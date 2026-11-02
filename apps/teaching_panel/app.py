"""Native Qt teaching console; opening the real UI never opens a device."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from PyQt5 import QtCore,QtGui,QtWidgets
from .camera import CameraWorker
from .controller import TeachingWorker,ROOT
from .recorder import read_episode

MODE={'disconnected':'未连接','readonly':'只读观察','hold':'位置保持','holding':'保持确认中',
      'teach':'拖动示教','returning':'回准备位中','error':'故障 · 保持未确认'}


class Bridge(QtCore.QObject):
    event=QtCore.pyqtSignal(str,object)


class ImageView(QtWidgets.QWidget):
    def __init__(self):
        super().__init__();self.setMinimumSize(540,280);self.pixmap=None;self.note='尚未打开 D435'
    def set_image(self,image,note):
        image=np.ascontiguousarray(image)
        self.pixmap=QtGui.QPixmap.fromImage(QtGui.QImage(image.data,image.shape[1],image.shape[0],
                    image.strides[0],QtGui.QImage.Format_RGB888).copy());self.note=note;self.update()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.fillRect(self.rect(),QtGui.QColor('#172c31'))
        if self.pixmap:
            scaled=self.pixmap.scaled(self.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation)
            p.drawPixmap((self.width()-scaled.width())//2,(self.height()-scaled.height())//2,scaled)
        else:
            p.setPen(QtGui.QColor('#adc4c6'));p.setFont(QtGui.QFont('Noto Sans CJK SC',14))
            p.drawText(self.rect(),QtCore.Qt.AlignCenter,'D435 实时画面\n点击“打开相机”开始预览')
        p.fillRect(0,self.height()-34,self.width(),34,QtGui.QColor(23,44,49,225))
        p.setPen(QtGui.QColor('#edf6f3'));p.setFont(QtGui.QFont('Noto Sans CJK SC',9))
        p.drawText(14,self.height()-12,self.note)


class TraceView(QtWidgets.QWidget):
    def __init__(self):
        super().__init__();self.setMinimumHeight(125);self.rows=[]
    def set_rows(self,rows):self.rows=rows;self.update()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.fillRect(self.rect(),QtGui.QColor('#ffffff'))
        area=QtCore.QRectF(42,22,self.width()-65,self.height()-52)
        p.setPen(QtGui.QPen(QtGui.QColor('#e0e8e5'),1));p.drawRect(area)
        p.setPen(QtGui.QColor('#677d7a'));p.drawText(8,16,'rad')
        colors=['#037c76','#c38333','#6b75ab','#ae5978','#5f8749','#816843','#152f37']
        if len(self.rows)>1:
            ts=np.array([r['relative_s'] for r in self.rows]);q=np.array([r['observation'] for r in self.rows])
            lo=float(q.min())-.08;hi=float(q.max())+.08;span=max(.01,ts[-1]-ts[0])
            for j,color in enumerate(colors):
                path=QtGui.QPainterPath()
                for i,t in enumerate(ts):
                    pt=QtCore.QPointF(area.left()+(t-ts[0])/span*area.width(),area.bottom()-(q[i,j]-lo)/(hi-lo)*area.height())
                    if i==0:path.moveTo(pt)
                    else:path.lineTo(pt)
                p.setPen(QtGui.QPen(QtGui.QColor(color),1.4));p.drawPath(path)
            p.setPen(QtGui.QColor('#677d7a'));p.drawText(int(area.right()-90),self.height()-5,f'{span:.1f} s')
        for i,color in enumerate(colors):
            p.setPen(QtGui.QColor(color));p.drawText(48+i*68,self.height()-5,f'J{i+1}' if i<6 else '夹爪')


class TeachingPanel(QtWidgets.QMainWindow):
    def __init__(self,*,demo=False,settings_path=None,store_base=None,dataset_base=None):
        super().__init__();self.demo=demo;self.setWindowTitle('ArmHub · 桌面抓取示教台'+(' · DEMO' if demo else ''))
        self.resize(1450,980)
        self.bridge=Bridge();self.bridge.event.connect(self.handle_event)
        self.camera=CameraWorker(self.bridge.event.emit,demo)
        self.worker=TeachingWorker(self.bridge.event.emit,self.camera,demo=demo,settings_path=settings_path,
                                  store_base=store_base,dataset_base=dataset_base)
        self.mode='disconnected';self.recording=False;self.exporting=False;self.closing=False
        self.current_folder=None;self.review_rows=[];self.review_meta=None;self.live_rows=[];self.plan_ready=False
        self.items={};self.fields={};self.review_mode=False;self.playing=False
        self.build()
        self.worker.start();self.camera.start()
        self.refresh_timer=QtCore.QTimer(self);self.refresh_timer.timeout.connect(self.refresh);self.refresh_timer.start(200)
        self.play_timer=QtCore.QTimer(self);self.play_timer.timeout.connect(self.play_step)
        self.camera_connected=False
        if demo:QtCore.QTimer.singleShot(200,self.start_demo)
        self.load_history()

    def start_demo(self):
        self.camera.open();self.worker.submit('connect');self.allowed.setChecked(True)

    def button(self,text,action,layout,style=None):
        button=QtWidgets.QPushButton(text);button.setMinimumHeight(35)
        if style:button.setObjectName(style)
        button.clicked.connect(lambda:self.send(action));layout.addWidget(button);return button

    def build(self):
        shell=QtWidgets.QWidget();self.setCentralWidget(shell);outer=QtWidgets.QVBoxLayout(shell);outer.setContentsMargins(20,16,20,14);outer.setSpacing(12)
        header=QtWidgets.QHBoxLayout()
        title=QtWidgets.QLabel('桌面抓取示教台');title.setObjectName('title');header.addWidget(title)
        subtitle=QtWidgets.QLabel('准备姿态  →  拖动与夹爪示教  →  回合审阅  →  LeRobot 数据集');header.addWidget(subtitle);header.addStretch()
        self.source=QtWidgets.QLabel('DEMO · 合成图像与反馈' if self.demo else 'Panthera-HT  /  D435');self.source.setObjectName('badge');header.addWidget(self.source);outer.addLayout(header)
        bar=QtWidgets.QHBoxLayout();self.mode_label=QtWidgets.QLabel('未连接');self.mode_label.setObjectName('mode');bar.addWidget(self.mode_label)
        self.camera_label=QtWidgets.QLabel('相机：未就绪');bar.addWidget(self.camera_label);bar.addStretch()
        self.allowed=QtWidgets.QCheckBox('允许实机动作' if not self.demo else '允许演示动作');self.allowed.toggled.connect(self.worker.permission);bar.addWidget(self.allowed)
        self.stop_button=QtWidgets.QPushButton('停止并保持  [空格]');self.stop_button.setObjectName('stop');self.stop_button.clicked.connect(self.stop);bar.addWidget(self.stop_button);outer.addLayout(bar)
        middle=QtWidgets.QHBoxLayout();outer.addLayout(middle,1)
        left=QtWidgets.QVBoxLayout();middle.addLayout(left,3)
        cam_bar=QtWidgets.QHBoxLayout();self.camera_button=QtWidgets.QPushButton('打开相机');self.camera_button.clicked.connect(self.toggle_camera);cam_bar.addWidget(self.camera_button)
        self.arm_button=QtWidgets.QPushButton('连接机械臂（只读）');self.arm_button.clicked.connect(lambda:self.send('connect' if self.mode=='disconnected' else 'disconnect'));cam_bar.addWidget(self.arm_button)
        self.live_button=QtWidgets.QPushButton('返回实时画面');self.live_button.clicked.connect(self.show_live);cam_bar.addWidget(self.live_button);left.addLayout(cam_bar)
        self.image=ImageView();left.addWidget(self.image,3)
        self.telemetry=QtWidgets.QLabel('J1–J6：—\n末端参考点 XYZ：—   |   夹爪实际 / 目标：—');self.telemetry.setObjectName('telemetry');self.telemetry.setMinimumHeight(58);left.addWidget(self.telemetry)
        self.progress=QtWidgets.QLabel('尚未录制 · 默认每回合手动开始、手动审阅');self.progress.setObjectName('progress');left.addWidget(self.progress)
        recordbar=QtWidgets.QHBoxLayout();self.record_button=self.button('开始录制  [F5]','record',recordbar,'primary');self.end_button=self.button('结束回合  [F6]','end',recordbar);left.addLayout(recordbar)
        self.trace=TraceView();left.addWidget(self.trace,1)
        self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.slider.setEnabled(False);self.slider.valueChanged.connect(self.review_frame);left.addWidget(self.slider)
        reviewbar=QtWidgets.QHBoxLayout();self.play_button=QtWidgets.QPushButton('播放记录');self.play_button.clicked.connect(self.toggle_play);reviewbar.addWidget(self.play_button)
        self.review_label=QtWidgets.QLabel('回看图像与曲线，不会驱动机械臂');reviewbar.addWidget(self.review_label);reviewbar.addStretch();left.addLayout(reviewbar)
        right_widget=QtWidgets.QWidget();right=QtWidgets.QVBoxLayout(right_widget);right.setContentsMargins(0,0,0,0)
        middle.addWidget(right_widget,2)
        prep=QtWidgets.QGroupBox('01  固定准备位');prep_layout=QtWidgets.QVBoxLayout(prep)
        self.prepare_label=QtWidgets.QLabel('尚未保存。先拖动到夹爪朝下的悬空姿态。');self.prepare_label.setWordWrap(True);prep_layout.addWidget(self.prepare_label)
        row=QtWidgets.QHBoxLayout();self.teach_button=self.button('进入拖动模式','teach',row,'primary');self.hold_button=QtWidgets.QPushButton('切回保持');self.hold_button.clicked.connect(self.stop);row.addWidget(self.hold_button);prep_layout.addLayout(row)
        row=QtWidgets.QHBoxLayout();self.save_button=self.button('保存当前为准备位','save_prepare',row);prep_layout.addLayout(row)
        row=QtWidgets.QHBoxLayout();self.preview_button=self.button('预览回准备位路径','preview',row);self.return_button=self.button('执行回准备位','return',row);prep_layout.addLayout(row)
        note=QtWidgets.QLabel('朝下姿态由你调整；回准备位保留当前夹爪开度。');note.setWordWrap(True);prep_layout.addWidget(note);right.addWidget(prep)
        grip=QtWidgets.QGroupBox('02  夹爪 · 目标与实际同步记录');grip_layout=QtWidgets.QHBoxLayout(grip)
        self.open_button=self.button('张开  [O]','open',grip_layout);self.close_button=self.button('闭合  [C]','close',grip_layout);right.addWidget(grip)
        config=QtWidgets.QGroupBox('03  采集与动作参数');form=QtWidgets.QFormLayout(config);form.setSpacing(3)
        for key,label in [('repo_id','数据集'),('task','任务说明')]:
            widget=QtWidgets.QLineEdit();self.fields[key]=widget;form.addRow(label,widget)
        for key,label,lo,hi,step,decimals in [('fps','采集 FPS',1,30,1,0),
             ('gripper_open','张开 / rad',0,2,.1,2),('gripper_close','闭合 / rad',0,2,.1,2),
             ('gripper_speed','夹爪速度上限 / rad/s',.01,.5,.05,2),('gripper_torque','夹爪力度上限 / Nm',.01,1,.05,2),
             ('return_scale','回准备位速度比例',.01,.3,.05,2),('return_duration_s','回位最短时长 / s',1,30,1,1)]:
            widget=QtWidgets.QDoubleSpinBox();widget.setRange(lo,hi);widget.setSingleStep(step);widget.setDecimals(decimals);self.fields[key]=widget;form.addRow(label,widget)
        floor_row=QtWidgets.QHBoxLayout();self.floor_enabled=QtWidgets.QCheckBox('启用 TCP Z 下限');self.floor_z=QtWidgets.QDoubleSpinBox();self.floor_z.setRange(-500,1500);self.floor_z.setSuffix(' mm');floor_row.addWidget(self.floor_enabled);floor_row.addWidget(self.floor_z);form.addRow(floor_row)
        note=QtWidgets.QLabel('Z 为模型末端参考点，不等于夹指尖离桌高度。\n桌面与指尖偏移尚需实测。');note.setWordWrap(True);form.addRow(note)
        self.right_tabs=QtWidgets.QTabWidget();config_scroll=QtWidgets.QScrollArea();config_scroll.setWidgetResizable(True);config_scroll.setFrameShape(QtWidgets.QFrame.NoFrame);config_scroll.setWidget(config);self.right_tabs.addTab(config_scroll,'参数设置');right.addWidget(self.right_tabs,1)
        history=QtWidgets.QGroupBox('04  回合审阅');hist_layout=QtWidgets.QVBoxLayout(history)
        self.history=QtWidgets.QListWidget();self.history.setMinimumHeight(90);self.history.currentItemChanged.connect(self.select_history);hist_layout.addWidget(self.history)
        row=QtWidgets.QHBoxLayout();self.keep_button=QtWidgets.QPushButton('成功 · 保留到训练集');self.keep_button.setObjectName('primary');self.keep_button.clicked.connect(lambda:self.decide('success'));row.addWidget(self.keep_button)
        self.fail_button=QtWidgets.QPushButton('失败 · 仅归档');self.fail_button.clicked.connect(lambda:self.decide('failure'));row.addWidget(self.fail_button)
        self.discard_button=QtWidgets.QPushButton('丢弃 / 重录');self.discard_button.clicked.connect(lambda:self.decide('discard'));row.addWidget(self.discard_button);hist_layout.addLayout(row)
        folder_button=QtWidgets.QPushButton('打开当前回合目录');folder_button.clicked.connect(self.open_folder);hist_layout.addWidget(folder_button);self.right_tabs.addTab(history,'回合列表')
        self.log=QtWidgets.QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumBlockCount(300);self.log.setMaximumHeight(85);outer.addWidget(self.log)
        self.statusBar().showMessage('启动不连接设备。先确认准备姿态与夹爪参数，再允许实机动作。')
        for key,fn in [('Space',self.stop),('F5',lambda:self.send('record')),('F6',lambda:self.send('end'))]:
            shortcut=QtWidgets.QShortcut(QtGui.QKeySequence(key),self);shortcut.setAutoRepeat(False);shortcut.activated.connect(fn)
        for key,action in [('O','open'),('C','close')]:
            shortcut=QtWidgets.QShortcut(QtGui.QKeySequence(key),self)
            shortcut.setAutoRepeat(False)
            shortcut.activated.connect(lambda a=action:self.gripper_shortcut(a))
        self.setStyleSheet('''
QMainWindow,QWidget {font-family:"Noto Sans CJK SC";font-size:12px;color:#233c3e;}
QMainWindow {background:#f2f5f1;} QGroupBox {background:white;border:1px solid #dce5de;border-radius:8px;margin-top:11px;padding:12px 10px 8px;font-weight:600;}
QGroupBox::title {subcontrol-origin:margin;left:12px;padding:0 4px;}
QPushButton {background:#edf3ee;border:1px solid #ccdad2;border-radius:5px;padding:7px 10px;}
QPushButton:hover {background:#dcebe2;} QPushButton:disabled {color:#9baaa3;background:#f1f3f0;}
QPushButton#primary {background:#087d72;color:white;border-color:#087d72;} QPushButton#primary:disabled {background:#b4ccc4;border-color:#b4ccc4;}
QPushButton#stop {background:#a34136;color:white;border:0;font-weight:700;padding:10px 24px;}
QLabel#title {font-size:24px;font-weight:700;} QLabel#badge {background:#dce9dd;border-radius:5px;padding:8px;color:#315f4e;}
QLabel#mode {background:#183e41;color:white;border-radius:5px;padding:7px 16px;font-weight:600;}
QLabel#telemetry,QLabel#progress {background:#e7efea;border-radius:5px;padding:8px;}
QDoubleSpinBox QLineEdit {padding:0;border:0;}
QLineEdit,QDoubleSpinBox,QListWidget,QPlainTextEdit {background:white;border:1px solid #d3dfd6;border-radius:4px;padding:4px;}
''')
        self.allowed.toggled.connect(self.refresh)
        self.refresh()

    def values(self):
        data={key:(widget.text() if isinstance(widget,QtWidgets.QLineEdit) else widget.value()) for key,widget in self.fields.items()}
        data['fps']=int(data['fps']);data['tcp_z_min_m']=self.floor_z.value()/1000 if self.floor_enabled.isChecked() else None
        return data

    def send(self,action):
        if action in ('teach','preview','record','open','close') and not self.recording:
            self.worker.submit('options',self.values())
        self.worker.submit(action)

    def stop(self):
        self.worker.request_stop();self.plan_ready=False;self.refresh()

    def gripper_shortcut(self,action):
        if isinstance(QtWidgets.QApplication.focusWidget(),(QtWidgets.QLineEdit,QtWidgets.QAbstractSpinBox)):
            return
        button=self.open_button if action=='open' else self.close_button
        if button.isEnabled():self.send(action)

    def toggle_camera(self):
        if self.camera.wanted.is_set():self.camera.close()
        else:self.camera.open();self.show_live()

    def show_live(self):self.review_mode=False;self.playing=False;self.play_timer.stop();self.play_button.setText('播放记录')

    def handle_event(self,event,value):
        if event=='mode':self.mode=value;self.mode_label.setText(MODE[value]);self.plan_ready=False
        elif event=='options':
            for key,widget in self.fields.items():
                if isinstance(widget,QtWidgets.QLineEdit):widget.setText(str(value[key]))
                else:widget.setValue(value[key])
            self.floor_enabled.setChecked(value['tcp_z_min_m'] is not None)
            if value['tcp_z_min_m'] is not None:self.floor_z.setValue(value['tcp_z_min_m']*1000)
        elif event=='prepared':
            q=' / '.join(f'{np.degrees(v):.1f}' for v in value['q'])
            self.prepare_label.setText(f'已保存 J1–J6 / °：{q}\nTCP Z：{value["position_m"][2]*1000:.1f} mm · {value["source"]}')
        elif event=='plan':
            self.plan_ready=True;self.log.appendPlainText(f'路径预检：{value["duration"]:.2f} s，最低 TCP Z {value["minimum_tcp_z_m"]*1000:.1f} mm；仅模型自碰撞检查，不含实测桌面。')
        elif event=='camera_state':
            if not self.review_mode:
                self.image.note={'opening':'正在连接 D435…','waiting':'等待 D435 新帧，正在重试…','closed':'相机预览已关闭','error':'相机预览已停止，请查看错误并重新打开'}.get(value,value)
                self.image.update()
        elif event=='camera_frame':
            if not self.review_mode:self.image.set_image(value['image'],'DEMO · 合成画面' if self.demo else 'D435 · RGB 640×480 · 实时预览')
        elif event=='feedback':
            q='  '.join(f'J{i+1} {np.degrees(v):6.1f}°' for i,v in enumerate(value['q']))
            xyz=' / '.join(f'{v*1000:.1f}' for v in value['pose']['position_m'])
            self.telemetry.setText(f'{q}\nTCP XYZ / mm：{xyz}    夹爪实际 / 目标：{value["gripper"]:.3f} / {value["target_gripper"]:.3f} rad    反馈年龄 {value["feedback_age_s"]*1000:.0f} ms')
            self.live_rows.append(dict(relative_s=value['monotonic_s'],observation=[*value['q'],value['gripper']]))
            self.live_rows=self.live_rows[-200:]
            if not self.review_mode:self.trace.set_rows(self.live_rows)
        elif event=='record_started':
            self.recording=True;self.review_mode=False;self.progress.setText('正在录制…');self.log.appendPlainText(f'原始回合：{value}')
        elif event=='record_progress':
            self.progress.setText(f'录制 {value["elapsed"]:.1f} s  ·  已写 {value["frames"]} 帧  ·  实测 {value["actual_fps"]:.1f} FPS')
        elif event=='record_stopping':self.recording=False;self.progress.setText('本回合结束，正在完成写盘和保持确认…')
        elif event=='review':
            self.recording=False;self.progress.setText(f'待审阅 · {value["frames"]} 帧 · {value.get("actual_fps",0):.1f} FPS · '+('采样完整' if value['quality_ok'] else '采样中断/不完整，不能进入训练集'))
            self.add_history(Path(value['folder']));self.select_folder(value['folder']);self.right_tabs.setCurrentIndex(1)
        elif event=='exporting':self.exporting=value;self.log.appendPlainText('正在编码并写入 LeRobot 数据集…' if value else '数据导出处理结束。')
        elif event=='exported':self.log.appendPlainText(f'已写入训练集：{value["root"]}，episode {value["episode_index"]}')
        elif event=='decision':
            self.add_history(Path(value['folder']));self.select_folder(value['folder']);self.progress.setText('回合已处理。回到准备位、摆好物体后再开始下一回合。')
        elif event in ('error','writer_error','log'):
            self.log.appendPlainText(('错误：' if event!='log' else '')+str(value))
            if event=='writer_error':self.recording=False;self.exporting=False
        elif event=='closed':self.log.appendPlainText('控制与数据线程已退出。')
        self.refresh()

    def refresh(self):
        if not hasattr(self,'record_button'):return
        frame=self.camera.latest();fresh=self.camera.wanted.is_set() and frame is not None and time.monotonic()-frame['monotonic_s']<.25
        self.camera_label.setText('相机：就绪' if fresh else '相机：未就绪 / 帧过期')
        self.camera_button.setText('关闭相机' if self.camera.wanted.is_set() else '打开相机')
        self.plan_ready=self.worker.plan is not None
        connected=self.mode!='disconnected';allowed=self.allowed.isChecked();busy=self.exporting or self.worker.writer_busy.is_set() or self.closing
        stable=self.mode in ('readonly','hold');teach=self.mode=='teach'
        self.arm_button.setText('断开机械臂' if connected else '连接机械臂（只读）')
        self.arm_button.setEnabled(not self.recording and not busy and self.mode not in ('holding','returning'))
        self.teach_button.setEnabled(connected and stable and allowed and not busy and not self.recording)
        self.hold_button.setEnabled(connected and not self.closing)
        self.save_button.setEnabled(connected and (stable or teach) and not self.recording and not busy)
        self.preview_button.setEnabled(connected and stable and not self.recording and not busy)
        self.return_button.setEnabled(connected and stable and allowed and self.plan_ready and not self.recording and not busy)
        for button in (self.open_button,self.close_button):button.setEnabled(connected and (stable or teach) and allowed and not busy)
        self.record_button.setEnabled(teach and allowed and fresh and not self.recording and not self.worker.pending_review.is_set() and self.worker.options['prepared'] is not None and not busy)
        self.end_button.setEnabled(self.recording and not self.closing)
        pending=(not connected or stable) and self.review_meta is not None and self.review_meta.get('status') in ('review','archived','error') and not self.review_meta.get('exported')
        for button in (self.keep_button,self.fail_button,self.discard_button):button.setEnabled(pending and not self.recording and not busy)
        self.keep_button.setEnabled(pending and self.review_meta.get('quality_ok',False) and not self.recording and not busy)
        for widget in [*self.fields.values(),self.floor_enabled,self.floor_z]:widget.setEnabled(not self.recording and not busy and self.mode not in ('returning','holding'))
        self.play_button.setEnabled(bool(self.review_rows) and not self.recording)
        if self.closing and not self.worker.is_alive() and not self.camera.is_alive():self.close()

    def load_history(self):
        base=self.worker.store.base/('demo' if self.demo else 'hardware')
        if base.exists():
            for path in sorted(base.glob('*/episode.json'))[-100:]:self.add_history(path.parent)

    def add_history(self,folder):
        meta=json.loads((folder/'episode.json').read_text())
        key=str(folder)
        if key not in self.items:
            item=QtWidgets.QListWidgetItem();item.setData(QtCore.Qt.UserRole,key);self.history.addItem(item);self.items[key]=item
        label={'review':'待审阅','exported':'成功 · 已入训练集','discarded':'已丢弃','archived':'失败 · 已归档','error':'异常 · 原始数据保留'}.get(meta['status'],meta['status'])
        self.items[key].setText(f'{folder.name}  |  {meta.get("frames",0)} 帧  |  {label}')

    def select_folder(self,folder):
        item=self.items.get(str(folder))
        if item:self.history.setCurrentItem(item);self.select_history(item,None)

    def select_history(self,item,previous):
        if not item or self.recording:return
        self.current_folder=Path(item.data(QtCore.Qt.UserRole))
        self.review_meta,self.review_rows=read_episode(self.current_folder)
        self.slider.setEnabled(bool(self.review_rows));self.slider.setRange(0,max(0,len(self.review_rows)-1));self.slider.setValue(0)
        self.trace.set_rows(self.review_rows);self.review_frame(0);self.refresh()

    def review_frame(self,index):
        if not self.review_rows or self.current_folder is None:return
        self.review_mode=True
        row=self.review_rows[index]
        pix=QtGui.QPixmap(str(self.current_folder/'images'/row['image']))
        self.image.pixmap=pix;self.image.note=f'回合回看 · 帧 {index+1}/{len(self.review_rows)} · {row["relative_s"]:.2f} s · 不驱动机械臂';self.image.update()
        self.review_label.setText(f'第 {index+1} 帧 · 夹爪实际 {row["observation"][6]:.3f} / 目标 {row["action"][6]:.3f} rad')

    def toggle_play(self):
        self.playing=not self.playing
        if self.playing:
            if self.slider.value()>=self.slider.maximum():self.slider.setValue(0)
            self.play_timer.start(int(1000/(self.review_meta or {}).get('fps',20)))
        else:self.play_timer.stop()
        self.play_button.setText('暂停回看' if self.playing else '播放记录')

    def play_step(self):
        if self.slider.value()>=self.slider.maximum():
            self.play_timer.stop();self.playing=False;self.play_button.setText('播放记录');return
        self.slider.setValue(self.slider.value()+1)

    def decide(self,outcome):
        if self.current_folder:
            self.worker.submit('decision',(str(self.current_folder),outcome))

    def open_folder(self):
        path=self.current_folder or self.worker.store.base
        if path.exists():QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))

    def closeEvent(self,event):
        if not self.closing:
            self.closing=True;self.play_timer.stop();self.worker.shutdown();self.camera.shutdown()
            self.statusBar().showMessage('正在结束回合、确认保持并释放设备，请等待…')
        if self.worker.is_alive() or self.camera.is_alive():event.ignore()
        else:self.refresh_timer.stop();event.accept()


def main():
    parser=argparse.ArgumentParser(description='ArmHub tabletop teaching panel')
    parser.add_argument('--demo',action='store_true',help='合成相机/机械臂，不访问设备')
    parser.add_argument('--camera-preview',action='store_true',help='只自动打开相机，不连接机械臂')
    parser.add_argument('--settings');parser.add_argument('--store-base');parser.add_argument('--dataset-base')
    parser.add_argument('--screenshot');parser.add_argument('--exit-after',type=float)
    args=parser.parse_args()
    app=QtWidgets.QApplication(sys.argv[:1]);app.setApplicationName('ArmHub Teaching')
    window=TeachingPanel(demo=args.demo,settings_path=args.settings,store_base=args.store_base,dataset_base=args.dataset_base);window.show()
    if args.camera_preview and not args.demo:QtCore.QTimer.singleShot(100,window.camera.open)
    if args.screenshot:
        def capture():
            path=Path(args.screenshot);path.parent.mkdir(parents=True,exist_ok=True);window.grab().save(str(path))
        QtCore.QTimer.singleShot(2200,capture)
    if args.exit_after:QtCore.QTimer.singleShot(int(args.exit_after*1000),window.close)
    return app.exec_()
