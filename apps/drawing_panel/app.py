"""Native drawing control panel. Qt owns the UI; one worker owns hardware."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .controller import DrawingSettings, HardwareWorker, ROOT
from apps.movel.moveL_error_analysis import load_run
from apps.movel.moveL_trajectories import PLANES

SHAPES={'正方形':'square','直线（平面第一轴）':'line_u','直线（平面第二轴）':'line_v','对角线':'diagonal','圆形':'circle','回字螺旋':'spiral'}


class Bridge(QtCore.QObject):
    event=QtCore.pyqtSignal(str,object)


class PathCanvas(QtWidgets.QWidget):
    def __init__(self):
        super().__init__();self.setMinimumSize(550,410)
        self.plane='xy';self.origin=None;self.ideal=[];self.planned=[]
        self.traces=[];self.current=None;self.cursor=None;self.reverse=False

    def clear(self):
        self.traces=[];self.current=None;self.update()

    def begin(self,direction):
        self.current={'direction':direction,'points':[]};self.traces.append(self.current)
        self.traces=self.traces[-8:];self.update()

    def point(self,value):
        self.cursor=value['position']
        if value.get('trace') and self.current is not None:
            self.current['points'].append(value['position'])
            if len(self.current['points'])>30000:self.current['points']=self.current['points'][::2]
        self.update()

    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(),QtGui.QColor('#ffffff'))
        area=QtCore.QRectF(64,34,self.width()-92,self.height()-94)
        i,j=PLANES[self.plane]
        ref=np.asarray(self.origin if self.origin is not None else (self.ideal[0] if len(self.ideal) else (self.cursor if self.cursor is not None else [0,0,0])))
        sets=[self.ideal,self.planned]+[t['points'] for t in self.traces]
        arrays=[(np.asarray(a)-ref)*1000 for a in sets if len(a)]
        if arrays:
            bounds=np.vstack(arrays)[:,[i,j]];lo=bounds.min(axis=0);hi=bounds.max(axis=0)
        else:lo=np.array([-10.,-10.]);hi=np.array([50.,50.])
        centre=(lo+hi)/2;span=max(20.,float(np.max(hi-lo))*1.18)
        scale=min(area.width(),area.height())/span
        def screen(position):
            uv=(np.asarray(position)-ref)[[i,j]]*1000
            return QtCore.QPointF(area.center().x()+(uv[0]-centre[0])*scale,
                                 area.center().y()-(uv[1]-centre[1])*scale)
        def local(u,v):
            pos=ref.copy();pos[i]+=u/1000;pos[j]+=v/1000;return screen(pos)
        tick=5 if span<=70 else (10 if span<=150 else 50)
        start=np.floor((centre-span/2)/tick)*tick
        p.setFont(QtGui.QFont('Noto Sans CJK SC',8))
        p.setClipRect(area.adjusted(-1,-1,1,1))
        for axis in (0,1):
            for v in np.arange(start[axis],centre[axis]+span/2+tick,tick):
                p.setPen(QtGui.QPen(QtGui.QColor('#edf0f2'),1))
                a,b=(local(v,centre[1]-span),local(v,centre[1]+span)) if axis==0 else (local(centre[0]-span,v),local(centre[0]+span,v))
                p.drawLine(a,b)
        for points,color,width,style in [(self.ideal,'#8c98a5',1.4,QtCore.Qt.DashLine),(self.planned,'#d69a32',1.3,QtCore.Qt.SolidLine)]+[(t['points'],'#b06aba' if t['direction']=='rev' else '#058c87',2.,QtCore.Qt.SolidLine) for t in self.traces]:
            if not len(points):continue
            path=QtGui.QPainterPath(screen(points[0]))
            for point in points[1:]:path.lineTo(screen(point))
            p.setPen(QtGui.QPen(QtGui.QColor(color),width,style));p.drawPath(path)
        if len(self.ideal):
            p.setPen(QtGui.QPen(QtGui.QColor('#1c3347'),1.5));p.setBrush(QtCore.Qt.white)
            p.drawEllipse(screen(self.ideal[0]),4,4)
        if self.cursor is not None:
            p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor('#1b354b'));p.drawEllipse(screen(self.cursor),3,3)
        p.setClipping(False);p.setBrush(QtCore.Qt.NoBrush);p.setPen(QtGui.QPen(QtGui.QColor('#d9e1e6'),1));p.drawRect(area)
        p.setPen(QtGui.QColor('#647585'))
        p.drawText(QtCore.QRectF(64,area.bottom()+28,area.width(),25),QtCore.Qt.AlignCenter,f'{"XYZ"[i]} / mm，相对绘图起点')
        p.save();p.translate(18,area.center().y());p.rotate(-90)
        p.drawText(QtCore.QRectF(-130,-12,260,24),QtCore.Qt.AlignCenter,f'{"XYZ"[j]} / mm，相对绘图起点');p.restore()
        for axis in (0,1):
            for v in np.arange(start[axis],centre[axis]+span/2+tick,tick):
                pt=local(v,centre[1]) if axis==0 else local(centre[0],v)
                if axis==0 and area.left()<=pt.x()<=area.right():
                    p.drawText(QtCore.QRectF(pt.x()-22,area.bottom()+4,44,20),QtCore.Qt.AlignCenter,f'{v:g}')
                elif axis==1 and area.top()<=pt.y()<=area.bottom():
                    p.drawText(QtCore.QRectF(22,pt.y()-10,36,20),QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter,f'{v:g}')
        p.drawText(66,22,'等比例轨迹 · 基座坐标系')


class DrawingPanel(QtWidgets.QMainWindow):
    def __init__(self, *, replay=None, auto_connect=False):
        super().__init__();self.setWindowTitle('ArmHub · 夹笔与绘图控制台');self.resize(1280,900)
        self.worker=None;self.connected=False;self.busy=False;self.closing=False;self.last_folder=None
        self.last_feedback=None;self.reference=None;self.replay_mode=replay is not None
        self.bridge=Bridge();self.bridge.event.connect(self.receive)
        self.motion_buttons=[];self.idle_buttons=[]
        self._build()
        self.setStyleSheet('''
            QMainWindow,QWidget#root {background:#f2f5f7;color:#203447;}
            QLabel {color:#203447;} QGroupBox {font-weight:600;background:white;border:1px solid #dae3e8;border-radius:8px;margin-top:13px;padding-top:12px;}
            QGroupBox::title {subcontrol-origin:margin;left:12px;padding:0 6px;}
            QPushButton {background:#fff;border:1px solid #c9d6dd;border-radius:6px;padding:5px 9px;color:#244154;min-height:14px;}
            QPushButton:hover {background:#edf7f6;border-color:#0d918c;} QPushButton:disabled {color:#a2adb5;background:#f3f5f6;}
            QPushButton#primary {background:#078b83;color:white;border:1px solid #078b83;font-weight:600;}
            QPushButton#primary:disabled {background:#e1e8eb;color:#98a8af;border-color:#d9e1e6;}
            QPushButton#stop {background:#b64043;color:white;border:none;font-weight:600;}
            QDoubleSpinBox,QComboBox {background:white;border:1px solid #cdd9df;border-radius:5px;padding:3px;min-height:16px;}
            QPlainTextEdit {background:#f7f9fa;border:1px solid #dde5ea;border-radius:6px;color:#476071;}
            QCheckBox {padding:7px 2px;} QCheckBox::indicator {width:17px;height:17px;}
        ''')
        QtWidgets.QShortcut(QtGui.QKeySequence('Escape'),self,activated=self.stop)
        if replay is not None:self.load_replay(replay)
        else:
            self.worker=HardwareWorker(self.bridge.event.emit);self.worker.start()
            self.log(f'本次数据目录：{self.worker.directory}')
            if auto_connect:QtCore.QTimer.singleShot(200,lambda:self.send('connect'))
        self.update_buttons()

    def group(self,title,parent):
        box=QtWidgets.QGroupBox(title);layout=QtWidgets.QGridLayout(box);layout.setSpacing(5);parent.addWidget(box)
        return layout

    def button(self,text,action,layout,row,column,span=1,motion=False,**kwargs):
        button=QtWidgets.QPushButton(text);button.clicked.connect(lambda:self.send(action,**kwargs));layout.addWidget(button,row,column,1,span)
        (self.motion_buttons if motion else self.idle_buttons).append(button)
        return button

    def spin(self,low,high,value,suffix='',decimals=1):
        w=QtWidgets.QDoubleSpinBox();w.setRange(low,high);w.setDecimals(decimals);w.setValue(value);w.setSuffix(suffix);return w

    def _build(self):
        root=QtWidgets.QWidget();root.setObjectName('root');self.setCentralWidget(root)
        whole=QtWidgets.QVBoxLayout(root);whole.setContentsMargins(24,16,24,16);whole.setSpacing(12)
        header=QtWidgets.QHBoxLayout();title=QtWidgets.QLabel('ArmHub  /  绘图工作台');title.setFont(QtGui.QFont('Noto Sans CJK SC',19,QtGui.QFont.Bold));header.addWidget(title);header.addStretch()
        self.status=QtWidgets.QLabel('● 未连接');header.addWidget(self.status);whole.addLayout(header)
        subtitle=QtWidgets.QLabel('Panthera-HT · 位置控制  |  反馈重建轨迹，与纸面笔迹对比');subtitle.setStyleSheet('color:#71828e;');whole.addWidget(subtitle)
        body=QtWidgets.QHBoxLayout();body.setSpacing(18);whole.addLayout(body,1)
        left_widget=QtWidgets.QWidget();left_widget.setFixedWidth(358);left=QtWidgets.QVBoxLayout(left_widget);left.setContentsMargins(0,0,0,0);left.setSpacing(9);body.addWidget(left_widget)
        connect_layout=QtWidgets.QHBoxLayout();self.connect_btn=QtWidgets.QPushButton('连接机械臂（只读）');self.connect_btn.clicked.connect(lambda:self.send('disconnect' if self.connected else 'connect'));connect_layout.addWidget(self.connect_btn);left.addLayout(connect_layout)
        self.allow=QtWidgets.QCheckBox('允许执行实机动作');self.allow.stateChanged.connect(self.permission_changed);left.addWidget(self.allow)
        g=self.group('01  准备与装笔',left)
        self.button('回到 Home','home',g,0,0,motion=True)
        self.button('到绘图准备位','prepare',g,0,1,motion=True)
        pos=QtWidgets.QHBoxLayout();self.prepare=[]
        for label,value in [('X',240),('Y',0),('Z',100)]:
            pos.addWidget(QtWidgets.QLabel(label));w=self.spin(-600,600,value,'',0);w.setMaximumWidth(78);self.prepare.append(w);pos.addWidget(w)
        g.addLayout(pos,1,0,1,2)
        self.joint_scale=self.spin(5,100,20,' %',0);g.addWidget(QtWidgets.QLabel('准备 / Home 速度'),2,0);g.addWidget(self.joint_scale,2,1)
        self.button('打开夹爪','open',g,3,0,motion=True);self.button('关闭夹爪','close',g,3,1,motion=True)
        self.button('↑ 上升 1 cm','up',g,4,0,motion=True);self.button('↓ 下降 1 cm','down',g,4,1,motion=True)
        note=QtWidgets.QLabel('准备位 XYZ：mm。回 Home / 准备位走关节路径，\n先抬笔离纸；升降保持姿态，不自动找纸面。');note.setStyleSheet('color:#7b868d;font-size:11px;');g.addWidget(note,5,0,1,2)
        g=self.group('02  图形与速度',left)
        self.shape=QtWidgets.QComboBox();self.shape.addItems(SHAPES);g.addWidget(QtWidgets.QLabel('图形'),0,0);g.addWidget(self.shape,0,1)
        self.plane=QtWidgets.QComboBox();self.plane.addItems(['xy','xz','yz']);g.addWidget(QtWidgets.QLabel('绘图平面'),1,0);g.addWidget(self.plane,1,1)
        self.size=self.spin(20,150,40,' mm');g.addWidget(QtWidgets.QLabel('边长 / 直径'),2,0);g.addWidget(self.size,2,1)
        self.speed=self.spin(1,100,20,' mm/s');g.addWidget(QtWidgets.QLabel('请求绘图速度'),3,0);g.addWidget(self.speed,3,1)
        self.pitch=self.spin(5,35,10,' mm');g.addWidget(QtWidgets.QLabel('螺旋圈距'),4,0);g.addWidget(self.pitch,4,1)
        self.z_min=self.spin(0,500,50,' mm');g.addWidget(QtWidgets.QLabel('TCP 最低高度'),5,0);g.addWidget(self.z_min,5,1)
        self.circle_mode=QtWidgets.QComboBox();self.circle_mode.addItem('SDK moveJ（试验）','sdk_movej');self.circle_mode.addItem('原连续参考','reference')
        g.addWidget(QtWidgets.QLabel('画圆方式'),6,0);g.addWidget(self.circle_mode,6,1)
        self.button('记为绘图起点','origin',g,7,0,2)
        self.origin_label=QtWidgets.QLabel('起点：未记录（装好笔并调整高度后记录）');self.origin_label.setWordWrap(True);self.origin_label.setStyleSheet('color:#71828e;font-size:11px');g.addWidget(self.origin_label,8,0,1,2)
        self.button('预检正向','preview',g,9,0,direction='fwd');self.button('预检反向','preview',g,9,1,direction='rev')
        self.draw_fwd=self.button('▶ 正向画','draw',g,10,0,motion=True,direction='fwd');self.draw_fwd.setObjectName('primary')
        self.draw_rev=self.button('◀ 反向画','draw',g,10,1,motion=True,direction='rev');self.draw_rev.setObjectName('primary')
        self.button('到正向起点','start',g,11,0,motion=True,direction='fwd');self.button('到反向起点','start',g,11,1,motion=True,direction='rev')
        self.stop_btn=QtWidgets.QPushButton('■ 保持停止  /  Esc');self.stop_btn.setObjectName('stop');self.stop_btn.clicked.connect(self.stop);left.addWidget(self.stop_btn)
        left.addStretch()
        right=QtWidgets.QVBoxLayout();right.setSpacing(10);body.addLayout(right,1)
        top=QtWidgets.QHBoxLayout();label=QtWidgets.QLabel('反馈轨迹');label.setFont(QtGui.QFont('Noto Sans CJK SC',13,QtGui.QFont.Bold));top.addWidget(label);top.addStretch()
        self.plot_plane=QtWidgets.QComboBox();self.plot_plane.addItems(['XY','XZ','YZ']);self.plot_plane.currentTextChanged.connect(self.set_plot_plane);top.addWidget(self.plot_plane)
        clear=QtWidgets.QPushButton('清空画布');clear.clicked.connect(lambda:self.canvas.clear());top.addWidget(clear);right.addLayout(top)
        self.canvas=PathCanvas();right.addWidget(self.canvas,1)
        legend=QtWidgets.QLabel('<span style="color:#8c98a5">┄ 理想路径</span>　 <span style="color:#d69a32">━ 规划 FK</span>　 <span style="color:#058c87">━ 正向反馈</span>　 <span style="color:#b06aba">━ 反向反馈</span>');legend.setStyleSheet('color:#526d78;padding:3px;');right.addWidget(legend)
        self.plan_label=QtWidgets.QLabel('预检后显示实际时长；速度受关节速度、加速度、jerk 限制。');self.plan_label.setWordWrap(True);right.addWidget(self.plan_label)
        self.pose_label=QtWidgets.QLabel('TCP  X —  Y —  Z — mm    |    夹爪 — rad');self.pose_label.setStyleSheet('background:white;padding:10px;border-radius:6px;');right.addWidget(self.pose_label)
        self.joint_label=QtWidgets.QLabel('J1 —   J2 —   J3 —   J4 —   J5 —   J6 —');self.joint_label.setStyleSheet('color:#71828e;');right.addWidget(self.joint_label)
        bottom=QtWidgets.QHBoxLayout();save=QtWidgets.QPushButton('保存当前画布 PNG');save.clicked.connect(self.save_canvas);bottom.addWidget(save)
        folder=QtWidgets.QPushButton('打开数据文件夹');folder.clicked.connect(self.open_folder);bottom.addWidget(folder);bottom.addStretch();right.addLayout(bottom)
        self.log_box=QtWidgets.QPlainTextEdit();self.log_box.setReadOnly(True);self.log_box.setMaximumHeight(112);self.log_box.setMaximumBlockCount(250);right.addWidget(self.log_box)
        footer=QtWidgets.QLabel('数据来源：六轴编码器关节角 → URDF FK。未标定笔尖偏移时图像代表 TCP 轨迹；不是外部测量，也没有接触力控制。');footer.setWordWrap(True);footer.setStyleSheet('color:#7b8993;font-size:11px;');whole.addWidget(footer)

    def settings(self):
        return DrawingSettings(shape=SHAPES[self.shape.currentText()],circle_mode=self.circle_mode.currentData(),plane=self.plane.currentText(),
            size_mm=self.size.value(),speed_mm_s=self.speed.value(),pitch_mm=self.pitch.value(),
            z_min_mm=self.z_min.value(),joint_scale=self.joint_scale.value()/100,
            prepare_mm=tuple(w.value() for w in self.prepare)).validate()

    def send(self,action,direction='fwd'):
        if self.worker is None:return
        try:s=self.settings()
        except ValueError as exc:self.log(str(exc));return
        if self.worker.submit(action,s,direction,self.allow.isChecked()):
            self.busy=True;self.update_buttons()
            self.log(f'请求：{action} {direction if action in ("draw","preview","start") else ""}')
        else:self.log('已有动作在进行，请等待或按保持停止。')

    def permission_changed(self,*_):
        if not self.allow.isChecked() and self.busy and self.worker is not None:
            self.worker.request_stop()
            self.log('执行许可已撤销，已请求保持停止。')
        self.update_buttons()

    def stop(self):
        if self.worker:self.worker.request_stop();self.log('已请求保持停止；等待控制线程确认。')

    def update_buttons(self,*_):
        ready=self.connected and not self.busy and not self.replay_mode
        for button in self.idle_buttons:button.setEnabled(ready)
        for button in self.motion_buttons:button.setEnabled(ready and self.allow.isChecked())
        self.connect_btn.setEnabled(not self.busy and not self.replay_mode)
        self.connect_btn.setText('断开连接' if self.connected else '连接机械臂（只读）')
        self.stop_btn.setEnabled(self.connected and not self.replay_mode)
        self.status.setText('● 执行 / 处理中' if self.busy else ('● 已连接 · 待命' if self.connected else ('● 历史反馈回放' if self.replay_mode else '● 未连接')))

    def log(self,message):self.log_box.appendPlainText(time.strftime('%H:%M:%S')+'  '+message)

    @QtCore.pyqtSlot(str,object)
    def receive(self,kind,value):
        if kind=='busy':self.busy=value;self.update_buttons()
        elif kind=='connected':
            self.connected=value
            if not value:
                self.allow.setChecked(False);self.reference=None;self.origin_label.setText('起点：未记录')
                self.pose_label.setText('未连接 · 画布保留之前的轨迹，不是实时反馈')
                self.joint_label.setText('J1 —   J2 —   J3 —   J4 —   J5 —   J6 —')
            self.update_buttons()
        elif kind=='log':self.log(value)
        elif kind=='error':self.plan_label.setText('未执行 / 已中止：'+value);self.plan_label.setStyleSheet('color:#a53d40;')
        elif kind=='feedback':
            self.last_feedback=value;self.canvas.point(value)
            x,y,z=np.array(value['position'])*1000
            grip=f"{value['gripper']:.3f} rad" if np.isfinite(value['gripper']) else '历史数据未记录'
            prefix='历史 TCP' if self.replay_mode else 'TCP'
            self.pose_label.setText(f'{prefix}   X {x:.2f}   Y {y:.2f}   Z {z:.2f} mm    |    夹爪 {grip}')
            self.joint_label.setText('    '.join(f'J{i+1} {q*180/np.pi:.1f}°' for i,q in enumerate(value['q'])))
        elif kind=='origin':
            self.reference=value;self.canvas.origin=value['position'];self.canvas.clear()
            self.canvas.ideal=[];self.canvas.planned=[]
            self.origin_label.setText('起点 XYZ：'+' / '.join(f'{v*1000:.2f}' for v in value['position'])+' mm')
        elif kind=='plan':
            self.canvas.ideal=value['waypoints'];self.canvas.planned=value['plan'];self.canvas.plane=value['plane']
            self.plot_plane.setCurrentText(value['plane'].upper());self.canvas.update()
            r=value['report'];self.plan_label.setText(f"预检通过  ·  路长 {r['length_mm']:.1f} mm  ·  实际参考时长 {r['duration_s']:.2f} s  ·  请求 {r['requested_speed_mm_s']:g} mm/s  ·  {r.get('command_mode','reference')}")
            self.plan_label.setStyleSheet('color:#087f78;')
        elif kind=='trace_start':self.canvas.begin(value)
        elif kind=='saved':self.last_folder=Path(value)
        elif kind=='closed':
            if self.closing:self.worker=None;self.close()

    def set_plot_plane(self,value):self.canvas.plane=value.lower();self.canvas.update()

    def save_canvas(self):
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,'保存反馈轨迹',str(self.last_folder/'canvas.png') if self.last_folder else str(ROOT/'logs/feedback_canvas.png'),'PNG (*.png)')
        if path:
            self.canvas.grab().save(path);self.log(f'画布已保存：{path}')

    def open_folder(self):
        path=self.last_folder or (self.worker.directory if self.worker else ROOT/'logs/movel')
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))

    def load_replay(self,folder):
        run,records=load_run(folder);self.last_folder=Path(folder)
        if not records:return
        for rec in records[-8:]:
            self.canvas.plane=rec.get('tags',{}).get('plane','xy')
            self.canvas.ideal=rec['waypoints'].tolist();self.canvas.planned=rec['plan_p'].tolist()
            self.canvas.begin(rec.get('tags',{}).get('direction','fwd'))
            self.canvas.current['points']=rec['fb_p'].tolist()
        rec=records[-1];self.canvas.origin=rec['waypoints'][0];self.canvas.cursor=rec['fb_p'][-1] if len(rec['fb_p']) else None
        self.plot_plane.setCurrentText(self.canvas.plane.upper())
        if len(rec['fb_q']):
            self.receive('feedback',dict(position=rec['fb_p'][-1],q=rec['fb_q'][-1],gripper=float('nan'),trace=False))
        self.plan_label.setText(f'历史实机反馈回放 · {len(records)} 条轨迹 · 当前界面不连接硬件')
        self.log(f'历史数据：{folder}')

    def closeEvent(self,event):
        if self.worker is not None and self.worker.is_alive():
            event.ignore();self.closing=True;self.allow.setChecked(False);self.worker.shutdown()
            self.log('正在停止并关闭连接，确认后自动退出。')
        else:event.accept()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--shape',choices=list(SHAPES.values()),help='初始选择的图形，不自动运动')
    p.add_argument('--connect',action='store_true',help='启动后自动只读连接，不运动')
    p.add_argument('--replay',type=Path,help='仅回放已有记录，不构造硬件会话')
    p.add_argument('--snapshot',type=Path,help='离线保存界面截图后退出；需与 --replay 同用')
    args=p.parse_args()
    if args.snapshot and not args.replay:p.error('--snapshot 仅用于离线回放')
    if args.connect and args.replay:p.error('--connect 和 --replay 不能同用')
    app=QtWidgets.QApplication(sys.argv[:1]);app.setFont(QtGui.QFont('Noto Sans CJK SC',10))
    panel=DrawingPanel(replay=args.replay,auto_connect=args.connect)
    if args.shape:panel.shape.setCurrentText(next(name for name,value in SHAPES.items() if value==args.shape))
    panel.show()
    if args.snapshot:
        def capture():
            args.snapshot.parent.mkdir(parents=True,exist_ok=True);panel.grab().save(str(args.snapshot));panel.close()
        QtCore.QTimer.singleShot(500,capture)
    sys.exit(app.exec_())


if __name__=='__main__':main()
