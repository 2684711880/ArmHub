"""Teaching state machine. Only this worker calls the mechanical-arm session."""
from collections import deque
import copy
import json
import math
from pathlib import Path
import queue
import re
import threading
import time

from .hardware import DemoSession, HardwareSession
from .recorder import EpisodeStore, atomic_json

ROOT=Path(__file__).resolve().parents[2]
DEFAULTS=dict(repo_id='local/panthera_teaching',task='pick an object from the tabletop and place it',
              fps=20,gripper_open=1.6,gripper_close=0.,gripper_speed=.3,
              gripper_torque=.3,return_scale=.15,return_duration_s=3.,tcp_z_min_m=None,prepared=None)


def validate_options(options):
    values=dict(DEFAULTS,**options)
    values.pop('duration_s',None)  # Retire the earlier per-episode timeout setting.
    if not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_-]+',values['repo_id']):raise ValueError('数据集名称使用 local/任务名，仅字母数字下划线和连字符')
    for key in ('gripper_open','gripper_close','gripper_speed','gripper_torque','return_scale','return_duration_s'):
        if not math.isfinite(float(values[key])):raise ValueError(f'{key} 必须是有限数')
    if not 1<=int(values['fps'])<=30:
        raise ValueError('采集 FPS 需为 1–30')
    if not 0<=values['gripper_open']<=2 or not 0<=values['gripper_close']<=2:
        raise ValueError('夹爪开合目标需在 0–2 原始电机 rad')
    if not 0<values['gripper_speed']<=.5 or not 0<values['gripper_torque']<=1:
        raise ValueError('夹爪速度需在 (0,.5]、力度上限需在 (0,1]')
    if not 0<values['return_scale']<=.3 or not 1<=values['return_duration_s']<=30:
        raise ValueError('回准备位速度比例需在 (0,.3]，最短时长需在 1–30 秒')
    if values['tcp_z_min_m'] is not None and not math.isfinite(float(values['tcp_z_min_m'])):
        raise ValueError('Z 下限必须是有限数')
    if not values['task'].strip():raise ValueError('请填写任务说明')
    p=values['prepared']
    if p is not None:
        for key,size in [('q',6),('position_m',3),('quaternion_xyzw',4)]:
            if len(p[key])!=size or not all(math.isfinite(float(v)) for v in p[key]):
                raise ValueError('保存的准备位配置无效')
        if p['source'] not in ('demo','hardware'):raise ValueError('准备位来源无效')
    return values


class TeachingWorker(threading.Thread):
    def __init__(self,emit,camera,*,demo=False,settings_path=None,store_base=None,dataset_base=None,session_factory=None):
        super().__init__(name='teaching-sdk-owner',daemon=False)
        self.emit=emit;self.camera=camera;self.demo=demo
        self.factory=session_factory or (DemoSession if demo else HardwareSession)
        self.settings_path=Path(settings_path) if settings_path else (
            ROOT/'logs/teaching_panel/demo_settings.json' if demo else
            ROOT/'integrations/panthera_lerobot/configs/teaching.json')
        self.options=validate_options(json.loads(self.settings_path.read_text()) if self.settings_path.exists() else {})
        if demo:self.options['repo_id']='demo/panthera_teaching'
        self.session=None;self.mode='disconnected';self.sample=None;self.plan=None
        self.commands=queue.Queue(maxsize=8);self.quit_event=threading.Event();self.stop_event=threading.Event()
        self.allowed=threading.Event();self.pending_review=threading.Event();self.writer_busy=threading.Event()
        self.epoch=0;self.epoch_lock=threading.Lock()
        self.recording=False;self.record_started=0.;self.next_frame=0.;self.ending=None
        self.pending_save=False;self.history=deque(maxlen=100);self.last_camera_sequence=-1
        self.store=EpisodeStore(self.store_event,store_base,dataset_base)

    def store_event(self,event,value):
        if event=='decision':self.pending_review.clear();self.writer_busy.clear()
        if event=='writer_error':
            if self.store.folder is None or not (self.store.folder/'samples.jsonl').is_file():self.pending_review.clear()
            self.writer_busy.clear();self.request_stop()
        self.emit(event,value)

    def submit(self,action,value=None):
        with self.epoch_lock:epoch=self.epoch
        try:self.commands.put_nowait((epoch,action,value));return True
        except queue.Full:self.emit('error','命令队列繁忙，请等待当前操作');return False

    def permission(self,enabled):
        if enabled:self.allowed.set()
        else:self.allowed.clear();self.request_stop()

    def request_stop(self):
        with self.epoch_lock:self.epoch+=1
        self.stop_event.set()

    def shutdown(self):self.quit_event.set();self.request_stop()
    def guard(self):return self.allowed.is_set() and not self.stop_event.is_set() and not self.quit_event.is_set()

    def set_mode(self,mode):self.mode=mode;self.emit('mode',mode)

    def hold(self):
        self.plan=None
        if self.session is not None:
            self.session.hold();self.set_mode('holding')

    def end_record(self,reason='completed'):
        if self.recording:
            self.recording=False;self.ending=reason
            self.emit('record_stopping',reason)

    def _flush_end(self):
        if self.ending and self.store.post('end',self.ending):self.ending=None

    def stationary(self):
        if len(self.history)<3 or self.history[-1][0]-self.history[0][0]<.45:return False
        recent=[q for stamp,q in self.history if stamp>=self.history[-1][0]-.5]
        return all(max(row[i] for row in recent)-min(row[i] for row in recent)<.003 for i in range(6))

    def save_prepare(self):
        s=self.sample
        if not self.stationary():raise ValueError('尚未确认静止，请先保持并等待反馈稳定')
        self.options['prepared']=dict(q=list(s['q']),position_m=list(s['pose']['position_m']),
             quaternion_xyzw=list(s['pose']['quaternion_xyzw']),source=s['source'],
             saved_unix_s=time.time(),orientation_policy='user_arranged; not an automatic downward-pose guarantee')
        atomic_json(self.settings_path,self.options)
        self.pending_save=False;self.plan=None
        self.emit('prepared',self.options['prepared']);self.emit('log','准备位已保存；夹爪方向由你手动确认。')

    def action(self,action,value):
        if action=='options':
            if self.recording or self.writer_busy.is_set():raise ValueError('录制/导出期间不能修改设置')
            prepared=self.options['prepared']
            self.options=validate_options(dict(self.options,**value));self.options['prepared']=prepared
            if self.demo:self.options['repo_id']='demo/panthera_teaching'
            atomic_json(self.settings_path,self.options);self.plan=None;return
        if action=='decision':
            if self.session and self.mode not in ('hold','readonly'):raise ValueError('请先保持，再归档或导出数据')
            folder,_=value
            resolved=Path(folder).resolve()
            if not resolved.is_relative_to(self.store.base.resolve()):raise ValueError('回合不属于本采集目录')
            if self.pending_review.is_set() and self.store.folder and resolved!=self.store.folder.resolve():
                raise ValueError('请先处理本次尚未决定的回合')
            if self.recording or self.ending:raise ValueError('回合尚未写完')
            self.writer_busy.set()
            if not self.store.post('decide',value):self.writer_busy.clear();raise RuntimeError('写盘队列繁忙')
            return
        if self.writer_busy.is_set():raise ValueError('正在归档/导出回合，请等待完成')
        if action=='connect':
            if self.session is not None:return
            session=self.factory()
            try:self.sample=session.connect()
            except BaseException:session.close();raise
            self.session=session;self.set_mode('readonly')
            self.emit('log','只读连接完成，没有回零、移动或开合夹爪。');return
        if self.session is None:raise ValueError('请先连接机械臂')
        if action=='disconnect':
            if self.recording:raise ValueError('请先结束本回合')
            self.session.close();self.session=None;self.set_mode('disconnected');return
        if action=='save_prepare':
            if self.recording:raise ValueError('录制中不能更改准备位')
            if self.mode=='teach':self.pending_save=True;self.hold()
            elif self.mode in ('readonly','hold'):self.save_prepare()
            else:raise ValueError('请先保持，再保存准备位')
            return
        if action=='preview':
            if self.mode not in ('readonly','hold'):raise ValueError('请先保持，再预览回准备位路径')
            p=self.options['prepared']
            if not p or p['source']!=self.sample['source']:raise ValueError('请先保存本运行模式的准备位')
            self.plan=self.session.preview(self.sample,p,self.options['return_scale'],
                                          self.options['return_duration_s'],self.options['tcp_z_min_m'])
            self.emit('plan',{k:v for k,v in self.plan.items() if k!='curve'});return
        if not self.allowed.is_set():raise ValueError('请先勾选“允许实机动作”')
        if self.mode in ('holding','returning','error'):raise ValueError('当前模式不允许开始新动作，请先等待保持确认')
        if action=='teach':
            self.plan=None;self.session.teach();self.set_mode('teach')
        elif action in ('open','close'):
            if self.mode not in ('teach','hold','readonly'):raise ValueError('当前不能操作夹爪')
            target=self.options['gripper_'+action]
            self.session.gripper(target,self.options['gripper_speed'],self.options['gripper_torque'],self.sample)
            self.emit('gripper_target',target);self.plan=None
            if self.mode=='readonly':self.set_mode('hold')
        elif action=='return':
            if self.recording:raise ValueError('录制中不能回准备位')
            if self.plan is None:raise ValueError('请先预览准备位路径')
            if max(abs(a-b) for a,b in zip(self.sample['q'],self.plan['start']))>.02:
                self.plan=None;raise ValueError('起点已变化，请重新预览')
            plan=self.plan;self.plan=None
            self.session.start_return(plan,self.guard);self.set_mode('returning')
        elif action=='record':
            if self.mode!='teach':raise ValueError('先进入拖动模式，再开始示教录制')
            if self.recording or self.ending or self.pending_review.is_set():raise ValueError('请先处理上一回合')
            prepared=self.options['prepared']
            if not prepared or prepared['source']!=self.sample['source']:raise ValueError('请先保存当前模式的准备位')
            if max(abs(a-b) for a,b in zip(prepared['q'],self.sample['q']))>.08:
                raise ValueError('当前不在准备位附近，请先结束拖动、预览并回准备位')
            frame=self.camera.latest()
            if frame is None or time.monotonic()-frame['monotonic_s']>.2:raise ValueError('相机帧未就绪或已过期')
            if frame['source']!=self.sample['source']:raise ValueError('相机与机械臂来源不一致')
            self.pending_review.set();self.record_started=time.monotonic();self.next_frame=self.record_started
            self.last_camera_sequence=-1
            meta=dict(repo_id=self.options['repo_id'],task=self.options['task'],fps=int(self.options['fps']),
                      prepared=copy.deepcopy(prepared),source=self.sample['source'],
                      options=copy.deepcopy(self.options),started_unix_s=time.time())
            if not self.store.post('begin',meta):self.pending_review.clear();raise RuntimeError('写盘队列繁忙')
            self.recording=True
        elif action=='end':
            self.end_record();self.hold()
        else:raise ValueError(f'未知操作：{action}')

    def tick(self):
        if self.session is None:return
        s=self.session.read();self.sample=s;now=time.monotonic()
        self.history.append((now,s['q']))
        floor=self.options['tcp_z_min_m']
        if floor is not None and s['pose']['position_m'][2]<floor and self.mode in ('teach','returning'):
            raise RuntimeError('实测末端参考点低于 Z 下限')
        if self.mode=='teach':self.session.teach_tick(s)
        elif self.mode=='holding' and self.session.poll_hold(s):self.set_mode('hold')
        elif self.mode=='returning' and self.session.advance_return(s):
            self.set_mode('hold');self.emit('log','已按新反馈确认到达准备位，夹爪保持原状。')
        if self.pending_save and self.mode=='hold' and self.stationary():self.save_prepare()
        public={k:v for k,v in s.items() if k!='states'}
        public['target_gripper']=self.session.gripper_target
        self.emit('feedback',public)
        if self.recording:
            elapsed=now-self.record_started
            if now>=self.next_frame:
                frame=self.camera.latest()
                if frame is None or now-frame['monotonic_s']>.2 or frame['sequence']==self.last_camera_sequence:
                    raise RuntimeError('录制相机帧过期或重复，已中断本回合')
                self.last_camera_sequence=frame['sequence']
                row=dict(monotonic_s=now,relative_s=elapsed,camera_host_receipt_s=frame['monotonic_s'],
                         image_age_s=now-frame['monotonic_s'],feedback_time=s['feedback_time'],
                         feedback_counters=s['feedback_counters'],
                         observation=[*s['q'],s['gripper']],
                         action=[*s['q'],float(self.session.gripper_target)])
                if not self.store.post('frame',(row,frame['image'])):raise RuntimeError('图像写盘队列已满，本回合中断')
                self.next_frame+=1/self.options['fps']
                self.emit('record_progress',dict(elapsed=elapsed,frames=self.store.count,
                                               actual_fps=self.store.count/max(elapsed,.001)))

    def run(self):
        self.store.start();self.emit('options',self.options)
        if self.options['prepared']:self.emit('prepared',self.options['prepared'])
        try:
            while not self.quit_event.is_set():
                began=time.monotonic()
                try:
                    if self.stop_event.is_set():
                        self.stop_event.clear();self.end_record()
                        if self.session:self.hold()
                    self._flush_end()
                    try:epoch,action,value=self.commands.get_nowait()
                    except queue.Empty:pass
                    else:
                        if epoch==self.epoch:
                            try:self.action(action,value)
                            except Exception as exc:
                                self.emit('error',str(exc))
                                if action in ('teach','open','close','return') and self.session:
                                    self.end_record('interrupted')
                                    try:self.hold()
                                    except Exception as hold_error:self.set_mode('error');self.emit('error',f'保持未确认：{hold_error}')
                    self.tick()
                except Exception as exc:
                    self.end_record('interrupted');self.emit('error',str(exc))
                    if self.session and self.mode not in ('holding','error'):
                        try:self.hold()
                        except Exception as stop_error:self.set_mode('error');self.emit('error',f'保持未确认：{stop_error}')
                    elif self.session:self.set_mode('error')
                self.quit_event.wait(max(0.,.02-(time.monotonic()-began)))
        finally:
            self.end_record('interrupted')
            while self.ending:
                self._flush_end()
                if self.ending:time.sleep(.02)
            if self.session:
                try:self.session.close()
                except Exception as exc:self.emit('error',f'退出时保持/关闭未确认：{exc}')
            self.store.shutdown();self.store.join()
            self.emit('closed',True)
