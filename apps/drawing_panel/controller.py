"""One hardware owner. UI publishes commands/events; only this worker touches the SDK."""
import csv
import hashlib
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import threading
import time

import numpy as np

from armhub import create_arm
from apps.movel.moveL_pos_control import PlanningRobot, CartesianCurve, checked
from apps.movel.circle_trajectory import plan_circle_curve
from apps.movel.moveL_trajectories import PLANES, SHAPES, shape_uv, to_waypoints, path_length, preflight
from apps.movel.moveL_error_analysis import record_cartesian_motion, save_run, analyze_run

ROOT=Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class DrawingSettings:
    shape: str='square'
    circle_mode: str='sdk_movej'
    plane: str='xy'
    size_mm: float=40.
    speed_mm_s: float=20.
    pitch_mm: float=10.
    z_min_mm: float=50.
    joint_scale: float=.2
    prepare_mm: tuple=(240.,0.,100.)

    def validate(self):
        values=(self.size_mm,self.speed_mm_s,self.pitch_mm,self.z_min_mm,self.joint_scale,*self.prepare_mm)
        if not all(math.isfinite(v) for v in values):raise ValueError('参数必须是有限数')
        if self.circle_mode not in ('reference','sdk_movej'):raise ValueError('未知画圆方式')
        if self.shape not in SHAPES or self.plane not in PLANES:raise ValueError('未知图形或平面')
        if not 20<=self.size_mm<=150:raise ValueError('尺寸需在 20–150 mm')
        if not 1<=self.speed_mm_s<=100:raise ValueError('绘图速度需在 1–100 mm/s')
        if self.z_min_mm<0 or not 0<self.joint_scale<=1:raise ValueError('高度或关节速度比例无效')
        return self


def drawing_waypoints(origin, settings, direction):
    settings.validate()
    if settings.shape=='spiral' and not 5<=settings.pitch_mm<=settings.size_mm/4:
        raise ValueError('螺旋圈距需在 5 mm 与尺寸/4 之间')
    if direction not in ('fwd','rev'):raise ValueError('方向必须为 fwd/rev')
    uv=shape_uv(settings.shape,settings.size_mm/1000,settings.pitch_mm/1000)
    uv=uv-uv[0]  # The saved origin is the first vertex, never the centre.
    if direction=='rev':uv=uv[::-1]
    return to_waypoints(origin['position'],origin['rotation'],settings.plane,uv)


def plan_drawing(robot, start, origin, settings, direction):
    if origin is None:raise ValueError('请先点击“记为绘图起点”')
    w=drawing_waypoints(origin,settings,direction)
    actual=robot.forward_kinematics(start)
    distance=np.linalg.norm(np.asarray(actual['position'])-w[0]['position'])
    if distance>.0015:
        raise ValueError(f'当前距本方向起点 {distance*1000:.1f} mm。先抬笔，再点“到所选方向起点”；不会自动画引入线。')
    angle=robot.kin.orientation_error(robot.kin.fk(start)['quaternion_xyzw'],origin['quaternion_xyzw'])
    if angle>.03:raise ValueError('末端姿态已改变，请重新设置绘图起点')
    check=preflight(robot,start,actual,w,settings.z_min_mm/1000)
    if not check['ok']:raise ValueError(check['reason'])
    duration=path_length(w)/(settings.speed_mm_s/1000)
    if settings.shape=='circle':
        curve=plan_circle_curve(robot,w,start,duration)
    else:
        qs=robot.path(w,start)
        curve=CartesianCurve(qs,duration,robot.arm.config)
    report=checked(robot,curve,start,settings.z_min_mm/1000)
    report.update(name=f'{settings.plane}_{settings.shape}_{direction}',requested_speed_mm_s=settings.speed_mm_s,
                  length_mm=path_length(w)*1000,preflight=check,start_distance_mm=distance*1000,
                  actual_average_speed_mm_s=path_length(w)*1000/curve.duration,
                  command_mode=settings.circle_mode if settings.shape=='circle' else 'reference',
                  policy='draw only; no lead, retreat, return or home')
    return w,curve,report


class HardwareWorker(threading.Thread):
    def __init__(self, emit, arm_factory=None, directory=None):
        super().__init__(name='armhub-drawing-owner',daemon=False)
        self.emit=emit;self.factory=arm_factory or (lambda:create_arm('panthera_ht','hardware'))
        self.commands=queue.Queue(maxsize=1)
        self.stop_event=threading.Event();self.quit_event=threading.Event()
        self.lock=threading.Lock();self.busy=False
        self.arm=None;self.robot=None;self.origin=None;self.last_states=None
        self.directory=Path(directory) if directory is not None else ROOT/'logs/drawing_panel'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True,exist_ok=False)
        self.csv_file=None;self.writer=None;self.tracing=False;self.action_index=0
        self.records=[]

    def submit(self,action,settings=None,direction='fwd',execute_allowed=False):
        with self.lock:
            if self.busy or self.quit_event.is_set():return False
            self.busy=True
            self.stop_event.clear()
            self.commands.put_nowait((action,settings,direction,execute_allowed))
        self.emit('busy',True)
        return True

    def request_stop(self):
        self.stop_event.set()

    def shutdown(self):
        self.quit_event.set();self.request_stop()

    def guard(self):return not self.stop_event.is_set() and not self.quit_event.is_set()

    def require_guard(self):
        if not self.guard():raise RuntimeError('已请求保持停止，取消本次动作')

    def log(self,message):self.emit('log',message)

    def observe(self,states):
        self.last_states=states
        q=[s.position for s in states[:6]]
        fk=self.robot.forward_kinematics(q)
        value=dict(position=list(fk['position']),q=q,dq=[s.velocity for s in states[:6]],
                   stamps=[s.time for s in states[:6]],gripper=states[6].position,
                   gripper_torque=states[6].torque,wall_time=time.time(),trace=self.tracing)
        if self.writer:
            self.writer.writerow([time.monotonic(),*value['stamps'],*q,*value['dq'],
                                  *value['position'],states[6].position,states[6].time,
                                  *[s.fault for s in states],*[s.num for s in states]])
        self.emit('feedback',value)

    def fresh(self):
        if self.arm is None:raise RuntimeError('请先连接机械臂')
        states=self.arm.read_feedback();self.observe(states)
        return np.array([s.position for s in states[:6]])

    def connect(self):
        if self.arm is not None:return
        arm=self.factory()
        try:
            arm.connect()
            start=np.asarray(arm.get_state().positions)
            self.robot=PlanningRobot(arm,start)
            arm._kinematics_cache=self.robot.kin
            arm.prepare_motion_model()
        except BaseException:
            arm.close();self.robot=None;raise
        self.arm=arm
        self.csv_file=(self.directory/'telemetry.csv').open('w',newline='',buffering=1)
        self.writer=csv.writer(self.csv_file)
        self.writer.writerow(['host_monotonic_s',*[f'J{i}_receive_unix_s' for i in range(1,7)],
            *[f'J{i}_rad' for i in range(1,7)],*[f'J{i}_velocity_rad_s' for i in range(1,7)],
            'tcp_x_m','tcp_y_m','tcp_z_m','gripper_raw_rad','gripper_receive_unix_s',
            *[f'fault_{i}' for i in range(1,8)],*[f'counter_{i}' for i in range(1,8)]])
        self.fresh();self.emit('connected',True)
        self.log('只读连接完成；未回零、未开合夹爪。')

    def close_arm(self):
        if self.arm is not None:
            try:self.arm.close()
            finally:self.arm=None;self.robot=None;self.origin=None;self.last_states=None
        if self.csv_file:self.csv_file.close();self.csv_file=None;self.writer=None
        self.emit('connected',False)

    def run_curve(self,curve):
        self.require_guard()
        self.arm.start_reference_trajectory(curve,expected_start=curve.sample(0)[0],command_guard=self.guard)
        while True:
            session=self.arm._reference_execution
            wait=session.next_due-time.monotonic()
            if not session.final_sent and wait>0:self.stop_event.wait(wait)
            result=self.arm.advance_reference_trajectory()
            self.observe(session.states)
            if result is not None:return result

    def snapshot_origin(self):
        q=self.fresh();fk=self.robot.forward_kinematics(q)
        self.origin=dict(position=list(fk['position']),rotation=np.asarray(fk['rotation']),
                         quaternion_xyzw=self.robot.kin.fk(q)['quaternion_xyzw'])
        self.emit('origin',self.origin)
        self.log('已用新反馈记录绘图起点。只影响后续路径，不移动机械臂。')

    def action(self,action,settings,direction,allowed):
        if action=='connect':return self.connect()
        if action=='disconnect':return self.close_arm()
        if action=='origin':return self.snapshot_origin()
        if action=='refresh':self.fresh();return
        s=(settings or DrawingSettings()).validate()
        moving=action not in ('preview',)
        if moving and not allowed:raise RuntimeError('请先勾选“允许执行实机动作”')
        q=self.fresh();self.robot.seed=q.copy()
        if action in ('preview','draw'):
            w,curve,report=plan_drawing(self.robot,q,self.origin,s,direction)
            plan_p=[self.robot.forward_kinematics(curve.sample(t)[0])['position'] for t in np.linspace(0,curve.duration,501)]
            self.emit('plan',dict(waypoints=[x['position'].tolist() for x in w],plan=plan_p,report=report,plane=s.plane))
            self.log(f"预检通过：{report['length_mm']:.1f} mm / {curve.duration:.2f} 秒，设定 {s.speed_mm_s:g} mm/s，方式 {report['command_mode']}")
            if action=='preview':return
            self.require_guard();self.arm.enable();self.tracing=True;self.emit('trace_start',direction)
            self.action_index+=1
            folder=self.directory/f'{self.action_index:03d}_{report["name"]}'
            records=[];error=None
            geometry=dict(origin=np.round(self.origin['position'],6).tolist(),rotation=np.round(self.origin['rotation'],6).tolist(),
                          shape=s.shape,plane=s.plane,size=s.size_mm,pitch=s.pitch_mm,speed=s.speed_mm_s,command_mode=report['command_mode'])
            pair_id=hashlib.sha256(json.dumps(geometry,sort_keys=True).encode()).hexdigest()[:10]
            try:
                record_cartesian_motion(self.robot,report['name'],w,curve.duration,.5,records,
                    tags=dict(plane=s.plane,shape=s.shape,direction=direction,pair_id=pair_id,command_mode=report['command_mode']),curve=curve,
                    command_guard=self.guard,on_feedback=self.observe,command_mode=report['command_mode'])
            except BaseException as exc:
                error=exc
            finally:
                self.tracing=False
                if records:
                    save_run(folder,records,dict(source='drawing_panel_hardware_encoder_fk',
                        requested_speed_mm_s=s.speed_mm_s,policy=report['policy'],preview=report,
                        status='failed' if error else 'completed',error=str(error) if error else None))
                    self.log(f'原始轨迹已保存：{folder}')
                    self.emit('saved',str(folder))
                    # Compute/export after all motor activity has stopped.
                    try:
                        analyze_run(folder)
                        export_xy(folder,records[0],s.plane)
                    except Exception as exc:self.log(f'出图失败，原始数据已保留：{exc}')
                    self.records.extend(records)
                    comparison=self.directory/'comparison'
                    save_run(comparison,self.records,dict(source='drawing_panel_session_encoder_fk',
                        note='Compare only matching geometry, size, origin and orientation.'))
                    if len(self.records)>1:
                        try:
                            analyze_run(comparison)
                            self.log(f'累计轨迹与正反向对比：{comparison}')
                        except Exception as exc:self.log(f'累计对比出图失败：{exc}')
            if error:raise error
            self.log('绘图完成；末端保持在本条路径终点。')
            return
        self.require_guard();self.arm.enable()
        if action in ('open','close'):
            target=1.6 if action=='open' else 0.
            self.arm.command_gripper(target,speed=.3,torque=.3,command_guard=self.guard)
            began=time.monotonic();anchor=None;since=began
            while True:
                self.require_guard()
                done=self.arm.poll_gripper();self.observe(self.arm._last_states)
                if done:break
                now=time.monotonic();grip=self.arm._last_states[6].position
                if anchor is None or abs(grip-anchor)>.003:anchor=grip;since=now
                if action=='close' and now-began>=1 and now-since>=.6:
                    self.arm.stop();self.log('关爪可能接触物体，已保持；请手动确认笔是否夹稳。');break
            self.log('夹爪动作结束，六轴保持原位。');return
        if action in ('up','down','start'):
            pose=self.robot.forward_kinematics(q)
            if action=='start':
                if self.origin is None:raise ValueError('请先设置绘图起点')
                target=drawing_waypoints(self.origin,s,direction)[0]
            else:
                pos=np.asarray(pose['position']);pos[2]+=.01 if action=='up' else -.01
                target=dict(position=pos,rotation=pose['rotation'])
            distance=np.linalg.norm(np.asarray(target['position'])-pose['position'])
            if distance<.0001:self.log('已在目标位置，无需运动。');return
            qs=self.robot.path([pose,target],q)
            curve=CartesianCurve(qs,max(1.,distance/(s.speed_mm_s/1000)),self.arm.config)
            checked(self.robot,curve,q,s.z_min_mm/1000)
        elif action in ('home','prepare'):
            if action=='home':
                preview=self.arm.preview_home(velocity_scale=s.joint_scale)
                curve=self.arm._build_transition(q,preview['target_joints_rad'],preview['duration_s'],preview['effective_max_speeds_rad_s'])
                # Home may be outside the drawing height floor; use its existing joint/self-collision checks.
            else:
                rotation=np.array([[0,0,1],[0,1,0],[-1,0,0]])
                goal=self.robot.solve(np.asarray(s.prepare_mm)/1000,rotation,q)
                limits=tuple(v*s.joint_scale for v in self.arm.config['velocity_limits_rad_s'])
                curve=self.arm._build_transition(q,goal,3.,limits)
                checked(self.robot,curve,q,s.z_min_mm/1000)
        else:raise ValueError(f'未知动作 {action}')
        self.require_guard();self.log(f'执行 {action}，参考时长 {curve.duration:.2f} 秒')
        self.tracing=True;self.emit('trace_start','motion')
        try:self.run_curve(curve)
        finally:self.tracing=False
        self.log('运动到位并已确认稳定。若要在新高度绘图，请重新记为绘图起点。')

    def run(self):
        try:
            while not self.quit_event.is_set():
                try:command=self.commands.get(timeout=.12)
                except queue.Empty:
                    if self.arm is not None:
                        try:self.fresh()
                        except Exception as exc:
                            self.log(f'只读反馈失败：{exc}；已断开，请检查后重新连接。')
                            self.close_arm()
                    continue
                try:self.action(*command)
                except BaseException as exc:
                    if self.arm is not None and self.arm._moving:
                        try:self.arm.stop()
                        except Exception as stop_error:self.log(f'停止未确认：{stop_error}')
                    self.log(f'动作未完成：{type(exc).__name__}: {exc}')
                    self.emit('error',str(exc))
                finally:
                    with self.lock:self.busy=False
                    self.emit('busy',False)
        finally:
            try:self.close_arm()
            finally:self.emit('closed',True)


def export_xy(folder,rec,plane):
    """Standalone position plot and CSV to compare against ink on paper."""
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    folder=Path(folder);i,j=PLANES[plane]
    reference=rec['waypoints'][0]
    fig,ax=plt.subplots(figsize=(7,7),layout='constrained')
    for key,label,color,style in [('waypoints','Ideal','#6b7280','--'),('plan_p','Planned FK','#d28a21','-'),('fb_p','Encoder FK','#067c83','-')]:
        points=rec[key]
        if len(points):
            p=(points-reference)*1000
            ax.plot(p[:,i],p[:,j],style,color=color,label=label,lw=1.2)
    ax.set(xlabel=f'{"XYZ"[i]} relative to start (mm)',ylabel=f'{"XYZ"[j]} relative to start (mm)',
           title=f'{rec["name"]} | encoder FK, not external metrology',aspect='equal')
    ax.grid(alpha=.25);ax.legend();fig.savefig(folder/'feedback_path.png',dpi=180);plt.close(fig)
    with (folder/'feedback_xyz.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['t_s','x_m','y_m','z_m','dx_mm','dy_mm','dz_mm'])
        for t,p in zip(rec['fb_t'],rec['fb_p']):writer.writerow([t,*p,*((p-reference)*1000)])
