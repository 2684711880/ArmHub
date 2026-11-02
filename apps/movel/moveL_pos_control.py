"""Panthera 实机平面轨迹实验；默认只读预览，--execute 执行，不自动回零。"""
import argparse
from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import make_interp_spline

from armhub import create_arm
from armhub.execution import CheckedCurve
from armhub.adapters.panthera_kinematics import PantheraKinematics
from .moveL_trajectories import PLANES, SHAPES, DIRECTIONS, to_waypoints, shape_uv, path_length, preflight
from .moveL_error_analysis import record_cartesian_motion, save_run, analyze_run
from .circle_trajectory import plan_circle_curve

ROOT = Path(__file__).resolve().parents[2]


class PlanningRobot:
    """只借用官方 SDK 的数学方法；不构造 Panthera，不打开串口。"""
    def __init__(self, arm, seed):
        self.arm = arm
        c = arm.config
        self.kin = PantheraKinematics(c['lower_rad'], c['upper_rad'])
        from Panthera_lib import Panthera
        self.sdk = self.kin.sdk
        for name in ['_interpolate_segment', '_compute_segment_steps', '_has_joint_jump']:
            setattr(self.sdk, name, getattr(Panthera,name).__get__(self.sdk))
        original_ik = self.sdk.inverse_kinematics
        def strict_ik(*args, **kwargs):
            kwargs.update(eps=1e-5, max_iter=1500)
            return original_ik(*args, **kwargs)
        self.sdk.inverse_kinematics = strict_ik
        self.sdk.eef_step = self.eef_step = .002
        self.sdk.jump_threshold = .15
        self.resample_dt = c['motion_smoothing']['control_period_s']
        self.motor_count = 6
        self.joint_limits = self.sdk.joint_limits
        self._interpolate_segment = self.sdk._interpolate_segment
        self.seed = np.asarray(seed,dtype=float)
        self.sdk.seed = self.seed.copy()

    def forward_kinematics(self, q=None):
        result = self.sdk.forward_kinematics(self.seed if q is None else q)
        # Pinocchio exposes views into mutable model data. Recorded poses must
        # survive subsequent FK/IK calls made for other samples or planning.
        return dict(result, rotation=np.array(result['rotation'],copy=True),
                    transform=np.array(result['transform'],copy=True),
                    joint_angles=np.array(result['joint_angles'],copy=True))

    def solve(self, position, rotation, seed):
        from scipy.spatial.transform import Rotation
        return np.asarray(self.kin.solve(seed,position,Rotation.from_matrix(rotation).as_quat())['target_joints_rad'])

    def path(self, waypoints, seed):
        qs = [np.asarray(seed)]
        for a,b in zip(waypoints[:-1],waypoints[1:]):
            segment, ok = self._interpolate_segment(a,b,qs[-1])
            if not ok or not segment:
                raise ValueError('笛卡尔路径 IK 失败或关节跳变')
            if np.max(np.abs(np.asarray(segment[0])-qs[-1])) > self.sdk.jump_threshold:
                raise ValueError('首个 IK 点关节跳变')
            qs.extend(segment)
        return np.asarray(qs)


class CartesianCurve:
    """对稠密 IK 点做五次样条，端点 qdot/qddot 为零；按 q/v/a/jerk 限制放慢。"""
    def __init__(self, qs, duration, config):
        length = np.linalg.norm(np.diff(qs,axis=0),axis=1)
        keep = np.r_[True, length > 1e-9]
        qs = qs[keep]
        if len(qs)<2:
            raise ValueError('路径没有有效位移')
        u = np.r_[0,np.cumsum(np.linalg.norm(np.diff(qs,axis=0),axis=1))]
        u /= u[-1]
        # 小路径补足五次插值点。
        if len(qs)<6:
            unew=np.linspace(0,1,6)
            qs=np.column_stack([np.interp(unew,u,qs[:,j]) for j in range(6)])
            u=unew
        zeros=np.zeros(6)
        self.spline=make_interp_spline(u,qs,k=5,bc_type=([(1,zeros),(2,zeros)],[(1,zeros),(2,zeros)]))
        self.speed_limits=tuple(config['velocity_limits_rad_s'])
        settings=config['motion_smoothing']
        self.duration=max(float(duration),.5)
        # 包含各 knot 邻域，避免圆弧密集点与角点处漏检峰值。
        grid=np.unique(np.r_[np.linspace(0,1,4001),u,(u[:-1]+u[1:])/2])
        peaks=[np.max(np.abs(self.spline(grid,nu=n)),axis=0) for n in [1,2,3]]
        self.duration=max(self.duration,
            float(np.max(peaks[0]/np.asarray(self.speed_limits))),
            float(np.sqrt(np.max(peaks[1]/np.asarray(settings['acceleration_limits_rad_s2'])))),
            float(np.cbrt(np.max(peaks[2]/np.asarray(settings['jerk_limits_rad_s3']))))) * 1.03

    def sample(self,t):
        u=float(np.clip(t/self.duration,0,1))
        if t<=0 or t>=self.duration:
            return self.spline(u),np.zeros(6),np.zeros(6)
        return tuple(self.spline(u,nu=n)/self.duration**n for n in [0,1,2])


def checked(robot, curve, start, z_min):
    c=CheckedCurve(curve,robot.arm.config,start)
    collision=robot.kin.check_trajectory(c,.02)
    samples=list(c.collision_samples(.02))
    heights=[robot.forward_kinematics(q)['position'][2] for q in samples]
    if min(heights)<z_min:
        raise ValueError(f'样条/转移段最低 TCP 高度 {min(heights):.4f} m < {z_min}')
    return dict(duration_s=curve.duration,min_tcp_z_m=min(heights),collision=collision,
                timing=getattr(curve,'timing_report',None))


def build_plan(robot,args,start):
    c=robot.arm.config
    from_current=getattr(args,'path_from_current',False) or args.square_from_current
    pose=robot.forward_kinematics(start)
    if args.anchor is None:
        aq=np.asarray(start);ap=np.asarray(pose['position']);ar=pose['rotation']
    else:
        ar=np.array([[0,0,1],[0,1,0],[-1,0,0]])
        aq=robot.solve(args.anchor,ar,start)
        actual=robot.forward_kinematics(aq);ap=np.asarray(actual['position']);ar=actual['rotation']
    anchor=dict(position=ap,rotation=ar)
    initial=None
    report=dict(anchor_m=ap.tolist(),anchor_q_rad=aq.tolist(),initial=None,motions=[])
    if np.max(np.abs(aq-start))>1e-8:
        initial=robot.arm._build_transition(start,aq,3.,None)
        report['initial']=checked(robot,initial,start,args.z_min)
    plans=[]
    for plane in args.plane:
        for shape in args.trajectories:
            uv=shape_uv(shape,args.size,args.pitch)
            for direction in DIRECTIONS if args.direction=='both' else (args.direction,):
                name=f'{plane}_{shape}_{direction}'
                w=to_waypoints(ap,ar,plane,uv if direction=='fwd' else uv[::-1])
                if from_current:
                    offset=np.asarray(w[0]['position'])-ap
                    w=[dict(position=np.asarray(point['position'])-offset,rotation=point['rotation']) for point in w]
                check=preflight(robot,aq,anchor,w,args.z_min)
                if not check['ok']:
                    raise ValueError(f'{name}: {check["reason"]}')
                lead_q=robot.path([anchor,w[0]],aq) if not from_current else np.array([aq])
                lead=(CartesianCurve(lead_q,max(1.,np.linalg.norm(w[0]['position']-ap)/args.speed),c)
                      if not from_current else None)
                if shape=='circle':
                    curve=plan_circle_curve(robot,w,lead_q[-1],path_length(w)/args.speed)
                    curve_q=np.array([curve.sample(curve.duration)[0]])
                else:
                    curve_q=robot.path(w,lead_q[-1])
                    curve=CartesianCurve(curve_q,path_length(w)/args.speed,c)
                back=robot.arm._build_transition(curve_q[-1],aq,2.,None) if not from_current else None
                entry=dict(name=name,tags=dict(plane=plane,shape=shape,direction=direction),
                           preflight=check,lead=checked(robot,lead,aq,args.z_min) if lead is not None else None,
                           motion=checked(robot,curve,lead_q[-1],args.z_min),
                           return_to_anchor=checked(robot,back,curve_q[-1],args.z_min) if back is not None else None)
                report['motions'].append(entry)
                plans.append(dict(name=name,tags=entry['tags'],waypoints=w,lead=lead,curve=curve,back=back))
                print(f"预检 {name}: 运动 {curve.duration:.2f}s，最小高度 {entry['motion']['min_tcp_z_m']:.3f}m",flush=True)
    return initial,plans,report


def execute(robot,curve):
    robot.arm.start_reference_trajectory(curve,expected_start=curve.sample(0)[0])
    while True:
        session=robot.arm._reference_execution
        delay=session.next_due-time.monotonic()
        if not session.final_sent and delay>0:time.sleep(delay)
        if robot.arm.advance_reference_trajectory() is not None:return


def parse_args():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plane',nargs='+',choices=PLANES,default=['xy'])
    p.add_argument('--trajectories',nargs='+',choices=SHAPES,default=list(SHAPES))
    p.add_argument('--direction',choices=['fwd','rev','both'],default='both')
    p.add_argument('--size',type=float,default=.08)
    p.add_argument('--pitch',type=float,default=.01)
    p.add_argument('--speed',type=float,default=.05)
    p.add_argument('--anchor',nargs=3,type=float,help='显式转移到此锚点与原脚本姿态；默认当前实测位置和姿态')
    p.add_argument('--z-min',type=float,default=.05)
    p.add_argument('--settle',type=float,default=.5)
    p.add_argument('--q',nargs=6,type=float,help='完全离线预检的起点 rad；不能与 --execute 同用')
    p.add_argument('--command-mode',choices=['reference','sdk_movej'],default='reference',help='圆形可试验官方 SDK moveJ 连续下发')
    p.add_argument('--path-from-current',action='store_true',help='当前点为路径首点，不添加引入或返回段；仅单平面/单图形/单方向')
    p.add_argument('--square-from-current',action='store_true',help='当前 TCP 为正方形首个角点，只走四边，不引入或返回中心')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    values=[args.size,args.pitch,args.speed,args.z_min,args.settle,*(args.anchor or []),*(args.q or [])]
    if not all(math.isfinite(x) for x in values):p.error('参数必须为有限数')
    if not .02<=args.size<=.15:p.error('--size 需在 .02–.15 m')
    if not .005<=args.pitch<=args.size/4:p.error('--pitch 需在 .005 与 size/4 m 之间')
    if not .01<=args.speed<=.1:p.error('--speed 需在 .01–.1 m/s')
    if args.z_min<0 or not 0<=args.settle<=5:p.error('z-min/settle 超出范围')
    if args.square_from_current and (args.trajectories!=['square'] or len(args.plane)!=1 or args.direction=='both' or args.anchor is not None):
        p.error('--square-from-current 只支持单平面 square、单方向，不能指定 --anchor')
    if args.path_from_current and (len(args.trajectories)!=1 or len(args.plane)!=1 or args.direction=='both' or args.anchor is not None):
        p.error('--path-from-current 需单平面/单图形/单方向，不能指定 anchor')
    if args.command_mode=='sdk_movej' and args.trajectories!=['circle']:
        p.error('SDK moveJ 试验目前只开放圆形')
    if args.q is not None and args.execute:p.error('实机执行必须使用实时反馈')
    return args


def main():
    args=parse_args()
    output=args.output or ROOT/'logs/movel'/time.strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True,exist_ok=False)
    arm=create_arm('panthera_ht','hardware')
    records=[];robot=None;meta=dict(started_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        backend='hardware',execute=args.execute,feedback='fresh host receive time, encoder FK; no external metrology',
        command_reference='continuous planned reference at feedback time; actual dispatched references also stored',
        command_mode=args.command_mode,
        return_policy=('path only; starts at current TCP; no lead or return' if args.square_from_current or args.path_from_current
                       else 'each shape returns to anchor; final stays at anchor; no automatic home'),
        tcp='SDK joint6 + local X 0.165 m',planes=args.plane,trajectories=args.trajectories,direction=args.direction,
        size_m=args.size,pitch_m=args.pitch,speed_m_s=args.speed,z_min_m=args.z_min,status='preflight')
    try:
        if args.q is None:
            arm.connect()
            start=np.asarray(arm.get_state().positions)
        else:
            start=np.asarray(args.q)
        robot=PlanningRobot(arm,start)
        print(f'起点关节 rad: {start.tolist()}',flush=True)
        # SDK 数学层有大量逐点输出；预检记录完整保存在独立日志。
        with (output/'preflight.log').open('w') as log,redirect_stdout(log):
            initial,plans,report=build_plan(robot,args,start)
        (output/'preview.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        for item in report['motions']:
            print(f"{item['name']}: {item['motion']['duration_s']:.2f}s",flush=True)
        meta.update(anchor_m=report['anchor_m'],eef_step_m=robot.eef_step,resample_dt_s=robot.resample_dt)
        if not args.execute:
            meta['status']='preview_passed'
            print(f'预检通过，未发送运动命令：{output}',flush=True)
            return
        arm._kinematics_cache=robot.kin
        arm.enable()
        if initial is not None:execute(robot,initial)
        for item in plans:
            print(f"执行 {item['name']}",flush=True)
            if item['lead'] is not None:execute(robot,item['lead'])
            record_cartesian_motion(robot,item['name'],item['waypoints'],item['curve'].duration,args.settle,
                                    records,dict(item['tags'],command_mode=args.command_mode),curve=item['curve'],command_mode=args.command_mode)
            if item['back'] is not None:execute(robot,item['back'])
        meta['status']='completed'
    except BaseException as exc:
        meta.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        if arm._moving:
            try:arm.stop()
            except Exception as stop_error:meta['stop_error']=str(stop_error)
        raise
    finally:
        arm.close()
        if records:
            save_run(output,records,meta)
            analyze_run(output)
        else:
            (output/'run.json').write_text(json.dumps(dict(meta,motions=[]),ensure_ascii=False,indent=2)+'\n')
        print(f'结果目录：{output}',flush=True)


if __name__=='__main__':main()
