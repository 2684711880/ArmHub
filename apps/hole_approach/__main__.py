"""洞口应用入口：离线、相机预览、只读机械臂预览、显式交互执行。"""
import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from .config import ROOT, DEFAULT_CONFIG, load_config, resolve, load_calibration


def arguments(argv=None):
    pre=argparse.ArgumentParser(add_help=False)
    pre.add_argument('--config',default=str(DEFAULT_CONFIG))
    known,_=pre.parse_known_args(argv)
    config=load_config(resolve(known.config))
    tool=json.loads(resolve(config['tool']).read_text())
    motion=config['motion_parameters']
    p=argparse.ArgumentParser(description='ArmHub 深度洞口两段任务；默认预览，--execute 才允许按键运动')
    p.add_argument('--config',default=str(resolve(known.config)))
    p.add_argument('--robot',choices=['panthera_ht'],default=config['robot_id'])
    p.add_argument('--calibration',default=str(resolve(config['calibration'])))
    p.add_argument('--camera-source',choices=['live','snapshot'],default=config['camera']['source'])
    p.add_argument('--camera-serial',default=config['camera']['serial'])
    p.add_argument('--snapshot',default=str(ROOT/'logs/camera/live_current.npz'))
    p.add_argument('--camera-only',action='store_true',help='仅相机与检测，不连接机械臂')
    p.add_argument('--probe',action='store_true',help='只列出相机，不启动采集或打开机械臂')
    p.add_argument('--dry-run',action='store_true',help='纯离线 fixture 两段规划，不打开串口或相机')
    p.add_argument('--fixture',default=str(ROOT/'validation/hole_approach/synthetic_geometry.json'))
    p.add_argument('--execute',action='store_true',help='允许 GO/TRACK/HOME/夹爪按键，启动时不自动运动')
    p.add_argument('--execute-once',action='store_true',help='看到稳定目标后自动 GO 一次，需同时 --execute')
    p.add_argument('--plan-only',action='store_true')
    p.add_argument('--no-gui',action='store_true')
    p.add_argument('--report',default=str(ROOT/'logs/hole_approach/last_plan.json'))
    p.add_argument('--orientation',choices=['align','keep'],default='align')
    p.add_argument('--align-clearance','--standoff',dest='standoff',type=float,default=motion['align_clearance_m'])
    p.add_argument('--feed-length','--feed-distance',dest='feed_distance',type=float,default=motion['feed_distance_m'])
    p.add_argument('--duration',type=float,default=motion['approach_duration_s'])
    p.add_argument('--feed-duration',type=float,default=motion['feed_duration_s'])
    p.add_argument('--recovery-speed',type=float,default=motion['recovery_speed_rad_s'])
    p.add_argument('--rod-length',type=float,default=tool['rod_length_m'])
    p.add_argument('--tcp-xyz-mm',type=float,nargs=3,default=tool['tcp_xyz_mm'])
    p.add_argument('--tcp-rpy-deg',type=float,nargs=3,default=tool['tcp_rpy_deg'])
    p.add_argument('--plane-offset-mm',type=float,nargs=2,default=config['correction_right_down_mm'])
    p.add_argument('--grip-max-torque',type=float)
    p.add_argument('--grip-speed',type=float)
    a=p.parse_args(argv)
    if a.execute_once and not a.execute:p.error('--execute-once 需要同时 --execute')
    if a.plan_only and a.execute:p.error('--plan-only 不接受 --execute')
    if (a.dry_run or a.camera_only or a.probe) and (a.execute or a.execute_once):p.error('离线／相机预览不接受运动参数')
    numbers=[a.duration,a.feed_duration,a.standoff,a.feed_distance,a.recovery_speed]
    if not all(math.isfinite(x) for x in numbers) or min(a.duration,a.feed_duration,a.standoff,a.recovery_speed)<=0 or not 0<=a.feed_distance<a.standoff:
        p.error('时长、间距和回收速度需为正；要求 0<=前送距离<对准间距')
    if a.recovery_speed>min(config['robot']['velocity_limits']):p.error('回收速度不能超过当前机械臂速度上限')
    if a.grip_speed is not None and (not math.isfinite(a.grip_speed) or not 0<a.grip_speed<=.5):p.error('夹爪速度需在 (0,0.5] rad/s')
    if a.grip_max_torque is not None and (not math.isfinite(a.grip_max_torque) or not 0<a.grip_max_torque<=1.):p.error('夹爪力矩需在 (0,1] Nm')
    a.camera_width=config['camera']['width'];a.camera_height=config['camera']['height'];a.camera_fps=config['camera']['fps']
    if any(not isinstance(v,int) or v<=0 for v in [a.camera_width,a.camera_height,a.camera_fps]):p.error('相机宽高和帧率必须为正整数')
    a.ruckig=True
    a.plane_offset_m=(np.asarray(a.plane_offset_mm)/1000).tolist()
    a.config=str(resolve(a.config))
    a.calibration=str(resolve(a.calibration))
    from armhub.tools import tool_transform
    tool_transform(a)
    return a


def camera_preview(a):
    import cv2
    from perception.cameras.realsense import LiveCamera, SnapshotCamera
    from perception.detectors.depth_hole import detect_depth_hole
    camera=LiveCamera(serial=a.camera_serial,width=a.camera_width,height=a.camera_height,fps=a.camera_fps) if a.camera_source=='live' else SnapshotCamera(a.snapshot)
    try:
        if not a.no_gui:cv2.namedWindow('ArmHub camera only',cv2.WINDOW_NORMAL)
        while True:
            rgb,depth,k,stamp=camera.read()
            image=cv2.applyColorMap(np.uint8(np.clip(np.nan_to_num(depth)/1.5,0,1)*255),cv2.COLORMAP_TURBO)
            try:
                hole=detect_depth_hole(depth,k)
                cv2.ellipse(image,hole['ellipse'],(0,255,0),2)
                report={'status':'detected','frame':'camera_optical','center_optical_m':hole['center'].tolist(),
                        'normal_optical':hole['normal'].tolist(),'radius_m':hole['radius'],'motion_sent':False}
            except ValueError as error:report={'status':'no_hole','error':str(error),'motion_sent':False}
            from .reporting import write_report
            write_report(a.report,report)
            if a.no_gui:print(json.dumps(report,ensure_ascii=False));return
            cv2.putText(image,'CAMERA ONLY - no arm connection; q quit',(8,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            cv2.imshow('ArmHub camera only',image)
            if cv2.waitKey(20)&255 in [ord('q'),27]:return
    finally:
        camera.close()
        if not a.no_gui:cv2.destroyAllWindows()


def main(argv=None):
    a=arguments(argv)
    if a.probe:
        import pyrealsense2 as rs
        devices=[{'name':d.get_info(rs.camera_info.name),'serial':d.get_info(rs.camera_info.serial_number)} for d in rs.context().query_devices()]
        print(json.dumps({'devices':devices,'arm_connected':False},ensure_ascii=False,indent=2));return
    if a.camera_only:return camera_preview(a)
    calibration,status=load_calibration(a.calibration)
    print(f'机械臂={a.robot} 标定状态={status} 运动许可={a.execute}',flush=True)
    if a.dry_run:
        from .planning import solve_two_stage
        from .reporting import write_report,show_plan
        d=json.loads(resolve(a.fixture).read_text())
        if d.get('kind')=='synthetic_kinematic_fixture':
            from armhub.task_runtime import OfflineKinematics
            from armhub.tools import tool_transform
            math_robot=OfflineKinematics(d['q_start'],load_config(a.config))
            fk=math_robot.forward_kinematics(d['q_start'])
            goal_fk=math_robot.forward_kinematics(d.get('q_target',d['q_start']))
            tf=np.asarray(goal_fk['transform'])@tool_transform(a)
            normal=-tf[:3,0];hole=tf[:3,3]-normal*a.standoff
            camera=np.asarray(fk['transform'])@calibration
            center=(np.linalg.inv(camera)@np.r_[hole,1])[:3]
            optical_normal=camera[:3,:3].T@normal
            d.update(center_optical_m=center.tolist(),normal_optical=optical_normal.tolist())
        q=d.get('q_start',d.get('arm_rad'))
        request={'options':vars(a),'q_start':q,'q_capture':q,'center':d['center_optical_m'],
                 'normal':d['normal_optical'],'calibration':calibration.tolist(),'calibration_status':status}
        np.random.seed(258)
        plan=solve_two_stage(request)
        plan.update(dry_run=True,source=d.get('kind','recorded_geometry_fixture'),motion_sent=False)
        write_report(a.report,plan);show_plan(plan)
        print('离线完成：未打开相机或机械臂串口。');return
    from .app import run_hardware
    run_hardware(a,calibration,status)


if __name__=='__main__':
    try:main()
    except (RuntimeError,ValueError,FileNotFoundError) as error:
        print(f'洞口应用失败：{error}',file=sys.stderr)
        raise SystemExit(1)
