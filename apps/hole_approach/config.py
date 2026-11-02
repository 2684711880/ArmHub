"""任务参数引用机械臂、工具和标定配置，物理限制只从机械臂配置读取。"""
import json
from pathlib import Path
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT/'apps/hole_approach/config.yaml'


def resolve(path):
    path=Path(path)
    return path if path.is_absolute() else ROOT/path


def load_config(path=DEFAULT_CONFIG):
    c=yaml.safe_load(Path(path).read_text())
    if c['robot_id']!='panthera_ht':
        raise ValueError('当前洞口任务已接入 Panthera；其他机械臂需增加运动学提供器与相机标定')
    hw=json.loads((ROOT/'robots'/c['robot_id']/'hardware/config.json').read_text())
    c['robot']={'joint_limits':{'lower':hw['lower_rad'],'upper':hw['upper_rad']},
                'velocity_limits':hw['velocity_limits_rad_s'],
                'acceleration_limits':hw['motion_smoothing']['acceleration_limits_rad_s2']}
    m=c.setdefault('motion_smoothing',{})
    m.update(control_period_s=hw['motion_smoothing']['control_period_s'],
             stream_min_speed_rad_s=hw['motion_smoothing']['stream_min_speed_rad_s'],
             max_jerk_rad_s3=hw['motion_smoothing']['jerk_limits_rad_s3'],
             max_tracking_error_rad=hw['motion_smoothing']['max_tracking_error_rad'],
             max_cycle_gap_s=hw['motion_smoothing']['max_cycle_gap_s'])
    c['home_joints_rad']=hw['home_joints_rad']
    return c


def load_calibration(path):
    c=json.loads(Path(path).read_text())
    if c.get('parent_frame')!='gripper_center' or c.get('translation_unit')!='m' or not c.get('child_frame','').endswith('optical'):
        raise ValueError('手眼标定必须为 gripper_center 到 optical 的米制变换')
    t=np.asarray(c['T_tcp_camera'],dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1]) or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(t[:3,:3]),1):
        raise ValueError('非法手眼刚体变换')
    return t,c['status']
