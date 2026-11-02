"""Call the official SDK moveJ math through the existing guarded ArmHub transport."""
import math
from pathlib import Path
import sys
import numpy as np


class SDKMoveJBridge:
    def __init__(self,arm,speed_limits,guard=None):
        # Importing the class does not instantiate a Robot or open a serial port.
        sdk=Path(__file__).resolve().parents[2]/'vendor/panthera/sdk/official_python'
        if str(sdk) not in sys.path:sys.path.insert(0,str(sdk))
        from Panthera_lib import Panthera
        self.official_moveJ=Panthera.moveJ
        self.arm=arm;self.speed_limits=np.asarray(speed_limits);self.guard=guard
        self.motor_count=6;self.max_torque=np.asarray(arm.config['torque_caps_nm'])
        self.joint_limits=dict(lower=np.asarray(arm.config['lower_rad']),upper=np.asarray(arm.config['upper_rad']))
        self.states=None;self.grip=None;self.floor=None;self.last_speed_caps=None

    def get_current_pos(self):
        return np.asarray([s.position for s in self.states[:6]])

    def Joint_Pos_Vel(self,pos,vel,max_tqu,iswait=False,tolerance=.1,timeout=15.):
        if iswait:raise ValueError('逐点 moveJ 试验禁止阻塞等待')
        target=np.asarray(pos,dtype=float);velocity=np.asarray(vel,dtype=float);torque=np.asarray(max_tqu,dtype=float)
        if any(x.shape!=(6,) or not np.isfinite(x).all() for x in [target,velocity,torque]):
            raise ValueError('moveJ 需要六轴有限数')
        if np.any(target<self.joint_limits['lower']) or np.any(target>self.joint_limits['upper']):
            raise ValueError('moveJ 目标超出关节限位')
        if np.any(torque<0) or np.any(torque>self.max_torque+1e-9):raise ValueError('moveJ 力矩超限')
        caps=np.maximum(np.abs(velocity),self.floor)
        if np.any(caps>self.speed_limits+1e-9):raise ValueError('moveJ 基于反馈计算的速度超限；已拒绝发送')
        if self.guard is not None and not self.guard():raise RuntimeError('任务已被新命令替代')
        self.arm._send(target,caps,self.grip)
        self.last_speed_caps=caps.tolist()
        return True

    def send(self,target,states,grip,duration,floor):
        self.arm._require_connection()
        if not self.arm._enabled or self.arm._gravity_active:raise RuntimeError('moveJ 需要已使能且退出重力补偿')
        if not math.isfinite(duration) or duration<=0:raise ValueError('moveJ duration 必须为有限正数')
        self.states=states;self.grip=grip;self.floor=floor
        if not self.official_moveJ(self,target,duration,max_tqu=self.max_torque,iswait=False):
            raise RuntimeError('官方 SDK moveJ 拒绝目标')
        return self.last_speed_caps
