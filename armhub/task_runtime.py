"""任务层使用的非阻塞 ArmHub 会话；单个后端拥有串口，不构造第二个 SDK Robot。"""
from pathlib import Path
import json
import numpy as np
from . import create_arm
from .adapters.panthera_kinematics import PantheraKinematics

ROOT=Path(__file__).resolve().parents[1]


class OfflineKinematics:
    def __init__(self,q,config):
        # Import configuration lazily; the kinematics provider never opens transport.
        self.config=config
        hw=json.loads((ROOT/'robots/panthera_ht/hardware/config.json').read_text())
        self.provider=PantheraKinematics(hw['lower_rad'],hw['upper_rad'])
        self.joint_limits={k:np.asarray(v) for k,v in self.config['robot']['joint_limits'].items()}
        self.q=np.asarray(q,dtype=float)

    def forward_kinematics(self,q):
        return self.provider.sdk.forward_kinematics(q)

    def inverse_kinematics(self,position,rotation,seed,**kwargs):
        self.provider.sdk.seed=np.asarray(seed)
        return self.provider.sdk.inverse_kinematics(position,rotation,seed,**kwargs)

    def check_trajectory(self,curve):
        from .execution import CheckedCurve
        hw=json.loads((ROOT/'robots/panthera_ht/hardware/config.json').read_text())
        checked=CheckedCurve(curve,hw,curve.sample(0)[0])
        return self.provider.check_trajectory(checked,hw['cartesian']['collision_step_rad'])


class ArmTaskRuntime:
    def __init__(self,robot,config,*,allow_motion=False):
        self.config=config
        if robot!=self.config['robot_id']:
            raise ValueError('任务配置的机械臂不匹配')
        self.arm=create_arm(robot,'hardware')
        self.allow_motion=allow_motion
        self.command_guard=None
        self.trajectory_done=True
        self.cached=None
        self.gripper_goal=None
        self.hold_pending=False
        self.arm.connect()
        try:
            self.math=OfflineKinematics((0.,)*6,config)
            self.joint_limits=self.math.joint_limits
            self.velocity_limits=np.array(self.config['robot']['velocity_limits'])
            if allow_motion:
                self.arm.prepare_motion_model()
                self.arm.enable()
            self.feedback()
        except BaseException:
            self.arm.close()
            raise

    def feedback(self):
        self.cached=self.arm.get_joint_feedback()
        if getattr(self,'hold_pending',False) and self.arm.poll_hold(feedback=self.cached):
            self.hold_pending=False
        if self.gripper_goal is not None and self.arm.poll_gripper():
            self.gripper_goal=None
        return self.cached

    def positions(self):
        if self.cached is None:self.feedback()
        return np.array([v.position for v in self.cached[:6]])

    def forward_kinematics(self,q):
        return self.math.forward_kinematics(q)

    def start_trajectory(self,curve,*,started_at=None):
        if not self.allow_motion:raise RuntimeError('当前是只读预览；启动时添加 --execute 才允许运动')
        if getattr(self,'hold_pending',False):raise RuntimeError('保持尚未确认，等待反馈后再规划')
        self.trajectory_done=False
        return self.arm.start_reference_trajectory(curve,replace=True,reference_started_at=started_at,
            command_guard=lambda:self.command_guard is None or self.command_guard())

    def advance_trajectory(self):
        result=self.arm.advance_reference_trajectory(feedback=self.cached)
        self.trajectory_done=result is not None
        return result

    def hold(self):
        if self.allow_motion and not getattr(self,'hold_pending',False):
            self.arm.request_hold()
            self.hold_pending=True
        self.trajectory_done=True
        self.gripper_goal=None

    def command_gripper(self,position,*,speed=.5,torque=.5):
        if not self.allow_motion:raise RuntimeError('夹爪运动需要 --execute')
        if self.command_guard is not None and not self.command_guard():raise RuntimeError('夹爪命令已被替代')
        result=self.arm.command_gripper(position,speed=speed,torque=torque,command_guard=self.command_guard)
        self.gripper_goal=position
        return result

    def close(self):
        self.arm.close()
