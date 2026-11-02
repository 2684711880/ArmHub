"""Bounded first policy trial. Evaluations only; never connect on import."""
import json
import math
from pathlib import Path
import time

KEYS = [f'joint{i}.pos' for i in range(1,7)] + ['gripper.pos']


class PolicyGuard:
    def __init__(self, config, prepared, kin):
        self.config, self.prepared, self.kin = config, prepared, kin
        self.previous = self.initial = None
        self.sent_at = None
        self.failed = False
        self.speed_since = [None]*7
        self.speed_count = [0]*7
        self.feedback_warnings = []

    def step(self, action, states, now):
        if self.failed:
            raise RuntimeError('策略保护已触发，拒绝继续发送')
        try:
            return self._step(action, states, now)
        except BaseException:
            self.failed = True
            raise

    def _step(self, action, states, now):
        if set(action) != set(KEYS):
            raise ValueError('策略输出必须包含六轴和夹爪七个目标')
        target = [float(action[k]) for k in KEYS]
        q = [s.position for s in states]
        if len(q)!=7 or not all(math.isfinite(v) for v in target+q):
            raise ValueError('策略目标或反馈存在无效数值')
        lower = self.config['lower_rad'] + [self.config['gripper_bounds_rad'][0]]
        upper = self.config['upper_rad'] + [self.config['gripper_bounds_rad'][1]]
        if any(not lo<=v<=hi for vector in (target,q) for v,lo,hi in zip(vector,lower,upper)):
            raise ValueError('策略目标或反馈超出关节限位')
        self.feedback_warnings = []
        for i,(state,cap) in enumerate(zip(states,self.config['torque_caps_nm']+[1.0])):
            label=f'motor{i+1} ({KEYS[i]})'
            if state.fault:
                raise RuntimeError(f'{label} 故障码={state.fault}')
            if not math.isfinite(state.velocity) or not math.isfinite(state.torque):
                raise RuntimeError(f'{label} 速度或力矩为非有限数')
            if abs(state.torque)>cap:
                raise RuntimeError(f'{label} 实测力矩={state.torque:.4f}，上限={cap:.4f} SDK Nm')
            speed=abs(state.velocity)
            if speed>1.2:
                raise RuntimeError(f'{label} 严重超速={state.velocity:.4f} rad/s，上限=1.2')
            if speed>.6:
                if self.speed_since[i] is None:
                    self.speed_since[i]=now
                self.speed_count[i]+=1
                duration=now-self.speed_since[i]
                self.feedback_warnings.append(dict(motor=i+1,velocity=state.velocity,
                    soft_limit=.6,duration_s=duration,count=self.speed_count[i]))
                if duration>=.12 and self.speed_count[i]>=3:
                    raise RuntimeError(f'{label} 持续超速={state.velocity:.4f} rad/s，'
                                       f'超过0.6已持续{duration:.3f}s/{self.speed_count[i]}帧')
            else:
                self.speed_since[i]=None
                self.speed_count[i]=0
        if self.previous is None:
            if max(abs(a-b) for a,b in zip(q[:6],self.prepared['q']))>.08 or abs(q[6]-1.6)>.15:
                raise RuntimeError('请先回保存的准备位并张开夹爪')
            self.initial=list(q)
            self.previous=list(q)
        elif now-self.sent_at > .3 or now <= self.sent_at:
            raise RuntimeError('策略执行周期超时或时钟异常')
        if max(abs(a-b) for a,b in zip(q,self.previous))>.1:
            raise RuntimeError('目标跟踪误差超过 0.1 rad')
        if max(abs(a-b) for a,b in zip(target[:6],q[:6]))>.3:
            raise RuntimeError('策略预测离实测姿态超过 0.3 rad，拒绝跳变')
        dt=.05 if self.sent_at is None else min(.05,now-self.sent_at)
        command=[p+max(-.15*dt,min(.15*dt,t-p)) for t,p in zip(target,self.previous)]
        if self.feedback_warnings:
            command=list(self.previous)  # Never advance the policy through a suspect speed sample.
        if max(abs(a-b) for a,b in zip(command[:6],self.initial[:6]))>.25:
            raise RuntimeError('首次试跑超过起点 0.25 rad 活动范围')
        # Check both measured-to-command and reference-to-command paths.
        self.kin.check_path(q[:6],command[:6],step_rad=.01)
        self.kin.check_path(self.previous[:6],command[:6],step_rad=.01)
        for vector in (q,command):
            if self.kin.fk(vector[:6])['position_m'][2]<.05:
                raise RuntimeError('TCP 低于基座坐标 0.05 m 首次试跑下限')
        self.previous=command
        self.sent_at=now
        return dict(zip(KEYS,command))


def install_policy_guard(root):
    """Use the SDK speed/torque limited mode; disable its competing idle writer."""
    from lerobot_robot_panthera.panthera_follower import PantheraFollower
    from armhub.adapters.panthera_kinematics import PantheraKinematics
    from armhub_lerobot_runtime import fresh_states
    from prepare_pose import load_prepared
    root=Path(root)
    config=json.loads((root/'robots/panthera_ht/hardware/config.json').read_text())
    kin=PantheraKinematics(config['lower_rad'],config['upper_rad'])
    kin.prepare_collision_geometry()
    prepared=load_prepared()['prepared']
    original_connect=PantheraFollower.connect
    instances=[]
    original_send=PantheraFollower.send_action
    original_loop=PantheraFollower._control_loop
    guards={}
    path=root/'logs/lerobot/policy'/time.strftime('trial_%Y%m%d_%H%M%S.jsonl')
    path.parent.mkdir(parents=True,exist_ok=True)
    def connect(robot):
        instances.append(robot)
        return original_connect(robot)
    def send(robot, action):
        guard=guards.setdefault(id(robot),PolicyGuard(config,prepared,kin))
        states=None
        def snapshot():
            return [dict(motor=i+1,position=float(s.position),velocity=float(s.velocity),
                         torque=float(s.torque),fault=int(s.fault),
                         counter=int(s.num),time=float(s.time)) for i,s in enumerate(states)] if states is not None else None
        try:
            states=fresh_states(robot.bus,timeout=.15)
            command=guard.step(action,states,time.monotonic())
            caps=config['torque_caps_nm']+[.3]
            for motor,key,cap in zip(robot.bus.htr_motors,KEYS,caps):
                motor.pos_vel_MAXtqe(command[key],.15,cap)
            robot.bus.robot.motor_send_cmd()
            robot.bus.target_positions=list(command.values())
            row=dict(unix_s=time.time(),raw=action,sent=command,actual=[s.position for s in states],
                     feedback=snapshot(),warnings=guard.feedback_warnings)
        except BaseException as exc:
            guard.failed=True
            with path.open('a') as stream:
                stream.write(json.dumps(dict(unix_s=time.time(),error=str(exc),feedback=snapshot(),
                    previous=guard.previous,raw=action),default=float)+'\n')
            raise
        with path.open('a') as stream:
            stream.write(json.dumps(row,default=float)+'\n')
        return command
    PantheraFollower.connect=connect
    PantheraFollower.send_action=send
    PantheraFollower._control_loop=lambda robot:None
    print(f'首次实物策略试跑：速度≤0.15 rad/s，起点六轴范围±0.25 rad；日志 {path}',flush=True)
    def uninstall():
        try:
            for robot in instances:
                if robot.bus.is_connected:
                    robot.disconnect()
        finally:
            PantheraFollower.connect=original_connect
            PantheraFollower.send_action=original_send
            PantheraFollower._control_loop=original_loop
    return uninstall
