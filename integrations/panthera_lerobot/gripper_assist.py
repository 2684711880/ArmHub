"""Opt-in position-triggered closing feedforward for the official replay path.

The six arm commands and native five-parameter packet mode remain upstream-owned.
This is grip assistance, not a contact sensor or a calibrated finger-force loop.
"""
from dataclasses import dataclass
import math
import threading
import time


@dataclass(frozen=True)
class AssistConfig:
    torque_nm: float = .1
    close_at_rad: float = .85
    open_at_rad: float = 1.2
    near_rad: float = .08
    release_delta_rad: float = .08
    closed_guard_rad: float = .1
    ramp_s: float = .3
    feedback_torque_limit_nm: float = 1.
    max_gap_s: float = .25
    max_speed_rad_s: float = .3
    max_position_lead_rad: float = .08

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('夹持辅助参数必须是有限数')
        if not 0 < self.torque_nm <= .3:
            raise ValueError('附加闭合力矩需在 (0, 0.3] SDK Nm')
        if not 0 <= self.closed_guard_rad < self.close_at_rad < self.open_at_rad <= 2:
            raise ValueError('夹爪开度阈值需满足闭合保护 < 加力位置 < 张开位置 <= 2 rad')
        if not (0 < self.near_rad <= .15 and 0 < self.release_delta_rad <= .2
                and .1 <= self.ramp_s <= 2 and 0 < self.feedback_torque_limit_nm <= 1
                and 0 < self.max_gap_s <= .5 and 0 < self.max_speed_rad_s <= .5
                and 0 < self.max_position_lead_rad <= .1):
            raise ValueError('夹持辅助的接近范围、释放阈值、斜坡或保护参数无效')


class PositionGripAssist:
    def __init__(self, config):
        self.config = config
        self.open_seen = False; self.active = False
        self.minimum_target = None; self.last_time = None
        self.started = None; self.last_counter = None
        self.status = dict(stage='position', feedforward_nm=0.)

    def step(self, target, state, now=None):
        now = time.monotonic() if now is None else now
        c = self.config; target = float(target)
        if not math.isfinite(now):
            raise RuntimeError('夹持辅助时间戳无效')
        if (state.ID != 7 or state.num <= 0 or state.fault
                or not -.02 <= time.time()-state.time <= .2
                or not all(math.isfinite(v) for v in (target,state.position,state.velocity,state.torque))
                or not 0 <= target <= 2 or not -.02 <= state.position <= 2.02):
            raise RuntimeError('夹持辅助的夹爪目标或反馈无效/过期/故障')
        if self.last_counter is not None and state.num <= self.last_counter:
            raise RuntimeError('夹持辅助未获得新的夹爪反馈')
        if self.last_time is not None and (now < self.last_time or now-self.last_time > c.max_gap_s):
            raise RuntimeError('夹持辅助控制周期超时')
        if abs(state.torque) > c.feedback_torque_limit_nm:
            raise RuntimeError(f'夹爪实测力矩超过辅助保护上限：反馈 {state.torque:.4f} Nm，'
                               f'上限 {c.feedback_torque_limit_nm:.4f} Nm，'
                               f'实际开度 {state.position:.4f} rad，目标 {target:.4f} rad')
        self.last_time = now; self.last_counter = state.num
        # Initial closed frames must not apply assist before the first real opening.
        if target >= c.open_at_rad and state.position >= c.open_at_rad-c.near_rad:
            self.open_seen = True; self.active = False; self.started = None
            self.minimum_target = target
        if self.minimum_target is not None:
            self.minimum_target = min(self.minimum_target,target)
        opening = (self.minimum_target is not None
                   and target-self.minimum_target >= c.release_delta_rad)
        if opening:
            self.active = False; self.started = None
            # Require an explicit full opening before the next assisted closure.
            self.open_seen = target >= c.open_at_rad and state.position >= c.open_at_rad-c.near_rad
            self.minimum_target = target
        if target <= c.closed_guard_rad or state.position <= c.closed_guard_rad:
            self.active = False; self.open_seen = False; self.started = None
        elif (not opening and not self.active and self.open_seen and target <= c.close_at_rad
              and abs(state.position-target) <= c.near_rad and abs(state.velocity) <= .2):
            self.active = True; self.started = now
        if self.active and abs(state.position-target) > .2:
            raise RuntimeError('夹持辅助期间夹爪位置偏差超过保护范围')
        ff = -c.torque_nm * min(1.,max(0.,(now-self.started)/c.ramp_s)) if self.active else 0.
        self.status = dict(stage='assist' if self.active else 'position', feedforward_nm=ff)
        return self.status


class PositionTargetLimiter:
    """Start at measured opening; bound command slew and PD position error."""
    def __init__(self, config):
        self.config=config; self.target=None; self.last_time=None

    def step(self, requested, measured, now):
        if self.target is None:
            self.target=float(measured); self.last_time=now
            return self.target
        dt=now-self.last_time
        if not 0 <= dt <= self.config.max_gap_s:
            raise RuntimeError('夹爪限速周期无效')
        step=self.config.max_speed_rad_s*dt
        lead=self.config.max_position_lead_rad
        low=max(0.,self.target-step,measured-lead)
        high=min(2.,self.target+step,measured+lead)
        if low > high:
            raise RuntimeError('夹爪实测位置变化超过限速跟踪范围，停止回放')
        self.target=max(low,min(high,float(requested)))
        self.last_time=now
        return self.target


class _MotorCommandProxy:
    """Override ID 7 in the same outgoing packet, only on this replay thread."""
    def __init__(self, motor, feedforward, position_target):
        self.motor = motor; self.feedforward = feedforward
        self.position_target = position_target
        self.thread_id = threading.get_ident(); self.used = False; self.error = None

    def __getattr__(self,name):
        return getattr(self.motor,name)

    def pos_vel_tqe_kp_kd(self, position, velocity, torque, kp, kd):
        if threading.get_ident() != self.thread_id:
            return self.motor.pos_vel_tqe_kp_kd(position,velocity,torque,kp,kd)
        try:
            if not all(math.isfinite(v) for v in (position,velocity,torque,kp,kd)) or torque != 0:
                raise RuntimeError('官方夹爪命令参数已变化，拒绝叠加未知前馈力矩')
            self.used = True
            return self.motor.pos_vel_tqe_kp_kd(self.position_target,velocity,self.feedforward,kp,kd)
        except BaseException as exc:
            self.error = exc
            raise


def install_gripper_assist(config, follower_class=None):
    if follower_class is None:
        from lerobot_robot_panthera.panthera_follower import PantheraFollower
        follower_class = PantheraFollower
    original = follower_class.send_action
    controllers = {}

    def send_action(device, action):
        if device.teaching_mode:
            raise RuntimeError('夹持辅助只用于实机回放，不用于拖动示教')
        if id(device) not in controllers:
            controllers[id(device)] = (PositionGripAssist(config),PositionTargetLimiter(config))
        controller, limiter = controllers[id(device)]
        state = device.bus.htr_motors[6].get_current_motor_state()
        now=time.monotonic()
        status = dict(controller.step(action['gripper.pos'],state,now))
        limited_target = limiter.step(action['gripper.pos'],state.position,now)
        if abs(limited_target-float(action['gripper.pos'])) > config.near_rad:
            status.update(stage='approaching',feedforward_nm=0.)
        status['limited_target_rad'] = limited_target
        device._armhub_gripper_assist_status = status
        motors = device.bus.robot.Motors
        if len(motors) != 7:
            raise RuntimeError('夹持辅助要求六轴和夹爪共七电机')
        motor = motors[6]
        proxy = _MotorCommandProxy(motor,status['feedforward_nm'],limited_target)
        motors[6] = proxy
        try:
            result = original(device,action)
            # Upstream catches motor-send exceptions. Propagate any proxy failure.
            if proxy.error is not None:
                raise proxy.error
            if not proxy.used:
                raise RuntimeError('官方执行路径未使用预期夹爪命令，停止辅助回放')
            return result
        finally:
            motors[6] = motor

    follower_class.send_action = send_action

    def uninstall():
        if follower_class.send_action is send_action:
            follower_class.send_action = original
        controllers.clear()
    return uninstall
