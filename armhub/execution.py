"""同一参考执行器：CLI 阻塞等待，视觉应用逐 tick 推进；只有后端能发送电机指令。"""
import math
from .types import MotionResult
from .motion import speed_limits


class HoldExecution:
    """当前位置保持的反馈确认；任务逐 tick 轮询，CLI 可阻塞等同一结果。"""
    def __init__(self, arm, states, clock):
        self.arm,self.states,self.clock=arm,states,clock
        self.target=tuple(s.position for s in states)
        self.deadline=clock.monotonic()+3.
        self.anchor=None;self.since=None;self.done=False
        self.raw_velocity_peak=0.

    def step(self,feedback=None):
        if self.done:return True
        if feedback is None or not self.arm._validate_snapshot(feedback,[s.num for s in self.states]):
            feedback=self.arm.read_feedback()
        self.states=feedback
        now=self.clock.monotonic()
        if now>=self.deadline:
            raise RuntimeError('当前姿态保持指令已发出，但停止反馈未确认')
        q=tuple(s.position for s in feedback)
        error=max(abs(a-b) for a,b in zip(q,self.target))
        self.raw_velocity_peak=max(self.raw_velocity_peak,max(abs(s.velocity) for s in feedback))
        # 单帧 SDK 速度存在尖峰；用持续新位置反馈的位移窗口确认停稳。
        if error>self.arm.config['position_tolerance_rad']:
            self.since=None;self.anchor=None
        elif self.anchor is None or max(abs(a-b) for a,b in zip(q,self.anchor))>=self.arm.config['stable_delta_rad']:
            self.since=now;self.anchor=q
        elif now-self.since>=self.arm.config['arrival_dwell_s']:
            self.done=True;self.arm._moving=False
            self.arm._record('hold_stop_confirmed',max_position_error_rad=error,
                             raw_velocity_peak_rad_s=self.raw_velocity_peak,
                             confirmation='fresh_position_stability')
        return self.done


class ReferenceExecution:
    def __init__(self, backend, curve, states, clock, guard=None, goal_validator=None, reference_started_at=None,
                 command_mode='reference'):
        self.arm, self.curve, self.states, self.clock = backend, curve, states, clock
        self.command_mode=command_mode
        self.movej=None
        if command_mode=='sdk_movej':
            from .adapters.panthera_movej import SDKMoveJBridge
            self.movej=SDKMoveJBridge(backend,curve.speed_limits,guard)
        self.guard, self.goal_validator = guard, goal_validator
        self.settings = backend.config['motion_smoothing']
        self.started = clock.monotonic() if reference_started_at is None else reference_started_at
        self.next_due = self.started
        self.deadline = self.started+curve.duration+backend.config['motion_timeout_margin_s']
        self.start = tuple(s.position for s in states[:6])
        self.target = tuple(curve.sample(curve.duration)[0])
        self.grip = states[6].position
        self.last_send = None
        self.sent = 0
        self.final_sent = False
        self.arrived = self.reference = None
        self.max_gap = self.max_error = 0.
        self.result = None
        self.completed_at = None

    def check_guard(self):
        if self.guard is not None and not self.guard():
            raise RuntimeError('任务已被新命令替代，拒绝发送旧轨迹')

    def step(self,feedback=None):
        if self.result is not None:
            return self.result
        self.check_guard()
        arm, clock = self.arm, self.clock
        if clock.monotonic() < self.next_due and not self.final_sent:
            return None
        if clock.monotonic() >= self.deadline:
            raise TimeoutError('平滑实物运动到位超时')
        if feedback is not None:
            previous=[s.num for s in self.states]
            if arm._validate_snapshot(feedback,previous):
                self.states=feedback
            else:
                self.states=arm.read_feedback(timeout_s=self.settings['feedback_timeout_s'])
        elif self.sent:
            self.states = arm.read_feedback(timeout_s=self.settings['feedback_timeout_s'])
        now = clock.monotonic()
        self.check_guard()
        if not self.final_sent and self.last_send is not None and now-self.last_send > self.settings['max_cycle_gap_s']:
            raise RuntimeError('平滑执行下发间隔超时，拒绝跳到较远参考点')
        actual = tuple(item.position for item in self.states[:6])
        guard=arm.config['position_guard_rad']
        if any(not low-guard<=q<=high+guard for q,low,high in zip(actual,self.curve.envelope_lower,self.curve.envelope_upper)):
            raise RuntimeError('实测位置超出本次参考轨迹区间')
        if abs(self.states[6].position-self.grip) > arm.config['position_guard_rad']:
            raise RuntimeError('夹爪保持位置偏差过大')
        position, velocity, acceleration = self.curve.sample(now-self.started)
        tracking_error = max(abs(a-b) for a,b in zip(actual,position))
        self.max_error = max(self.max_error,tracking_error)
        if tracking_error > self.settings['max_tracking_error_rad']:
            raise RuntimeError('平滑执行跟踪误差超限')
        if not self.final_sent:
            self.check_guard()
            # MAXtqe 的 velocity 是固件速度上限，不是速度前馈。终点导数为零，
            # 仍须给实际滞后留下到位能力，沿用昨天 moveJ 的保持速度下限。
            terminal=now-self.started>=self.curve.duration
            floor=arm.config['hold_speed_rad_s'] if terminal else self.settings['stream_min_speed_rad_s']
            caps = [min(limit,max(abs(v),floor)) for v,limit in zip(velocity,self.curve.speed_limits)]
            arm._moving = True
            if self.movej is not None:
                caps=self.movej.send(position,self.states,self.grip,self.settings['control_period_s'],floor)
            else:
                arm._send(position,caps,self.grip)
            sent_at = clock.monotonic()
            gap = sent_at-self.last_send if self.last_send is not None else 0.
            if sent_at-now > self.settings['max_cycle_gap_s'] or gap > self.settings['max_cycle_gap_s']:
                raise RuntimeError('平滑执行发送耗时超时')
            self.max_gap = max(self.max_gap,gap)
            arm._record('stream_reference',elapsed_s=now-self.started,position_rad=position,
                        velocity_reference_rad_s=velocity,acceleration_reference_rad_s2=acceleration,
                        tracking_error_rad=tracking_error,command_gap_s=gap,
                        command_mode=self.command_mode,velocity_caps_rad_s=caps)
            self.last_send = sent_at
            self.sent += 1
            self.final_sent = now-self.started >= self.curve.duration
            self.next_due = self.started+(math.floor((sent_at-self.started)/self.settings['control_period_s'])+1)*self.settings['control_period_s']
        error = max(abs(a-b) for a,b in zip(actual,self.target))
        if self.final_sent and error <= arm.config['position_tolerance_rad'] and (self.goal_validator is None or self.goal_validator(actual)):
            if self.arrived is None or max(abs(a-b) for a,b in zip(actual,self.reference)) >= arm.config['stable_delta_rad']:
                self.arrived,self.reference = now,actual
            if now-self.arrived >= arm.config['arrival_dwell_s']:
                self.completed_at = now
                arm._moving = False
                arm._record('stream_completed',final_positions=actual,reference_commands=self.sent,
                            max_command_gap_s=self.max_gap,max_tracking_error_rad=self.max_error,
                            elapsed_s=now-self.started,max_error_rad=error)
                self.result = MotionResult('succeeded','hardware',actual,
                    f'{self.command_mode} + 官方 SDK；新反馈确认到位和稳定；实物动态性能需测量',trajectory_points=self.sent)
        else:
            self.arrived = None
        return self.result


class CheckedCurve:
    """把任务曲线检查成后端接受的有限、限位内参考；预检同时覆盖自碰撞。"""
    def __init__(self, curve, config, start):
        self.curve, self.config, self.start = curve, config, tuple(start)
        self.duration = float(curve.duration)
        self.period = config['motion_smoothing']['control_period_s']
        self.envelope_lower=list(start);self.envelope_upper=list(start)
        self.speed_limits=speed_limits(config,getattr(curve,'speed_limits',getattr(curve,'speed_limit',None)))
        self.speed_limit=max(self.speed_limits)
        if len(self.start)!=6 or any(not math.isfinite(q) or not low-.01<=q<=high+.01 for q,low,high in zip(self.start,config['lower_rad'],config['upper_rad'])):
            raise ValueError('参考轨迹起点超出反馈限位')
        if not math.isfinite(self.speed_limit) or self.speed_limit<=0:
            raise ValueError('参考轨迹速度上限无效')
        if not math.isfinite(self.duration) or self.duration <= 0:
            raise ValueError('参考轨迹时长必须为正的有限数')
        self.sample(0.);self.sample(self.duration)

    def sample(self,t):
        result = tuple(tuple(float(x) for x in v) for v in self.curve.sample(t))
        if len(result)!=3 or any(len(v)!=6 or not all(math.isfinite(x) for x in v) for v in result):
            raise ValueError('参考轨迹需要六轴有限 q/v/a')
        q,v,a = result
        if any(not min(low,s)-1e-8 <= x <= max(high,s)+1e-8 for x,low,high,s in
               zip(q,self.config['lower_rad'],self.config['upper_rad'],self.start)):
            raise ValueError('参考轨迹超出物理关节限位')
        if any(abs(x)>limit+1e-6 for x,limit in zip(v,self.speed_limits)):
            raise ValueError('参考轨迹速度超限')
        if any(abs(x)>limit+1e-6 for x,limit in zip(a,self.config['motion_smoothing']['acceleration_limits_rad_s2'])):
            raise ValueError('参考轨迹加速度超限')
        self.envelope_lower=[min(a,b) for a,b in zip(self.envelope_lower,q)]
        self.envelope_upper=[max(a,b) for a,b in zip(self.envelope_upper,q)]
        return result

    def collision_samples(self,step):
        segments=max(1,math.ceil(self.duration/min(self.period,step/self.speed_limit)))
        previous_a = None
        dt=self.duration/segments
        for i in range(segments+1):
            q,v,a = self.sample(dt*i)
            if previous_a is not None and any(abs(x-y)/dt > limit+1e-4 for x,y,limit in
                    zip(a,previous_a,self.config['motion_smoothing']['jerk_limits_rad_s3'])):
                raise ValueError('参考轨迹 jerk 超限')
            previous_a = a
            yield q

    def summary(self):
        return {'execution_backend':'checked_reference_stream','duration_s':self.duration,
                'reference_period_s':self.period,'reference_max_velocity_rad_s':list(self.speed_limits),'planned_reference_samples':math.ceil(self.duration/self.period)+1}
