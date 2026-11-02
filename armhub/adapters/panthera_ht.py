"""Panthera 官方 SDK 的关节、空间到点与重力补偿适配器；不导入旧应用，不自动回零。"""
import fcntl
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import time
from collections import deque
from datetime import datetime, timezone

from ..interfaces import ArmBackend
from ..types import ArmState, GravityResult, MotionResult, JointFeedback, UnsupportedCapability

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'robots/panthera_ht/hardware/config.json'


class PantheraHardwareBackend(ArmBackend):
    backend_name = 'hardware'

    def __init__(self, profile):
        super().__init__(profile)
        self.config = json.loads(CONFIG.read_text())
        self.device = os.environ.get('ARMHUB_PANTHERA_DEVICE', self.config['device'])
        self._driver = None
        self._motors = []
        self._lock = None
        self._temp = None
        self._log = None
        self._enabled = False
        self._moving = False
        self._gravity_active = False
        self._kinematics_cache = None
        self._reference_execution = None
        self._hold_execution = None
        self._gripper_action = None
        self._feedback_stability = None
        self._last_states = None
        self.log_path = None

    @property
    def capabilities(self):
        return frozenset({'state', 'joint_motion', 'stop', 'gravity_compensation', 'cartesian_target', 'home', 'reference_stream', 'gripper_control'})

    def _record(self, event, **data):
        if self._log:
            self._log.write(json.dumps({'event': event, 'monotonic_s': time.monotonic(),
                'utc': datetime.now(timezone.utc).isoformat(), **data}, ensure_ascii=False) + '\n')

    def _device_preflight(self):
        path = Path(self.device)
        if not path.exists():
            raise RuntimeError(f'未发现机械臂串口 {path}；接通电源和 USB 后先运行只读 state')
        resolved = path.resolve()
        if not resolved.name.startswith(('ttyACM', 'ttyUSB')) or not stat.S_ISCHR(path.stat().st_mode):
            raise RuntimeError('设备必须是已确认的 USB 串口，不接受普通文件或其他设备')
        if not os.access(path, os.R_OK | os.W_OK):
            raise PermissionError(f'没有串口读写权限：{path}')
        # 旧 SDK 不共享本工程的 advisory lock，因此另查正在打开的串口。
        result = subprocess.run(['fuser', '-s', str(resolved)], check=False)
        if result.returncode == 0:
            raise RuntimeError('串口已被其他进程占用；先退出旧控制器')
        if result.returncode != 1:
            raise RuntimeError('无法检查串口占用，拒绝建立第二个控制连接')

    def _open_driver(self):
        import yaml
        sdk = ROOT / 'vendor/panthera/sdk'
        sys.path.insert(0, str(sdk / 'python'))
        import hightorque_robot as htr
        if not getattr(htr, 'commission_safe_binding', False):
            raise RuntimeError('必须使用 ArmHub 编译的 passive-lifecycle/snapshot-lock SDK')
        if not Path(htr.__file__).resolve().is_relative_to(sdk):
            raise RuntimeError('加载了工程外的 SDK，拒绝连接')
        motors = yaml.safe_load((CONFIG.parent / self.config['motor_config']).read_text())
        motors['robot']['Serial_Type'] = self.device
        if motors['robot']['motor_timeout_ms'] != 0:
            raise RuntimeError('本接入阶段沿用已验证的 timeout=0 配置，禁止静默改变固件策略')
        self._temp = tempfile.TemporaryDirectory(prefix='armhub_panthera_')
        folder = Path(self._temp.name)
        (folder / 'motors.yaml').write_text(yaml.safe_dump(motors))
        native = {'robot': {'name': 'Panthera-HT', 'param_file': str(folder / 'motors.yaml'),
                            'commission_passive_lifecycle': True}}
        (folder / 'session.yaml').write_text(yaml.safe_dump(native))
        return htr.Robot(str(folder / 'session.yaml'))

    def connect(self):
        if self._driver is not None:
            return
        self._device_preflight()
        key = hashlib.sha256(str(Path(self.device).resolve()).encode()).hexdigest()[:16]
        self._lock = open(f'/tmp/armhub-panthera-{key}.lock', 'a')
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            directory = ROOT / 'logs/panthera_ht'
            directory.mkdir(parents=True, exist_ok=True)
            self.log_path = directory / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '.jsonl')
            self._log = self.log_path.open('w', buffering=1)
            self._record('connect_begin', device=self.device, config=self.config)
            self._driver = self._open_driver()
            self._motors = self._driver.get_motors()
            if len(self._motors) != 7 or [motor.get_motor_id() for motor in self._motors] != list(range(1, 8)):
                raise RuntimeError('必须识别 ID 1…7：六轴机械臂和夹爪')
            self.read_feedback()
            self._record('connected_read_only', note='No motion, reset, home, brake or torque enable command')
        except BaseException:
            self.close()
            raise

    def _require_connection(self):
        if self._driver is None:
            raise RuntimeError('请先 connect()')

    def _validate_snapshot(self, states, previous):
        if len(states) != 7:
            raise RuntimeError('必须具有七个电机反馈')
        fresh = True
        now = time.time()
        for index, item in enumerate(states):
            values = [item.position, item.velocity, item.torque, item.time]
            if not all(math.isfinite(value) for value in values) or item.fault or item.ID != index + 1:
                raise RuntimeError(f'ID {index+1} 的反馈非有限数、故障或身份错误')
            if item.num <= previous[index] or not -0.02 <= now-item.time <= self.config['max_feedback_age_s']:
                fresh = False
            low, high = ((self.config['lower_rad'][index], self.config['upper_rad'][index])
                         if index < 6 else self.config['gripper_bounds_rad'])
            # 只读阶段记录真实原始值；超限不掩盖、不将反馈裁剪成合法值。
            margin = 0.01 if index < 6 else 0.05
            if not low-margin <= item.position <= high+margin:
                raise RuntimeError(f'ID {index+1} 实测位置 {item.position:.6f} 超出配置限制')
        return fresh

    def read_feedback(self, *, query_period_s=None, timeout_s=None, record=True):
        self._require_connection()
        previous = [motor.get_current_motor_state().num for motor in self._motors]
        deadline = time.monotonic() + (self.config['feedback_acquire_timeout_s'] if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            self._driver.send_get_motor_state_cmd()
            time.sleep(self.config['feedback_query_period_s'] if query_period_s is None else query_period_s)
            states = [motor.get_current_motor_state() for motor in self._motors]
            # 初始零缓存只在收到帧后检查模型范围，防止用默认 0 冒充反馈。
            if any(item.num <= 0 for item in states):
                continue
            if self._validate_snapshot(states, previous):
                self._last_states = states
                self._observe_stability(states)
                if record:
                    self._record('feedback', motors=[{
                    'id': item.ID, 'position_rad': item.position, 'velocity_rad_s': item.velocity,
                    'torque_nm': item.torque, 'fault': item.fault, 'mode': item.mode,
                    'sample_counter': item.num, 'age_s': time.time()-item.time} for item in states])
                return states
        self._record('feedback_failure')
        raise RuntimeError('七个电机的新反馈未在期限内到齐；没有更新运动目标')

    def get_state(self):
        states = self.read_feedback()
        oldest_age = max(time.time()-item.time for item in states)
        return ArmState(self.profile.joint_names, tuple(item.position for item in states[:6]),
                        time.monotonic()-oldest_age, self._enabled, 'hardware')

    def get_joint_feedback(self):
        return tuple(JointFeedback(**{name:getattr(s,name) for name in JointFeedback.__dataclass_fields__})
                     for s in self.read_feedback())

    def command_gripper(self, position, *, speed=.5, torque=.5, command_guard=None):
        self._require_connection()
        if not self._enabled or self._moving or self._gravity_active:
            raise RuntimeError('夹爪命令需要已使能且没有其他运动')
        low,high=self.config['gripper_bounds_rad']
        if not all(math.isfinite(v) for v in [position,speed,torque]) or not low<=position<=high or not 0<speed<=.5 or not 0<torque<=self.config['gripper_torque_cap_nm']:
            raise ValueError('夹爪位置、速度或力矩越界')
        states=self._stable_feedback()
        if command_guard is not None and not command_guard():raise RuntimeError('夹爪命令已被替代')
        q=tuple(s.position for s in states[:6])
        self._gripper_action={'arm_hold':q,'target':position,'deadline':time.monotonic()+abs(position-states[6].position)/speed+5,'arrived':None}
        self._moving=True
        try:
            self._send(q,[self.config['hold_speed_rad_s']]*6,position,gripper_speed=speed,gripper_torque=torque)
        except BaseException:
            self.stop()
            raise
        return {'status':'submitted','target_raw_rad':position,'grasp_confirmed':False}

    def poll_gripper(self):
        action=self._gripper_action
        if action is None:return True
        try:
            states=self.read_feedback()
            if max(abs(s.position-q) for s,q in zip(states[:6],action['arm_hold']))>self.config['position_guard_rad']:
                raise RuntimeError('夹爪动作时机械臂保持偏差超限')
            if abs(states[6].position-action['target'])<=self.config['position_tolerance_rad']:
                if action['arrived'] is None:action['arrived']=time.monotonic()
                if time.monotonic()-action['arrived']>=self.config['arrival_dwell_s']:
                    self._gripper_action=None;self._moving=False
                    return True
            else:action['arrived']=None
            if time.monotonic()>action['deadline']:raise TimeoutError('夹爪到位超时；不代表物体已夹稳')
            return False
        except BaseException:
            self.stop()
            raise

    def _observe_stability(self, states):
        """复用持续采集的新反馈；观测断档或位置变化即清除已有稳定结论。"""
        now=time.monotonic()
        positions=tuple(s.position for s in states)
        counters=tuple(s.num for s in states)
        previous=self._feedback_stability
        if (previous is None or now-previous['last_observed']>self.config['max_feedback_age_s'] or
                max(abs(a-b) for a,b in zip(positions,previous['positions']))>=self.config['stable_delta_rad']):
            previous={'positions':positions,'counters':counters,'since':now,'last_observed':now,'ready':False}
            self._feedback_stability=previous
        previous['last_observed']=now
        previous['ready']=(now-previous['since']>=self.config['stable_window_s'] and
                           all(s.num>=n+8 for s,n in zip(states,previous['counters'])))

    def _stable_feedback(self):
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            states=self.read_feedback()
            if self._feedback_stability['ready']:
                return states
        raise RuntimeError('未确认机械臂和夹爪静止，拒绝开始运动')

    def enable(self):
        self._stable_feedback()
        self._enabled = True
        self._record('software_execution_enabled', note='Does not send a hardware enable or home command')

    def _send(self, target, speed_caps, gripper_position, *, gripper_speed=None, gripper_torque=None):
        # SDK 七电机共享命令缓冲；每次明确填写全部轴，夹爪沿用实测原始角度。
        for index, motor in enumerate(self._motors[:6]):
            motor.pos_vel_MAXtqe(float(target[index]), float(speed_caps[index]), self.config['torque_caps_nm'][index])
        self._motors[6].pos_vel_MAXtqe(float(gripper_position),
            self.config['hold_speed_rad_s'] if gripper_speed is None else gripper_speed,
            self.config['gripper_torque_cap_nm'] if gripper_torque is None else gripper_torque)
        self._driver.motor_send_cmd()
        self._record('command', target_rad=list(target), speed_caps_rad_s=list(speed_caps),
                     gripper_hold_rad=gripper_position)

    def move_joints(self, positions, *, duration_s):
        return self._execute_joint_target(positions, duration_s=duration_s,
            max_step_rad=self.config['max_step_rad'], single_joint=True)

    def _execute_joint_target(self, positions, *, duration_s, max_step_rad, single_joint,
                              expected_start=None, goal_validator=None, smooth=False, speed_limit=None):
        self._require_connection()
        if self._gravity_active:
            raise RuntimeError('重力补偿正在占用执行通道，请先退出补偿')
        if not self._enabled:
            raise RuntimeError('请先显式 enable()')
        target = tuple(float(value) for value in positions)
        if len(target) != 6 or not all(math.isfinite(value) for value in target):
            raise ValueError('需要六个有限关节角（rad）')
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError('duration_s 必须为正的有限数')
        states = self._stable_feedback()
        start = tuple(item.position for item in states[:6])
        if expected_start is not None and max(abs(a-b) for a,b in zip(start,expected_start)) > self.config['cartesian']['start_tolerance_rad']:
            raise RuntimeError('规划后实测姿态发生变化，拒绝执行旧起点的逆解；请重新预览')
        checked = []
        for value, actual, low, high in zip(target, start, self.config['lower_rad'], self.config['upper_rad']):
            if low <= value <= high:
                checked.append(value)
            elif (abs(value-actual) <= self.config['stable_delta_rad']
                  and low-0.01 <= actual <= high+0.01):
                # 零位附近的编码器偏差只允许保持实测位置，不能据此扩大主动运动范围。
                checked.append(actual)
            else:
                raise ValueError('目标超出实物模型限位')
        target = tuple(checked)
        delta = [abs(a-b) for a, b in zip(start, target)]
        if max_step_rad is not None and max(delta) > max_step_rad:
            raise ValueError(f'单次关节变化超过 {math.degrees(max_step_rad):.1f}° 配置限制')
        if single_joint and sum(value > self.config['stable_delta_rad'] for value in delta) > 1:
            raise ValueError('首次接入仅允许一个关节运动，其他轴保持当前位置')
        if smooth:
            return self._execute_smoothed_target(start, target, duration_s, speed_limit, goal_validator)
        caps = [max(value/duration_s, self.config['hold_speed_rad_s']) for value in delta]
        from ..motion import speed_limits
        allowed=([self.config['jog_max_speed_rad_s']]*6 if single_joint else speed_limits(self.config))
        if any(cap>limit+1e-9 for cap,limit in zip(caps,allowed)):
            raise ValueError('运动速度超过该接口的每轴配置上限；请增加运动时间')
        grip = states[6].position
        self._moving = True
        try:
            self._send(target, caps, grip)
            deadline = time.monotonic() + duration_s + self.config['motion_timeout_margin_s']
            arrived = None
            arrival_reference = None
            while time.monotonic() < deadline:
                feedback = self.read_feedback()
                actual = tuple(item.position for item in feedback[:6])
                guard = self.config['position_guard_rad']
                if any(not min(a, b)-guard <= q <= max(a, b)+guard for q, a, b in zip(actual, start, target)):
                    raise RuntimeError('实测位置超出本次运动区间')
                if abs(feedback[6].position-grip) > guard:
                    raise RuntimeError('夹爪保持位置偏差过大')
                error = max(abs(a-b) for a, b in zip(actual, target))
                self._record('motion_progress', target_rad=list(target), max_error_rad=error)
                if error <= self.config['position_tolerance_rad'] and (goal_validator is None or goal_validator(actual)):
                    if (arrived is None or max(abs(a-b) for a,b in zip(actual,arrival_reference)) >= self.config['stable_delta_rad']):
                        arrived = time.monotonic()
                        arrival_reference = actual
                    if time.monotonic()-arrived >= self.config['arrival_dwell_s']:
                        self._moving = False
                        self._record('motion_completed', final_positions=actual, max_error_rad=error)
                        return MotionResult('succeeded', 'hardware', actual,
                            '官方 SDK POS_VEL_MAX_TORQUE 执行；新反馈确认到位；固件速度上限不代表精确轨迹时序', trajectory_points=1)
                else:
                    arrived = None
            raise TimeoutError('实物运动到位超时')
        except BaseException:
            try:
                self.stop()
            except Exception as stop_error:
                self._record('stop_unconfirmed', reason=str(stop_error), action_required='Check hardware and physical emergency stop')
            raise

    def _build_transition(self, start, target, duration_s, speed_limit):
        from ..motion import JointTransition
        return JointTransition(start, target, self.config, speed_limit=speed_limit,
                               minimum_duration=duration_s)

    def start_reference_trajectory(self, curve, *, expected_start=None, command_guard=None,
                                   replace=False, goal_validator=None, reference_started_at=None,
                                   command_mode='reference'):
        from ..execution import ReferenceExecution, CheckedCurve
        self._require_connection()
        if command_mode not in ('reference','sdk_movej'):
            raise ValueError('未知轨迹下发方式')
        if not self._enabled or self._gravity_active:
            raise RuntimeError('执行连续参考需要软件使能且退出重力补偿')
        if self._gripper_action is not None:
            raise RuntimeError('请先结束夹爪动作')
        if self._moving and not replace:
            raise RuntimeError('已有运动正在执行')
        if self._hold_execution is not None and not self._hold_execution.done:
            raise RuntimeError('保持仍在等待新反馈确认，拒绝启动下一轨迹')
        settings = self.config['motion_smoothing']
        for key in ['control_period_s','stream_min_speed_rad_s','max_tracking_error_rad','max_cycle_gap_s','feedback_timeout_s']:
            if not math.isfinite(settings[key]) or settings[key]<=0:
                raise ValueError(f'无效平滑执行参数 {key}')
        if settings['control_period_s']>=settings['max_cycle_gap_s']:
            raise ValueError('平滑执行周期必须小于最大间隔')
        previous=self._reference_execution
        handoff=replace and previous is not None and previous.completed_at is not None and time.monotonic()-previous.completed_at<1.
        states = self.read_feedback() if self._moving or handoff else self._stable_feedback()
        start = tuple(s.position for s in states[:6])
        if expected_start is not None and max(abs(a-b) for a,b in zip(start,expected_start)) > self.config['cartesian']['start_tolerance_rad']:
            raise RuntimeError('轨迹检查前实测起点发生变化，请重新规划')
        checked = CheckedCurve(curve,self.config,start)
        if settings['stream_min_speed_rad_s'] > min(checked.speed_limits):
            raise ValueError('最低参考速度上限超过轨迹速度上限')
        initial,initial_velocity,initial_acceleration = checked.sample(0)
        tolerance = settings['max_tracking_error_rad'] if self._moving else self.config['cartesian']['start_tolerance_rad']
        if max(abs(a-b) for a,b in zip(start,initial)) > tolerance:
            raise RuntimeError('参考轨迹起点与实测反馈不一致')
        collision = self._kinematics().check_trajectory(checked,self.config['cartesian']['collision_step_rad'])
        states = self.read_feedback() if self._moving or handoff else self._stable_feedback()
        if max(abs(s.position-q) for s,q in zip(states[:6],initial)) > tolerance:
            raise RuntimeError('轨迹检查后实测起点发生变化，请重新规划')
        if command_guard is not None and not command_guard():
            raise RuntimeError('任务已被新命令替代')
        if self._moving and previous is not None and not previous.final_sent and previous.last_send is not None and time.monotonic()-previous.last_send>settings['max_cycle_gap_s']:
            self.stop()
            raise RuntimeError('轨迹替换检查超时，已请求当前位置保持')
        now=time.monotonic()
        # 静止出发的曲线应在预检结束后计时；运动中 q/v/a 接续必须沿用原时间轴。
        rest_start=(not self._moving and
                    all(abs(x)<=1e-9 for x in (*initial_velocity,*initial_acceleration)))
        if reference_started_at is not None and (not math.isfinite(reference_started_at) or
                reference_started_at>now or (not rest_start and now-reference_started_at>settings['max_cycle_gap_s'])):
            if self._moving:self.stop()
            raise RuntimeError('参考起始时刻已过期，拒绝采用旧时间轴')
        session = ReferenceExecution(self,checked,states,time,command_guard,goal_validator,
                                     None if rest_start else reference_started_at,command_mode=command_mode)
        self._reference_execution = session
        self._record('stream_start',**checked.summary(),collision_check=collision,command_mode=command_mode)
        return session.started

    def advance_reference_trajectory(self,feedback=None):
        session = self._reference_execution
        if session is None:
            raise RuntimeError('没有活动参考轨迹')
        try:
            return session.step(feedback=feedback)
        except BaseException as error:
            self._record('stream_failure',reason=str(error),reference_commands=session.sent)
            if self._moving:
                try:
                    self.stop()
                except Exception as stop_error:
                    self._record('stop_unconfirmed',reason=str(stop_error))
            raise

    def _execute_smoothed_target(self, start, target, duration_s, speed_limit, goal_validator):
        curve = self._build_transition(start,target,duration_s,speed_limit)
        self.start_reference_trajectory(curve,expected_start=start,goal_validator=goal_validator)
        try:
            while True:
                session = self._reference_execution
                wait = session.next_due-time.monotonic()
                if not session.final_sent and wait>0:
                    time.sleep(wait)
                result = self.advance_reference_trajectory()
                if result is not None:
                    return result
        except BaseException:
            if self._moving:
                try:self.stop()
                except Exception as error:self._record('stop_unconfirmed',reason=str(error))
            raise

    def execute_trajectory(self, points):
        raise UnsupportedCapability('实物连续轨迹尚未验收；当前开放受限关节微动和空间点到点')

    def prepare_motion_model(self):
        """在进入交互控制前建立碰撞几何；不查询设备或发送运动命令。"""
        started=time.monotonic()
        result=self._kinematics().prepare_collision_geometry()
        self._record('motion_model_ready',initialization_s=time.monotonic()-started,**result)
        return result

    def _kinematics(self):
        if self._kinematics_cache is None:
            from .panthera_kinematics import PantheraKinematics
            self._kinematics_cache = PantheraKinematics(self.config['lower_rad'], self.config['upper_rad'])
        return self._kinematics_cache

    def get_pose(self):
        return self._kinematics().fk(self.get_state().positions)

    def preview_move_to(self, position_m, *, quaternion_xyzw=None, velocity_scale=1.0):
        self._require_connection()
        if self._gravity_active or self._moving:
            raise RuntimeError('请先退出重力补偿或其他运动，再规划空间目标')
        if not math.isfinite(velocity_scale) or not 0 < velocity_scale <= 1:
            raise ValueError('velocity_scale 必须位于 (0, 1]')
        model = self._kinematics()
        target = model.vector(position_m, 3, '空间目标')
        states = self._stable_feedback()
        start = tuple(item.position for item in states[:6])
        before = model.fk(start)
        limits = self.config['cartesian']
        distance = math.dist(before['position_m'], target)
        plan = model.solve(start, target, quaternion_xyzw)
        delta = tuple(abs(a-b) for a,b in zip(start,plan['target_joints_rad']))
        from ..motion import speed_limits
        speed_limit=tuple(max(self.config['hold_speed_rad_s'],v*velocity_scale) for v in speed_limits(self.config))
        duration = max(limits['minimum_duration_s'], max(d/v for d,v in zip(delta,speed_limit)))
        curve = self._build_transition(start, plan['target_joints_rad'], duration, speed_limit)
        collision = model.check_trajectory(curve, limits['collision_step_rad'])
        plan.update(curve.summary())
        plan.update({'source':'hardware_preview', 'start_joints_rad': start,
                     'translation_m': distance, 'joint_delta_degrees':[math.degrees(v) for v in delta],
                     'duration_s':curve.duration, 'velocity_scale':velocity_scale,
                     'effective_max_speed_rad_s':max(speed_limit), 'effective_max_speeds_rad_s':speed_limit, 'collision_check':collision,
                     'motion_sent':False})
        self._record('cartesian_preview', **plan)
        return plan

    def move_to(self, position_m, *, quaternion_xyzw=None, velocity_scale=1.0):
        self._require_connection()
        if not self._enabled:
            raise RuntimeError('请先显式 enable()')
        plan = self.preview_move_to(position_m, quaternion_xyzw=quaternion_xyzw, velocity_scale=velocity_scale)
        model = self._kinematics()
        measured = {}
        def reached(actual):
            pose = model.fk(actual)
            error = math.dist(pose['position_m'], plan['target_position_m'])
            angle = model.orientation_error(pose['quaternion_xyzw'], plan['target_quaternion_xyzw'])
            measured.update(pose=pose, error=error, angle=angle)
            return error <= self.config['cartesian']['position_tolerance_m'] and angle <= self.config['cartesian']['orientation_tolerance_rad']
        result = self._execute_joint_target(plan['target_joints_rad'], duration_s=plan['duration_s'],
            max_step_rad=None, single_joint=False,
            expected_start=plan['start_joints_rad'], goal_validator=reached, smooth=True,
            speed_limit=plan['effective_max_speeds_rad_s'])
        self._record('cartesian_completed', actual_position_m=measured['pose']['position_m'],
                     position_error_m=measured['error'], orientation_error_rad=measured['angle'])
        return MotionResult('succeeded','hardware',result.final_positions,
            '官方 IK + Ruckig 平滑参考 + SDK；到位反馈已核对；参考轨迹自碰撞检查，不含外部障碍物',
            target_position_m=tuple(plan['target_position_m']), actual_position_m=measured['pose']['position_m'],
            position_error_m=measured['error'], trajectory_points=result.trajectory_points)

    def preview_home(self, *, velocity_scale=1.0):
        self._require_connection()
        if self._gravity_active or self._moving:
            raise RuntimeError('请先退出重力补偿或其他运动，再回 home')
        if not math.isfinite(velocity_scale) or not 0 < velocity_scale <= 1:
            raise ValueError('velocity_scale 必须位于 (0, 1]')
        model = self._kinematics()
        target = model.vector(self.config['home_joints_rad'], 6, 'home 关节角')
        if any(not low <= value <= high for value, low, high in
               zip(target, self.config['lower_rad'], self.config['upper_rad'])):
            raise ValueError('home 目标超出物理关节限位')
        states = self._stable_feedback()
        start = tuple(item.position for item in states[:6])
        delta = tuple(abs(a-b) for a,b in zip(start,target))
        from ..motion import speed_limits
        speed=tuple(max(self.config['hold_speed_rad_s'],v*velocity_scale) for v in speed_limits(self.config))
        duration = max(self.config['cartesian']['minimum_duration_s'],max(d/v for d,v in zip(delta,speed)))
        curve = self._build_transition(start, target, duration, speed)
        collision = model.check_trajectory(curve, self.config['cartesian']['collision_step_rad'])
        plan = {'source':'hardware_home_preview', 'start_joints_rad':start,
                'target_joints_rad':target, 'target_pose':model.fk(target),
                'joint_delta_degrees':[math.degrees(v) for v in delta],
                'duration_s':curve.duration, 'velocity_scale':velocity_scale,
                'effective_max_speed_rad_s':max(speed), 'effective_max_speeds_rad_s':speed, 'gripper_hold_raw_rad':states[6].position,
                'collision_check':collision, 'motion_sent':False,
                'note':'回到配置中的关节姿态，不重设编码器零点'}
        plan.update(curve.summary())
        self._record('home_preview', **plan)
        return plan

    def home(self, *, velocity_scale=1.0):
        self._require_connection()
        if not self._enabled:
            raise RuntimeError('请先显式 enable()')
        plan = self.preview_home(velocity_scale=velocity_scale)
        result = self._execute_joint_target(plan['target_joints_rad'], duration_s=plan['duration_s'],
            max_step_rad=None, single_joint=False, expected_start=plan['start_joints_rad'],
            smooth=True, speed_limit=plan['effective_max_speeds_rad_s'])
        self._record('home_completed', final_positions=result.final_positions,
                     target_joints_rad=plan['target_joints_rad'])
        return MotionResult('succeeded', 'hardware', result.final_positions,
            'Ruckig 平滑回 home + 官方 SDK；新反馈确认到位与稳定；夹爪保持，不重设编码器零点',
            trajectory_points=result.trajectory_points)

    def _gravity_model(self):
        from .panthera_gravity import PantheraGravityModel
        return PantheraGravityModel()

    def _send_gravity(self, torques):
        if len(torques) != 6 or not all(math.isfinite(value) for value in torques):
            raise ValueError('需要六个有限重力力矩')
        # 五参数协议与位置/速度上限协议共用整帧缓冲，禁止在同帧混用夹爪位置命令。
        # 原生 SDK 模式切换先填 0x8000 未更新标志；只填六轴，夹爪保持先前模式。
        for motor, torque in zip(self._motors[:6], torques):
            motor.pos_vel_tqe_kp_kd(0.0, 0.0, float(torque), 0.0, 0.0)
        self._driver.motor_send_cmd()

    def run_gravity_compensation(self, *, duration_s=None):
        self._require_connection()
        if not self._enabled:
            raise RuntimeError('请先显式 enable()')
        if self._moving or self._gravity_active:
            raise RuntimeError('执行通道已被运动或重力补偿占用')
        if duration_s is not None and (not math.isfinite(duration_s) or duration_s <= 0):
            raise ValueError('duration_s 必须为空或正的有限数')
        settings = self.config['gravity_compensation']
        for key in ['period_s', 'query_period_s', 'feedback_timeout_s', 'max_command_gap_s',
                    'max_velocity_rad_s', 'log_period_s']:
            if not math.isfinite(settings[key]) or settings[key] <= 0:
                raise ValueError(f'重力补偿参数 {key} 必须是正的有限数')
        if settings['max_command_gap_s'] < settings['period_s']:
            raise ValueError('重力补偿最大周期间隔不能小于目标周期')
        caps = tuple(float(value) for value in settings['torque_caps_nm'])
        if (len(caps) != 6 or any(not math.isfinite(value) or not 0 < value <= allowed
                                for value, allowed in zip(caps, self.config['torque_caps_nm']))):
            raise ValueError('重力补偿力矩上限必须为正且不超过该机型既有限制')
        model = self._gravity_model()  # 离线构造，失败时还没有力矩命令
        self._stable_feedback()
        self._gravity_active = True
        started = time.monotonic()
        last_send = None
        last_log = -math.inf
        last_print = started
        cycles = clipped_cycles = 0
        recent_feedback = deque(maxlen=64)
        max_gap = 0.0
        reason = 'duration_elapsed'
        hold_confirmed = False
        self._record('gravity_start', settings=settings, model='official_get_Gravity',
                     note='Six arm joints only; no payload calibration or friction compensation')
        print('重力补偿即将启用：仅六轴，Ctrl+C 切回当前位置保持。物理急停保持可操作。', flush=True)
        try:
            while duration_s is None or time.monotonic()-started < duration_s:
                cycle_start = time.monotonic()
                states = self.read_feedback(query_period_s=settings['query_period_s'],
                                            timeout_s=settings['feedback_timeout_s'], record=False)
                positions = tuple(item.position for item in states[:6])
                observed = time.monotonic()
                recent_feedback.append({
                    'monotonic_s': observed,
                    'position_rad': positions,
                    'velocity_rad_s': [item.velocity for item in states[:6]],
                    'sample_counters': [item.num for item in states[:6]],
                    'sample_time_unix_s': [item.time for item in states[:6]],
                })
                offending = [
                    {'joint': self.profile.joint_names[index], 'motor_id': item.ID,
                     'position_rad': item.position, 'velocity_rad_s': item.velocity,
                     'sample_counter': item.num, 'feedback_age_s': time.time()-item.time,
                     'fault': item.fault, 'mode': item.mode}
                    for index, item in enumerate(states[:6])
                    if abs(item.velocity) > settings['max_velocity_rad_s']
                ]
                if offending:
                    self._record('gravity_speed_guard',
                                 limit_rad_s=settings['max_velocity_rad_s'],
                                 offending_joints=offending,
                                 recent_feedback=[sample for sample in recent_feedback
                                                  if observed-sample['monotonic_s'] <= 0.15])
                    details = ', '.join(f"{item['joint']}={item['velocity_rad_s']:.4f} rad/s"
                                        for item in offending)
                    raise RuntimeError('重力补偿时实测速度超过配置保护阈值：'
                                       f"{details}；阈值={settings['max_velocity_rad_s']:.3f} rad/s")
                requested = tuple(float(value) for value in model(positions))
                if len(requested) != 6 or not all(math.isfinite(value) for value in requested):
                    raise RuntimeError('重力模型返回无效力矩，拒绝下发')
                applied = tuple(max(-cap, min(cap, value)) for value, cap in zip(requested, caps))
                clipped = applied != requested
                now = time.monotonic()
                gap = now-last_send if last_send is not None else 0.0
                if gap > settings['max_command_gap_s']:
                    raise RuntimeError('重力补偿循环超时，拒绝继续下发旧周期力矩')
                self._moving = True  # 后续即使发送异常，也必须尝试保持停止
                self._send_gravity(applied)
                last_send = now
                cycles += 1
                clipped_cycles += int(clipped)
                max_gap = max(max_gap, gap)
                if now-last_log >= settings['log_period_s']:
                    self._record('gravity_tick', cycle=cycles, position_rad=positions,
                                 velocity_rad_s=[item.velocity for item in states[:6]],
                                 requested_torque_nm=requested, applied_torque_nm=applied,
                                 clipped=clipped, command_gap_s=gap,
                                 feedback_counters=[item.num for item in states])
                    last_log = now
                if now-last_print >= 1.0:
                    frequency = cycles / max(time.monotonic()-started, 1e-6)
                    print(f'补偿运行：q(rad)={[round(v,4) for v in positions]} '
                          f'力矩(Nm)={[round(v,3) for v in applied]} '
                          f'平均频率={frequency:.1f} Hz，累计饱和周期={clipped_cycles}', flush=True)
                    last_print = now
                remaining = settings['period_s']-(time.monotonic()-cycle_start)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            reason = 'user_interrupt'
        except BaseException as error:
            self._record('gravity_failure', reason=str(error))
            raise
        finally:
            elapsed = time.monotonic()-started
            try:
                if self._moving:
                    self.stop()
                    hold_confirmed = True
                    self._record('gravity_exit_hold_confirmed', cycles=cycles, duration_s=elapsed)
            finally:
                self._gravity_active = False
        result = GravityResult('stopped' if cycles else 'not_started', 'hardware', elapsed, cycles,
                               cycles/elapsed if elapsed else 0.0,
                               max_gap, clipped_cycles, reason, hold_confirmed)
        self._record('gravity_result', **result.__dict__)
        return result

    def request_hold(self):
        """只下发一次保持；是否停稳由 poll_hold 的持续新反馈确认。"""
        from ..execution import HoldExecution
        self._reference_execution=None
        self._gripper_action=None
        self._require_connection()
        states=self.read_feedback()
        self._send(tuple(s.position for s in states[:6]),
                   [self.config['hold_speed_rad_s']]*6,states[6].position)
        self._moving=True
        self._hold_execution=HoldExecution(self,states,time)
        self._record('hold_requested',target_rad=[s.position for s in states])

    def poll_hold(self,feedback=None):
        if self._hold_execution is None:return True
        return self._hold_execution.step(feedback)

    def stop(self):
        self.request_hold()
        while not self.poll_hold():
            pass

    def close(self):
        error = None
        if self._driver is not None and self._moving:
            try:
                self.stop()
            except Exception as exception:
                error = exception
                self._record('close_with_stop_unconfirmed', reason=str(exception))
        self._record('transport_closing', note='Passive SDK destructor; no auto brake/reset/home. Does not prove torque disable.')
        self._motors = []
        self._driver = None
        gc.collect()
        self._enabled = False
        self._feedback_stability = None
        if self._temp:
            self._temp.cleanup()
            self._temp = None
        if self._lock:
            self._lock.close()
            self._lock = None
        if self._log:
            self._log.close()
            self._log = None
        if error:
            raise RuntimeError('关闭连接时未确认停止；请立即检查实物并使用物理急停') from error
