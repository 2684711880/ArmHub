"""Replay diagnostics only: observe native gripper feedback without replacing control."""
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time


class GripperTrace:
    fields = ('monotonic_s', 'feedback_unix_s', 'feedback_age_s', 'feedback_counter',
              'requested_target_rad', 'previous_target_rad', 'position_rad',
              'velocity_rad_s', 'feedback_torque_nm', 'position_error_rad', 'fault',
              'assist_stage', 'requested_feedforward_nm', 'limited_position_target_rad')

    def __init__(self, path):
        self.path = Path(path).resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', newline='')
        self.writer = csv.DictWriter(self.stream, fieldnames=self.fields)
        self.writer.writeheader(); self.stream.flush()
        self.count = 0; self.previous_target = None
        self.extrema = {}; self.class_restore = None
        self.assist_config = None
        self.failure = None

    def observe(self, target, state, assist=None):
        target = float(target)
        age = time.time() - state.time
        if (state.ID != 7 or state.num <= 0 or state.fault or not -.02 <= age <= .2
                or not all(math.isfinite(v) for v in
                           (target, state.position, state.velocity, state.torque))):
            raise RuntimeError('夹爪诊断收到过期、故障或无效反馈，停止回放')
        assist = assist or dict(stage='position',feedforward_nm=0.)
        row = dict(monotonic_s=time.monotonic(), feedback_unix_s=state.time,
                   feedback_age_s=age, feedback_counter=state.num,
                   requested_target_rad=target, previous_target_rad=self.previous_target,
                   position_rad=state.position, velocity_rad_s=state.velocity,
                   feedback_torque_nm=state.torque, position_error_rad=target-state.position,
                   fault=state.fault, assist_stage=assist['stage'],
                   requested_feedforward_nm=assist['feedforward_nm'],
                   limited_position_target_rad=assist.get('limited_target_rad',target))
        self.writer.writerow(row); self.stream.flush()
        self.previous_target = target; self.count += 1
        for key in ('position_rad', 'velocity_rad_s', 'feedback_torque_nm', 'position_error_rad'):
            low, high = self.extrema.get(key, (row[key], row[key]))
            self.extrema[key] = (min(low,row[key]), max(high,row[key]))

    def close(self):
        if self.stream.closed:
            return
        if self.class_restore:
            cls, wrapper, original = self.class_restore
            if cls.send_action is wrapper:
                cls.send_action = original
        self.stream.close()
        summary = dict(samples=self.count, extrema=self.extrema,
                       control_changed=self.assist_config is not None,
                       assist_config=self.assist_config, contact_threshold_calibrated=False,
                       failure=self.failure,
                       torque_note='native SDK motor torque feedback, not calibrated finger force',
                       alignment_note='feedback precedes requested target; previous_target is prior request')
        self.path.with_suffix(self.path.suffix+'.summary.json').write_text(
            json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        mode = '位置加力辅助已启用' if self.assist_config is not None else '未改变官方控制'
        print(f'夹爪日志：{self.path}，{self.count} 个采样；{mode}。',flush=True)


def install_gripper_trace(path, follower_class=None):
    if follower_class is None:
        from lerobot_robot_panthera.panthera_follower import PantheraFollower
        follower_class = PantheraFollower
    trace = GripperTrace(path)
    original = follower_class.send_action

    def send_action(device, action):
        if device.teaching_mode:
            raise RuntimeError('夹爪回放诊断不适用于手动示教模式')
        # get_observation() already acquired fresh seven-motor feedback this cycle.
        # Snapshot it without issuing another query or creating a second SDK owner.
        state = device.bus.htr_motors[6].get_current_motor_state()
        trace.observe(action['gripper.pos'],state,getattr(device,'_armhub_gripper_assist_status',None))
        return original(device, action)

    follower_class.send_action = send_action
    trace.class_restore = (follower_class, send_action, original)
    return trace


def default_trace_path(root):
    return Path(root)/'logs/lerobot/gripper'/('replay_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.csv')
