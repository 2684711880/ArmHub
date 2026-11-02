"""Official LeRobot control chain with ArmHub transport ownership and exit policy."""
import fcntl
import hashlib
import logging
import math
import os
from pathlib import Path
import subprocess
import time


def fresh_states(bus, timeout=1.0):
    previous = [m.get_current_motor_state().num for m in bus.htr_motors]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        bus.robot.send_get_motor_state_cmd()
        time.sleep(0.01)
        states = [m.get_current_motor_state() for m in bus.htr_motors]
        if any(s.num <= n or not -0.02 <= time.time()-s.time <= 0.2
               for s, n in zip(states, previous)):
            continue
        if len(states) != 7 or any(s.ID != i+1 or s.fault or not all(
            math.isfinite(v) for v in (s.position, s.velocity, s.torque))
            for i, s in enumerate(states)):
            raise RuntimeError('七电机反馈身份、故障或数值检查失败')
        return states
    raise RuntimeError('七电机新反馈超时')


def hold_current(bus):
    states = fresh_states(bus)
    target = [s.position for s in states]
    # SDK position/velocity/max-torque mode; zero speed is not used as a stop.
    import yaml
    cfg = yaml.safe_load(Path(bus.config_path).read_text())
    caps = [*cfg['robot']['max_torque'], 0.5]
    for motor, q, cap in zip(bus.htr_motors, target, caps):
        motor.pos_vel_MAXtqe(q, 0.1, cap)
    bus.robot.motor_send_cmd()
    deadline = time.monotonic() + 2.0
    stable_since = None
    while time.monotonic() < deadline:
        states = fresh_states(bus)
        stable = all(abs(s.position-q) < 0.04 and abs(s.velocity) < 0.08
                     for s, q in zip(states, target))
        stable_since = (stable_since or time.monotonic()) if stable else None
        if stable_since is not None and time.monotonic()-stable_since >= 0.2:
            return
    raise RuntimeError('已发送当前位置保持，但尚未确认稳定；不代表失能或物理急停')


def disconnect_device(device):
    """Stop the writer, hold measured pose, then close cameras and transport."""
    thread = device._thread
    if thread is not None:
        device._stop_event.set()
        thread.join(timeout=3.0)
        if thread.is_alive():
            raise RuntimeError('控制线程未退出，拒绝并发发送退出保持命令')
        device._thread = None
    try:
        if device.bus.is_connected:
            hold_current(device.bus)
    finally:
        # Passive native destructor; never resets, homes or changes motor zero.
        device.bus.disconnect()
        for camera in getattr(device, 'cameras', {}).values():
            if camera.is_connected:
                camera.disconnect()


def configure_runtime():
    import yaml
    import numpy as np
    import hightorque_robot as htr
    from lerobot_robot_panthera.motors.panthera_motors_bus import PantheraMotorsBus
    if not getattr(htr, 'commission_safe_binding', False):
        raise RuntimeError('需要 ArmHub 编译的 passive-lifecycle/snapshot-lock SDK')
    original_connect = PantheraMotorsBus.connect
    original_disconnect = PantheraMotorsBus.disconnect
    original_read = PantheraMotorsBus.sync_read

    def connect(bus):
        if bus.is_connected:
            return
        cfg = yaml.safe_load(Path(bus.config_path).read_text())
        if cfg['robot'].get('commission_passive_lifecycle') is not True:
            raise RuntimeError('SDK 配置必须启用 commission_passive_lifecycle')
        motors = yaml.safe_load(Path(cfg['robot']['param_file']).read_text())
        device = Path(motors['robot']['Serial_Type'])
        if not device.exists():
            raise RuntimeError(f'串口不存在：{device}')
        if subprocess.run(['fuser', '-s', str(device.resolve())], check=False).returncode != 1:
            raise RuntimeError('串口占用或无法检查占用，请先退出现有控制器')
        key = hashlib.sha256(str(device.resolve()).encode()).hexdigest()[:16]
        bus._armhub_lock = open(f'/tmp/armhub-panthera-{key}.lock', 'a')
        try:
            fcntl.flock(bus._armhub_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            original_connect(bus)
            if [m.get_motor_id() for m in bus.htr_motors] != list(range(1, 8)):
                raise RuntimeError('必须识别 ID 1…7')
            states = fresh_states(bus)
            bus.target_positions = np.array([s.position for s in states])
        except BaseException:
            disconnect(bus)
            raise

    def disconnect(bus):
        original_disconnect(bus)
        lock = getattr(bus, '_armhub_lock', None)
        if lock is not None:
            lock.close()
            bus._armhub_lock = None

    def sync_read(bus, data_name):
        if data_name not in ('Present_Position', 'Present_Velocity', 'Present_Torque'):
            return original_read(bus, data_name)
        attribute = {'Present_Position':'position', 'Present_Velocity':'velocity',
                     'Present_Torque':'torque'}[data_name]
        return {name:getattr(state, attribute) for name,state in
                zip(bus.motor_names, fresh_states(bus))}

    PantheraMotorsBus.connect = connect
    PantheraMotorsBus.disconnect = disconnect
    PantheraMotorsBus.sync_read = sync_read
    logging.info('ArmHub LeRobot: official control chain; automatic preset motions disabled')
