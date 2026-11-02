"""Explicit return to the teaching panel's persisted pose, with one SDK owner."""
import json
from pathlib import Path
import threading
import time

from apps.teaching_panel.controller import ROOT, validate_options
from apps.teaching_panel.hardware import HardwareSession

SETTINGS = ROOT/'integrations/panthera_lerobot/configs/teaching.json'


def load_prepared(path=SETTINGS):
    options = validate_options(json.loads(Path(path).read_text()))
    prepared = options['prepared']
    if prepared is None or prepared['source'] != 'hardware':
        raise ValueError('请先在示教窗口保存真实机械臂的准备位')
    return options


def return_prepared(*, execute=False, settings_path=SETTINGS, session_factory=HardwareSession):
    options = load_prepared(settings_path)
    print('准备位文件：', settings_path, flush=True)
    print('六轴目标 / rad：', options['prepared']['q'], flush=True)
    session = session_factory()
    cancelled = threading.Event()
    try:
        session.connect()
        session.arm._stable_feedback()
        sample = session.read()
        plan = session.preview(sample, options['prepared'], options['return_scale'],
                               options['return_duration_s'], options['tcp_z_min_m'])
        print('六轴到位后张开夹爪：',dict(target_rad=options['gripper_open'],
              speed_rad_s=options['gripper_speed'],torque_cap_nm=options['gripper_torque']),flush=True)
        print(json.dumps({k:v for k,v in plan.items() if k!='curve'},
                         ensure_ascii=False, indent=2), flush=True)
        if not execute:
            print('只读预览完成，未发送运动命令。执行请加 --execute。', flush=True)
            return plan
        session.start_return(plan, lambda:not cancelled.is_set())
        while not session.advance_return(session.read()):
            time.sleep(.005)
        print('六轴已到准备位，开始受限速/限力矩张开夹爪。',flush=True)
        session.arm.command_gripper(options['gripper_open'],speed=options['gripper_speed'],
                                    torque=options['gripper_torque'],
                                    command_guard=lambda:not cancelled.is_set())
        while not session.arm.poll_gripper():
            time.sleep(.005)
        print('新反馈已确认：六轴在准备位、夹爪已张开。', flush=True)
        return plan
    finally:
        cancelled.set()
        session.close()
