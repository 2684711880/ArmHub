"""实物驱动替身 + 实际任务会话/控制器/参考执行器；重现已稳定后启动追踪被旧帧取消。"""
import unittest
from pathlib import Path
from types import SimpleNamespace
import tempfile
from unittest.mock import patch
import numpy as np
from armhub.task_runtime import ArmTaskRuntime
from apps.hole_approach.__main__ import arguments
from apps.hole_approach.config import load_config
from apps.hole_approach.controller import Controller
from motion_fixture import virtual_time
from test_panthera_cartesian import FakeKinematics
import test_panthera_hardware as fixtures


class TrackingStartupTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm
    def test_stable_streaming_feedback_does_not_expire_camera_on_start(self):
        clock=virtual_time(self)
        arm,driver=self.make_arm()
        arm.config.update(feedback_query_period_s=.025,stable_window_s=.5,arrival_dwell_s=.3)
        arm._kinematics_cache=FakeKinematics()
        # 模拟运行中的稳定反馈：尚未开始任何运动，已有完整历史。
        for _ in range(30):arm.get_joint_feedback()
        arm.enable()
        runtime=ArmTaskRuntime.__new__(ArmTaskRuntime)
        runtime.arm=arm;runtime.allow_motion=True;runtime.command_guard=None;runtime.trajectory_done=True
        runtime.cached=arm.get_joint_feedback();runtime.gripper_goal=None
        runtime.config=load_config()
        runtime.joint_limits={k:np.array(v) for k,v in runtime.config['robot']['joint_limits'].items()}
        runtime.velocity_limits=np.array(runtime.config['robot']['velocity_limits'])
        runtime.positions=lambda:np.array([s.position for s in runtime.cached[:6]])
        runtime.forward_kinematics=lambda q:{'transform':np.eye(4),'rotation':np.eye(3),'position':[0,0,0]}
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        a=arguments(['--execute','--report',str(Path(temp.name)/'plan.json')])
        c=Controller(runtime,a,np.eye(4),'fixture',lambda task:None)
        # Camera and motor clocks share a test timestamp; only the startup duration matters.
        now=clock.monotonic()
        c.tracking=True;c.vision='visible';c.frame_stamp=clock.time()
        goal=[.12,.1,.1,.1,.1,.1]
        epoch=clock.time()
        controller_clock=SimpleNamespace(monotonic=clock.monotonic,time=lambda:epoch+clock.monotonic()-now)
        with patch('apps.hole_approach.controller.time',controller_clock):
            c.send({'target_joints_rad':goal,'duration_s':2.},'APPROACH')
        # Image was fresh when the target was accepted; preflight must not burn its 0.7s lifetime.
        self.assertLess(clock.monotonic()-now,.7)
        self.assertEqual(driver.transmissions,[])
        with patch('apps.hole_approach.controller.time',controller_clock):
            c.tick(runtime.positions())
        self.assertEqual(c.state,'APPROACH')
        self.assertTrue(c.tracking)
        self.assertEqual(len(driver.transmissions),1)


    def test_cold_start_still_requires_full_stable_window(self):
        clock=virtual_time(self);arm,driver=self.make_arm()
        arm.config.update(feedback_query_period_s=.025,stable_window_s=.5)
        now=clock.monotonic();arm.enable()
        self.assertGreaterEqual(clock.monotonic()-now,.5)
        self.assertEqual(driver.transmissions,[])

    def test_feedback_gap_invalidates_stability_cache(self):
        clock=virtual_time(self);arm,driver=self.make_arm()
        arm.config.update(feedback_query_period_s=.025,stable_window_s=.5)
        arm.enable();clock.sleep(.2)
        now=clock.monotonic();arm.enable()
        self.assertGreaterEqual(clock.monotonic()-now,.5)
        self.assertEqual(driver.transmissions,[])

    def test_position_change_invalidates_stability_cache(self):
        clock=virtual_time(self);arm,driver=self.make_arm()
        arm.config.update(feedback_query_period_s=.025,stable_window_s=.5)
        arm.enable();driver.states[1].position+=.02
        now=clock.monotonic();arm.enable()
        self.assertGreaterEqual(clock.monotonic()-now,.5)
        self.assertEqual(driver.transmissions,[])
