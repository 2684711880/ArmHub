"""真实 Controller→ArmTaskRuntime→Panthera 后端→Ruckig 执行器，只有电机与碰撞模型为替身。"""
import copy
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from armhub.task_runtime import ArmTaskRuntime
from apps.hole_approach.__main__ import arguments
from apps.hole_approach.config import load_config
from apps.hole_approach.controller import Controller
from motion_fixture import virtual_time
from test_panthera_cartesian import FakeKinematics
import test_panthera_hardware as fixtures


class TrackingRuntimeChainTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm
    def test_tracking_retarget_after_final_reference_keeps_running(self):
        clock=virtual_time(self);wall=clock.time();origin=clock.monotonic()
        clock.time=lambda:wall+clock.monotonic()-origin
        arm,driver=self.make_arm();arm._kinematics_cache=FakeKinematics()
        arm.config.update(velocity_limits_rad_s=[1.68,2.,2.,1.68,1.48,1.48],arrival_dwell_s=.3,feedback_query_period_s=.025)
        original=driver.send_get_motor_state_cmd
        def feedback():
            original()
            for s in driver.states:s.time=clock.time()
        driver.send_get_motor_state_cmd=feedback
        arm.enable()
        runtime=ArmTaskRuntime.__new__(ArmTaskRuntime)
        runtime.arm=arm;runtime.config=load_config();runtime.allow_motion=True
        runtime.command_guard=None;runtime.trajectory_done=True;runtime.gripper_goal=None
        runtime.cached=arm.get_joint_feedback()
        runtime.joint_limits={k:np.array(v) for k,v in runtime.config['robot']['joint_limits'].items()}
        runtime.velocity_limits=np.array(runtime.config['robot']['velocity_limits'])
        runtime.forward_kinematics=lambda q:{'transform':np.eye(4),'position':[0,0,0],'rotation':np.eye(3)}
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        a=arguments(['--execute','--report',str(Path(tmp.name)/'plan.json')])
        controller_clock=SimpleNamespace(time=clock.time,monotonic=clock.monotonic)
        patcher=patch('apps.hole_approach.controller.time',controller_clock);patcher.start();self.addCleanup(patcher.stop)
        c=Controller(runtime,a,np.eye(4),'fixture',lambda task:None)
        c.tracking=True;c.vision='visible'
        c.observation={'world_center':np.array([.5,0,.5]),'world_normal':np.array([0,0,-1]),'stamp':clock.time()}
        c.same_target=lambda plan:True
        def plan(goal):
            stage={'target_joints_rad':goal,'target_position_m':[.2,0,.3],'duration_s':3.}
            return dict(stage,stage1=stage,stage2_waypoints=[stage],feed_trajectory=None,
                        hole_center_base_m=[.5,0,.5],outward_normal_base=[0,0,-1])
        c.begin_plan(plan([.14,.1,.1,.1,.1,.1]))
        self.assertEqual(c.active['started'],arm._reference_execution.started)
        for _ in range(300):
            q=c.feedback();c.frame_stamp=clock.time();c.tick(q);clock.sleep(.005)
            if arm._reference_execution.final_sent:break
        old=arm._reference_execution
        self.assertTrue(old.final_sent);self.assertTrue(arm._moving)
        # 模拟固件仍差 0.012 rad 才到目标；控制循环继续收反馈，保持最后目标。
        driver.states[0].position=old.target[0]-.012
        for _ in range(8):
            q=c.feedback();c.frame_stamp=clock.time();c.tick(q);clock.sleep(.005)
        self.assertGreater(clock.monotonic()-old.last_send,.2)
        c.begin_plan(plan([.18,.1,.1,.1,.1,.1]))
        count=len(driver.transmissions)
        q=c.feedback();c.frame_stamp=clock.time();c.tick(q)
        self.assertTrue(c.tracking);self.assertEqual(c.state,'APPROACH')
        self.assertGreater(len(driver.transmissions),count)
        self.assertIsNot(old,arm._reference_execution)

    def test_runtime_preloads_geometry_before_allowing_control(self):
        order=[]
        class Backend:
            log_path=None
            def connect(self):order.append('connect')
            def prepare_motion_model(self):order.append('warmup')
            def enable(self):order.append('enable')
            def get_joint_feedback(self):return [SimpleNamespace(position=.1)]*7
            def close(self):order.append('close')
        math=SimpleNamespace(joint_limits={'lower':np.zeros(6),'upper':np.ones(6)})
        with patch('armhub.task_runtime.create_arm',return_value=Backend()),patch('armhub.task_runtime.OfflineKinematics',return_value=math):
            runtime=ArmTaskRuntime('panthera_ht',load_config(),allow_motion=True)
            runtime.close()
        self.assertLess(order.index('warmup'),order.index('enable'))
        self.assertEqual(order.count('warmup'),1)

if __name__=='__main__':unittest.main()
