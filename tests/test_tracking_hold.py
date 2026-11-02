"""重现重新看见洞后的阻塞保持，以及位置稳定但速度反馈抖动的现场条件。"""
import unittest
from types import SimpleNamespace
from armhub.task_runtime import ArmTaskRuntime
from motion_fixture import virtual_time
import test_panthera_hardware as fixtures


class TrackingHoldTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm

    def setup_clock(self):
        clock=virtual_time(self);wall=clock.time();origin=clock.monotonic()
        clock.time=lambda:wall+clock.monotonic()-origin
        arm,driver=self.make_arm();arm.config.update(feedback_query_period_s=.025,arrival_dwell_s=.3)
        original=driver.send_get_motor_state_cmd
        def feedback():
            original()
            for s in driver.states:s.time=clock.time()
            # 实际日志中 J3 位置稳定，而速度在约 ±0.25 rad/s 跳动。
            driver.states[2].velocity=.25 if driver.states[2].num%2 else -.25
        driver.send_get_motor_state_cmd=feedback
        return clock,arm,driver

    def test_position_stable_hold_is_confirmed_despite_velocity_spikes(self):
        clock,arm,driver=self.setup_clock();arm._moving=True
        arm.stop()
        self.assertFalse(arm._moving)

    def test_task_hold_returns_then_confirms_on_regular_feedback_ticks(self):
        clock,arm,driver=self.setup_clock();runtime=ArmTaskRuntime.__new__(ArmTaskRuntime)
        runtime.arm=arm;runtime.allow_motion=True;runtime.gripper_goal=None
        started=clock.monotonic();runtime.hold()
        self.assertLess(clock.monotonic()-started,.1)
        self.assertTrue(runtime.hold_pending)
        for _ in range(30):
            runtime.feedback()
            if not runtime.hold_pending:break
        self.assertFalse(runtime.hold_pending);self.assertFalse(arm._moving)
        self.assertEqual(len(driver.transmissions),1)

    def test_recorded_reacquire_feedback_replay_confirms_position_hold(self):
        import json
        from pathlib import Path
        from armhub.execution import HoldExecution
        record=json.loads((Path(__file__).parent/'fixtures/tracking_reacquire_hold.json').read_text())
        clock,arm,driver=self.setup_clock()
        origin=record['hold_command']['monotonic_s'];wall=clock.time();clock.now=origin
        clock.time=lambda:wall+clock.now-origin
        def states(frame):
            return [SimpleNamespace(ID=m['id'],num=m['sample_counter'],time=clock.time()-m['age_s'],
                    position=m['position_rad'],velocity=m['velocity_rad_s'],torque=m['torque_nm'],fault=m['fault'])
                    for m in frame['motors']]
        initial=states(record['initial'])
        for state,q in zip(initial,record['hold_command']['target_rad']):state.position=q
        session=HoldExecution(arm,initial,clock);arm._moving=True
        for frame in record['frames']:
            clock.now=frame['monotonic_s']
            if session.step(states(frame)):break
        self.assertTrue(session.done);self.assertFalse(arm._moving)
        self.assertGreater(session.raw_velocity_peak,.05)
        self.assertLess(clock.now-origin,3.)

    def test_continuing_position_drift_cannot_confirm_hold(self):
        clock,arm,driver=self.setup_clock()
        original=driver.send_get_motor_state_cmd
        def drift():
            original();driver.states[0].position+=.001
        driver.send_get_motor_state_cmd=drift
        arm._moving=True
        with self.assertRaisesRegex(RuntimeError,'停止反馈未确认'):arm.stop()
        driver.send_get_motor_state_cmd=original
        self.assertTrue(arm._moving)

class ReacquireRuntimeChainTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm
    setup_clock=TrackingHoldTests.setup_clock

    def test_repeated_lost_recovery_reacquire_runs_through_real_backend(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        import numpy as np
        from apps.hole_approach.controller import Controller
        from apps.hole_approach.config import load_config
        from apps.hole_approach.__main__ import arguments
        from test_panthera_cartesian import FakeKinematics
        clock,arm,driver=self.setup_clock();arm._kinematics_cache=FakeKinematics();arm.enable()
        runtime=ArmTaskRuntime.__new__(ArmTaskRuntime)
        runtime.arm=arm;runtime.config=load_config();runtime.allow_motion=True
        runtime.command_guard=None;runtime.trajectory_done=True;runtime.gripper_goal=None;runtime.hold_pending=False
        runtime.cached=arm.get_joint_feedback()
        runtime.joint_limits={k:np.array(v) for k,v in runtime.config['robot']['joint_limits'].items()}
        runtime.velocity_limits=np.array(runtime.config['robot']['velocity_limits'])
        runtime.forward_kinematics=lambda q:{'transform':np.eye(4),'position':[0,0,0],'rotation':np.eye(3)}
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        args=arguments(['--execute','--report',str(Path(tmp.name)/'report.json')])
        controller_clock=SimpleNamespace(time=clock.time,monotonic=clock.monotonic)
        with patch('apps.hole_approach.controller.time',controller_clock):
            tasks=[];c=Controller(runtime,args,np.eye(4),'fixture',tasks.append)
            c.tracking=True
            def frame(visible):
                q=c.feedback()
                c.observe({'stamp':clock.time(),'observation':
                    {'center':[.1,0,.5],'normal':[0,0,-1],'plane_rms':.001} if visible else None})
                c.tick(q)
            frame(False)
            for cycle in range(2):
                for _ in range(50):
                    if c.active is not None and c.state=='RECOVER':break
                    frame(False)
                self.assertEqual(c.state,'RECOVER');self.assertIsNotNone(c.active)
                started=clock.monotonic();frame(True)
                self.assertLess(clock.monotonic()-started,.1)
                self.assertTrue(runtime.hold_pending);self.assertTrue(c.tracking)
                for _ in range(30):
                    frame(True)
                    if c.pending is not None:break
                self.assertFalse(runtime.hold_pending);self.assertEqual(c.state,'PLANNING')
                self.assertGreaterEqual(c.good,3)
                task=tasks[-1];goal=runtime.positions();goal[0]+=.02
                stage={'target_joints_rad':goal.tolist(),'target_position_m':[.1,0,.5],'duration_s':3.}
                plan=dict(stage,stage1=stage,stage2_waypoints=[stage],feed_trajectory=None,
                          hole_center_base_m=[.1,0,.5],outward_normal_base=[0,0,-1])
                c.planned({'id':task['id'],'epoch':0,'plan':plan});frame(True)
                self.assertEqual(c.state,'APPROACH');self.assertTrue(c.tracking)
                c.estimator.reset();c.observation=None
                frame(False)  # 再次丢洞：非阻塞保持结束后才能启动回收。
                self.assertTrue(c.tracking);self.assertEqual(c.state,'RECOVER')
            # 回零按键也需先完成正在进行的保持，不能同一 tick 又启动轨迹。
            c.command('r',0)
            self.assertEqual(c.state,'HOME_WAIT')
            for _ in range(50):
                frame(True)
                if c.state=='HOME':break
            self.assertEqual(c.state,'HOME');self.assertFalse(runtime.hold_pending)
            c.cancel()
            for _ in range(30):
                runtime.feedback()
                if not runtime.hold_pending:break


class TerminalArrivalTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm

    def test_terminal_speed_cap_can_close_measured_lag_before_timeout(self):
        from armhub.motion import JointTransition
        from test_panthera_cartesian import FakeKinematics
        clock=virtual_time(self);wall=clock.time();origin=clock.monotonic()
        clock.time=lambda:wall+clock.monotonic()-origin
        arm,driver=self.make_arm();arm.config.update(feedback_query_period_s=.025,arrival_dwell_s=.3)
        original_query=driver.send_get_motor_state_cmd
        def timestamped_feedback():
            original_query()
            for state in driver.states:state.time=clock.time()
        driver.send_get_motor_state_cmd=timestamped_feedback
        arm._kinematics_cache=FakeKinematics();arm.enable()
        curve=JointTransition((.1,)*6,(.3,.1,.1,.1,.1,.1),arm.config,speed_limit=.1,minimum_duration=3.)
        arm.start_reference_trajectory(curve)
        session=arm._reference_execution
        while not session.final_sent:
            clock.sleep(max(0,session.next_due-clock.monotonic()));arm.advance_reference_trajectory()
        # 日志中的最终参考已结束，但实测 J2 仍滞后约 0.04 rad。
        driver.states[0].position=session.target[0]-.04
        cap=driver.transmissions[-1][0][1]
        original=driver.send_get_motor_state_cmd
        previous=[clock.monotonic()]
        def physical_feedback():
            dt=clock.monotonic()-previous[0];previous[0]=clock.monotonic()
            driver.states[0].position=min(session.target[0],driver.states[0].position+cap*dt)
            original()
            for state in driver.states:state.time=clock.time()
        driver.send_get_motor_state_cmd=physical_feedback
        result=None
        for _ in range(180):
            clock.sleep(.025);result=arm.advance_reference_trajectory()
            if result is not None:break
        self.assertIsNotNone(result)
        self.assertFalse(arm._moving)

if __name__=='__main__':unittest.main()
