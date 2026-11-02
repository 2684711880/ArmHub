import unittest
from motion_fixture import virtual_time
import test_panthera_hardware as fixtures
from test_panthera_cartesian import FakeKinematics
from armhub.motion import JointTransition


class TaskExecutionTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm
    def ready(self):
        clock=virtual_time(self);arm,driver=self.make_arm();arm._kinematics_cache=FakeKinematics()
        arm.enable()
        curve=JointTransition((.1,)*6,(.12,.1,.1,.1,.1,.1),arm.config,speed_limit=.1,minimum_duration=2.)
        return arm,driver,clock,curve
    def test_stale_generation_prevents_first_reference(self):
        arm,driver,clock,curve=self.ready()
        with self.assertRaisesRegex(RuntimeError,'替代'):
            arm.start_reference_trajectory(curve,command_guard=lambda:False)
        self.assertEqual(driver.transmissions,[])
    def test_cancel_between_ticks_holds_and_stops_old_stream(self):
        arm,driver,clock,curve=self.ready();current=[True]
        arm.start_reference_trajectory(curve,command_guard=lambda:current[0])
        arm.advance_reference_trajectory();current[0]=False
        with self.assertRaisesRegex(RuntimeError,'替代'):
            arm.advance_reference_trajectory()
        self.assertEqual(len(driver.transmissions),2)
        self.assertFalse(arm._moving)
    def test_replanning_cannot_hide_a_long_reference_gap(self):
        arm,driver,clock,curve=self.ready()
        arm.start_reference_trajectory(curve)
        arm.advance_reference_trajectory()
        clock.now+=.25
        with self.assertRaisesRegex(RuntimeError,'替换检查超时'):
            arm.start_reference_trajectory(curve,replace=True)
        self.assertFalse(arm._moving)

    def test_ctrl_c_between_blocking_reference_ticks_holds(self):
        arm,driver,clock,curve=self.ready()
        original_sleep=clock.sleep;interrupted=[False]
        def interrupt_sleep(seconds):
            if len(driver.transmissions)==1 and not interrupted[0]:
                interrupted[0]=True
                raise KeyboardInterrupt()
            original_sleep(seconds)
        clock.sleep=interrupt_sleep
        with self.assertRaises(KeyboardInterrupt):arm.home()
        self.assertFalse(arm._moving)
        self.assertEqual(len(driver.transmissions),2)

    def test_telemetry_is_detached_from_native_snapshot(self):
        arm,driver,clock,curve=self.ready();values=arm.get_joint_feedback()
        driver.states[0].position=.2
        self.assertEqual(values[0].position,.1)
    def test_gripper_fills_seven_slots_and_holds_arm(self):
        arm,driver,clock,curve=self.ready()
        result=arm.command_gripper(1.6,speed=.3,torque=.5)
        self.assertEqual(result['status'],'submitted')
        self.assertEqual([driver.transmissions[0][i][0] for i in range(6)],[.1]*6)
        self.assertEqual(driver.transmissions[0][6],(1.6,.3,.5))
        for _ in range(20):
            if arm.poll_gripper():break
        self.assertFalse(arm._moving)
    def test_gripper_rejects_superseded_command_without_packet(self):
        arm,driver,clock,curve=self.ready()
        with self.assertRaisesRegex(RuntimeError,'替代'):
            arm.command_gripper(1.,command_guard=lambda:False)
        self.assertEqual(driver.transmissions,[])
    def test_active_gripper_excludes_arm_reference(self):
        arm,driver,clock,curve=self.ready();arm.command_gripper(1.6)
        with self.assertRaisesRegex(RuntimeError,'夹爪'):
            arm.start_reference_trajectory(curve,replace=True)
        self.assertEqual(len(driver.transmissions),1)
        arm.stop()

if __name__=='__main__':unittest.main()
