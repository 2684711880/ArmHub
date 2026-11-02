"""回放实测日志中的：参考结束、仍等待到位，此时目标更新不能被当作断流。"""
import unittest
from motion_fixture import virtual_time
from test_panthera_cartesian import FakeKinematics
import test_panthera_hardware as fixtures
from armhub.motion import JointTransition


class TrackingReferenceHandoffTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm
    def test_target_update_during_final_target_hold_is_not_stream_timeout(self):
        clock=virtual_time(self);arm,driver=self.make_arm()
        arm.config['arrival_dwell_s']=.3
        arm._kinematics_cache=FakeKinematics();arm.enable()
        first=JointTransition((.1,)*6,(.12,.1,.1,.1,.1,.1),arm.config,speed_limit=.1,minimum_duration=2.)
        arm.start_reference_trajectory(first)
        old=arm._reference_execution
        while not old.final_sent:
            clock.sleep(max(0,old.next_due-clock.monotonic()))
            arm.advance_reference_trajectory()
        self.assertTrue(arm._moving)
        for _ in range(10):
            clock.sleep(.025);arm.get_joint_feedback()
        self.assertGreater(clock.monotonic()-old.last_send,arm.config['motion_smoothing']['max_cycle_gap_s'])
        second=JointTransition((.12,.1,.1,.1,.1,.1),(.14,.1,.1,.1,.1,.1),arm.config,speed_limit=.1,minimum_duration=2.)
        arm.start_reference_trajectory(second,replace=True)
        self.assertIsNot(arm._reference_execution,old)
        count=len(driver.transmissions);arm.advance_reference_trajectory()
        self.assertEqual(len(driver.transmissions),count+1)
        self.assertTrue(arm._moving)



class CachedFeedbackTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm
    def test_latest_tick_feedback_is_used_without_second_serial_query(self):
        clock=virtual_time(self);arm,driver=self.make_arm();arm._kinematics_cache=FakeKinematics();arm.enable()
        curve=JointTransition((.1,)*6,(.12,.1,.1,.1,.1,.1),arm.config,speed_limit=.1,minimum_duration=2.)
        arm.start_reference_trajectory(curve);arm.advance_reference_trajectory()
        clock.sleep(.05);feedback=arm.get_joint_feedback()
        counters=[s.num for s in driver.states]
        arm.advance_reference_trajectory(feedback=feedback)
        self.assertEqual([s.num for s in driver.states],counters)
        self.assertEqual(len(driver.transmissions),2)
    def test_feedback_fault_in_current_tick_stops_before_new_reference(self):
        from dataclasses import replace
        clock=virtual_time(self);arm,driver=self.make_arm();arm._kinematics_cache=FakeKinematics();arm.enable()
        curve=JointTransition((.1,)*6,(.12,.1,.1,.1,.1,.1),arm.config,speed_limit=.1,minimum_duration=2.)
        arm.start_reference_trajectory(curve);arm.advance_reference_trajectory()
        clock.sleep(.05);feedback=list(arm.get_joint_feedback());feedback[0]=replace(feedback[0],fault=1)
        with self.assertRaises(RuntimeError):arm.advance_reference_trajectory(feedback=feedback)
        self.assertEqual(len(driver.transmissions),2)  # 首参考 + 基于真实新反馈的保持
        self.assertFalse(arm._moving)

if __name__=='__main__':unittest.main()
