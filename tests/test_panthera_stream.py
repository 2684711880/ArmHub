import math
import unittest

from armhub.motion import JointTransition
from motion_fixture import virtual_time
from test_panthera_cartesian import FakeKinematics
import test_panthera_hardware as hardware_fixture


class StreamTests(unittest.TestCase):
    make_arm = hardware_fixture.HardwareTests.make_arm

    def prepared(self):
        clock = virtual_time(self)
        arm, driver = self.make_arm()
        arm._kinematics_cache = FakeKinematics()
        events = []
        arm._record = lambda event, **data: events.append(dict(event=event, **data))
        arm.enable()
        return arm, driver, clock, events

    def test_real_ruckig_stream_is_timed_smooth_and_ends_exactly(self):
        arm, driver, clock, events = self.prepared()
        result = arm.home()
        refs = [e for e in events if e['event']=='stream_reference']
        self.assertEqual(refs[0]['position_rad'], (.1,)*6)
        self.assertEqual(refs[0]['velocity_reference_rad_s'], (0.,)*6)
        self.assertEqual(refs[-1]['position_rad'], (0.,)*6)
        self.assertEqual(refs[-1]['velocity_reference_rad_s'], (0.,)*6)
        self.assertEqual(result.trajectory_points,len(refs))
        for previous,current in zip(refs,refs[1:]):
            dt=current['elapsed_s']-previous['elapsed_s']
            self.assertGreaterEqual(dt, .049)
            self.assertLessEqual(dt, .051)
            self.assertLess(max(abs(a-b) for a,b in zip(previous['position_rad'],current['position_rad'])), .006)
        self.assertTrue(all(packet[6][0]==.5 for packet in driver.transmissions))
        self.assertTrue(all(0 < value[1] <= .1 for packet in driver.transmissions for value in packet.values()))

    def test_dense_trajectory_satisfies_v_a_jerk_and_bounds(self):
        arm, driver, clock, events = self.prepared()
        goal=(.8,1.2,.4,-.2,-.4,.2)
        curve=JointTransition((.1,)*6,goal,arm.config,speed_limit=.1,minimum_duration=2)
        samples=[curve.sample(curve.duration*i/2000) for i in range(2001)]
        dt=curve.duration/2000
        for i,(q,v,a) in enumerate(samples):
            self.assertLessEqual(max(abs(x) for x in v), .10000001)
            self.assertLessEqual(max(abs(x) for x in a), .80000001)
            self.assertTrue(all(min(.1,g)-1e-9<=x<=max(.1,g)+1e-9 for x,g in zip(q,goal)))
            if i:
                self.assertLessEqual(max(abs(x-y)/dt for x,y in zip(a,samples[i-1][2])),4.00001)
        self.assertEqual(samples[-1][0],goal)
        self.assertEqual(driver.transmissions, [])

    def test_late_cycle_holds_instead_of_catching_up_in_burst(self):
        arm, driver, clock, events = self.prepared()
        original=driver.motor_send_cmd
        def delayed_send():
            original()
            if len(driver.transmissions)==1:
                clock.now += .25
        driver.motor_send_cmd=delayed_send
        with self.assertRaisesRegex(RuntimeError,'超时'):
            arm.home()
        refs=[e for e in events if e['event']=='stream_reference']
        self.assertEqual(len(refs),0)
        self.assertEqual(len(driver.transmissions),2)  # 首帧 + 实测位置保持
        self.assertFalse(arm._moving)

    def test_feedback_cycle_gap_holds_before_sending_next_reference(self):
        arm, driver, clock, events = self.prepared()
        original=arm.read_feedback
        def delayed_feedback(**kwargs):
            if len(driver.transmissions)==1:
                clock.now += .25
            return original(**kwargs)
        arm.read_feedback=delayed_feedback
        with self.assertRaisesRegex(RuntimeError,'间隔超时'):
            arm.home()
        self.assertEqual(sum(e['event']=='stream_reference' for e in events),1)
        self.assertEqual(len(driver.transmissions),2)
        self.assertFalse(arm._moving)

    def test_nontracking_driver_fails_then_holds(self):
        arm, driver, clock, events = self.prepared()
        def no_motion():
            driver.transmissions.append(dict(driver.staged))
            driver.staged={}
        driver.motor_send_cmd=no_motion
        with self.assertRaisesRegex(RuntimeError,'跟踪误差'):
            arm.move_to((.8,.1,.1))
        self.assertEqual([driver.transmissions[-1][i][0] for i in range(6)],[.1]*6)
        self.assertFalse(arm._moving)

    def test_feedback_stall_does_not_send_future_references(self):
        arm, driver, clock, events = self.prepared()
        original=driver.motor_send_cmd
        def stall():
            original()
            if len(driver.transmissions)==1:
                driver.increment=False
        driver.motor_send_cmd=stall
        with self.assertRaisesRegex(RuntimeError,'新反馈'):
            arm.home()
        self.assertEqual(len(driver.transmissions),1)
        self.assertTrue(any(e['event']=='stop_unconfirmed' for e in events))
        driver.increment=True
        arm.stop()

    def test_ctrl_c_switches_to_measured_position_hold(self):
        arm, driver, clock, events = self.prepared()
        original=driver.motor_send_cmd
        def interrupt():
            original()
            if len(driver.transmissions)==3:
                raise KeyboardInterrupt()
        driver.motor_send_cmd=interrupt
        with self.assertRaises(KeyboardInterrupt):
            arm.home()
        self.assertFalse(arm._moving)
        self.assertEqual(len(driver.transmissions),4)
        self.assertEqual([driver.transmissions[-1][i][0] for i in range(6)],
                         [driver.states[i].position for i in range(6)])

    def test_trajectory_rejects_physical_limit_violation(self):
        arm, driver, clock, events = self.prepared()
        with self.assertRaises(ValueError):
            JointTransition((.1,)*6,(3.,)*6,arm.config,speed_limit=.1,minimum_duration=2)
        self.assertEqual(driver.transmissions,[])


if __name__ == '__main__':
    unittest.main()
