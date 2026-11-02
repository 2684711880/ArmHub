import math
import time
import unittest

from armhub import create_arm
from armhub.types import UnsupportedCapability
import test_panthera_hardware as hardware_fixture


class GravityControlTests(unittest.TestCase):
    make_arm = hardware_fixture.HardwareTests.make_arm

    def prepare(self, model=lambda q: (0.0, 1.0, 4.0, 0.9, 0.0, 0.0)):
        arm, driver = self.make_arm()
        arm.enable()
        arm._gravity_model = lambda: model
        return arm, driver

    def test_torque_frames_only_cover_six_axes_then_exit_to_full_hold(self):
        arm, driver = self.prepare(lambda q: (999.0, -999.0, 4.0, 0.0, 0.0, 999.0))
        result = arm.run_gravity_compensation(duration_s=0.012)
        self.assertTrue(result.hold_stop_confirmed)
        self.assertGreater(result.clipped_cycles, 0)
        gravity = [packet for packet in driver.transmissions if len(packet[0]) == 5]
        self.assertGreater(len(gravity), 0)
        for packet in gravity:
            self.assertEqual(set(packet), set(range(6)))
            self.assertEqual(packet[0], (0.0,0.0,15.0,0.0,0.0))
            self.assertEqual(packet[1][2], -30.0)
            self.assertEqual(packet[5][2], 3.0)
        self.assertEqual(set(driver.transmissions[-1]), set(range(7)))
        self.assertTrue(all(len(command) == 3 for command in driver.transmissions[-1].values()))
        self.assertFalse(arm._gravity_active)
        self.assertFalse(arm._moving)

    def test_invalid_model_output_does_not_enter_torque_mode(self):
        arm, driver = self.prepare(lambda q: (math.nan,)*6)
        with self.assertRaisesRegex(RuntimeError, '无效力矩'):
            arm.run_gravity_compensation(duration_s=0.01)
        self.assertEqual(driver.transmissions, [])

    def test_interrupt_after_output_is_followed_by_hold(self):
        calls = [0]
        def model(q):
            calls[0] += 1
            if calls[0] > 1:
                raise KeyboardInterrupt()
            return (0.0,1.0,4.0,0.0,0.0,0.0)
        arm, driver = self.prepare(model)
        result = arm.run_gravity_compensation()
        self.assertEqual(result.exit_reason, 'user_interrupt')
        self.assertTrue(result.hold_stop_confirmed)
        self.assertEqual(len(driver.transmissions[-1]), 7)

    def test_slow_cycle_is_rejected_then_held(self):
        calls = [0]
        def model(q):
            calls[0] += 1
            if calls[0] > 1:
                time.sleep(0.02)
            return (0.0,)*6
        arm, driver = self.prepare(model)
        arm.config['gravity_compensation']['max_command_gap_s'] = 0.01
        with self.assertRaisesRegex(RuntimeError, '循环超时'):
            arm.run_gravity_compensation(duration_s=0.1)
        self.assertEqual(sum(len(packet[0]) == 5 for packet in driver.transmissions), 1)
        self.assertEqual(len(driver.transmissions[-1]), 7)

    def test_missing_feedback_never_claims_hold_stop_confirmed(self):
        arm, driver = self.prepare()
        original = arm._send_gravity
        def send_then_lose_feedback(values):
            original(values)
            driver.increment = False
        arm._send_gravity = send_then_lose_feedback
        try:
            with self.assertRaisesRegex(RuntimeError, '新反馈'):
                arm.run_gravity_compensation(duration_s=0.1)
            self.assertTrue(arm._moving)
            self.assertFalse(arm._gravity_active)
        finally:
            driver.increment = True
            arm.stop()

    def test_position_control_cannot_enter_during_gravity_mode(self):
        arm, driver = self.prepare()
        arm._gravity_active = True
        with self.assertRaisesRegex(RuntimeError, '占用执行通道'):
            arm.move_joints((0.1,)*6, duration_s=2)
        self.assertEqual(driver.transmissions, [])
        arm._gravity_active = False

    def test_unknown_robot_gravity_is_explicitly_unsupported(self):
        arm = create_arm('nero7','hardware')
        self.assertNotIn('gravity_compensation', arm.capabilities)
        with self.assertRaises(UnsupportedCapability):
            arm.run_gravity_compensation()

    def test_speed_guard_records_exact_joint_and_sample_before_hold(self):
        arm, driver = self.prepare()
        limit = arm.config['gravity_compensation']['max_velocity_rad_s']
        overspeed = limit + 0.2
        events = []
        arm._record = lambda event, **data: events.append((event, data))
        original = arm._send_gravity
        def send_then_report_speed(values):
            original(values)
            driver.states[3].velocity = overspeed
        arm._send_gravity = send_then_report_speed
        with self.assertRaisesRegex(RuntimeError, f'joint4={overspeed:.4f} rad/s'):
            arm.run_gravity_compensation(duration_s=0.1)
        guards = [data for event, data in events if event == 'gravity_speed_guard']
        self.assertEqual(len(guards), 1)
        self.assertEqual(guards[0]['limit_rad_s'], limit)
        self.assertEqual(guards[0]['offending_joints'][0]['joint'], 'joint4')
        self.assertEqual(guards[0]['offending_joints'][0]['velocity_rad_s'], overspeed)
        self.assertGreaterEqual(len(guards[0]['recent_feedback']), 2)
        self.assertEqual(len(driver.transmissions[-1]), 7)
        self.assertFalse(arm._moving)


if __name__ == '__main__':
    unittest.main()
