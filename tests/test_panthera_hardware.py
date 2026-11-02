import copy
import math
import time
from types import SimpleNamespace
import unittest

from armhub.adapters.panthera_ht import PantheraHardwareBackend
from armhub.profiles import load_profile
from armhub.types import UnsupportedCapability


class FakeMotor:
    def __init__(self, driver, index):
        self.driver, self.index = driver, index

    def get_motor_id(self):
        return self.index + 1

    def get_current_motor_state(self):
        return copy.copy(self.driver.states[self.index])

    def pos_vel_MAXtqe(self, position, velocity, torque):
        self.driver.staged[self.index] = (position, velocity, torque)

    def pos_vel_tqe_kp_kd(self, position, velocity, torque, kp, kd):
        self.driver.staged[self.index] = (position, velocity, torque, kp, kd)


class FakeDriver:
    def __init__(self):
        self.states = [SimpleNamespace(ID=i+1, num=1, time=time.time(), position=0.1 if i < 6 else 0.5,
                                      velocity=0.0, torque=0.0, fault=0, mode=10) for i in range(7)]
        self.motors = [FakeMotor(self, i) for i in range(7)]
        self.staged = {}
        self.transmissions = []
        self.increment = True

    def send_get_motor_state_cmd(self):
        for state in self.states:
            state.time = time.time()
            if self.increment:
                state.num += 1

    def motor_send_cmd(self):
        self.transmissions.append(dict(self.staged))
        for index, values in self.staged.items():
            if len(values) == 3:
                self.states[index].position = values[0]
                self.states[index].velocity = 0.0
        self.staged = {}


class HardwareTests(unittest.TestCase):
    def make_arm(self):
        arm = PantheraHardwareBackend(load_profile('panthera_ht'))
        arm.config.update({'velocity_limits_rad_s':[.1]*6, 'feedback_query_period_s': 0.0001, 'feedback_acquire_timeout_s': 0.01,
                           'stable_window_s': 0.001, 'arrival_dwell_s': 0.001})
        driver = FakeDriver()
        arm._driver, arm._motors = driver, driver.motors
        self.addCleanup(arm.close)
        return arm, driver

    def test_read_and_enable_never_send_motion(self):
        arm, driver = self.make_arm()
        state = arm.get_state()
        self.assertEqual(state.source, 'hardware')
        self.assertFalse(state.enabled)
        arm.enable()
        self.assertEqual(driver.transmissions, [])
        arm.close()
        self.assertEqual(driver.transmissions, [])

    def test_freshness_requires_new_frames_not_fresh_cache_time(self):
        arm, driver = self.make_arm()
        driver.increment = False
        with self.assertRaisesRegex(RuntimeError, '新反馈'):
            arm.get_state()
        self.assertEqual(driver.transmissions, [])

    def test_fault_and_motor_identity_are_rejected(self):
        arm, driver = self.make_arm()
        driver.states[2].fault = 1
        with self.assertRaises(RuntimeError):
            arm.enable()
        driver.states[2].fault = 0
        driver.states[2].ID = 4
        with self.assertRaises(RuntimeError):
            arm.get_state()
        self.assertEqual(driver.transmissions, [])

    def test_large_multijoint_and_nonfinite_goals_never_transmit(self):
        arm, driver = self.make_arm()
        arm.enable()
        for target in [(0.1,)*5, (math.nan,)*6,
                       (0.1,0.1,0.1,0.1,0.1,0.3),
                       (0.14,0.14,0.1,0.1,0.1,0.1),
                       (0.1,0.1,0.1,0.1,0.1,2.6)]:
            with self.subTest(target=target):
                with self.assertRaises(ValueError):
                    arm.move_joints(target, duration_s=2.0)
        self.assertEqual(driver.transmissions, [])

    def test_motion_requires_explicit_enable(self):
        arm, driver = self.make_arm()
        with self.assertRaises(RuntimeError):
            arm.move_joints((0.1,)*6, duration_s=2.0)
        self.assertEqual(driver.transmissions, [])

    def test_bounded_jog_fills_all_seven_slots_and_keeps_gripper(self):
        arm, driver = self.make_arm()
        arm.enable()
        result = arm.move_joints((0.1,0.1,0.1,0.1,0.1,0.14), duration_s=2.0)
        self.assertEqual(result.status, 'succeeded')
        packet = driver.transmissions[0]
        self.assertEqual(set(packet), set(range(7)))
        self.assertEqual([packet[index][0] for index in range(5)], [0.1]*5)
        self.assertEqual(packet[5][0], 0.14)
        self.assertEqual(packet[6][0], 0.5)
        self.assertTrue(all(0 < values[1] <= 0.1 for values in packet.values()))
        arm.stop()
        self.assertFalse(arm._moving)

    def test_unverified_functions_remain_unavailable(self):
        arm, _ = self.make_arm()
        with self.assertRaises(UnsupportedCapability):
            arm.execute_trajectory([])
        with self.assertRaises(RuntimeError):
            arm.move_to((0.25,0,0.3))

    def test_encoder_zero_offset_is_held_without_expanding_motion_limits(self):
        arm, driver = self.make_arm()
        driver.states[2].position = -0.0025
        arm.enable()
        arm.move_joints((0.1,0.1,-0.0025,0.1,0.1,0.14), duration_s=2)
        self.assertEqual(driver.transmissions[0][2][0], -0.0025)
        count = len(driver.transmissions)
        with self.assertRaises(ValueError):
            arm.move_joints((0.1,0.1,-0.008,0.1,0.1,0.14), duration_s=2)
        self.assertEqual(len(driver.transmissions), count)


if __name__ == '__main__':
    unittest.main()
