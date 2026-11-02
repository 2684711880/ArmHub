import math
import unittest

from motion_fixture import virtual_time

from test_panthera_cartesian import FakeKinematics
import test_panthera_hardware as hardware_fixture


class HomeTests(unittest.TestCase):
    make_arm = hardware_fixture.HardwareTests.make_arm

    def prepared(self):
        virtual_time(self)
        arm, driver = self.make_arm()
        arm._kinematics_cache = FakeKinematics()
        return arm, driver

    def test_preview_uses_configured_joint_home_and_never_sends(self):
        arm, driver = self.prepared()
        plan = arm.preview_home()
        self.assertEqual(plan['target_joints_rad'], (0.,)*6)
        self.assertEqual(plan['start_joints_rad'], (.1,)*6)
        self.assertEqual(plan['gripper_hold_raw_rad'], .5)
        self.assertFalse(plan['motion_sent'])
        self.assertEqual(driver.transmissions, [])
        self.assertFalse(arm.get_state().enabled)

    def test_home_returns_all_axes_to_zero_and_keeps_gripper(self):
        arm, driver = self.prepared()
        arm.enable()
        result = arm.home()
        self.assertEqual(result.status, 'succeeded')
        self.assertEqual(result.final_positions, (0.,)*6)
        self.assertEqual(set(driver.transmissions[0]), set(range(7)))
        self.assertEqual(driver.transmissions[0][6][0], .5)
        self.assertGreater(len(driver.transmissions), 2)

    def test_long_return_is_timed_from_measured_start(self):
        arm, driver = self.prepared()
        driver.states[1].position = 3.1
        plan = arm.preview_home()
        self.assertGreaterEqual(plan['duration_s'], 31.)
        self.assertEqual(driver.transmissions, [])

    def test_home_requires_enable(self):
        arm, driver = self.prepared()
        with self.assertRaisesRegex(RuntimeError, 'enable'):
            arm.home()
        self.assertEqual(driver.transmissions, [])

    def test_invalid_home_and_speed_rejected_without_sending(self):
        arm, driver = self.prepared()
        for scale in [0, math.nan, 1.1]:
            with self.assertRaises(ValueError):
                arm.preview_home(velocity_scale=scale)
        arm.config['home_joints_rad'] = [99.]*6
        with self.assertRaises(ValueError):
            arm.preview_home()
        self.assertEqual(driver.transmissions, [])

    def test_collision_prevents_home(self):
        arm, driver = self.prepared()
        def collision(*args):
            raise ValueError('collision')
        arm._kinematics_cache.check_trajectory = collision
        arm.enable()
        with self.assertRaisesRegex(ValueError, 'collision'):
            arm.home()
        self.assertEqual(driver.transmissions, [])

    def test_gravity_blocks_home(self):
        arm, driver = self.prepared()
        arm._gravity_active = True
        with self.assertRaises(RuntimeError):
            arm.preview_home()
        arm._gravity_active = False
        self.assertEqual(driver.transmissions, [])


if __name__ == '__main__':
    unittest.main()
