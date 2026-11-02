"""实物执行通道回归；驱动与运动学均为替身，不接触硬件。"""
import math
import unittest

from motion_fixture import virtual_time

import test_panthera_hardware as hardware_fixture
from armhub.adapters.panthera_kinematics import PantheraKinematics


class FakeKinematics:
    vector = staticmethod(PantheraKinematics.vector)
    orientation_error = staticmethod(PantheraKinematics.orientation_error)

    def fk(self, q):
        return {'frame_id':'base_link', 'tcp_frame':'gripper_center',
                'position_m': tuple(q[:3]), 'quaternion_xyzw': (0,0,0,1)}

    def solve(self, q, target, quaternion):
        return {'start_pose':self.fk(q), 'target_position_m':tuple(target),
                'target_quaternion_xyzw':(0,0,0,1), 'target_joints_rad':tuple(target)+tuple(q[3:]),
                'orientation_policy':'hold_current'}

    def check_trajectory(self, trajectory, step):
        return {'samples':2, 'scope':'fake_only'}


class CartesianTests(unittest.TestCase):
    make_arm = hardware_fixture.HardwareTests.make_arm

    def prepared(self):
        virtual_time(self)
        arm, driver = self.make_arm()
        arm._kinematics_cache = FakeKinematics()
        return arm, driver

    def test_preview_never_sends_or_enables(self):
        arm, driver = self.prepared()
        plan = arm.preview_move_to((.12,.13,.1))
        self.assertFalse(plan['motion_sent'])
        self.assertFalse(arm.get_state().enabled)
        self.assertEqual(driver.transmissions, [])
        self.assertEqual(plan['start_joints_rad'], (.1,)*6)

    def test_coupled_target_feedback_and_gripper(self):
        arm, driver = self.prepared()
        arm.enable()
        result = arm.move_to((.12,.13,.1))
        self.assertEqual(result.status, 'succeeded')
        self.assertLess(result.position_error_m, 1e-10)
        self.assertEqual(driver.transmissions[-1][0][0], .12)
        self.assertEqual(driver.transmissions[-1][1][0], .13)
        self.assertEqual(driver.transmissions[0][6][0], .5)
        self.assertGreater(len(driver.transmissions), 2)

    def test_invalid_and_large_targets_do_not_send(self):
        arm, driver = self.prepared()
        arm.enable()
        for target in [(math.nan,0,0), (3.0,.1,.1), (.1,.1)]:
            with self.assertRaises(ValueError):
                arm.move_to(target)
        self.assertEqual(driver.transmissions, [])

    def test_preview_accepts_translation_over_previous_15cm_limit(self):
        arm, driver = self.prepared()
        plan = arm.preview_move_to((.2,.2,.2))
        self.assertAlmostEqual(plan['translation_m'], math.sqrt(.03))
        self.assertFalse(plan['motion_sent'])
        self.assertEqual(driver.transmissions, [])

    def test_cartesian_can_execute_over_30_degrees(self):
        arm, driver = self.prepared()
        arm.enable()
        result = arm.move_to((.8,.1,.1))
        self.assertEqual(result.status, 'succeeded')
        self.assertEqual(driver.transmissions[-1][0][0], .8)

    def test_cartesian_duration_can_exceed_30_seconds(self):
        arm, driver = self.prepared()
        plan = arm.preview_move_to((.1,3.2,.1))
        self.assertGreater(plan['duration_s'], 30)
        self.assertEqual(driver.transmissions, [])

    def test_cartesian_orientation_can_exceed_30_degrees(self):
        arm, driver = self.prepared()
        solve = arm._kinematics_cache.solve
        def rotated(q, target, quaternion):
            plan = solve(q, target, quaternion)
            plan['target_quaternion_xyzw'] = (0,0,1,0)
            return plan
        arm._kinematics_cache.solve = rotated
        plan = arm.preview_move_to((.12,.1,.1), quaternion_xyzw=(0,0,1,0))
        self.assertEqual(plan['target_quaternion_xyzw'], (0,0,1,0))
        self.assertEqual(driver.transmissions, [])

    def test_collision_rejection_does_not_send(self):
        arm, driver = self.prepared()
        def collision(*args):
            raise ValueError('collision')
        arm._kinematics_cache.check_trajectory = collision
        arm.enable()
        with self.assertRaisesRegex(ValueError, 'collision'):
            arm.move_to((.12,.1,.1))
        self.assertEqual(driver.transmissions, [])

    def test_changed_start_rejected_before_send(self):
        arm, driver = self.prepared()
        arm.enable()
        with self.assertRaisesRegex(RuntimeError, '起点'):
            arm._execute_joint_target((.12,.1,.1,.1,.1,.1), duration_s=2,
                max_step_rad=.5, single_joint=False, expected_start=(.2,)*6)
        self.assertEqual(driver.transmissions, [])

    def test_gravity_excludes_cartesian_preview(self):
        arm, driver = self.prepared()
        arm._gravity_active = True
        with self.assertRaises(RuntimeError):
            arm.preview_move_to((.12,.1,.1))
        arm._gravity_active = False
        self.assertEqual(driver.transmissions, [])

    def test_fk_failure_to_reach_prevents_false_success(self):
        arm, driver = self.prepared()
        arm.config['motion_timeout_margin_s'] = -.999
        arm.enable()
        with self.assertRaises(TimeoutError):
            arm._execute_joint_target((.1,)*6, duration_s=1, max_step_rad=.5,
                                     single_joint=False, goal_validator=lambda q: False)
        self.assertTrue(driver.transmissions)  # 首次目标与失败后的实测保持
        self.assertFalse(arm._moving)


if __name__ == '__main__':
    unittest.main()
