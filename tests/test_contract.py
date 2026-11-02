import math
import unittest

from armhub import Pose, TrajectoryPoint, UnsupportedCapability, create_arm, list_robots


class ContractTests(unittest.TestCase):
    def test_same_task_runs_for_both_joint_counts(self):
        from examples.joint_motion import run
        for robot, count in [("panthera_ht", 6), ("nero7", 7)]:
            with self.subTest(robot=robot):
                report = run(robot)
                self.assertEqual(len(report["after"]["positions"]), count)
                self.assertEqual(report["after"]["positions"][-1], 0.05)
                self.assertEqual(report["result"]["source"], "mock")
                self.assertEqual(report["result"]["status"], "succeeded")

    def test_motion_requires_connection_then_explicit_enable(self):
        arm = create_arm("panthera_ht")
        with self.assertRaises(RuntimeError):
            arm.move_joints([0] * 6, duration_s=1)
        arm.connect()
        self.assertFalse(arm.get_state().enabled)
        with self.assertRaises(RuntimeError):
            arm.move_joints([0] * 6, duration_s=1)
        arm.enable()
        arm.close()
        arm.connect()
        self.assertFalse(arm.get_state().enabled)

    def test_bad_trajectory_does_not_partially_change_state(self):
        with create_arm("panthera_ht") as arm:
            arm.enable()
            first = TrajectoryPoint((0.05,) * 6, 1)
            bad_points = [
                TrajectoryPoint((0.1,) * 5, 2),
                TrajectoryPoint((math.nan,) * 6, 2),
                TrajectoryPoint((math.inf,) * 6, 2),
                TrajectoryPoint((0.3,) * 6, 2),
                TrajectoryPoint((0.1,) * 6, 1),
                TrajectoryPoint((0.1,) * 6, math.inf),
                TrajectoryPoint((0.1,) * 6, math.nan),
                TrajectoryPoint((0.1,) * 6, -1),
            ]
            for bad in bad_points:
                with self.subTest(point=bad):
                    with self.assertRaises(ValueError):
                        arm.execute_trajectory([first, bad])
                    self.assertEqual(arm.get_state().positions, (0.0,) * 6)
            with self.assertRaises(ValueError):
                arm.execute_trajectory([])

    def test_valid_multisegment_trajectory(self):
        with create_arm("nero7") as arm:
            arm.enable()
            result = arm.execute_trajectory([
                TrajectoryPoint((0.05,) * 7, 1),
                TrajectoryPoint((0.1,) * 7, 2),
            ])
            self.assertEqual(result.final_positions, (0.1,) * 7)

    def test_linear_motion_is_not_silently_replaced(self):
        with create_arm("panthera_ht") as arm:
            arm.enable()
            pose = Pose((0.1, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0), "base_link", "tool0")
            with self.assertRaises(UnsupportedCapability):
                arm.move_linear(pose, speed_m_s=0.05)
            with self.assertRaises(UnsupportedCapability):
                arm.set_teach_mode(True)
            self.assertEqual(arm.get_state().positions, (0.0,) * 6)

    def test_pending_backends_fail_instead_of_fake_success(self):
        for robot in ['nero7']:
            for backend in ["hardware"]:
                with self.subTest(robot=robot, backend=backend):
                    arm = create_arm(robot, backend)
                    self.assertEqual(arm.capabilities, frozenset())
                    with self.assertRaises(UnsupportedCapability):
                        arm.connect()
                    with self.assertRaises(UnsupportedCapability):
                        arm.move_joints([0] * len(arm.profile.joint_names), duration_s=1)

    def test_unknown_selection_is_rejected(self):
        for robot, backend in [("unknown", "mock"), ("../nero7", "mock"),
                               ("panthera_ht", "typo")]:
            with self.subTest(robot=robot, backend=backend):
                with self.assertRaises(ValueError):
                    create_arm(robot, backend)


if __name__ == "__main__":
    unittest.main()
