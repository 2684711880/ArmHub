"""丢洞回收及重新捕获；覆盖实际后端静止预检使恢复时间轴过期的问题。"""
import unittest
from unittest.mock import patch
from armhub.motion import JointTransition
from motion_fixture import virtual_time
from test_panthera_cartesian import FakeKinematics
import test_panthera_hardware as fixtures
import test_hole_controller as workflow


class RecoveryBackendTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm
    def test_rest_recovery_starts_after_stationary_preflight(self):
        clock=virtual_time(self);arm,driver=self.make_arm();arm._kinematics_cache=FakeKinematics();arm.enable()
        curve=JointTransition((.1,)*6,(0.,)*6,arm.config,speed_limit=.1,minimum_duration=3.)
        started=clock.monotonic()
        original=arm._stable_feedback
        def delayed():
            clock.sleep(.3)
            return original()
        with patch.object(arm,'_stable_feedback',side_effect=delayed):
            actual=arm.start_reference_trajectory(curve,replace=True,reference_started_at=started)
        self.assertGreater(actual-started,.2)
        arm.advance_reference_trajectory()
        self.assertEqual(len(driver.transmissions),1)
        self.assertTrue(arm._moving)

    def test_expired_moving_handoff_still_rejected(self):
        clock=virtual_time(self);arm,driver=self.make_arm();arm._kinematics_cache=FakeKinematics();arm.enable()
        curve=JointTransition((.1,)*6,(.12,.1,.1,.1,.1,.1),arm.config,speed_limit=.1,minimum_duration=3.)
        arm.start_reference_trajectory(curve);arm.advance_reference_trajectory()
        started=clock.monotonic();clock.sleep(.3)
        with self.assertRaisesRegex(RuntimeError,'超时|过期'):
            arm.start_reference_trajectory(curve,replace=True,reference_started_at=started)
        self.assertFalse(arm._moving)


class RecoveryWorkflowTests(unittest.TestCase):
    setUp=workflow.ControllerTests.setUp
    key=workflow.ControllerTests.key
    observe=workflow.ControllerTests.observe
    plan=workflow.ControllerTests.plan

    def lost(self):
        self.clock.now+=.1;self.c.feedback()
        self.c.observe({'stamp':self.clock.now,'observation':None})
        self.c.tick(self.robot.positions())

    def test_lost_retracts_and_visible_stops_then_confirms(self):
        self.key('t');self.lost()
        self.assertEqual(self.c.state,'RECOVER');self.assertTrue(self.c.tracking)
        self.assertEqual(len(self.robot.starts),1)
        self.clock.now+=.1;self.lost()
        self.assertEqual(len(self.robot.starts),1)
        self.observe()
        self.assertEqual(self.c.state,'REACQUIRE');self.assertIsNone(self.c.active)
        self.assertGreater(self.robot.holds,0)
        self.c.tick(self.robot.positions());self.assertEqual(len(self.tasks),0)
        self.observe();self.observe();self.c.tick(self.robot.positions())
        self.assertEqual(len(self.tasks),1);self.assertTrue(self.c.tracking)

    def test_feed_finishes_before_lost_hole_recovery(self):
        from armhub.trajectory import ContinuousFeed
        self.c.report=self.plan();self.c.tracking=True
        self.c.send(self.c.report['stage2_waypoints'][-1],'FEED_CONTINUOUS',
                    ContinuousFeed(self.c.report['feed_trajectory']))
        self.lost()
        self.assertEqual(self.c.state,'FEED_CONTINUOUS')
        self.assertEqual(len(self.robot.starts),1)
        self.c.finish_or_advance(self.clock.now)
        self.lost()
        self.assertEqual(self.c.state,'RECOVER')
        self.assertEqual(len(self.robot.starts),2)

    def test_stale_camera_holds_instead_of_blind_recovery(self):
        self.key('t');self.c.frame_stamp=self.clock.now-2;self.c.vision='lost'
        self.c.tick(self.robot.positions())
        self.assertEqual(self.c.state,'WAIT_CAMERA');self.assertEqual(len(self.robot.starts),0)

if __name__=='__main__':unittest.main()
