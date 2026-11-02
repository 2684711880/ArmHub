from contextlib import redirect_stdout
import io
import unittest
import numpy as np

from armhub import create_arm
from apps.movel.moveL_pos_control import PlanningRobot,checked
from apps.movel.circle_trajectory import TimedCircleCurve,plan_circle_curve
from apps.drawing_panel.controller import DrawingSettings,drawing_waypoints,plan_drawing


class CircleTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm=create_arm('panthera_ht','hardware')
        cls.robot=PlanningRobot(cls.arm,[0]*6)
        rotation=np.array([[0,0,1],[0,1,0],[-1,0,0]])
        cls.q=cls.robot.solve([.24,0,.1],rotation,[0]*6)
        pose=cls.robot.forward_kinematics(cls.q)
        cls.origin=dict(position=pose['position'],rotation=pose['rotation'],
                        quaternion_xyzw=cls.robot.kin.fk(cls.q)['quaternion_xyzw'])
        cls.settings=DrawingSettings(shape='circle',speed_mm_s=20)
        cls.w=drawing_waypoints(cls.origin,cls.settings,'fwd')
        with redirect_stdout(io.StringIO()):
            cls.curve=plan_circle_curve(cls.robot,cls.w,cls.q,.04*np.pi/.02)

    def test_4cm_circle_at_20mm_s_takes_under_eight_seconds(self):
        self.assertLess(self.curve.duration,8.)
        self.assertGreaterEqual(self.curve.duration,.04*np.pi/.02)
        checked(self.robot,self.curve,self.q,.05)
        self.assertIsNone(self.arm._driver)

    def test_endpoints_rest_and_continuous_start(self):
        for t in (-1.,0.,self.curve.duration,self.curve.duration+1):
            q,v,a=self.curve.sample(t)
            np.testing.assert_allclose(v,0.,atol=1e-12)
            np.testing.assert_allclose(a,0.,atol=1e-12)
            np.testing.assert_allclose(self.curve.jerk(t),0.,atol=1e-12)
        np.testing.assert_allclose(self.curve.sample(0)[0],self.q,atol=1e-12)
        self.assertLess(np.max(np.abs(self.curve.sample(1e-4)[1])),1e-8)

    def test_independent_dense_derivative_and_position_checks(self):
        c=self.arm.config
        for t in np.linspace(0,self.curve.duration,4001):
            q,v,a=self.curve.sample(t);j=self.curve.jerk(t)
            self.assertTrue(np.all(np.abs(v)<=np.asarray(c['velocity_limits_rad_s'])+1e-7))
            self.assertTrue(np.all(np.abs(a)<=np.asarray(c['motion_smoothing']['acceleration_limits_rad_s2'])+1e-7))
            self.assertTrue(np.all(np.abs(j)<=np.asarray(c['motion_smoothing']['jerk_limits_rad_s3'])+1e-7))
            self.assertTrue(np.all(q>=np.asarray(c['lower_rad'])-1e-9))
            self.assertTrue(np.all(q<=np.asarray(c['upper_rad'])+1e-9))
        centre=np.asarray(self.origin['position'])+[0,.02,0]
        for t in np.linspace(0,self.curve.duration,1501):
            fk=self.robot.forward_kinematics(self.curve.sample(t)[0])
            p=np.asarray(fk['position'])
            self.assertLess(abs(np.linalg.norm((p-centre)[:2])-.02),.00005)
            self.assertLess(abs(p[2]-centre[2]),.00005)

    def test_reverse_uses_same_circle_and_ui_keeps_sdk_movej(self):
        with redirect_stdout(io.StringIO()):
            w,curve,report=plan_drawing(self.robot,self.q,self.origin,self.settings,'rev')
        self.assertIsInstance(curve,TimedCircleCurve)
        self.assertLess(curve.duration,8.)
        np.testing.assert_allclose(np.asarray([x['position'] for x in w]),
                                   np.asarray([x['position'] for x in self.w])[::-1],atol=1e-12)
        self.assertEqual(report['command_mode'],'sdk_movej')
        self.assertEqual(report['timing']['parameterization'],'circle_geometry_plus_C3_progress')
        self.assertIsNone(self.arm._driver)

    def test_high_requested_speed_still_retimes_to_same_jerk_limit(self):
        with redirect_stdout(io.StringIO()):
            fast=plan_circle_curve(self.robot,self.w,self.q,.04*np.pi/.1)
        self.assertLess(fast.duration,self.curve.duration)
        self.assertGreater(fast.duration,.04*np.pi/.1)
        peak=np.max(np.abs([fast.jerk(t) for t in np.linspace(0,fast.duration,8001)]),axis=0)
        self.assertTrue(np.all(peak<=np.asarray(self.arm.config['motion_smoothing']['jerk_limits_rad_s3'])+1e-6))
        checked(self.robot,fast,self.q,.05)

    def test_open_path_and_invalid_parameters_rejected(self):
        with self.assertRaisesRegex(ValueError,'闭合'):
            plan_circle_curve(self.robot,self.w[:-2],self.q,6.)
        with self.assertRaises(ValueError):
            TimedCircleCurve([0,1],np.zeros((2,6)),1.,self.arm.config)


if __name__=='__main__':unittest.main()
