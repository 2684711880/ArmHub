"""恢复的是六轴各自的上限；用驱动替身验证规划与下发均不遗留 0.1 全局帽。"""
import unittest
from armhub.execution import CheckedCurve
from armhub.motion import JointTransition
from armhub.profiles import load_profile
from armhub.adapters.panthera_ht import PantheraHardwareBackend
from apps.hole_approach.__main__ import arguments
from apps.hole_approach.config import load_config
from motion_fixture import virtual_time
from test_panthera_cartesian import FakeKinematics
import test_panthera_hardware as fixtures

LIMITS=[1.68,2.,2.,1.68,1.48,1.48]


class RestoredSpeedTests(unittest.TestCase):
    make_arm=fixtures.HardwareTests.make_arm
    def ready(self):
        virtual_time(self)
        arm,driver=self.make_arm()
        arm.config['velocity_limits_rad_s']=LIMITS.copy()
        arm._kinematics_cache=FakeKinematics()
        return arm,driver
    def test_production_profile_matches_reference_limits(self):
        arm=PantheraHardwareBackend(load_profile('panthera_ht'))
        self.assertEqual(arm.config['velocity_limits_rad_s'],LIMITS)
        self.assertNotIn('max_speed_rad_s',arm.config)
        self.assertEqual(load_config()['robot']['velocity_limits'],LIMITS)
        self.assertEqual(arguments([]).recovery_speed,.6)
    def test_home_defaults_to_restored_limits_and_scale_is_per_axis(self):
        arm,driver=self.ready()
        plan=arm.preview_home()
        self.assertEqual(list(plan['effective_max_speeds_rad_s']),LIMITS)
        self.assertEqual(plan['reference_max_velocity_rad_s'],LIMITS)
        half=arm.preview_home(velocity_scale=.5)
        self.assertEqual(list(half['effective_max_speeds_rad_s']),[v*.5 for v in LIMITS])
        self.assertEqual(driver.transmissions,[])
    def test_actual_stream_exceeds_old_cap_and_respects_each_axis(self):
        arm,driver=self.ready()
        start=[-2.2,3.1,3.2,1.4,1.6,2.4]
        for state,q in zip(driver.states[:6],start):state.position=q
        arm.enable()
        result=arm.home()
        self.assertEqual(result.status,'succeeded')
        self.assertGreater(max(packet[i][1] for packet in driver.transmissions for i in range(6)),.1)
        for packet in driver.transmissions:
            self.assertTrue(all(0<packet[i][1]<=LIMITS[i]+1e-9 for i in range(6)))
            self.assertEqual(packet[6][0],.5)
    def test_validation_rejects_speed_above_specific_axis_even_below_global_max(self):
        arm,driver=self.ready()
        class Curve:
            duration=1.
            def sample(self,t):return (.1,)*6,(1.9,0,0,0,0,0),(0,)*6
        with self.assertRaisesRegex(ValueError,'速度超限'):
            CheckedCurve(Curve(),arm.config,(.1,)*6)
        self.assertEqual(driver.transmissions,[])
    def test_ruckig_retains_distinct_joint_limits(self):
        arm,driver=self.ready()
        curve=JointTransition((.1,)*6,(1.,1.,1.,1.,1.,1.),arm.config,speed_limit=LIMITS,minimum_duration=2.)
        self.assertEqual(list(curve.speed_limits),LIMITS)
        self.assertEqual(curve.summary()['reference_max_velocity_rad_s'],LIMITS)

if __name__=='__main__':unittest.main()
