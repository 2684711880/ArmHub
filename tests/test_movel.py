import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

from armhub import create_arm
from armhub.execution import CheckedCurve
from apps.movel.moveL_pos_control import CartesianCurve, PlanningRobot, checked
from apps.movel.moveL_trajectories import to_waypoints, shape_uv, path_length
from apps.movel.moveL_error_analysis import record_cartesian_motion, save_run, analyze_run
from motion_fixture import virtual_time
import test_panthera_hardware as hardware_fixture
from test_panthera_cartesian import FakeKinematics


class MoveLTests(unittest.TestCase):
    make_arm = hardware_fixture.HardwareTests.make_arm

    def test_real_model_curve_derivatives_and_collision(self):
        arm=create_arm('panthera_ht','hardware')
        robot=PlanningRobot(arm,[0]*6)
        rot=np.array([[0,0,1],[0,1,0],[-1,0,0]])
        q=robot.solve([.24,0,.1],rot,[0]*6)
        fk=robot.forward_kinematics(q)
        w=to_waypoints(fk['position'],fk['rotation'],'xy',shape_uv('line_u',.02,.005))
        q0=robot.solve(w[0]['position'],rot,q)
        curve=CartesianCurve(robot.path(w,q0),2,arm.config)
        checked(robot,curve,q0,.05)
        for t in [0,curve.duration]:
            _,v,a=curve.sample(t)
            np.testing.assert_array_equal(v,np.zeros(6));np.testing.assert_array_equal(a,np.zeros(6))
        with self.assertRaisesRegex(ValueError,'高度'):
            checked(robot,curve,q0,.5)
        self.assertIsNone(arm._driver)
        self.assertAlmostEqual(path_length(w),.02)

    def test_square_from_current_has_no_diagonal_transfer(self):
        from types import SimpleNamespace
        from contextlib import redirect_stdout
        import io
        from apps.movel.moveL_pos_control import build_plan
        arm=create_arm('panthera_ht','hardware')
        robot=PlanningRobot(arm,[0]*6)
        rotation=np.array([[0,0,1],[0,1,0],[-1,0,0]])
        q=robot.solve([.24,0,.1],rotation,[0]*6)
        args=SimpleNamespace(anchor=None,plane=['xy'],trajectories=['square'],direction='fwd',
                             size=.04,pitch=.01,speed=.01,z_min=.05,square_from_current=True)
        with redirect_stdout(io.StringIO()):
            initial,plans,report=build_plan(robot,args,q)
        item=plans[0]
        self.assertIsNone(initial);self.assertIsNone(item['lead']);self.assertIsNone(item['back'])
        np.testing.assert_allclose(item['waypoints'][0]['position'],robot.forward_kinematics(q)['position'])
        np.testing.assert_allclose(item['waypoints'][-1]['position'],item['waypoints'][0]['position'])
        edges=np.diff([w['position'] for w in item['waypoints']],axis=0)
        np.testing.assert_allclose(edges,[[.04,0,0],[0,.04,0],[-.04,0,0],[0,-.04,0]],atol=1e-10)
        self.assertIsNone(arm._driver)

    def test_fk_snapshots_are_not_rewritten_by_later_model_calls(self):
        arm=create_arm('panthera_ht','hardware')
        robot=PlanningRobot(arm,[0]*6)
        pose=robot.forward_kinematics([0,.8,1.,-.3,0,0])
        rotation=pose['rotation'].copy();transform=pose['transform'].copy()
        robot.forward_kinematics([.3,.9,1.1,-.2,.2,.3])
        np.testing.assert_array_equal(pose['rotation'],rotation)
        np.testing.assert_array_equal(pose['transform'],transform)

    def test_guarded_recording_and_report_roundtrip(self):
        clock=virtual_time(self)
        # The recorder and executor share the same time module under this fixture.
        from unittest.mock import patch
        arm,driver=self.make_arm();arm._kinematics_cache=FakeKinematics();arm.enable()
        robot=type('RecordingRobot',(),{})()
        robot.arm=arm;robot.resample_dt=.05
        robot.forward_kinematics=lambda q:dict(position=np.asarray(q[:3]),rotation=np.eye(3))
        qs=np.array([[.1]*6,[.12]*6])
        curve=CartesianCurve(qs,2.,arm.config)
        w=[dict(position=q[:3],rotation=np.eye(3)) for q in qs]
        records=[]
        # Fake driver uses its virtual wall clock; align host wall time to it.
        wall_base, mono_base = clock.time(), clock.now
        clock.time = lambda: wall_base + clock.now - mono_base
        with patch('apps.movel.moveL_error_analysis.time',clock), patch('test_panthera_hardware.time',clock):
            record_cartesian_motion(robot,'xy_line_u_fwd',w,curve.duration,.1,records,
                                    dict(plane='xy',shape='line_u',direction='fwd'),curve=curve)
        rec=records[0]
        self.assertTrue(rec['completed'])
        self.assertGreater(len(rec['fb_t']),10)
        self.assertEqual(len(rec['dispatch_t']),len(rec['dispatched_q']))
        self.assertEqual(len(rec['dispatch_t']),len(rec['command_sample_t']))
        self.assertGreater(len(driver.transmissions),10)
        with tempfile.TemporaryDirectory() as folder:
            save_run(folder,records,dict(test_data='synthetic_not_hardware'))
            metrics=analyze_run(folder)
            self.assertEqual(len(metrics),1)
            self.assertTrue((Path(folder)/'summary.png').exists())
            self.assertTrue((Path(folder)/'01_xy_line_u_fwd.csv').exists())


if __name__=='__main__':unittest.main()
