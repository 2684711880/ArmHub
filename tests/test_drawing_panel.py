import csv
from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from armhub import create_arm
from apps.drawing_panel.controller import DrawingSettings, HardwareWorker, drawing_waypoints, plan_drawing, export_xy
from apps.movel.moveL_pos_control import PlanningRobot, CartesianCurve
from apps.movel.moveL_error_analysis import record_cartesian_motion, find_direction_pairs
from motion_fixture import virtual_time
import test_panthera_hardware as hardware_fixture
from test_panthera_cartesian import FakeKinematics


class DrawingGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm=create_arm('panthera_ht','hardware')
        cls.robot=PlanningRobot(cls.arm,[0]*6)
        rot=np.array([[0,0,1],[0,1,0],[-1,0,0]])
        cls.q=cls.robot.solve([.24,0,.1],rot,[0]*6)
        pose=cls.robot.forward_kinematics(cls.q)
        cls.origin=dict(position=pose['position'],rotation=pose['rotation'],quaternion_xyzw=cls.robot.kin.fk(cls.q)['quaternion_xyzw'])

    def test_reverse_line_returns_over_same_geometry(self):
        s=DrawingSettings(shape='line_u')
        f=drawing_waypoints(self.origin,s,'fwd');r=drawing_waypoints(self.origin,s,'rev')
        np.testing.assert_allclose(f[0]['position'],self.origin['position'])
        np.testing.assert_allclose(r[0]['position'],f[-1]['position'])
        np.testing.assert_allclose(r[-1]['position'],f[0]['position'])
        self.assertIsNone(self.arm._driver)

    def test_reverse_from_wrong_endpoint_is_rejected_without_motion(self):
        with self.assertRaisesRegex(ValueError,'不会自动画引入线'):
            plan_drawing(self.robot,self.q,self.origin,DrawingSettings(shape='line_u'),'rev')
        self.assertIsNone(self.arm._driver)

    def test_square_closed_real_model_checked_and_height_rejection(self):
        with redirect_stdout(io.StringIO()):
            w,curve,report=plan_drawing(self.robot,self.q,self.origin,DrawingSettings(),'fwd')
        self.assertAlmostEqual(report['length_mm'],160.)
        self.assertGreater(curve.duration,0)
        self.assertNotIn('lead',report)
        np.testing.assert_allclose(w[0]['position'],w[-1]['position'])
        with self.assertRaisesRegex(ValueError,'最低点'):
            plan_drawing(self.robot,self.q,self.origin,DrawingSettings(z_min_mm=150),'fwd')
        self.assertIsNone(self.arm._driver)

    def test_geometry_pair_ids_do_not_mix_different_sizes(self):
        def rec(direction,pair_id):return dict(tags=dict(plane='xy',shape='square',direction=direction,pair_id=pair_id))
        self.assertEqual(find_direction_pairs([rec('fwd','40mm'),rec('rev','80mm')]),[])
        self.assertEqual(find_direction_pairs([rec('fwd','40mm'),rec('rev','40mm')]),[('xy','square',0,1)])

    def test_export_uses_feedback_xyz_even_when_plan_differs(self):
        rec=dict(name='synthetic',waypoints=np.array([[0.,0.,.1],[.04,0,.1]]),
                 plan_p=np.array([[0.,0.,.1],[.04,0,.1]]),fb_t=np.array([0.,1.]),
                 fb_p=np.array([[.001,.002,.1],[.039,.003,.1]]))
        with tempfile.TemporaryDirectory() as folder:
            export_xy(folder,rec,'xy')
            with (Path(folder)/'feedback_xyz.csv').open() as stream:
                rows=list(csv.reader(stream))
            self.assertEqual(float(rows[-1][1]),.039)
            self.assertEqual(float(rows[-1][2]),.003)
            self.assertTrue((Path(folder)/'feedback_path.png').exists())


class WorkerAndStopTests(unittest.TestCase):
    make_arm=hardware_fixture.HardwareTests.make_arm

    def test_duplicate_commands_cannot_queue_motions(self):
        with tempfile.TemporaryDirectory() as folder:
            worker=HardwareWorker(lambda *x:None,directory=Path(folder)/'session')
            self.assertTrue(worker.submit('connect'))
            self.assertFalse(worker.submit('home',execute_allowed=True))
            self.assertEqual(worker.commands.qsize(),1)
            worker.request_stop();self.assertFalse(worker.guard())

    def test_missing_execution_permission_never_touches_motor(self):
        with tempfile.TemporaryDirectory() as folder:
            worker=HardwareWorker(lambda *x:None,directory=Path(folder)/'session')
            arm,driver=self.make_arm();worker.arm=arm
            with self.assertRaisesRegex(RuntimeError,'允许执行'):
                worker.action('home',DrawingSettings(),'fwd',False)
            self.assertEqual(driver.transmissions,[])

    def test_stop_hook_saves_partial_feedback_and_confirms_hold(self):
        clock=virtual_time(self);arm,driver=self.make_arm()
        arm._kinematics_cache=FakeKinematics();arm.enable()
        robot=SimpleNamespace(arm=arm,resample_dt=.05,
            forward_kinematics=lambda q:dict(position=np.array(q[:3]),rotation=np.eye(3)))
        curve=CartesianCurve(np.array([[.1]*6,[.12]*6]),2.,arm.config)
        w=[dict(position=[.1]*3,rotation=np.eye(3)),dict(position=[.12]*3,rotation=np.eye(3))]
        records=[];running=[True];count=[0]
        def observed(states):
            count[0]+=1
            if count[0]==5:running[0]=False
        wall,mono=clock.time(),clock.now;clock.time=lambda:wall+clock.now-mono
        with patch('apps.movel.moveL_error_analysis.time',clock),patch('test_panthera_hardware.time',clock):
            with self.assertRaisesRegex(RuntimeError,'替代'):
                record_cartesian_motion(robot,'cancelled',w,curve.duration,.1,records,
                    curve=curve,command_guard=lambda:running[0],on_feedback=observed)
        self.assertFalse(arm._moving)
        self.assertFalse(records[0]['completed'])
        self.assertEqual(len(records[0]['fb_t']),5)
        self.assertEqual(len(driver.transmissions),6) # five references + measured hold
        self.assertIsNotNone(records[0]['error'])


if __name__=='__main__':unittest.main()
