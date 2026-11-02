import contextlib
import copy
import io
import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from apps.hole_approach.__main__ import main,arguments
from apps.hole_approach.config import ROOT,load_config,load_calibration
from armhub.trajectory import ContinuousFeed


class ApplicationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.report=Path(self.temp.name)/'report.json'
    def test_execute_once_requires_explicit_execution_flag(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):arguments(['--execute-once'])
    def test_camera_only_rejects_execution_flag(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):arguments(['--camera-only','--execute'])
    def test_current_calibration_and_zero_patch_are_preserved(self):
        a=arguments([])
        matrix,status=load_calibration(a.calibration)
        self.assertEqual(status,'calibrated_pending_independent_measurement')
        self.assertEqual(a.plane_offset_m,[0.,0.])
        self.assertEqual(load_config(a.config)['robot']['velocity_limits'],[1.68,2.,2.,1.68,1.48,1.48])
        np.testing.assert_allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-6)
    def test_dry_run_never_constructs_arm_and_validates_continuous_feed(self):
        with patch('armhub.task_runtime.create_arm',side_effect=AssertionError('unexpected hardware')),contextlib.redirect_stdout(io.StringIO()):
            main(['--dry-run','--report',str(self.report)])
        plan=json.loads(self.report.read_text())
        self.assertFalse(plan['motion_sent']);self.assertTrue(plan['dry_run'])
        self.assertGreater(np.max(np.abs(np.array(plan['stage1']['target_joints_rad'])-
                                   np.array(plan['start_joints_rad']))),.1)
        self.assertGreaterEqual(len(plan['stage2_waypoints']),90)
        n=np.array(plan['outward_normal_base']);hole=np.array(plan['hole_center_base_m'])
        self.assertAlmostEqual((np.array(plan['stage1']['target_position_m'])-hole)@n,.2)
        self.assertAlmostEqual((np.array(plan['stage2_waypoints'][-1]['target_position_m'])-hole)@n,.02)
        spec=plan['feed_trajectory'];curve=ContinuousFeed(spec)
        velocities=[np.linalg.norm(curve.sample(t)[1]) for t in np.linspace(curve.duration*.25,curve.duration*.75,30)]
        self.assertGreater(min(velocities),1e-4)
        self.assertLess(spec['sampled_max_cartesian_error_m'],.002)
        for phase in ['approach','feed']:
            self.assertEqual(plan['collision_checks'][phase]['scope'],'ruckig_reference_self_collision_only')
    def test_camera_only_reads_without_opening_arm(self):
        k=np.array([[462.,0,320],[0,462.,240],[0,0,1.]])
        depth=np.full((480,640),2.);depth[65:415,145:495]=.5;cv2.circle(depth,(320,240),46,2.,-1)
        class Camera:
            def read(self):return np.zeros((480,640,3),np.uint8),depth,k,time.time()
            def close(self):pass
        with patch('perception.cameras.realsense.LiveCamera',return_value=Camera()),patch('armhub.task_runtime.create_arm',side_effect=AssertionError('unexpected hardware')),contextlib.redirect_stdout(io.StringIO()):
            main(['--camera-only','--no-gui','--report',str(self.report)])
        report=json.loads(self.report.read_text())
        self.assertFalse(report['motion_sent']);self.assertEqual(report['status'],'detected')
    def test_invalid_handeye_frame_rejected(self):
        source=json.loads((ROOT/'calibration/panthera_ht/wrist_camera/hand_eye.json').read_text())
        source['parent_frame']='world'
        path=Path(self.temp.name)/'bad.json';path.write_text(json.dumps(source))
        with self.assertRaises(ValueError):load_calibration(path)

if __name__=='__main__':unittest.main()
