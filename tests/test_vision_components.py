"""迁入的离线视觉与坐标回归；不连接相机或机械臂。"""
import unittest, time, queue, multiprocessing as mp
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
from types import SimpleNamespace
from unittest.mock import patch
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from perception.detectors.depth_hole import detect_depth_hole
from perception.tracking.hole_estimator import HolePositionEstimator
from apps.hole_approach.overlay import HoleOverlay, project
from apps.hole_approach.app import publish_key, publish_settings, ToolCalibrationPanel
from armhub.tools import tool_transform, ToolKinematics, validate_settings


class DepthTests(unittest.TestCase):
    def scene(self,kind='hole'):
        k=np.array([[462.,0,320],[0,462.,240],[0,0,1.]])
        d=np.full((480,640),2.);d[65:415,145:495]=.5
        if kind!='solid':cv2.circle(d,(320,240),46,0. if kind=='missing' else (.3 if kind=='occluder' else 2.),-1)
        return d,k
    def test_depth_rim_not_background(self):
        d,k=self.scene();r=detect_depth_hole(d,k)
        np.testing.assert_allclose(r['center'],[0,0,.5],atol=.001)
        self.assertAlmostEqual(r['radius'],.05,delta=.003)
        self.assertEqual(r['source'],'depth_only')
    def test_invalid_hole_center_still_uses_valid_rim(self):
        d,k=self.scene('missing');r=detect_depth_hole(d,k);self.assertAlmostEqual(r['center'][2],.5,places=5)
    def test_solid_board_and_foreground_object_are_not_holes(self):
        for kind in ['solid','occluder']:
            with self.assertRaises(ValueError):detect_depth_hole(*self.scene(kind))
    def test_multiple_holes_rejected(self):
        d,k=self.scene('solid')
        cv2.circle(d,(260,240),30,2.,-1);cv2.circle(d,(380,240),30,2.,-1)
        with self.assertRaisesRegex(ValueError,'multiple'):detect_depth_hole(d,k)
    def test_tilted_depth_board_normal(self):
        d,k=self.scene('solid');y,x=np.mgrid[:480,:640]
        rays=np.stack(((x-320)/462,(y-240)/462,np.ones_like(d)),axis=-1)
        n=np.array([.1,-.2,-1.]);n/=np.linalg.norm(n)
        plane=np.dot([0,0,.5],n)/(rays@n);d[65:415,145:495]=plane[65:415,145:495]
        cv2.circle(d,(320,240),46,2.,-1)
        r=detect_depth_hole(d,k)
        self.assertGreater(r['normal']@n,.999);self.assertLess(np.linalg.norm(r['center']-[0,0,.5]),.005)
    def test_recorded_real_depth_frame(self):
        with np.load(ROOT/'validation/hole_approach/recorded_rgbd.npz') as f:r=detect_depth_hole(f['depth_m'],f['K'])
        self.assertTrue(.0425<2*r['radius']<.12);self.assertLess(r['plane_rms'],.004)

class EstimatorTests(unittest.TestCase):
    def test_stationary_measurement_noise_is_reduced(self):
        rng=np.random.default_rng(258);k=HolePositionEstimator();truth=np.array([.3,.1,.5]);raw=[];filtered=[]
        for i in range(150):
            measurement=truth+rng.normal(0,.005,3)
            if k.update(measurement,10+i*.1):
                if i>30:raw.append(measurement-truth);filtered.append(k.estimate(10+i*.1)['position']-truth)
        self.assertLess(np.sqrt(np.mean(np.square(filtered))),np.sqrt(np.mean(np.square(raw)))*.75)
        self.assertGreaterEqual(np.linalg.eigvalsh(k.covariance).min(),-1e-12)
    def test_constant_velocity_predicts_during_dropout(self):
        k=HolePositionEstimator();v=np.array([.02,-.01,0.])
        for i in range(40):k.update(np.array([.3,0,.5])+v*i*.1,10+i*.1)
        e=k.estimate(14.9)
        np.testing.assert_allclose(e['velocity'],v,atol=.001)
        np.testing.assert_allclose(e['position'],np.array([.3,0,.5])+v*4.9,atol=.002)
        self.assertTrue(e['prediction_allowed'])
    def test_expiry_uncertainty_and_displacement_limits(self):
        for mode in ['time','uncertainty','displacement']:
            options={'max_position_std_m':.003} if mode=='uncertainty' else ({'max_prediction_displacement_m':.001} if mode=='displacement' else {})
            k=HolePositionEstimator(**options)
            for i in range(40):k.update([.3+.01*i*.1,0,.5],10+i*.1)
            self.assertFalse(k.estimate(k.stamp+(3.1 if mode=='time' else 1.))['prediction_allowed'])
    def test_prediction_reads_do_not_reject_delayed_measurements(self):
        k=HolePositionEstimator();k.update([0,0,.5],10.)
        k.estimate(10.2)
        self.assertTrue(k.update([0,0,.5],10.1));self.assertEqual(k.stamp,10.1)
        self.assertFalse(k.update([0,0,.5],10.05))
    def test_outlier_does_not_extend_prediction_window(self):
        k=HolePositionEstimator()
        for i in range(40):k.update([0,0,.5],10+i*.1)
        stamp=k.stamp
        self.assertFalse(k.update([1,1,1],stamp+.1));self.assertEqual(k.stamp,stamp)
        self.assertFalse(k.estimate(stamp+3.01)['prediction_allowed'])
    def test_insufficient_samples_cannot_predict(self):
        k=HolePositionEstimator()
        for i in range(3):k.update([0,0,.5],10+i*.1)
        self.assertFalse(k.estimate(10.3)['prediction_allowed'])
    def test_expired_track_reinitializes_on_actual_measurement(self):
        k=HolePositionEstimator();k.update([0,0,.5],10.)
        self.assertTrue(k.update([.5,.5,.5],14.));self.assertEqual(k.samples,1)
        np.testing.assert_allclose(k.estimate(14.)['velocity'],[0,0,0])

class OverlayTests(unittest.TestCase):
    def scene(self):
        stamp=time.time();t=np.eye(4);t[:3,3]=[.2,.1,.3]
        # Optical x maps to base -Y, y to -Z, z to +X.
        t[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        camera=np.array([.02,.01,.5]);base=t[:3,:3]@camera+t[:3,3]
        packet={'stamp':stamp,'K':[[462,0,320],[0,462,240],[0,0,1]],'measured_center_optical_m':camera.tolist()}
        status={'camera_pose_history':[{'stamp':stamp,'T_base_camera':t.tolist()}],
                'hole_estimate_base_m':base.tolist(),'plan_hole_base_m':base.tolist(),
                'tcp_goal_base_m':(base-np.array([.12,0,0])).tolist(),
                'current_tcp_base_m':(base-np.array([.2,0,0])).tolist(),
                'plan_plane_normal_base':[-1,0,0],'hole_plane_normal_base':[-1,0,0],
                'hole_estimate_mode':'MEASURED','hole_estimate_age_s':.03,'hole_position_std_m':.005,'overlay_plan_mode':'GO LOCKED'}
        return packet,status,t,base,camera
    def test_coordinates_switch_but_projection_and_distances_do_not(self):
        packet,status,t,base,camera=self.scene();view=HoleOverlay()
        a,_=view.points(packet,status);view.toggle();b,_=view.points(packet,status)
        for left,right in zip(a,b):
            np.testing.assert_allclose(left['coordinates'],left['camera'])
            np.testing.assert_allclose(right['coordinates'],right['base'])
            np.testing.assert_allclose(left['camera'],right['camera'])
            self.assertAlmostEqual(left['camera_range_m'],right['camera_range_m'])
            self.assertAlmostEqual(left['tcp_distance_m'],right['tcp_distance_m'])
            self.assertAlmostEqual(left['frame_range_m'],np.linalg.norm(left['camera']))
            self.assertAlmostEqual(right['frame_range_m'],np.linalg.norm(right['base']))
        np.testing.assert_allclose(a[0]['base'],base);np.testing.assert_allclose(b[1]['camera'],camera)
    def test_normal_gap_and_euclidean_distance_are_distinct(self):
        packet,status,t,base,camera=self.scene();status['current_tcp_base_m']=(base+[-.2,.05,0]).tolist()
        rows,_=HoleOverlay().points(packet,status);row=next(r for r in rows if r['name']=='PLAN HOLE')
        self.assertAlmostEqual(row['plane_gap_m'],.2)
        self.assertAlmostEqual(row['tcp_distance_m'],np.sqrt(.2**2+.05**2))
    def test_old_camera_pose_is_not_used_to_project_new_image(self):
        packet,status,*_=self.scene();status['camera_pose_history'][0]['stamp']-=.5
        rows,_=HoleOverlay().points(packet,status)
        self.assertIsNone(rows[0]['base']);self.assertIsNone(rows[1]['camera'])
        self.assertIsNotNone(rows[1]['base'])
    def test_point_behind_or_outside_image_is_not_clamped_to_false_pixel(self):
        k=np.array([[462,0,320],[0,462,240],[0,0,1]])
        self.assertEqual(project([0,0,-1],k,(480,640))[1],'BEHIND CAMERA')
        self.assertEqual(project([10,0,1],k,(480,640))[1],'OUTSIDE IMAGE')
        self.assertEqual(project([0,0,.5],k,(480,640))[0],(320,240))
    def test_render_camera_and_base_previews(self):
        packet,status,*_=self.scene();image=np.zeros((480,640,3),np.uint8);view=HoleOverlay()
        camera=view.render(image,packet,status);view.toggle();base=view.render(image,packet,status)
        self.assertEqual(camera.shape,(480,1060,3));self.assertGreater(np.count_nonzero(camera),0)
        self.assertFalse(np.array_equal(camera,base));self.assertFalse(np.any(image))

class ToolTests(unittest.TestCase):
    def test_default_rod_tip_four_cm_forward(self):
        np.testing.assert_allclose(tool_transform(SimpleNamespace())[:3,3],[.04,0,0])
    def test_offset_is_in_gripper_axes_and_rotation_independent(self):
        t=tool_transform(SimpleNamespace(rod_length=.04,tcp_xyz_mm=[1,2,3],tcp_rpy_deg=[0,0,90]))
        np.testing.assert_allclose(t[:3,3],[.041,.002,.003])
        np.testing.assert_allclose(t[:3,:3],Rotation.from_euler('z',90,degrees=True).as_matrix())
    def test_tool_fk_ik_transform_roundtrip_preserves_gripper_model(self):
        grip=np.eye(4);grip[:3,:3]=Rotation.from_euler('xyz',[.2,-.3,.4]).as_matrix();grip[:3,3]=[.1,.2,.3]
        class Robot:
            def forward_kinematics(self,q):return {'transform':grip.copy()}
            def inverse_kinematics(self,p,r,*args,**kwargs):
                np.testing.assert_allclose(p,grip[:3,3],atol=1e-12)
                np.testing.assert_allclose(r,grip[:3,:3],atol=1e-12)
                return [1]*6
        raw=Robot();tool=ToolKinematics(raw,tool_transform(SimpleNamespace(tcp_rpy_deg=[4,5,6])))
        fk=tool.forward_kinematics([0]*6)
        self.assertEqual(tool.inverse_kinematics(fk['position'],fk['rotation']),[1]*6)
        np.testing.assert_allclose(raw.forward_kinematics([])['transform'],grip)
    def test_invalid_calibration_is_rejected(self):
        for length in [float('nan'),-.1,.31]:
            with self.assertRaises(ValueError):tool_transform(SimpleNamespace(rod_length=length))
    def test_ui_apply_publishes_values_and_invalidates_old_epoch(self):
        commands=queue.Queue();generation=mp.get_context('spawn').Value('L',3)
        values=dict(rod_length=.04,tcp_xyz_mm=[0,0,0],tcp_rpy_deg=[0,0,0],plane_offset_m=[-.05,-.05])
        publish_settings(values,commands,generation)
        packet=commands.get_nowait()
        self.assertEqual(packet['epoch'],4);self.assertEqual(generation.value,4)
        self.assertEqual(packet['settings'],validate_settings(values))
    def test_following_go_carries_applied_settings_when_apply_is_coalesced(self):
        commands=queue.Queue(maxsize=2);context=mp.get_context('spawn')
        generation=context.Value('L',0);quit_event=context.Event()
        values=dict(rod_length=.06,tcp_xyz_mm=[1,2,3],tcp_rpy_deg=[0,0,0],plane_offset_m=[-.05,-.05])
        publish_settings(values,commands,generation)
        publish_key('g',commands,generation,quit_event,values)
        commands.get_nowait();go=commands.get_nowait()
        self.assertEqual(go['key'],'g');self.assertEqual(go['settings']['rod_length'],.06)
    def test_ui_slider_roundtrip_and_apply_button_without_sdk(self):
        args=SimpleNamespace(rod_length=.04,tcp_xyz_mm=[1,2,3],tcp_rpy_deg=[0,2,0],plane_offset_m=[-.05,-.05])
        panel=ToolCalibrationPanel(args);positions={};callbacks={}
        def create(name,window,value,maximum,callback):positions[name]=value
        with patch('apps.hole_approach.app.cv2.namedWindow'),patch('apps.hole_approach.app.cv2.resizeWindow'),patch('apps.hole_approach.app.cv2.imshow'),patch('apps.hole_approach.app.cv2.createTrackbar',side_effect=create),patch('apps.hole_approach.app.cv2.setMouseCallback',side_effect=lambda window,cb:callbacks.update(mouse=cb)),patch('apps.hole_approach.app.cv2.getTrackbarPos',side_effect=lambda name,window:positions[name]):
            panel.open();positions['Rod mm']=45;positions['Offset Right']=160
            values=panel.values();self.assertEqual(values['rod_length'],.045)
            self.assertEqual(values['plane_offset_m'],[-.04,-.05])
            import cv2
            callbacks['mouse'](cv2.EVENT_LBUTTONDOWN,500,55,0,None)
            self.assertTrue(panel.apply_requested)
