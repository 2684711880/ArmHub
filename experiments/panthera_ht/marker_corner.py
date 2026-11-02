"""Fresh D405 observation -> official hand-eye -> bounded ArmHub move_to."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
from armhub import create_arm
from apps.hole_approach.config import ROOT, load_calibration


def target_point(transform, rotation, translation, size, distance, tool_offset):
    corner = rotation @ np.array([-size/2, size/2, 0.]) + translation
    normal = rotation[:, 2]
    # Positive marker Z points out of its printed face, towards the observer.
    if normal @ translation >= 0:
        raise ValueError('标记法向背向相机，拒绝目标')
    target = transform[:3, :3] @ (corner + distance*normal) + transform[:3, 3]
    return target - tool_offset, corner


def solve_marker_pose(intr, pixels, obj):
    if intr.model not in (rs.distortion.none,rs.distortion.brown_conrady,rs.distortion.inverse_brown_conrady):
        raise ValueError('不支持的 RealSense 畸变模型')
    rays=np.array([rs.rs2_deproject_pixel_to_point(intr,[float(u),float(v)],1.) for u,v in pixels])
    normalized=rays[:,:2]/rays[:,2,None]
    ok,r,t=cv2.solvePnP(obj,normalized,np.eye(3),np.zeros(5))
    if not ok or not np.isfinite(t).all() or not .1<t[2,0]<2:raise RuntimeError('码位姿求解失败')
    projected=cv2.projectPoints(obj,r,t,np.eye(3),np.zeros(5))[0].reshape(4,2)
    rms=float(np.sqrt(np.mean(np.sum(((projected-normalized)*[intr.fx,intr.fy])**2,axis=1))))
    return r,t,rms


def main():
    p=argparse.ArgumentParser(description='黑色 ArUco 左上角沿纸面法向向外指定距离；默认仅测量和规划')
    p.add_argument('--standoff', type=float, default=.10, help='到码平面的法向距离，米；例如 0.10 = 10 cm')
    p.add_argument('--marker-size', type=float, default=.10, help='黑色方块边长，米，不含白边')
    p.add_argument('--marker-id', type=int, default=23)
    p.add_argument('--serial', default='262622274345')
    p.add_argument('--calibration', type=Path, default=ROOT/'calibration/panthera_ht/wrist_camera/hand_eye.json')
    p.add_argument('--tcp-offset-x', type=float, default=0., help='验证点相对 gripper_center 沿 +X 的延伸，米；无杆为 0')
    p.add_argument('--velocity-scale', type=float, default=.10)
    p.add_argument('--execute', action='store_true')
    a=p.parse_args()
    if not np.isfinite([a.standoff,a.marker_size,a.tcp_offset_x,a.velocity_scale]).all() or not .03<=a.standoff<=.5 or not .01<=a.marker_size<=.5 or not 0<=a.tcp_offset_x<=.2 or not 0<a.velocity_scale<=.15:
        p.error('距离范围 0.03..0.5 m，码边长 0.01..0.5 m，工具偏移 0..0.2 m，速度比例 (0,0.15]')
    calibration,status=load_calibration(a.calibration)
    meta=json.loads(a.calibration.read_text())
    if meta.get('camera_serial')!=a.serial:raise ValueError('相机序列号与手眼配置不一致')
    out=ROOT/'validation/marker_corner'/time.strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True)
    pipe=rs.pipeline();cfg=rs.config();cfg.enable_device(a.serial)
    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),cv2.aruco.DetectorParameters())
    with create_arm('panthera_ht','hardware') as arm:
        pipe.start(cfg)
        try:
            for _ in range(30):pipe.wait_for_frames(5000)
            before=arm.get_joint_feedback()
            frame=pipe.wait_for_frames(5000).get_color_frame()
            captured=time.monotonic()
            after=arm.get_joint_feedback()
            if max(abs(x.velocity) for x in (*before[:6],*after[:6]))>.03 or max(abs(x.position-y.position) for x,y in zip(before[:6],after[:6]))>.003:
                raise RuntimeError('拍摄时机械臂未静止；请静止后重试')
            im=np.asanyarray(frame.get_data()).copy();intr=frame.profile.as_video_stream_profile().intrinsics
            cv2.imwrite(str(out/'raw.png'),im)
            k=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]])
            dist=np.array(intr.coeffs)
            corners,ids,_=detector.detectMarkers(im)
            hits=[] if ids is None else np.where(ids.ravel()==a.marker_id)[0]
            if len(hits)!=1:raise RuntimeError('画面未找到唯一目标码')
            pixels=corners[hits[0]].reshape(4,2).astype(float)
            if np.any(pixels<8) or np.any(pixels[:,0]>632) or np.any(pixels[:,1]>472):raise RuntimeError('目标码过近画面边界')
            if min(np.linalg.norm(pixels-np.roll(pixels,-1,axis=0),axis=1))<40:raise RuntimeError('目标码像素尺寸过小')
            half=a.marker_size/2;obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]])
            r,t,rms=solve_marker_pose(intr,pixels,obj)
            if rms>1:raise RuntimeError('重投影误差超过 1 像素')
            cv2.aruco.drawDetectedMarkers(im,corners,ids);cv2.circle(im,tuple(pixels[0].astype(int)),9,(0,0,255),2)
            cv2.imwrite(str(out/'detect.png'),im)
        finally:pipe.stop()
        q=tuple(x.position for x in after[:6]);model=arm._kinematics();fk=model.fk(q)
        R=Rotation.from_quat(fk['quaternion_xyzw']).as_matrix()
        base_tcp=np.eye(4);base_tcp[:3,:3]=R;base_tcp[:3,3]=fk['position_m']
        target,corner=target_point(base_tcp@calibration,cv2.Rodrigues(r)[0],t.ravel(),a.marker_size,a.standoff,R@np.array([a.tcp_offset_x,0,0]))
        (out/'ik_request.json').write_text(json.dumps({'q':q,'target':target.tolist(),'quaternion':fk['quaternion_xyzw'],'K':k.tolist(),'distortion':dist.tolist(),'T_tcp_camera':calibration.tolist()},indent=2))
        plan=arm.preview_move_to(target.tolist(),quaternion_xyzw=fk['quaternion_xyzw'],velocity_scale=a.velocity_scale)
        report={'standoff_m':a.standoff,'tcp_offset_x_m':a.tcp_offset_x,'orientation':'keep_current','calibration_status':status,'T_tcp_camera':calibration.tolist(),'camera_serial':a.serial,'marker_size_m':a.marker_size,'corner_camera_m':corner.tolist(),'target_gripper_center_base_m':target.tolist(),'reprojection_rms_px':rms,'plan':plan,'motion_sent':False}
        (out/'plan.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2));print('输出目录:',out,flush=True)
        if a.execute:
            current=arm.get_state().positions
            if max(abs(x-y) for x,y in zip(current,q))>.01:raise RuntimeError('拍摄后机械臂发生移动，请重新运行')
            if time.monotonic()-captured>30:raise RuntimeError('拍摄已超过 30 秒，请重新运行')
            arm.enable()
            try:result=arm.move_to(target.tolist(),quaternion_xyzw=fk['quaternion_xyzw'],velocity_scale=a.velocity_scale)
            except BaseException:
                arm.stop()
                raise
            report['motion_sent']=True;report['result']=asdict(result)
            (out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(asdict(result),indent=2))


if __name__=='__main__':main()
