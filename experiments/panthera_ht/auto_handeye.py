"""15-pose D405 eye-in-hand calibration; explicit execution, no automatic return."""
import argparse
from dataclasses import asdict
from datetime import datetime
from zoneinfo import ZoneInfo
import json
from pathlib import Path
import time
import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
from armhub import create_arm
from apps.hole_approach.config import ROOT
from .auto_handeye_core import poses, excitation, scatter, solve


def save(path,data):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2));tmp.replace(path)


class Camera:
    def __init__(self,serial):
        self.pipe=rs.pipeline();c=rs.config();c.enable_device(serial);c.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
        p=self.pipe.start(c)
        try:
            self.intr=p.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
            if self.intr.model not in (rs.distortion.none,rs.distortion.brown_conrady,rs.distortion.inverse_brown_conrady):raise ValueError('不支持的相机畸变模型')
            self.detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),cv2.aruco.DetectorParameters())
            for _ in range(30):self.pipe.wait_for_frames(2000)
        except BaseException:self.pipe.stop();raise
        self.metadata={'serial':serial,'distortion_model':str(self.intr.model),'coeffs':list(self.intr.coeffs),'fx':self.intr.fx,'fy':self.intr.fy,'cx':self.intr.ppx,'cy':self.intr.ppy,'resolution':[640,480],'pose_method':'RealSense deprojection to normalized rays, then OpenCV solvePnP with zero distortion'}

    def observe(self,marker_id,size):
        # Polling is bounded; don't drain indefinitely on a live stream.
        for _ in range(8):
            if not self.pipe.poll_for_frames():break
        frame=self.pipe.wait_for_frames(2000).get_color_frame();image=np.asanyarray(frame.get_data()).copy()
        corners,ids,_=self.detector.detectMarkers(image);hits=[] if ids is None else np.where(ids.ravel()==marker_id)[0]
        if len(hits)!=1:raise ValueError('未检测到唯一目标码')
        pix=corners[hits[0]].reshape(4,2).astype(float)
        if np.any(pix<8) or np.any(pix[:,0]>632) or np.any(pix[:,1]>472) or min(np.linalg.norm(pix-np.roll(pix,-1,axis=0),axis=1))<40:raise ValueError('码太小或靠近边界')
        rays=np.array([rs.rs2_deproject_pixel_to_point(self.intr,[float(u),float(v)],1.) for u,v in pix]);norm=rays[:,:2]/rays[:,2,None]
        s=size/2;obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]])
        ok,r,t=cv2.solvePnP(obj,norm,np.eye(3),np.zeros(5))
        if not ok or not np.isfinite(t).all() or not .1<t[2,0]<1.5:raise ValueError('码位姿非法')
        projected=cv2.projectPoints(obj,r,t,np.eye(3),np.zeros(5))[0].reshape(4,2)
        rms=float(np.sqrt(np.mean(np.sum(((projected-norm)*[self.intr.fx,self.intr.fy])**2,axis=1))))
        if rms>1.:raise ValueError('重投影误差超过1像素')
        T=np.eye(4);T[:3,:3]=cv2.Rodrigues(r)[0];T[:3,3]=t.ravel()
        return image,{'T_camera_marker':T.tolist(),'corners_px':pix.tolist(),'reprojection_rms_px':rms,'camera_frame_number':frame.get_frame_number(),'camera_timestamp_ms':frame.get_timestamp(),'timestamp_domain':str(frame.get_frame_timestamp_domain()),'host_receive_time':time.time()}

    def close(self):self.pipe.stop()


def transform(model,q):
    f=model.fk(q);t=np.eye(4);t[:3,:3]=Rotation.from_quat(f['quaternion_xyzw']).as_matrix();t[:3,3]=f['position_m'];return t


def capture(arm,model,camera,args,out,index):
    arm._stable_feedback();burst=[];images=[]
    for _ in range(5):
        before=arm.get_joint_feedback();image,o=camera.observe(args.marker_id,args.marker_size);after=arm.get_joint_feedback()
        q0=np.array([s.position for s in before[:6]]);q1=np.array([s.position for s in after[:6]])
        if np.max(np.abs(q1-q0))>.003 or max(abs(s.velocity) for s in (*before[:6],*after[:6]))>.03:raise RuntimeError('拍摄时关节移动，采样已拒绝')
        q=(q0+q1)/2;o.update(q_rad=q.tolist(),T_base_tcp=transform(model,q).tolist(),feedback_before=[asdict(s) for s in before],feedback_after=[asdict(s) for s in after]);burst.append(o);images.append(image)
    metrics,_=scatter([s['T_camera_marker'] for s in burst])
    if metrics['position_max_mm']>3 or metrics['angle_max_deg']>2 or np.max(np.ptp([s['q_rad'] for s in burst],axis=0))>.003:raise RuntimeError('静止采样不稳定: '+str(metrics))
    for j,im in enumerate(images):
        if not cv2.imwrite(str(out/f'pose_{index:02d}_{j}.png'),im):raise RuntimeError('保存图片失败')
    return dict(burst[-1],burst=burst,stationary_metrics=metrics,image=f'pose_{index:02d}_4.png')


def main():
    p=argparse.ArgumentParser(description=__doc__);mode=p.add_mutually_exclusive_group();mode.add_argument('--execute',action='store_true');mode.add_argument('--dry-run',action='store_true');mode.add_argument('--solve',type=Path)
    p.add_argument('--serial',default='262622274345');p.add_argument('--marker-id',type=int,default=23);p.add_argument('--marker-size',type=float,default=.10);p.add_argument('--amplitude-deg',type=float,default=10.);p.add_argument('--speed',type=float,default=.04,help='六轴最大速度 rad/s');p.add_argument('--joints',nargs=6,type=float);p.add_argument('--output',type=Path)
    a=p.parse_args()
    if not 0<a.marker_size<.5 or not 0<=a.marker_id<250 or not np.isfinite([a.marker_size,a.speed,a.amplitude_deg]).all() or not .005<=a.speed<=.06:p.error('非法码尺寸/ID/速度，速度范围 .005.. .06 rad/s')
    if a.joints is not None and not a.dry_run:p.error('--joints 仅用于离线预览')
    if a.solve:
        data=json.loads(a.solve.read_text());result=solve(data['samples']);result.update(camera_model='Intel RealSense D405',camera_serial=data['camera']['serial'],source_samples=str(a.solve.resolve()),marker_size_m=data['marker_size_m']);save(a.solve.parent/'calibration_result.json',result);print(json.dumps(result,ensure_ascii=False,indent=2));return
    out=a.output or ROOT/'calibration/panthera_ht/wrist_camera/sessions'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=False)
    print('SESSION:',out,flush=True)
    if a.dry_run:
        from armhub.adapters.panthera_kinematics import PantheraKinematics
        h=json.loads((ROOT/'robots/panthera_ht/hardware/config.json').read_text());model=PantheraKinematics(h['lower_rad'],h['upper_rad']);q=a.joints or [0.,.6,.6,0.,.3,0.];plan=poses(q,h['lower_rad'],h['upper_rad'],a.amplitude_deg);ex=excitation([transform(model,x['q_rad']) for x in plan]);save(out/'plan.json',{'synthetic':True,'poses':plan,'excitation':ex,'motion_sent':False});print('离线15姿态包络/旋转激励通过；未连接设备');return
    samples=[];camera=None
    with create_arm('panthera_ht','hardware') as arm:
        try:
            model=arm._kinematics();q=arm.get_state().positions;plan=poses(q,arm.config['lower_rad'],arm.config['upper_rad'],a.amplitude_deg)
            checks=[];previous=q
            for pose in plan:
                curve=arm._build_transition(previous,pose['q_rad'],max(3.,max(abs(x-y) for x,y in zip(previous,pose['q_rad']))/a.speed),[a.speed]*6)
                checks.append(model.check_trajectory(curve));previous=pose['q_rad']
            ex=excitation([transform(model,x['q_rad']) for x in plan]);save(out/'plan.json',{'poses':plan,'path_checks':checks,'excitation':ex,'start_rad':q,'motion_requested':a.execute,'automatic_return':False})
            camera=Camera(a.serial);image,obs=camera.observe(a.marker_id,a.marker_size);cv2.imwrite(str(out/'preview.png'),image);save(out/'camera.json',dict(camera.metadata,preview=obs));print('15姿态路径/激励/相机检查通过。',flush=True)
            if not a.execute:print('仅预览；执行请加 --execute');return
            if max(abs(x-y) for x,y in zip(arm.get_state().positions,q))>.003:raise RuntimeError('预检期间起点改变，请重新运行')
            arm.enable()
            for pose in plan:
                start=arm.get_state().positions;delta=max(abs(x-y) for x,y in zip(start,pose['q_rad']))
                if delta>.001:
                    curve=arm._build_transition(start,pose['q_rad'],max(3.,delta/a.speed),[a.speed]*6)
                    arm.start_reference_trajectory(curve,expected_start=start,command_mode='sdk_movej')
                    last_visible=time.monotonic();last_camera=0.
                    while True:
                        result=arm.advance_reference_trajectory()
                        if result is not None:break
                        if time.monotonic()-last_camera>.1:
                            # Nonblocking visibility check; capture waits only after arrival.
                            frames=camera.pipe.poll_for_frames();last_camera=time.monotonic()
                            if frames:
                                im=np.asanyarray(frames.get_color_frame().get_data());_,ids,_=camera.detector.detectMarkers(im)
                                if ids is not None and a.marker_id in ids.ravel():last_visible=time.monotonic()
                            if time.monotonic()-last_visible>1.:raise RuntimeError('运动中标记丢失超过1秒')
                        time.sleep(.003)
                time.sleep(.3);sample=capture(arm,model,camera,a,out,pose['index']);sample.update(index=pose['index'],role=pose['role']);samples.append(sample)
                save(out/'samples.json',{'samples':samples,'camera':camera.metadata,'marker_size_m':a.marker_size,'marker_id':a.marker_id,'timestamp_policy':'stationary frames bracketed by actual feedback; no hardware sync claim'})
                print(f"已保存 {len(samples)}/15 姿态：{pose['role']}",flush=True)
            result=solve(samples);result.update(camera_model='Intel RealSense D405',camera_serial=a.serial,source_samples=str(out/'samples.json'),marker_size_m=a.marker_size);save(out/'calibration_result.json',result);print(json.dumps(result,ensure_ascii=False,indent=2));print('未覆盖当前 hand_eye.json。结果：',out/'calibration_result.json')
        except BaseException as e:
            if a.execute:
                try:arm.stop()
                except Exception as stop_error:save(out/'stop_error.json',{'error':str(stop_error)})
            save(out/'failure.json',{'error':str(e),'completed_poses':len(samples),'automatic_return':False});raise
        finally:
            if camera is not None:camera.close()

if __name__=='__main__':main()
