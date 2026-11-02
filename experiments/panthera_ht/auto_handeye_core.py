"""Pure stationary eye-in-hand planning and held-out validation."""
import itertools
import numpy as np
import cv2
from scipy.spatial.transform import Rotation


def poses(q, lower, upper, amplitude_deg=10.):
    q=np.asarray(q,dtype=float);lo=np.asarray(lower);hi=np.asarray(upper)
    if q.shape!=(6,) or not np.isfinite(q).all() or not 6<=amplitude_deg<=15:
        raise ValueError('需要六轴有限角度、6..15 度采样幅度')
    if np.any(q<lo) or np.any(q>hi):
        raise ValueError('当前关节在模型限位外，请先调整到限位内的展开姿态；不自动移动到准备位')
    offsets=[(0,0,0)]+[tuple(sign if j==axis else 0 for j in range(3)) for axis in range(3) for sign in (-1,1)]+list(itertools.product((-1,1),repeat=3))
    candidates=[q+np.r_[np.zeros(3),v]*np.deg2rad(amplitude_deg) for v in offsets]
    if any(np.any(x<lo) or np.any(x>hi) for x in candidates):
        raise ValueError('腕部采样包络跨越限位，请手动调整起点或减小 --amplitude-deg')
    ordered=[];current=q
    while candidates:
        i=min(range(len(candidates)),key=lambda i:np.linalg.norm(candidates[i]-current));current=candidates.pop(i);ordered.append(current)
    return [{'index':i,'q_rad':x.tolist(),'role':'validation' if i in (4,9,14) else 'train'} for i,x in enumerate(ordered)]


def scatter(transforms, reference=None):
    ts=np.asarray(transforms)
    if reference is None:
        reference=np.eye(4);reference[:3,:3]=Rotation.from_matrix(ts[:,:3,:3]).mean().as_matrix();reference[:3,3]=ts[:,:3,3].mean(axis=0)
    d=np.linalg.norm(ts[:,:3,3]-reference[:3,3],axis=1)*1000
    a=np.rad2deg(Rotation.from_matrix(np.array([reference[:3,:3].T@x[:3,:3] for x in ts])).magnitude())
    return {'position_rms_mm':float(np.sqrt(np.mean(d*d))),'position_max_mm':float(d.max()),'angle_rms_deg':float(np.sqrt(np.mean(a*a))),'angle_max_deg':float(a.max())},reference


def excitation(h):
    h=np.asarray(h);r=Rotation.from_matrix(np.array([h[0,:3,:3].T@x[:3,:3] for x in h[1:]])).as_rotvec()
    singular=np.linalg.svd(r,compute_uv=False)
    if len(singular)<2 or singular[1]<.08 or singular[1]/max(singular[0],1e-9)<.05:
        raise ValueError('旋转激励不足，腕部轴可能接近平行；请改变展开起点后重新预览')
    return singular.tolist()


def solve(samples):
    train=[s for s in samples if s['role']=='train'];val=[s for s in samples if s['role']=='validation']
    if len(train)<12 or len(val)<3:raise ValueError('至少需要12组求解和3组留出验证')
    h=[np.array(s['T_base_tcp']) for s in train];v=[np.array(s['T_camera_marker']) for s in train];ex=excitation(h)
    candidates=[];errors={}
    for name,method in [('PARK',cv2.CALIB_HAND_EYE_PARK),('TSAI',cv2.CALIB_HAND_EYE_TSAI),('HORAUD',cv2.CALIB_HAND_EYE_HORAUD)]:
        try:
            r,t=cv2.calibrateHandEye([x[:3,:3] for x in h],[x[:3,3] for x in h],[x[:3,:3] for x in v],[x[:3,3] for x in v],method=method)
            x=np.eye(4);x[:3,:3]=r;x[:3,3]=t.ravel()
            if not np.isfinite(x).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1.) or np.linalg.norm(t)>.6:raise ValueError('非法求解结果')
            metrics,ref=scatter([a@x@b for a,b in zip(h,v)]);candidates.append((metrics['position_rms_mm']+metrics['angle_rms_deg'],name,x,metrics,ref))
        except (ValueError,cv2.error) as e:errors[name]=str(e)
    if not candidates:raise ValueError('所有求解方法失败: '+str(errors))
    _,name,x,metrics,ref=min(candidates,key=lambda c:c[0]);check,_=scatter([np.array(s['T_base_tcp'])@x@np.array(s['T_camera_marker']) for s in val],ref)
    passed=check['position_max_mm']<=3 and check['angle_max_deg']<=2
    return {'T_tcp_camera':x.tolist(),'xyz_mm':(x[:3,3]*1000).tolist(),'rpy_xyz_deg':Rotation.from_matrix(x[:3,:3]).as_euler('xyz',degrees=True).tolist(),'parent_frame':'gripper_center','child_frame':'handeye_color_optical','translation_unit':'m','status':'calibrated_pending_independent_measurement' if passed else 'validation_failed','method':name,'train_count':len(train),'validation_count':len(val),'train':metrics,'validation':dict(check,passed=bool(passed)),'rotation_excitation_singular_values':ex,'solver_errors':errors,'independent_absolute_measurement_done':False}
