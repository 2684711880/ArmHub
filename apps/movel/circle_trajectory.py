"""Circle geometry and start/stop timing are independent of each other.

No hardware calls. A sparse accurate IK path avoids amplifying dense IK stopping
noise; a C3 progress law supplies zero endpoint velocity/acceleration/jerk.
"""
import math
import numpy as np
from scipy.interpolate import make_interp_spline

from armhub.trajectory import progress


class TimedCircleCurve:
    def __init__(self, fractions, qs, duration, config, ramp_fraction=.2):
        self.fractions=np.asarray(fractions,dtype=float)
        qs=np.asarray(qs,dtype=float)
        if (qs.shape!=(len(self.fractions),6) or len(qs)<6 or
            not np.isfinite(qs).all() or not np.isfinite(self.fractions).all() or
            np.any(np.diff(self.fractions)<=0) or
            self.fractions[0]!=0 or self.fractions[-1]!=1):
            raise ValueError('圆形路径需要递增的 0–1 参数与六轴有限关节角')
        if not math.isfinite(duration) or duration<=0 or not 0<ramp_fraction<.5:
            raise ValueError('圆形时长和启停比例无效')
        self.ramp_fraction=ramp_fraction
        self.speed_limits=tuple(config['velocity_limits_rad_s'])
        self.spline=make_interp_spline(self.fractions,qs,k=5)
        # Derivatives here are geometric derivatives dq/du, never endpoint
        # rest conditions. Time scaling introduces the rest conditions later.
        self.duration=1.
        grid=np.unique(np.r_[np.linspace(0,1,8001),ramp_fraction,1-ramp_fraction])
        unit=np.array([progress(t,1.,ramp_fraction) for t in grid])
        u,du,ddu,dddu=unit.T
        d1,d2,d3=[self.spline(u,nu=n) for n in (1,2,3)]
        v=d1*du[:,None]
        a=d2*du[:,None]**2+d1*ddu[:,None]
        j=d3*du[:,None]**3+3*d2*(du*ddu)[:,None]+d1*dddu[:,None]
        peaks=[np.max(np.abs(values),axis=0) for values in (v,a,j)]
        settings=config['motion_smoothing']
        bounds=[float(np.max(peaks[0]/self.speed_limits)),
                float(np.sqrt(np.max(peaks[1]/settings['acceleration_limits_rad_s2']))),
                float(np.cbrt(np.max(peaks[2]/settings['jerk_limits_rad_s3'])))]
        self.duration=max(duration,*bounds)*1.03
        self.timing_report=dict(
            parameterization='circle_geometry_plus_C3_progress',
            requested_duration_s=float(duration),velocity_required_duration_s=bounds[0],
            acceleration_required_duration_s=bounds[1],jerk_required_duration_s=bounds[2],
            duration_s=self.duration,ramp_fraction=ramp_fraction,ik_stations=len(qs),
            peak_velocity_rad_s=(peaks[0]/self.duration).tolist(),
            peak_acceleration_rad_s2=(peaks[1]/self.duration**2).tolist(),
            peak_jerk_rad_s3=(peaks[2]/self.duration**3).tolist())

    def sample(self,t):
        u,v,a,_=progress(t,self.duration,self.ramp_fraction)
        d1=self.spline(u,nu=1)
        return self.spline(u),d1*v,self.spline(u,nu=2)*v*v+d1*a

    def jerk(self,t):
        u,v,a,j=progress(t,self.duration,self.ramp_fraction)
        return self.spline(u,nu=3)*v**3+3*self.spline(u,nu=2)*v*a+self.spline(u,nu=1)*j


def plan_circle_curve(robot,waypoints,start,duration):
    """Fit accurate IK stations on a complete circle, with no serial connection."""
    from Panthera_lib import Panthera
    positions=np.asarray([w['position'] for w in waypoints],dtype=float)
    if len(positions)<13 or np.linalg.norm(positions[-1]-positions[0])>1e-7:
        raise ValueError('圆轨迹必须是一条完整闭合路径')
    # The generator supplies evenly spaced angular samples. 90 intervals have
    # enough geometric resolution, while removing sub-millimetre IK jitter.
    indices=np.unique(np.linspace(0,len(positions)-1,min(91,len(positions)),dtype=int))
    fractions=indices/(len(positions)-1)
    qs=[np.asarray(start,dtype=float).copy()]
    lower,upper=robot.joint_limits['lower'],robot.joint_limits['upper']
    for index in indices[1:]:
        w=waypoints[index]
        q=Panthera.inverse_kinematics(robot.sdk,w['position'],w['rotation'],init_q=qs[-1],
                                    multi_init=False,eps=1e-8,max_iter=1500)
        if q is None:raise ValueError(f'圆形精确 IK 失败：站点 {index}')
        q=np.asarray(q,dtype=float)
        if (q.shape!=(6,) or not np.isfinite(q).all() or np.any(q<lower) or np.any(q>upper) or
            np.max(np.abs(q-qs[-1]))>robot.sdk.jump_threshold):
            raise ValueError(f'圆形精确 IK 限位或关节跳变：站点 {index}')
        qs.append(q)
    curve=TimedCircleCurve(fractions,qs,duration,robot.arm.config)
    # Check interpolated FK against the original circle rather than accepting
    # a faster joint curve just because its dynamic bounds happen to pass.
    centre=positions[:-1].mean(axis=0)
    radius=float(np.linalg.norm(positions[0]-centre))
    normal_axis=int(np.argmin(np.ptp(positions,axis=0)))
    axes=[i for i in range(3) if i!=normal_axis]
    start_error=float(np.linalg.norm(np.asarray(robot.forward_kinematics(start)['position'])-positions[0]))
    allowed=max(.00005,start_error*1.2+.00001)
    max_error=0.;max_angle=0.
    rotation=np.asarray(waypoints[0]['rotation'])
    for u in np.linspace(0,1,1001):
        fk=robot.forward_kinematics(curve.spline(u))
        delta=np.asarray(fk['position'])-centre
        error=math.hypot(np.linalg.norm(delta[axes])-radius,delta[normal_axis])
        max_error=max(max_error,error)
        # Matrix trace gives the relative rotation angle in radians.
        angle=math.acos(float(np.clip((np.trace(rotation.T@fk['rotation'])-1)/2,-1,1)))
        max_angle=max(max_angle,angle)
    if max_error>allowed or max_angle>.01:
        raise ValueError(f'圆形拟合复核失败：路径误差 {max_error*1000:.3f} mm，姿态误差 {max_angle:.5f} rad')
    curve.timing_report.update(planning_circle_error_max_mm=max_error*1000,
                               planning_orientation_error_max_rad=max_angle,
                               allowed_circle_error_mm=allowed*1000)
    return curve
