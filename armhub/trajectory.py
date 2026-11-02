"""Continuous reference trajectories. Firmware tracking must be measured separately."""
import math
import numpy as np
from scipy.interpolate import make_interp_spline


def motion_config(config):
    robot=config['robot']
    for name in ['velocity_limits','acceleration_limits']:
        values=np.asarray(robot[name],dtype=float)
        if values.shape!=(6,) or not np.isfinite(values).all() or np.min(values)<=0:raise ValueError(f'Six positive {name} required')
    c=config.get('motion_smoothing',{})
    out={'stream_min_speed_rad_s':float(c.get('stream_min_speed_rad_s',.005)),
         'control_period_s':float(c.get('control_period_s',.05)),
         'max_jerk_rad_s3':np.asarray(c.get('max_jerk_rad_s3',[4.]*6),dtype=float),
         'tracking_replan_period_s':float(c.get('tracking_replan_period_s',.3)),
         'state_filter_tau_s':float(c.get('state_filter_tau_s',.12)),
         'max_tracking_error_rad':float(c.get('max_tracking_error_rad',.15)),
         'max_cycle_gap_s':float(c.get('max_cycle_gap_s',.2)),
         'feed_ramp_fraction':float(c.get('feed_ramp_fraction',.2))}
    if out['max_jerk_rad_s3'].shape!=(6,) or not np.isfinite(out['max_jerk_rad_s3']).all() or np.min(out['max_jerk_rad_s3'])<=0:
        raise ValueError('Six positive jerk limits required')
    if not all(math.isfinite(v) and v>0 for k,v in out.items() if k!='max_jerk_rad_s3') or not 0<out['feed_ramp_fraction']<.5:
        raise ValueError('Invalid motion smoothing configuration')
    if out['stream_min_speed_rad_s']>float(np.min(robot['velocity_limits'])):raise ValueError('Stream minimum speed exceeds velocity limit')
    return out


class JointMotionEstimate:
    def __init__(self,tau=.12):
        self.tau=tau;self.q=None;self.v=np.zeros(6);self.a=np.zeros(6);self.stamp=None
    def update(self,q,stamp):
        q=np.asarray(q,dtype=float)
        if self.q is None or stamp-self.stamp>.2:
            self.q=q.copy();self.stamp=stamp;self.v[:]=0;self.a[:]=0;return
        dt=stamp-self.stamp
        if dt<=0:return
        alpha=1-math.exp(-dt/self.tau)
        velocity=self.v+alpha*((q-self.q)/dt-self.v)
        self.a+=alpha*((velocity-self.v)/dt-self.a)
        self.v=velocity;self.q=q.copy();self.stamp=stamp


class JointTransition:
    """Ruckig trajectory from current reference q/v/a to a new IK goal."""
    def __init__(self,q,v,a,goal,config,minimum_duration=0.):
        from ruckig import Ruckig, InputParameter, Trajectory
        c=motion_config(config);robot=config['robot']
        self.input=InputParameter(6);self.trajectory=Trajectory(6)
        self.input.current_position=np.asarray(q).tolist()
        self.input.current_velocity=np.asarray(v).tolist();self.input.current_acceleration=np.asarray(a).tolist()
        self.input.target_position=np.asarray(goal).tolist();self.input.target_velocity=[0.]*6;self.input.target_acceleration=[0.]*6
        self.input.max_velocity=robot['velocity_limits'];self.input.max_acceleration=robot['acceleration_limits'];self.input.max_jerk=c['max_jerk_rad_s3'].tolist()
        if minimum_duration>0:self.input.minimum_duration=float(minimum_duration)
        result=Ruckig(6,c['control_period_s']).calculate(self.input,self.trajectory)
        if int(result)<0:raise ValueError(f'Ruckig rejected trajectory: {result}')
        self.duration=self.trajectory.duration
        lower=np.asarray(robot['joint_limits']['lower']);upper=np.asarray(robot['joint_limits']['upper'])
        extrema=self.trajectory.position_extrema
        if any(bound.min<lower[i]-1e-8 or bound.max>upper[i]+1e-8 for i,bound in enumerate(extrema)):
            raise ValueError('Smoothed transition would cross a joint position limit')
    def sample(self,t):
        return tuple(np.asarray(x) for x in self.trajectory.at_time(float(np.clip(t,0,self.duration))))


def progress(t,duration,ramp_fraction):
    """C3 distance profile: smooth speed ramp, constant cruise, smooth deceleration."""
    t=float(np.clip(t,0,duration));ramp=duration*ramp_fraction;peak=1/(duration-ramp)
    if ramp<=t<=duration-ramp:
        return peak*(ramp*.5+t-ramp),peak,0.,0.
    ending=t>duration-ramp;u=(duration-t if ending else t)/ramp
    s=10*u**3-15*u**4+6*u**5
    ds=30*u**2-60*u**3+30*u**4
    dds=60*u-180*u**2+120*u**3
    integral=2.5*u**4-3*u**5+u**6
    return ((1-peak*ramp*integral) if ending else peak*ramp*integral,
            peak*s,(-1 if ending else 1)*peak/ramp*ds,peak/ramp**2*dds)


class ContinuousFeed:
    def __init__(self,spec):
        self.spec=spec;self.duration=float(spec['duration_s']);self.ramp=float(spec['ramp_fraction'])
        self.spline=make_interp_spline(np.asarray(spec['path_fraction']),np.asarray(spec['joint_path_rad']),k=5)
        self.derivatives=[self.spline.derivative(i) for i in [1,2,3]]
    def sample(self,t):
        u,v,a,j=progress(t,self.duration,self.ramp)
        q=self.spline(u);d1,d2,d3=(f(u) for f in self.derivatives)
        return q,d1*v,d2*v*v+d1*a
    def jerk(self,t):
        u,v,a,j=progress(t,self.duration,self.ramp);d1,d2,d3=(f(u) for f in self.derivatives)
        return d3*v**3+3*d2*v*a+d1*j
    def tcp_position(self,t):
        u=progress(t,self.duration,self.ramp)[0]
        return np.asarray(self.spec['start_position_m'])-np.asarray(self.spec['normal_base'])*self.spec['distance_m']*u


def retime_feed(spec,config):
    c=motion_config(config);trajectory=ContinuousFeed(spec)
    times=np.linspace(0,trajectory.duration,1201)
    sampled=[trajectory.sample(t) for t in times]
    q=np.array([x[0] for x in sampled]);v=np.array([x[1] for x in sampled]);a=np.array([x[2] for x in sampled]);j=np.array([trajectory.jerk(t) for t in times])
    limits=config['robot'];lower=np.asarray(limits['joint_limits']['lower']);upper=np.asarray(limits['joint_limits']['upper'])
    if np.any(q<lower-1e-8) or np.any(q>upper+1e-8):raise ValueError('Interpolated feed crosses a joint position limit')
    factor=max(1.,float(np.max(np.abs(v)/limits['velocity_limits'])),float(np.sqrt(np.max(np.abs(a)/limits['acceleration_limits']))),float(np.cbrt(np.max(np.abs(j)/c['max_jerk_rad_s3']))))
    scale=max(1.,factor*1.02)
    spec['duration_s']*=scale
    spec['reference_peak_velocity_rad_s']=(np.max(np.abs(v),axis=0)/scale).tolist()
    spec['reference_peak_acceleration_rad_s2']=(np.max(np.abs(a),axis=0)/scale**2).tolist()
    spec['reference_peak_jerk_rad_s3']=(np.max(np.abs(j),axis=0)/scale**3).tolist()
    spec['reference_cruise_speed_m_s']=spec['distance_m']/(spec['duration_s']*(1-spec['ramp_fraction']))
    spec['nominal_control_period_s']=c['control_period_s']
    return spec
