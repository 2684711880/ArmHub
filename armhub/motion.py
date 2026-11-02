"""静止起点到静止终点的 Ruckig 参考；固件跟踪性能另行测量。"""
import math


def speed_limits(config, requested=None):
    physical=tuple(float(v) for v in config['velocity_limits_rad_s'])
    if len(physical)!=6 or any(not math.isfinite(v) or v<=0 for v in physical):
        raise ValueError('机型需要六轴有限正速度上限')
    if requested is None:return physical
    values=(float(requested),)*6 if isinstance(requested,(int,float)) else tuple(float(v) for v in requested)
    if len(values)!=6 or any(not math.isfinite(v) or v<=0 for v in values):
        raise ValueError('参考速度需要有限正数或六轴上限')
    return tuple(min(a,b) for a,b in zip(physical,values))


class JointTransition:
    def __init__(self, start, goal, config, *, speed_limit, minimum_duration):
        from ruckig import Ruckig, InputParameter, Trajectory
        settings = config['motion_smoothing']
        self.period = float(settings['control_period_s'])
        self.speed_limits = speed_limits(config,speed_limit)
        self.speed_limit = max(self.speed_limits)
        n = 6
        vectors = [start, goal, settings['acceleration_limits_rad_s2'], settings['jerk_limits_rad_s3']]
        if any(len(v) != n or not all(math.isfinite(x) for x in v) for v in vectors):
            raise ValueError('平滑轨迹需要六轴有限参数')
        if (not math.isfinite(self.period) or self.period <= 0 or
                not math.isfinite(minimum_duration) or minimum_duration <= 0 or
                min(settings['acceleration_limits_rad_s2']) <= 0 or min(settings['jerk_limits_rad_s3']) <= 0):
            raise ValueError('平滑轨迹周期、速度、加速度、jerk 和时长必须有效')
        if any(not low <= q <= high for q,low,high in zip(goal,config['lower_rad'],config['upper_rad'])):
            raise ValueError('平滑轨迹目标超出物理限位')
        if any(not low-.01 <= q <= high+.01 for q,low,high in zip(start,config['lower_rad'],config['upper_rad'])):
            raise ValueError('平滑轨迹起点超出反馈容许范围')
        inputs = InputParameter(n)
        inputs.current_position = list(start)
        inputs.current_velocity = [0.]*n
        inputs.current_acceleration = [0.]*n
        inputs.target_position = list(goal)
        inputs.target_velocity = [0.]*n
        inputs.target_acceleration = [0.]*n
        inputs.max_velocity = list(self.speed_limits)
        inputs.max_acceleration = settings['acceleration_limits_rad_s2']
        inputs.max_jerk = settings['jerk_limits_rad_s3']
        inputs.minimum_duration = minimum_duration
        self.trajectory = Trajectory(n)
        status = Ruckig(n, self.period).calculate(inputs, self.trajectory)
        if int(status) < 0:
            raise ValueError(f'Ruckig 轨迹生成失败：{status}')
        self.duration = float(self.trajectory.duration)
        self.start, self.goal = tuple(start), tuple(goal)
        self.acceleration_limits = list(settings['acceleration_limits_rad_s2'])
        self.jerk_limits = list(settings['jerk_limits_rad_s3'])
        # 编码器零位噪声只允许从实测起点向模型内收敛，不能向更外侧运动。
        for bound,low,high,q in zip(self.trajectory.position_extrema,config['lower_rad'],config['upper_rad'],start):
            if bound.min < min(low,q)-1e-8 or bound.max > max(high,q)+1e-8:
                raise ValueError('Ruckig 轨迹越过关节限位或实测起点边界')

    def sample(self, elapsed):
        t = min(self.duration, max(0., float(elapsed)))
        if t >= self.duration:
            return self.goal, (0.,)*6, (0.,)*6
        return tuple(tuple(float(v) for v in values) for values in self.trajectory.at_time(t))

    def collision_samples(self, step_rad):
        dt = min(self.period, step_rad/self.speed_limit)
        segments = max(1, math.ceil(self.duration/dt))
        return (self.sample(self.duration*i/segments)[0] for i in range(segments+1))

    def summary(self):
        return {'execution_backend':'ruckig_stream', 'duration_s':self.duration,
                'reference_period_s':self.period,
                'reference_max_velocity_rad_s':list(self.speed_limits),
                'reference_max_acceleration_rad_s2':self.acceleration_limits,
                'reference_max_jerk_rad_s3':self.jerk_limits,
                'planned_reference_samples':math.ceil(self.duration/self.period)+1,
                'reference_start_velocity_rad_s':[0.]*6,
                'reference_end_velocity_rad_s':[0.]*6}
