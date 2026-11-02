"""One SDK owner, with the existing ArmHub feedback/hold/reference contracts."""
import math
import time
import numpy as np
from armhub import create_arm
from armhub.motion import JointTransition


class HardwareSession:
    def __init__(self):
        self.arm = create_arm('panthera_ht', 'hardware')
        self.mode = 'readonly'
        self.last_send = None
        self.gripper_target = None

    def connect(self):
        self.arm.connect()
        # Build models once, before any continuous control starts.
        self.kin = self.arm._kinematics()
        self.kin.prepare_collision_geometry()
        self.gravity = self.arm._gravity_model()
        sample = self.read()
        self.gripper_target = sample['gripper']
        return sample

    def read(self):
        settings = self.arm.config['gravity_compensation']
        states = self.arm.read_feedback(query_period_s=settings['query_period_s'],
                                       timeout_s=settings['feedback_timeout_s'], record=False)
        q = [s.position for s in states[:6]]
        return dict(q=q, dq=[s.velocity for s in states[:6]], gripper=states[6].position,
                    gripper_velocity=states[6].velocity, pose=self.kin.fk(q),
                    feedback_age_s=max(time.time()-s.time for s in states),
                    feedback_time=[s.time for s in states],
                    feedback_counters=[s.num for s in states], states=states,
                    monotonic_s=time.monotonic(), source='hardware')

    def teach(self):
        self.arm.enable()
        self.arm._gravity_active = True
        self.last_send = None
        self.mode = 'teach'

    def teach_tick(self, sample):
        settings = self.arm.config['gravity_compensation']
        if max(abs(v) for v in sample['dq']) > settings['max_velocity_rad_s']:
            raise RuntimeError('拖动速度超过既有重力补偿保护阈值')
        now = time.monotonic()
        if self.last_send is not None and now-self.last_send > settings['max_command_gap_s']:
            raise RuntimeError('拖动补偿循环超时，停止继续下发')
        raw = tuple(float(value) for value in self.gravity(sample['q']))
        if len(raw) != 6 or not all(math.isfinite(v) for v in raw):
            raise RuntimeError('重力模型输出无效')
        caps = settings['torque_caps_nm']
        if len(caps)!=6 or any(not math.isfinite(v) or not 0<v<=allowed
                              for v,allowed in zip(caps,self.arm.config['torque_caps_nm'])):
            raise ValueError('拖动力矩上限不符合既有机型限制')
        torques = [max(-cap, min(cap,value)) for value,cap in zip(raw,caps)]
        self.arm._moving = True
        self.arm._send_gravity(torques)
        self.last_send = time.monotonic()

    def gripper(self, target, speed, torque, sample):
        c = self.arm.config
        if not c['gripper_bounds_rad'][0] <= target <= c['gripper_bounds_rad'][1]:
            raise ValueError('夹爪目标超出机型范围')
        if not 0 < torque <= c['gripper_torque_cap_nm'] or not 0 < speed <= .5:
            raise ValueError('夹爪速度或力度超出允许范围')
        if self.mode == 'teach':
            # Different native packet mode: send only gripper in its own frame.
            # The patched SDK puts unchanged arm slots at the sentinel value.
            self.arm._motors[6].pos_vel_MAXtqe(target, speed, torque)
            self.arm._driver.motor_send_cmd()
        else:
            self.arm._send(sample['q'], [c['hold_speed_rad_s']]*6, target,
                           gripper_speed=speed, gripper_torque=torque)
        self.gripper_target = target
        self.arm._moving = True

    def hold(self):
        self.arm._gravity_active = False
        self.arm.request_hold()
        self.mode = 'holding'
        self.last_send = None

    def poll_hold(self, sample):
        done = self.arm.poll_hold(sample['states'])
        if done:
            self.mode = 'hold'
            self.gripper_target = sample['gripper']
        return done

    def preview(self, sample, prepared, scale, minimum_duration, z_min):
        config = self.arm.config
        curve = JointTransition(sample['q'], prepared['q'], config,
             speed_limit=[v*scale for v in config['velocity_limits_rad_s']],
             minimum_duration=minimum_duration)
        collision = self.kin.check_trajectory(curve, config['cartesian']['collision_step_rad'])
        zs = [self.kin.fk(curve.sample(t)[0])['position_m'][2]
              for t in np.linspace(0,curve.duration,max(3,math.ceil(curve.duration/.05)))]
        if z_min is not None and min(zs) < z_min:
            raise ValueError('准备位路径越过设定的末端参考点 Z 下限')
        return dict(curve=curve, start=list(sample['q']), duration=curve.duration,
                    target=list(prepared['q']), minimum_tcp_z_m=min(zs), collision=collision)

    def start_return(self, plan, guard):
        self.arm.enable()
        self.arm.start_reference_trajectory(plan['curve'], expected_start=plan['start'], command_guard=guard)
        self.mode = 'returning'

    def advance_return(self, sample):
        done = self.arm.advance_reference_trajectory(sample['states']) is not None
        if done:
            self.mode = 'hold'
        return done

    def close(self):
        self.arm._gravity_active = False
        self.arm.close()


class DemoSession:
    """Synthetic UI/data fixture, never a physical or dynamics validation."""
    def __init__(self):
        self.q = np.array([0., .6, 1., 0., .5, 0.])
        self.mode = 'readonly'
        self.gripper_target = self.grip = 1.6
        self.started = time.monotonic()

    def connect(self):
        return self.read()

    def read(self):
        now = time.monotonic()
        if self.mode == 'teach':
            self.q[0] = .025*math.sin((now-self.started)*.8)
        self.grip += max(-.04,min(.04,self.gripper_target-self.grip))
        return dict(q=self.q.tolist(),dq=[0.]*6,gripper=self.grip,gripper_velocity=0.,
                    pose=dict(position_m=(.25+self.q[0],0.,.20),quaternion_xyzw=(0.,1.,0.,0.),
                              frame_id='base_link',tcp_frame='gripper_center'),
                    feedback_age_s=0.,feedback_time=[time.time()]*7,
                    feedback_counters=[int(now*1000)]*7,monotonic_s=now,source='demo')

    def teach(self): self.mode='teach'
    def teach_tick(self,sample): pass
    def gripper(self,target,speed,torque,sample): self.gripper_target=target
    def hold(self): self.mode='holding'
    def poll_hold(self,sample):
        self.mode='hold'; self.gripper_target=sample['gripper']; return True
    def preview(self,sample,prepared,scale,minimum_duration,z_min):
        if z_min is not None and .2<z_min: raise ValueError('合成路径低于 Z 下限')
        return dict(start=sample['q'],target=prepared['q'],duration=.5,
                    minimum_tcp_z_m=.2,collision={'scope':'synthetic_only'})
    def start_return(self,plan,guard):
        self.plan=plan;self.return_start=time.monotonic();self.guard=guard;self.mode='returning'
    def advance_return(self,sample):
        if not self.guard(): raise RuntimeError('演示路径已取消')
        u=min(1.,(time.monotonic()-self.return_start)/self.plan['duration'])
        self.q=np.array(self.plan['start'])*(1-u)+np.array(self.plan['target'])*u
        if u>=1:self.mode='hold'
        return u>=1
    def close(self): pass
