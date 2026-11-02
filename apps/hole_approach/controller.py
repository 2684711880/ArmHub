"""Single motor-owner state machine. No GUI, camera waits, IK, or arrival waits."""
import copy
import math
import time
from collections import deque
import numpy as np
from armhub.tools import tool_transform
from perception.tracking.hole_estimator import HolePositionEstimator
from armhub.trajectory import motion_config,JointMotionEstimate,JointTransition,ContinuousFeed
from .reporting import write_report, show_plan


class Controller:
    def __init__(self, robot, args, calibration, status, submit, epoch=lambda: 0):
        self.robot, self.args, self.calibration = robot, args, np.asarray(calibration)
        self.calibration_status, self.submit, self.current_epoch = status, submit, epoch
        self.epoch = epoch(); self.holding = False
        robot.command_guard = lambda: self.holding or self.current_epoch() == self.epoch
        self.pose_history=deque(maxlen=20)
        self.history = deque(maxlen=400); self.target_history=deque(maxlen=128); self.states = None
        self.observation = None; self.frame_stamp = None; self.good = 0
        self.vision = 'waiting'; self.tracking = False; self.state = 'IDLE'
        self.preview = None; self.pending = None; self.request_id = 0
        self.active = None; self.sequence = []; self.report = None; self.anchor = None
        self.message = 'p: preview; g: two stages; t: track; space: hold'
        self.want_home=False
        self.want_preview = False; self.want_gripper = None; self.auto_go = False
        self.stationary_since = None; self.stationary_q = None; self.go_snapshot = None
        config=getattr(robot,'config',{}).get('hole_estimator',{})
        self.estimator=HolePositionEstimator(**config); self.rejection_reason=None
        self.motion_config=getattr(robot,'config',None)
        if not self.motion_config:
            import yaml
            from pathlib import Path
            self.motion_config=yaml.safe_load(Path(args.config).read_text())
        self.smoothing=motion_config(self.motion_config)
        self.joint_estimate=JointMotionEstimate(self.smoothing['state_filter_tau_s'])
        self.reference_state=None; self.last_reference_send=None; self.stream_intervals=deque(maxlen=3000); self.last_replan=-float('inf')

    def feedback(self):
        self.states = self.robot.feedback()
        q = np.array([s.position for s in self.states[:6]])
        stamps = [getattr(s, 'time', time.time()) for s in self.states[:6]]
        sample_time=float(np.mean(stamps))
        self.history.append((sample_time, q.copy()))
        self.joint_estimate.update(q,sample_time)
        fk=self.robot.forward_kinematics(q)
        self.current_tcp_base=(np.asarray(fk['transform'])@tool_transform(self.args))[:3,3].tolist()
        self.pose_history.append({'stamp':sample_time,'T_base_camera':(np.asarray(fk['transform'])@self.calibration).tolist()})
        if self.stationary_q is None or np.max(np.abs(q-self.stationary_q)) > .003:
            self.stationary_q = q.copy(); self.stationary_since = time.monotonic()
        return q

    def hold(self):
        self.holding = True
        try:
            self.robot.hold()
        finally:
            self.holding = False
        self.active=None;self.sequence=[];self.reference_state=None;self.last_reference_send=None

    def cancel(self, hold=True):
        moving = self.active is not None or self.tracking
        self.request_id += 1; self.pending = None; self.want_preview = False; self.want_gripper = None; self.want_home=False; self.auto_go = False; self.go_snapshot = None
        self.tracking = False; self.preview = None; self.sequence = []; self.active = None; self.anchor = None
        if hold and moving:
            self.hold()
        self.active = None; self.state = 'IDLE'

    def command(self, key, epoch, click_time=None, settings=None):
        if epoch != self.current_epoch():
            return
        if key in ['g','t','r','c','o'] and not self.robot.allow_motion:
            raise RuntimeError('当前只读预览；启动时添加 --execute 才允许运动')
        was_tracking = self.tracking
        clicked=time.time() if click_time is None else click_time
        selected=None
        if key=='g':
            for stamp,snapshot in reversed(self.target_history):
                if stamp<=clicked:
                    if snapshot is not None and 0<=clicked-stamp<=.7 and snapshot['confirmed_samples']>=3:selected=copy.deepcopy(snapshot)
                    break
        self.epoch = epoch
        self.cancel()
        if settings is not None:
            from armhub.tools import validate_settings
            for name,value in validate_settings(settings).items():setattr(self.args,name,value)
        if key == 'calibration':
            if settings is None:raise ValueError('Missing calibration settings')
            self.message='TCP / offset applied; tracking OFF. Press p to preview, g to execute'
        elif key in ['q', ' ']:
            self.hold()
            self.message = 'Position hold requested; physical stop not confirmed'
        elif key == 't':
            self.tracking = not was_tracking
            if self.active is None:self.state = 'SEARCH' if self.tracking else 'IDLE'
            self.good = 0; self.anchor = None
            if self.tracking:self.estimator.reset(); self.observation=None
            self.message = 'Tracking ON: depth acquisition / home recovery' if self.tracking else 'Tracking OFF'
        elif key == 'p':
            self.want_preview = True; self.state = 'PREVIEW_WAIT'
            self.message = 'Waiting for stationary feedback and three fresh depth observations'
        elif key == 'g':
            if selected is None:raise ValueError('GO：当前没有连续三帧确认的新鲜洞口，请看到洞口后再点击。')
            self.go_snapshot=selected; self.go_snapshot['click_time']=clicked
            self.want_preview = True; self.auto_go = True; self.state = 'GO_PLAN_WAIT'
            self.message = 'GO: click-time target locked; planning both stages'
        elif key == 'r':
            self.want_home=True;self.state='HOME_WAIT'
            if not getattr(self.robot,'hold_pending',False):self.start_home()
        elif key in ['c', 'o']:
            self.want_gripper = key; self.state = 'GRIPPER_WAIT'
            self.message = 'Waiting for stationary arm before gripper command'

    def start_home(self):
        q=self.robot.positions()
        duration=max(self.args.duration,float(np.max(np.abs(q)/self.robot.velocity_limits)))
        self.want_home=False
        self.report={'action':'home','motion_sent':False}
        self.send({'target_joints_rad':self.motion_config['home_joints_rad'],'duration_s':duration},'HOME')

    def observe(self, packet):
        stamp = packet['stamp']
        if stamp == self.frame_stamp:
            return
        self.frame_stamp = stamp
        if packet.get('camera_error') or not 0<=time.time()-stamp<=.7:
            self.target_history.append((stamp,None))
            self.vision = 'unavailable'; self.good = 0; self.observation = None
            return
        if packet.get('observation') is None:
            self.target_history.append((stamp,None))
            self.vision = 'lost'; self.good = 0
            return
        if not self.history:
            return
        nearest = min(self.history, key=lambda item: abs(item[0]-stamp))
        if abs(nearest[0]-stamp) > .12:
            self.target_history.append((stamp,None))
            self.vision = 'unsynchronized'; self.good = 0; self.observation = None
            return
        r = packet['observation']; capture_q = nearest[1]
        transform = np.asarray(self.robot.forward_kinematics(capture_q)['transform']) @ self.calibration
        center = (transform @ np.r_[r['center'], 1])[:3]
        normal = transform[:3, :3] @ r['normal']; normal /= np.linalg.norm(normal)
        previous = self.observation
        estimator_fresh=self.estimator.stamp is not None and stamp-self.estimator.stamp<=self.estimator.max_prediction_s
        if estimator_fresh and previous is not None and normal@previous['world_normal']<math.cos(math.radians(self.estimator.normal_gate_deg)):
            self.target_history.append((stamp,None))
            self.vision='lost'; self.good=0; self.rejection_reason='Normal outlier'; return
        if not self.estimator.update(center,stamp,float(r.get('plane_rms',0.))):
            self.target_history.append((stamp,None))
            self.vision='lost'; self.good=0; self.rejection_reason='Position innovation outlier'; return
        self.rejection_reason=None
        raw_center=center.copy(); center=self.estimator.estimate(stamp)['position']
        optical_center=transform[:3,:3].T@(center-transform[:3,3])
        # Three observations of the same hole, rather than isolated depth defects.
        consistent = previous is not None and np.linalg.norm(center-previous['world_center']) < .03 and normal @ previous['world_normal'] > math.cos(math.radians(10))
        self.good = self.good+1 if consistent else 1
        self.observation = {'center': optical_center.tolist(), 'raw_world_center': raw_center, 'normal': np.asarray(r['normal']).tolist(),
                            'capture_q': capture_q.copy(), 'world_center': center, 'world_normal': normal, 'stamp': stamp}
        self.vision = 'visible'
        snapshot=copy.deepcopy(self.observation); estimate=self.estimator.estimate(stamp)
        snapshot['estimate']={key:(value.tolist() if isinstance(value,np.ndarray) else value) for key,value in estimate.items()}
        snapshot['estimate']['mode']='measured_filtered'; snapshot['confirmed_samples']=self.good
        self.target_history.append((stamp,snapshot))
        if self.tracking and self.state in ['RECOVER', 'SEARCH_HOME']:
            self.hold(); self.pending = None; self.request_id += 1
            self.anchor = None; self.state = 'REACQUIRE'; self.message = 'Hole found: recovery held; confirming three frames'

    def prediction_usable(self):
        if not self.tracking or self.observation is None or self.vision!='lost':
            return False
        if self.frame_stamp is None or not 0<=time.time()-self.frame_stamp<=.7:
            return False
        estimate=self.estimator.estimate(time.time())
        return estimate is not None and estimate['prediction_allowed']

    def fresh(self):
        measured=self.vision=='visible' and self.observation is not None and 0<=time.time()-self.observation['stamp']<=.7
        return measured or self.prediction_usable()

    def estimated_observation(self):
        if not self.fresh():return None
        r=dict(self.observation); estimate=self.estimator.estimate(time.time())
        transform=np.asarray(self.robot.forward_kinematics(r['capture_q'])['transform'])@self.calibration
        r['world_center']=estimate['position']
        r['center']=(transform[:3,:3].T@(estimate['position']-transform[:3,3])).tolist()
        r['estimate']={key:(value.tolist() if isinstance(value,np.ndarray) else value) for key,value in estimate.items()}
        r['estimate']['mode']='predicted' if self.vision=='lost' else 'measured_filtered'
        return r

    def same_target(self, plan):
        r=self.estimated_observation()
        tolerance=self.estimator.prediction_replan_m if self.prediction_usable() else .01
        return r is not None and np.linalg.norm(r['world_center']-plan['hole_center_base_m'])<tolerance and r['world_normal']@plan['outward_normal_base']>math.cos(math.radians(3))

    def request(self, preview=False):
        locked=self.auto_go and self.go_snapshot is not None
        if not locked and (not self.fresh() or (self.good < 3 and not self.prediction_usable())):
            return
        self.request_id += 1
        self.pending = {'id': self.request_id, 'epoch': self.epoch, 'preview': preview, 'submitted': time.monotonic(), 'locked_target':locked}
        r = self.go_snapshot if locked else self.estimated_observation()
        task = dict(self.pending, q_start=self.robot.positions().tolist(), q_capture=r['capture_q'].tolist(),
                    center=r['center'], normal=r['normal'], hole_estimate=r['estimate'], calibration=self.calibration.tolist(),
                    calibration_status=self.calibration_status, options=vars(self.args).copy())
        if locked:task['go_click_time']=r['click_time']
        self.submit(task); self.last_replan=time.monotonic()
        if self.active is None:self.state = 'PLANNING'
        self.message = 'IK worker solving configured alignment / feed; buttons remain active'

    def planned(self, result):
        if self.pending is None or result['id'] != self.pending['id'] or result['epoch'] != self.current_epoch():
            return
        preview = self.pending['preview']; locked=self.pending.get('locked_target',False); self.pending = None
        if result.get('error'):
            self.cancel(); self.message = 'IK failed: '+result['error']; return
        if self.state=='FEED_CONTINUOUS':
            return  # 完成当前前送后再使用新观测规划，旧异步结果不能打断前送
        plan = result['plan']; plan['wall_time'] = time.time()
        if not locked and not self.same_target(plan):
            if self.active is None:self.state = 'SEARCH' if self.tracking else 'IDLE'
            self.message = 'Target moved during IK: discarded stale solution'; return
        if locked:
            plan['target_lock_mode']='go_click_snapshot'; plan['go_click_time']=self.go_snapshot['click_time']; plan['locked_observation_time']=self.go_snapshot['stamp']
        self.want_preview = False; self.auto_go = False; self.go_snapshot=None
        if preview:
            self.preview = plan; self.state = 'PREVIEW_READY'
            write_report(self.args.report, plan); show_plan(plan)
            final = plan['stage2_waypoints'][-1]
            if plan.get('feed_trajectory'):
                print('连续前送参考时长(s):',round(plan['feed_trajectory']['duration_s'],3),'匀速段(m/s):',round(plan['feed_trajectory']['reference_cruise_speed_m_s'],4),flush=True)
            print('第二段前送(m):', plan['feed_distance_m'], '最终 XYZ(m):', np.round(final['target_position_m'], 5).tolist(), '最终距孔(m):', plan['final_standoff_m'], flush=True)
            self.message = 'Preview ready; GO locks fresh depth and executes configured alignment / feed'
        else:
            self.begin_plan(plan)

    def reference_now(self,now):
        if self.active is not None and 'curve' in self.active:
            return self.active['curve'].sample(now-self.active['started'])
        if self.reference_state is not None:return self.reference_state
        q=np.clip(self.robot.positions(),self.robot.joint_limits['lower'],self.robot.joint_limits['upper'])
        if self.joint_estimate.q is None or (self.stationary_since is not None and now-self.stationary_since>=.5) or np.max(np.abs(self.joint_estimate.v))<.02:
            return q,np.zeros(6),np.zeros(6)
        return q,self.joint_estimate.v.copy(),self.joint_estimate.a.copy()

    def send(self, segment, phase,curve=None):
        if self.current_epoch()!=self.epoch:return
        now=time.monotonic();q,v,a=self.reference_now(now)
        if curve is None:
            minimum=0. if self.tracking and self.active is not None else segment['duration_s']
            config=copy.deepcopy(self.motion_config)
            if phase=='RECOVER':
                config['robot']['velocity_limits']=np.minimum(config['robot']['velocity_limits'],self.args.recovery_speed).tolist()
            curve=JointTransition(q,v,a,segment['target_joints_rad'],config,minimum)
        started=self.robot.start_trajectory(curve,started_at=now)
        self.active={'segment':segment,'phase':phase,'started':started,
                     'start':q.copy(),'last_emit':None,'backend':'ArmHub continuous reference','curve':curve}
        self.last_reference_send=None
        self.state=phase;self.message=phase
        if self.report is not None:
            self.report.update(motion_sent=False,phase=phase,reference_duration_s=curve.duration,
                               active_backend=self.active['backend'])
            write_report(self.args.report,self.report)

    def begin_plan(self, plan):
        self.preview=None;self.report=copy.deepcopy(plan);self.anchor=plan
        # Only one transition at the phase boundary; no waypoint arrival loop.
        self.sequence=[]
        self.send(plan['stage1'],'APPROACH')

    def emit_reference(self,q,now):
        self.robot.advance_trajectory()
        curve=self.active['curve']
        self.reference_state=curve.sample(now-self.active['started'])
        if self.last_reference_send is not None:self.stream_intervals.append(now-self.last_reference_send)
        self.last_reference_send=now
        self.active['last_emit']=now
        if self.report is not None:self.report['motion_sent']=True

    def finish_or_advance(self,now):
        phase=self.active['phase'];self.active=None
        if phase=='APPROACH' and self.report.get('feed_trajectory') is not None and self.report['feed_trajectory']['distance_m']>0:
            spec=self.report['feed_trajectory'];curve=ContinuousFeed(spec)
            segment=copy.deepcopy(self.report['stage2_waypoints'][-1]);segment['duration_s']=curve.duration
            self.send(segment,'FEED_CONTINUOUS',curve=curve);return
        self.state=('SEARCH_HOME' if phase=='RECOVER' else 'TRACK_HOLD') if self.tracking else 'IDLE';self.message='Feedback arrival confirmed'
        self.reference_state=None;self.last_reference_send=None
        if self.report is not None:
            self.report['arrival_confirmed']=True
            if self.stream_intervals:
                dt=np.asarray(self.stream_intervals)
                self.report['stream_cycle_s']={'mean':float(dt.mean()),'p95':float(np.percentile(dt,95)),'p99':float(np.percentile(dt,99)),'max':float(dt.max())}
            write_report(self.args.report,self.report)

    def recovery(self, q):
        if self.active is not None or getattr(self.robot,'hold_pending',False):
            return
        if np.max(np.abs(q)) <= .01:
            self.state = 'SEARCH_HOME'; self.message = 'At home; waiting for depth hole'; return
        target = np.zeros(6)
        target = np.clip(target, self.robot.joint_limits['lower'], self.robot.joint_limits['upper'])
        duration = max(.3, float(np.max(np.abs(target-q)))/self.args.recovery_speed, float(np.max(np.abs(target-q)/self.robot.velocity_limits)))
        self.report = {'action': 'lost_hole_home_recovery', 'motion_sent': False, 'target_joints_rad': target.tolist()}
        self.send({'target_joints_rad': target.tolist(), 'duration_s': duration}, 'RECOVER')

    def tick(self, q):
        if self.current_epoch() != self.epoch:
            # Input received while IK/serial work was running: invalidate before sending.
            if self.active is not None or self.tracking:
                self.hold()
            return
        if getattr(self.robot,'hold_pending',False):
            self.message='Hold requested: confirming fresh position feedback; tracking intent retained'
            return
        if self.want_home:self.start_home()
        if self.pending is not None and time.monotonic()-self.pending['submitted'] > 15:
            self.cancel(); self.message = 'IK worker timeout: task cancelled, press GO to retry'; return
        if self.tracking and self.state!='FEED_CONTINUOUS':
            if self.frame_stamp is None or time.time()-self.frame_stamp > .7 or self.vision in ['unavailable', 'unsynchronized']:
                if self.active is not None:
                    self.hold()
                self.pending = None; self.anchor = None; self.state = 'WAIT_CAMERA'; self.message = 'Camera unavailable: hold, no blind recovery'; return
            if self.vision == 'lost':
                if self.prediction_usable():
                    self.message='PREDICTING: short occlusion; maintaining filtered hole target'
                    if self.active is not None and self.anchor and not self.same_target(self.anchor):
                        if self.pending is None and time.monotonic()-self.last_replan>=self.smoothing['tracking_replan_period_s']:self.request()
                else:
                    if self.state not in ['RECOVER', 'SEARCH_HOME']:
                        if self.active is not None:self.hold()
                        self.pending=None; self.anchor=None; self.request_id+=1; self.state='RECOVER'
                    self.sequence=[]
            elif self.active is not None and self.state not in ['RECOVER', 'HOME'] and self.anchor and not self.same_target(self.anchor):
                if self.pending is None and time.monotonic()-self.last_replan>=self.smoothing['tracking_replan_period_s']:self.request()
        if self.active is not None:
            now=time.monotonic()
            self.emit_reference(q,now)
            # 后端统一核验新反馈、到位窗口和超时；任务层不再重复等待另一个窗口。
            if self.robot.trajectory_done:self.finish_or_advance(now)
        if self.tracking and self.state!='FEED_CONTINUOUS' and self.vision == 'lost' and not self.prediction_usable():
            self.recovery(q); return
        stationary = self.stationary_since is not None and time.monotonic()-self.stationary_since >= .5
        if self.want_gripper and stationary:
            key = self.want_gripper; self.want_gripper = None
            from .reporting import execute_gripper
            execute_gripper(self.robot, self.args, {}, key == 'c'); self.state = 'IDLE'
        if self.want_preview and stationary and self.pending is None:
            self.request(preview=not self.auto_go)
        if self.tracking and self.active is None and self.pending is None and (self.good >= 3 or self.prediction_usable()) and self.fresh():
            if self.anchor is None or not self.same_target(self.anchor):
                self.request()

    def status(self):
        result = {'motion_enabled':self.robot.allow_motion, 'state': self.state, 'tracking': self.tracking, 'message': self.message, 'vision': self.vision}
        result['depth_confirmed_frames']=self.good
        result['camera_age_s']=None if self.frame_stamp is None else max(0.,time.time()-self.frame_stamp)
        result['pending_age_s']=None if self.pending is None else max(0.,time.monotonic()-self.pending['submitted'])
        result['vision_rejection']=self.rejection_reason
        result['hold_pending']=getattr(self.robot,'hold_pending',False)
        result['camera_pose_history']=list(self.pose_history)
        result['ruckig_enabled']=getattr(self.args,'ruckig',False)
        result['tool_settings']={name:getattr(self.args,name,default) for name,default in [('rod_length',.04),('tcp_xyz_mm',[0.,0.,0.]),('tcp_rpy_deg',[0.,0.,0.]),('plane_offset_m',[0.,0.])]}
        result['T_gripper_tool']=tool_transform(self.args).tolist()
        result['current_tcp_base_m']=getattr(self,'current_tcp_base',None)
        if self.observation is not None:result['hole_plane_normal_base']=self.observation['world_normal'].tolist()
        plan=self.preview or self.anchor
        if plan is not None:
            result['plan_hole_base_m']=plan['hole_center_base_m']; result['plan_plane_normal_base']=plan['outward_normal_base']
            result['overlay_plan_mode']='GO LOCKED' if plan.get('target_lock_mode')=='go_click_snapshot' else ('TRACKING' if self.tracking else 'PREVIEW')
        elif self.go_snapshot is not None:
            result['plan_hole_base_m']=self.go_snapshot['world_center'].tolist(); result['overlay_plan_mode']='GO LOCKED / PLANNING'
        if self.active is not None:
            segment=self.active['segment']
            result['tcp_goal_base_m']=segment.get('target_position_m') or (np.asarray(self.robot.forward_kinematics(segment['target_joints_rad'])['transform'])@tool_transform(self.args))[:3,3].tolist()
        elif plan is not None:
            result['tcp_goal_base_m']=plan['stage2_waypoints'][-1].get('target_position_m') or plan.get('target_position_m')
        estimate=self.estimator.estimate(time.time())
        if estimate is not None:
            result.update(hole_estimate_mode=('PREDICTED' if self.prediction_usable() else ('MEASURED' if self.vision=='visible' else 'EXPIRED')),hole_estimate_age_s=estimate['age_s'],hole_position_std_m=estimate['position_std_m'],hole_estimate_base_m=estimate['position'].tolist(),hole_velocity_m_s=estimate['velocity'].tolist())
        if self.reference_state is not None:
            result['reference_velocity_rad_s']=self.reference_state[1].tolist();result['reference_acceleration_rad_s2']=self.reference_state[2].tolist()
        if self.joint_estimate.q is not None:
            result['estimated_velocity_rad_s']=self.joint_estimate.v.tolist();result['estimated_acceleration_rad_s2']=self.joint_estimate.a.tolist()
        if self.states is not None:
            result.update(gripper_position=float(self.states[6].position), gripper_torque=float(self.states[6].torque))
        return result
