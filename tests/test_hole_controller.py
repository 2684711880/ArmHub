"""点击锁定、前送策略与取消回归；只用任务会话替身，无 SDK/相机。"""
import copy
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from armhub.trajectory import ContinuousFeed
from apps.hole_approach.__main__ import arguments
from apps.hole_approach.config import load_config
from apps.hole_approach.controller import Controller


class ControlClock:
    def __init__(self):self.now=1000.
    def monotonic(self):return self.now
    def time(self):return self.now


class TaskDouble:
    def __init__(self,clock,options):
        self.clock=clock;self.config=load_config(options.config)
        self.q=np.array([0.,.5,.5,0.,0.,0.])
        self.joint_limits={k:np.array(v) for k,v in self.config['robot']['joint_limits'].items()}
        self.velocity_limits=np.array(self.config['robot']['velocity_limits'])
        self.allow_motion=options.execute;self.holds=0;self.starts=[];self.command_guard=None
        self.trajectory_done=True;self.curve=None
    def positions(self):return self.q.copy()
    def feedback(self):
        return [SimpleNamespace(position=v,velocity=0.,torque=0.,time=self.clock.now) for v in [*self.q,.5]]
    def forward_kinematics(self,q):
        t=np.eye(4);t[:3,3]=np.asarray(q)[:3]
        return {'transform':t,'position':t[:3,3].tolist(),'rotation':t[:3,:3]}
    def hold(self):self.holds+=1;self.curve=None;self.trajectory_done=True
    def start_trajectory(self,curve,*,started_at=None):
        if self.command_guard and not self.command_guard():raise RuntimeError('cancelled')
        self.curve=curve;self.started=self.clock.now if started_at is None else started_at;self.trajectory_done=False;self.starts.append(curve)
        return self.started
    def advance_trajectory(self):
        self.q=np.asarray(self.curve.sample(self.clock.now-self.started)[0])
        self.trajectory_done=self.clock.now-self.started>=self.curve.duration
        return self.trajectory_done


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock=ControlClock();p=patch('apps.hole_approach.controller.time',self.clock);p.start();self.addCleanup(p.stop)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.a=arguments(['--execute','--report',str(Path(self.temp.name)/'report.json')])
        self.robot=TaskDouble(self.clock,self.a);self.tasks=[];self.epoch=[0]
        self.c=Controller(self.robot,self.a,np.eye(4),'synthetic',self.tasks.append,lambda:self.epoch[0])
        self.c.feedback()
    def key(self,key):
        self.epoch[0]+=1
        self.c.command(key,self.epoch[0],self.clock.now)
    def observe(self,center=(.1,0,.5)):
        self.clock.now+=.1;self.c.feedback()
        self.c.observe({'stamp':self.clock.now,'observation':{'center':list(center),'normal':[0,0,-1],'plane_rms':.001}})
    def ready(self):
        for _ in range(3):self.observe()
        self.c.stationary_since=self.clock.now-.6
    def plan(self):
        q=self.robot.q.copy();q[0]+=.02;end=q.copy();end[0]+=.02
        spec={'duration_s':2.,'ramp_fraction':.2,'path_fraction':np.linspace(0,1,7).tolist(),
              'joint_path_rad':[(q+(end-q)*f).tolist() for f in np.linspace(0,1,7)],
              'distance_m':.02,'normal_base':[-1,0,0],'start_position_m':[.1,0,.5]}
        stage={'target_joints_rad':q.tolist(),'target_position_m':[.1,0,.5],'target_rpy_deg':[0,0,0],'duration_s':2.}
        final=dict(stage,target_joints_rad=end.tolist())
        center=self.c.observation['world_center'].tolist() if self.c.observation else [.1,.5,1.]
        return dict(stage,stage1=stage,stage2_waypoints=[final],feed_trajectory=spec,
                    hole_center_base_m=center,outward_normal_base=[0,0,-1],feed_distance_m=.02,final_standoff_m=.18)
    def test_go_locks_click_observation_and_ignores_later_hole_motion(self):
        self.ready();original=copy.deepcopy(self.c.observation)
        self.key('g');self.c.tick(self.robot.positions())
        task=self.tasks[-1]
        np.testing.assert_allclose(task['center'],original['center'])
        self.observe((.3,0,.5))
        self.c.planned({'id':task['id'],'epoch':task['epoch'],'plan':self.plan()})
        self.assertEqual(len(self.robot.starts),1)
        self.assertEqual(self.c.report['target_lock_mode'],'go_click_snapshot')
        self.assertFalse(self.c.tracking)
    def test_stop_discards_late_ik_result(self):
        self.ready();self.key('g');self.c.tick(self.robot.positions());task=self.tasks[-1]
        self.key(' ')
        self.c.planned({'id':task['id'],'epoch':task['epoch'],'plan':self.plan()})
        self.assertEqual(self.robot.starts,[]);self.assertIsNone(self.c.active)
    def test_read_only_rejects_all_motion_keys(self):
        self.robot.allow_motion=False
        for key in ['g','t','r','c','o']:
            with self.assertRaisesRegex(RuntimeError,'只读'):self.key(key)
        self.assertEqual(self.robot.starts,[])
        self.key('p');self.assertTrue(self.c.want_preview)
    def test_feed_continues_through_target_loss_and_stale_camera(self):
        plan=self.plan();self.c.report=plan;self.c.tracking=True
        curve=ContinuousFeed(plan['feed_trajectory'])
        self.c.send(plan['stage2_waypoints'][-1],'FEED_CONTINUOUS',curve)
        self.c.vision='lost';self.c.frame_stamp=self.clock.now-5
        self.c.tick(self.robot.positions())
        self.assertEqual(self.robot.holds,0)
        self.assertEqual(self.c.state,'FEED_CONTINUOUS')
        self.assertEqual(len(self.robot.starts),1)
    def test_old_tracking_ik_cannot_interrupt_feed(self):
        plan=self.plan();self.c.report=plan
        self.c.send(plan['stage2_waypoints'][-1],'FEED_CONTINUOUS',ContinuousFeed(plan['feed_trajectory']))
        self.c.pending={'id':7,'epoch':0,'preview':False,'submitted':self.clock.now}
        self.c.planned({'id':7,'epoch':0,'plan':plan})
        self.assertEqual(self.c.state,'FEED_CONTINUOUS');self.assertEqual(len(self.robot.starts),1)
    def test_home_is_ruckig_reference_through_shared_runtime(self):
        self.key('r')
        self.assertEqual(self.c.state,'HOME')
        np.testing.assert_allclose(self.robot.starts[-1].sample(self.robot.starts[-1].duration)[0],0,atol=1e-12)
    def test_status_explains_tracking_observation_readiness(self):
        self.ready()
        status=self.c.status()
        self.assertGreaterEqual(status['depth_confirmed_frames'],3)
        self.assertEqual(status['vision'],'visible')
        self.assertAlmostEqual(status['camera_age_s'],0.)
        self.assertIsNone(status['pending_age_s'])
    def test_settings_cancel_pending_motion(self):
        self.ready();self.key('g');self.c.tick(self.robot.positions())
        task=self.tasks[-1];self.epoch[0]+=1
        self.c.command('calibration',self.epoch[0],settings={'rod_length':.04,'tcp_xyz_mm':[1,2,3],
                       'tcp_rpy_deg':[0,0,0],'plane_offset_m':[0,0]})
        self.c.planned({'id':task['id'],'epoch':task['epoch'],'plan':self.plan()})
        self.assertEqual(self.robot.starts,[]);self.assertFalse(self.c.tracking)


if __name__=='__main__':unittest.main()
