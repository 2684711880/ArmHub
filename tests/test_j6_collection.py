"""采集流程使用替身电机；动力学特征使用真实离线模型。不会连接串口。"""
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from examples.collect_j6_gravity import (
    Observations, capture_pose, collect, make_plan, move_one_axis, parse_args, preflight,
)
import test_panthera_hardware as hardware_fixture


class Kinematics:
    def check_path(self, start, goal):
        return {'samples': 2}


class Clock:
    def __init__(self):
        self.value = 0.

    def monotonic(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


def synthetic_observations(q):
    return {'model_gravity_nm': [0.]*6, 'gravity_link6_m_s2': [0., 1., -9.],
            'regressor_hy_hz_bias': [9., 1., 1.]}


class CollectionTests(unittest.TestCase):
    make_arm = hardware_fixture.HardwareTests.make_arm

    def options(self):
        return SimpleNamespace(settle_seconds=.002, settle_timeout=.1,
                               sample_seconds=.01, min_samples=5, sample_period=.001, speed_deg_s=2.)

    def test_plan_has_opposite_approaches_and_separate_validation_group(self):
        start = (.1, .5, .6, -.2, .3, .4)
        plan = make_plan(start, [0, 10, 20], [-30, -20, -10, 0, 10, 20, 30], 3)
        samples = [n for n in plan if n['kind'] == 'sample']
        self.assertEqual(len(samples), 42)
        self.assertEqual(sum(n['role']=='validation' for n in samples), 14)
        previous = start
        for node in plan:
            changed = [i for i,(a,b) in enumerate(zip(previous,node['target_rad'])) if abs(a-b)>1e-9]
            self.assertLessEqual(len(changed), 1)
            self.assertTrue(all(i in (2,5) for i in changed))
            if node['kind'] == 'sample':
                delta = node['target_rad'][5]-previous[5]
                self.assertGreater(delta if node['direction']=='positive' else -delta, 0)
            previous = node['target_rad']

    def test_steps_reuse_bounded_single_axis_and_preserve_gripper_and_caps(self):
        arm, driver = self.make_arm()
        arm.enable()
        target = [.1]*6
        target[5] += math.radians(15)
        move_one_axis(arm, target, math.radians(2), Kinematics())
        self.assertGreater(len(driver.transmissions), 3)
        previous = [.1]*6
        for packet in driver.transmissions:
            self.assertEqual(set(packet), set(range(7)))
            self.assertEqual(packet[6][0], .5)
            current = [packet[i][0] for i in range(6)]
            self.assertLessEqual(max(abs(a-b) for a,b in zip(current,previous)), arm.config['max_step_rad'])
            self.assertEqual(current[:5], [.1]*5)
            self.assertEqual([packet[i][2] for i in range(6)], arm.config['torque_caps_nm'])
            previous = current
        target[2] += math.radians(10)
        move_one_axis(arm, target, math.radians(2), Kinematics())
        self.assertAlmostEqual(driver.states[2].position, target[2])
        self.assertAlmostEqual(driver.states[5].position, target[5])

    def test_hold_targets_do_not_ratchet_toward_quantized_feedback(self):
        arm, driver = self.make_arm()
        arm.enable()
        original_send = driver.motor_send_cmd
        tick = 2*math.pi/10000
        def quantized_hold():
            commanded_j4 = driver.staged[3][0]
            original_send()
            # 保持误差仅一格；若逐帧把反馈当新目标，误差会反复累积。
            driver.states[3].position = commanded_j4-tick
        driver.motor_send_cmd = quantized_hold
        target = [.1]*6
        target[5] += math.radians(70)
        move_one_axis(arm,target,math.radians(2),Kinematics())
        held = [packet[3][0] for packet in driver.transmissions]
        self.assertGreater(len(held),15)
        self.assertTrue(all(v == .1 for v in held))
        self.assertAlmostEqual(driver.states[3].position,.1-tick)

    def test_zero_boundary_noise_uses_fixed_legal_hold_without_zeroing_encoder(self):
        arm, driver = self.make_arm()
        driver.states[2].position = -.0018
        arm.enable()
        target = [.1,.1,-.0018,.1,.1,.25]
        move_one_axis(arm,target,math.radians(2),Kinematics())
        self.assertTrue(all(packet[2][0] == 0. for packet in driver.transmissions))
        self.assertTrue(all(len(command)==3 for packet in driver.transmissions for command in packet.values()))

    def test_limits_and_collisions_fail_before_any_motion(self):
        arm, driver = self.make_arm()
        plan = make_plan([.1]*6, [0,10,20], [-30,-20,-10,0,10], 3)
        plan[-1]['target_rad'][5] = 4.
        with self.assertRaisesRegex(ValueError, '越界'):
            preflight([.1]*6, plan, arm.config, synthetic_observations, Kinematics())
        self.assertEqual(driver.transmissions, [])
        plan = make_plan([.1]*6, [0,10,20], [-30,-20,-10,0,10], 3)
        blocked = Kinematics()
        blocked.check_path = lambda a,b: (_ for _ in ()).throw(ValueError('碰撞'))
        with self.assertRaisesRegex(ValueError, '碰撞'):
            preflight([.1]*6, plan, arm.config, synthetic_observations, blocked)
        self.assertEqual(driver.transmissions, [])

    def test_capture_uses_measured_torque_and_preserves_all_feedback(self):
        arm, driver = self.make_arm()
        driver.states[5].torque = .23
        node = {'target_rad': [.1]*6, 'pose_id':'test', 'group':0, 'role':'train',
                'direction':'positive', 'j3_offset_deg':0, 'j6_offset_deg':0}
        raw = io.StringIO()
        result = capture_pose(arm, node, synthetic_observations, self.options(), raw, clock=Clock())
        self.assertAlmostEqual(result['j6_residual_nm_mean'], .23)
        rows = [json.loads(line) for line in raw.getvalue().splitlines()]
        self.assertTrue(any(r['phase']=='settle' for r in rows))
        self.assertEqual(len(rows[0]['motors']), 7)
        self.assertGreater(rows[-1]['motors'][5]['counter'], rows[0]['motors'][5]['counter'])
        self.assertEqual(driver.transmissions, [])

    def test_hardware_trace_velocity_spike_retries_same_pose_and_recovers(self):
        trace = json.loads((Path(__file__).parent/'fixtures/j6_static_velocity_spike.json').read_text())
        arm, driver = self.make_arm()
        clock = Clock()
        index = 0
        def replay(**kwargs):
            nonlocal index
            frame = trace['frames'][min(index,len(trace['frames'])-2)]
            if index < len(trace['frames']):
                frame = trace['frames'][index]
                clock.value = max(clock.value,frame['elapsed_s'])
            index += 1
            return [SimpleNamespace(ID=m['id'],position=m['position_rad'],velocity=m['velocity_rad_s'],
                                    torque=m['torque_nm'],mode=m['mode'],fault=m['fault'],
                                    num=index,time=0.) for m in frame['motors']]
        arm.read_feedback = replay
        options = self.options()
        options.settle_seconds, options.settle_timeout = .6, 10.
        options.sample_seconds, options.min_samples, options.sample_period = 2.,20,.025
        node = {'target_rad':trace['target_rad'],'pose_id':'hardware_replay','group':0,'role':'train',
                'direction':'positive','j3_offset_deg':0,'j6_offset_deg':-30}
        raw = io.StringIO()
        result = capture_pose(arm,node,synthetic_observations,options,raw,clock=clock)
        self.assertGreaterEqual(result['accepted_samples'],20)
        self.assertEqual(result['window_restarts'],0)
        self.assertEqual(driver.transmissions, [])
        rows = [json.loads(s) for s in raw.getvalue().splitlines()]
        spikes = [r for r in rows if abs(r['velocity_rad_s'][3])>.03]
        self.assertTrue(spikes)
        self.assertTrue(all(r['accepted'] for r in spikes))
        self.assertGreater(result['reported_velocity_peak_rad_s'][3],.03)

    def test_small_position_drift_discards_old_window_and_waits_in_place(self):
        arm, driver = self.make_arm()
        original_read = arm.read_feedback
        calls = 0
        def drift_then_settle(**kwargs):
            nonlocal calls
            calls += 1
            if calls == 7:
                driver.states[5].position += .004
            return original_read(**kwargs)
        arm.read_feedback = drift_then_settle
        node = {'target_rad':[.1]*6,'pose_id':'drift_then_settle','group':0,'role':'train',
                'direction':'positive','j3_offset_deg':0,'j6_offset_deg':0}
        raw = io.StringIO()
        result = capture_pose(arm,node,synthetic_observations,self.options(),raw,clock=Clock())
        self.assertEqual(result['window_restarts'],1)
        self.assertGreater(result['discarded_samples'],0)
        self.assertAlmostEqual(result['position_rad_mean'][5],.104)
        self.assertEqual(driver.transmissions,[])
        self.assertIn('discarded_window_id',raw.getvalue())

    def test_continuous_position_motion_times_out_without_accepting_window(self):
        arm, driver = self.make_arm()
        original_read = arm.read_feedback
        def moving(**kwargs):
            driver.states[5].position = .1 if driver.states[5].num%2 else .104
            return original_read(**kwargs)
        arm.read_feedback = moving
        node = {'target_rad':[.1]*6,'pose_id':'moving','group':0,'role':'train',
                'direction':'positive','j3_offset_deg':0,'j6_offset_deg':0}
        with self.assertRaises(TimeoutError):
            capture_pose(arm,node,synthetic_observations,self.options(),io.StringIO(),clock=Clock())
        self.assertEqual(driver.transmissions,[])

    def test_near_limit_fault_stale_or_drift_cannot_be_reported_as_static(self):
        for problem in ['limit','fault','stale','drift']:
            with self.subTest(problem=problem):
                arm, driver = self.make_arm()
                if problem=='limit': driver.states[5].torque = 2.9
                if problem=='fault': driver.states[5].fault = 1
                if problem=='stale': driver.increment = False
                if problem=='drift': driver.states[5].position += .02
                node = {'target_rad':[.1]*6, 'pose_id':'bad', 'group':0, 'role':'train',
                        'direction':'positive', 'j3_offset_deg':0, 'j6_offset_deg':0}
                with self.assertRaises((RuntimeError,TimeoutError)):
                    capture_pose(arm,node,synthetic_observations,self.options(),io.StringIO(),clock=Clock())
                self.assertEqual(driver.transmissions, [])

    def test_interrupt_keeps_partial_files_and_confirms_hold(self):
        arm, driver = self.make_arm()
        node = {'kind':'sample','target_rad':[.1]*6,'pose_id':'p','group':0,'role':'train',
                'direction':'positive','j3_offset_deg':0,'j6_offset_deg':0}
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            def interrupt(*args):
                args[-1].write('{"event":"partial_sample"}\n')
                raise KeyboardInterrupt()
            with patch('examples.collect_j6_gravity.capture_pose', side_effect=interrupt):
                with self.assertRaises(KeyboardInterrupt):
                    collect(arm,[node],synthetic_observations,Kinematics(),self.options(),folder)
            report = json.loads((folder/'summary.json').read_text())
            self.assertEqual(report['status'], 'interrupted')
            self.assertTrue(report['hold_stop_confirmed'])
            self.assertIn('partial_sample', (folder/'samples.jsonl').read_text())
            self.assertTrue(all(len(command)==3 for command in driver.transmissions[-1].values()))

    def test_complete_session_retains_both_directions_and_holdout_data(self):
        arm, driver = self.make_arm()
        plan = make_plan([.1]*6,[0,2,4],[-10,-5,0,5,10],3)
        original_capture = capture_pose
        def accelerated(*args):
            return original_capture(*args,clock=Clock())
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            with patch('examples.collect_j6_gravity.capture_pose',side_effect=accelerated), patch('sys.stdout',io.StringIO()):
                result = collect(arm,plan,synthetic_observations,Kinematics(),self.options(),folder)
            self.assertEqual(result['status'],'completed')
            self.assertEqual(result['completed_poses'],30)
            records = [json.loads(line) for line in (folder/'poses.jsonl').read_text().splitlines()]
            self.assertEqual(len(records),30)
            self.assertEqual(sum(r['role']=='validation' for r in records),10)
            self.assertEqual({r['direction'] for r in records},{'positive','negative'})
            self.assertTrue(result['hold_stop_confirmed'])
            self.assertEqual(driver.states[6].position,.5)
            self.assertTrue((folder/'poses.csv').exists())

    def test_hold_failure_is_saved_and_never_claims_completion(self):
        arm, _ = self.make_arm()
        arm.stop = lambda: (_ for _ in ()).throw(RuntimeError('模拟停止反馈失败'))
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            with self.assertRaisesRegex(RuntimeError,'保持未确认'):
                collect(arm,[],synthetic_observations,Kinematics(),self.options(),folder)
            result = json.loads((folder/'summary.json').read_text())
            self.assertEqual(result['status'],'stop_unconfirmed')
            self.assertFalse(result['hold_stop_confirmed'])

    def test_parser_rejects_execute_with_offline_or_invalid_scan(self):
        for argv in [['--execute','--offline-q']+['0']*6,
                     ['--j6-offsets-deg','0','10','5','20','30'],
                     ['--j3-offsets-deg','0','nan','20'],['--speed-deg-s','10']]:
            with self.subTest(argv=argv), self.assertRaises(SystemExit), patch('sys.stderr',io.StringIO()):
                parse_args(argv)


class DynamicsFeatureTests(unittest.TestCase):
    def test_payload_first_moment_features_match_pinocchio_added_mass(self):
        observation = Observations()
        model = observation.gravity
        joint = model.model.getJointId('joint6')
        original = model.pin.Inertia(model.model.inertias[joint])
        mass, lever = .3, model.np.array([.08,.03,-.04])
        extra = model.pin.Inertia(mass,lever,model.np.eye(3)*1e-5)
        for pose in [(.1,.5,.6,-.2,.3,.4),(-.2,1.2,1.4,.5,-.6,.8)]:
            data = observation(pose)
            model.model.inertias[joint] = original+extra
            actual = model(pose)[5]-data['model_gravity_nm'][5]
            model.model.inertias[joint] = original
            row = data['regressor_hy_hz_bias']
            self.assertAlmostEqual(actual,row[0]*mass*lever[1]+row[1]*mass*lever[2],places=9)


if __name__ == '__main__':
    unittest.main()
