import json
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from policy_guard import PolicyGuard,KEYS

ROOT=Path(__file__).resolve().parents[1]
class GuardTests(unittest.TestCase):
    def make(self):
        config=json.loads((ROOT/'robots/panthera_ht/hardware/config.json').read_text())
        prepared=json.loads((ROOT/'integrations/panthera_lerobot/configs/teaching.json').read_text())['prepared']
        kin=Mock();kin.fk.return_value={'position_m':[0,0,.15]}
        guard=PolicyGuard(config,prepared,kin)
        states=[SimpleNamespace(position=q,velocity=0.,torque=0.,fault=0) for q in prepared['q']+[1.6]]
        action={k:s.position for k,s in zip(KEYS,states)}
        return guard,states,action
    def test_rate_limit_and_feedback_path_checked(self):
        guard,states,action=self.make();action[KEYS[0]]+=.2
        out=guard.step(action,states,1.)
        self.assertAlmostEqual(out[KEYS[0]]-states[0].position,.0075)
        self.assertEqual(guard.kin.check_path.call_count,2)
    def test_reject_jump_and_latch(self):
        guard,states,action=self.make();action[KEYS[0]]+=.31
        with self.assertRaisesRegex(RuntimeError,'跳变'):guard.step(action,states,1.)
        action[KEYS[0]]=states[0].position
        with self.assertRaisesRegex(RuntimeError,'已触发'):guard.step(action,states,1.05)
    def test_reject_limits_nan_and_wrong_keys(self):
        for key,value in [(KEYS[0],float('nan')),(KEYS[1],-1)]:
            guard,states,action=self.make();action[key]=value
            with self.assertRaises(ValueError):guard.step(action,states,1.)
        guard,states,action=self.make();action.pop(KEYS[0])
        with self.assertRaises(ValueError):guard.step(action,states,1.)
    def test_reject_wrong_start_tracking_fault_and_gap(self):
        for kind in ['start','tracking','fault','gap']:
            guard,states,action=self.make()
            if kind=='start':states[0].position+=.1
            if kind=='fault':states[0].fault=1
            if kind in ('gap','tracking'):guard.step(action,states,1.)
            if kind=='tracking':states[0].position+=.11
            with self.assertRaises(RuntimeError):guard.step(action,states,1.4 if kind=='gap' else 1.05)
    def test_reject_collision_and_low_tcp(self):
        guard,states,action=self.make();guard.kin.check_path.side_effect=ValueError('collision')
        with self.assertRaises(ValueError):guard.step(action,states,1.)
        guard,states,action=self.make();guard.kin.fk.return_value={'position_m':[0,0,.01]}
        with self.assertRaisesRegex(RuntimeError,'TCP'):guard.step(action,states,1.)
    def test_single_speed_spike_freezes_reference_and_recovers(self):
        guard,states,action=self.make()
        guard.step(action,states,1.)
        previous=list(guard.previous)
        action[KEYS[0]]+=.1
        states[0].velocity=.7
        frozen=guard.step(action,states,1.05)
        self.assertEqual(list(frozen.values()),previous)
        self.assertEqual(guard.feedback_warnings[0]['motor'],1)
        states[0].velocity=0
        resumed=guard.step(action,states,1.10)
        self.assertGreater(resumed[KEYS[0]],previous[0])
    def test_sustained_and_severe_speed_trip(self):
        guard,states,action=self.make()
        guard.step(action,states,1.)
        states[2].velocity=.7
        guard.step(action,states,1.05)
        guard.step(action,states,1.10)
        with self.assertRaisesRegex(RuntimeError,'motor3.*持续'):
            guard.step(action,states,1.18)
        guard,states,action=self.make();states[4].velocity=1.21
        with self.assertRaisesRegex(RuntimeError,'motor5.*严重超速'):
            guard.step(action,states,1.)
    def test_fault_torque_and_invalid_feedback_never_debounced(self):
        for field,value,pattern in [('fault',1,'motor2.*故障'),('torque',37,'motor2.*力矩'),('velocity',float('nan'),'motor2.*非有限')]:
            guard,states,action=self.make();setattr(states[1],field,value)
            with self.assertRaisesRegex(RuntimeError,pattern):guard.step(action,states,1.)
    def test_spikes_on_different_motors_do_not_accumulate(self):
        guard,states,action=self.make();guard.step(action,states,1.)
        states[0].velocity=.7;guard.step(action,states,1.05)
        states[0].velocity=0;states[1].velocity=.7;guard.step(action,states,1.10)
        states[1].velocity=0;states[0].velocity=.7;guard.step(action,states,1.18)
        states[0].velocity=0;guard.step(action,states,1.23)
        self.assertFalse(guard.failed)

    def test_real_geometry_saved_start_without_hardware(self):
        from armhub.adapters.panthera_kinematics import PantheraKinematics
        guard,states,action=self.make()
        guard.kin=PantheraKinematics(guard.config['lower_rad'],guard.config['upper_rad'])
        self.assertEqual(guard.step(action,states,1.),action)

if __name__=='__main__':unittest.main()
