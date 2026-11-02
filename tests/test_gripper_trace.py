"""Exercise replay call seam: feedback logging must preserve outgoing actions."""
import csv
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import Mock

from integrations.panthera_lerobot.gripper_trace import GripperTrace, install_gripper_trace


class GripperTraceTests(unittest.TestCase):
    def state(self, **kwargs):
        values=dict(ID=7,num=10,fault=0,time=time.time(),position=.8,velocity=-.1,torque=-.2)
        return SimpleNamespace(**dict(values,**kwargs))

    def test_real_send_seam_preserves_action_identity_and_result(self):
        sent=[]
        class Follower:
            teaching_mode=False
            def send_action(self, action):
                sent.append(action)
                return action
        original=Follower.send_action
        device=Follower()
        motor=Mock();motor.get_current_motor_state.side_effect=lambda:self.state()
        device.bus=SimpleNamespace(htr_motors=[Mock() for _ in range(6)]+[motor])
        with tempfile.TemporaryDirectory() as temp:
            trace=install_gripper_trace(Path(temp)/'trace.csv',Follower)
            try:
                for target in (.9,.7):
                    action={'gripper.pos':target,**{f'joint{i}.pos':i*.1 for i in range(1,7)}}
                    self.assertIs(device.send_action(action),action)
                    self.assertIs(sent[-1],action)
                with trace.path.open() as stream:
                    rows=list(csv.DictReader(stream))
                self.assertEqual(len(rows),2)
                self.assertEqual(rows[0]['previous_target_rad'],'')
                self.assertEqual(float(rows[1]['previous_target_rad']),.9)
                self.assertEqual(float(rows[1]['feedback_torque_nm']),-.2)
                motor.pos_vel_tqe_kp_kd.assert_not_called()
            finally:trace.close()
            self.assertIs(Follower.send_action,original)
            summary=json.loads(trace.path.with_suffix('.csv.summary.json').read_text())
            self.assertEqual(summary['samples'],2)
            self.assertFalse(summary['control_changed'])

    def test_bad_feedback_rejected_before_original_send(self):
        for overrides in ({'time':time.time()-1},{'fault':1},{'torque':float('nan')},{'ID':6},{'num':0}):
            with self.subTest(overrides=overrides),tempfile.TemporaryDirectory() as temp:
                trace=GripperTrace(Path(temp)/'trace.csv')
                try:
                    with self.assertRaises(RuntimeError):trace.observe(.5,self.state(**overrides))
                    self.assertEqual(trace.count,0)
                finally:trace.close()

    def test_teaching_mode_is_rejected_without_sending(self):
        original=Mock()
        class Follower:send_action=original
        device=Follower();device.teaching_mode=True
        with tempfile.TemporaryDirectory() as temp:
            trace=install_gripper_trace(Path(temp)/'trace.csv',Follower)
            try:
                with self.assertRaises(RuntimeError):device.send_action({'gripper.pos':0})
                original.assert_not_called()
            finally:trace.close()

    def test_existing_log_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'trace.csv';path.write_text('preserve')
            with self.assertRaises(FileExistsError):GripperTrace(path)
            self.assertEqual(path.read_text(),'preserve')

    def test_send_failure_still_restores_class_via_finally(self):
        class Follower:
            teaching_mode=False
            def send_action(self, action):raise RuntimeError('send failed')
        original=Follower.send_action
        device=Follower();motor=Mock();motor.get_current_motor_state.side_effect=lambda:self.state()
        device.bus=SimpleNamespace(htr_motors=[Mock() for _ in range(6)]+[motor])
        with tempfile.TemporaryDirectory() as temp:
            trace=install_gripper_trace(Path(temp)/'trace.csv',Follower)
            try:
                with self.assertRaisesRegex(RuntimeError,'send failed'):device.send_action({'gripper.pos':.3})
            finally:trace.close()
            self.assertIs(Follower.send_action,original)


if __name__=='__main__':unittest.main()
