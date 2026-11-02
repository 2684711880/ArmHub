"""Prepared return and offline viewing: no real devices."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
from integrations.panthera_lerobot.prepare_pose import load_prepared, return_prepared
from integrations.panthera_lerobot.view_episode import validate_episode

class TrialTests(unittest.TestCase):
    def settings(self, folder, source='hardware'):
        path=Path(folder)/'settings.json'
        path.write_text(json.dumps({'prepared':dict(q=[0]*6,position_m=[0]*3,
            quaternion_xyzw=[0,0,0,1],source=source)}))
        return path

    def session(self):
        session=Mock()
        session.preview.return_value={'curve':object(),'start':[0]*6,'target':[0]*6,'duration':3.}
        session.read.return_value={'q':[0]*6,'gripper':.7}
        session.advance_return.side_effect=[False,True]
        return session

    def test_preview_never_starts_motion_and_closes_transport(self):
        with tempfile.TemporaryDirectory() as temp:
            session=self.session()
            return_prepared(settings_path=self.settings(temp),session_factory=lambda:session)
            session.start_return.assert_not_called();session.arm.enable.assert_not_called()
            session.arm.command_gripper.assert_not_called()
            session.close.assert_called_once()

    def test_execute_uses_saved_target_and_waits_for_feedback_completion(self):
        with tempfile.TemporaryDirectory() as temp:
            session=self.session()
            return_prepared(execute=True,settings_path=self.settings(temp),session_factory=lambda:session)
            self.assertEqual(session.preview.call_args.args[1]['q'],[0]*6)
            plan,guard=session.start_return.call_args.args
            self.assertEqual(plan['target'],[0]*6)
            self.assertEqual(session.advance_return.call_count,2)
            session.arm.command_gripper.assert_called_once()
            self.assertEqual(session.arm.command_gripper.call_args.args,(1.6,))
            self.assertEqual(session.arm.command_gripper.call_args.kwargs['speed'],.3)
            self.assertEqual(session.arm.command_gripper.call_args.kwargs['torque'],.3)
            self.assertTrue(session.arm.poll_gripper.called)
            self.assertFalse(guard())
            session.close.assert_called_once()

    def test_preflight_failure_never_starts_and_still_closes(self):
        with tempfile.TemporaryDirectory() as temp:
            session=self.session();session.preview.side_effect=ValueError('collision')
            with self.assertRaisesRegex(ValueError,'collision'):
                return_prepared(execute=True,settings_path=self.settings(temp),session_factory=lambda:session)
            session.start_return.assert_not_called();session.close.assert_called_once()
            session.arm.command_gripper.assert_not_called()

    def test_interrupted_execution_closes_and_cancels_guard(self):
        with tempfile.TemporaryDirectory() as temp:
            session=self.session();session.advance_return.side_effect=KeyboardInterrupt
            with self.assertRaises(KeyboardInterrupt):
                return_prepared(execute=True,settings_path=self.settings(temp),session_factory=lambda:session)
            self.assertFalse(session.start_return.call_args.args[1]())
            session.close.assert_called_once()
            session.arm.command_gripper.assert_not_called()

    def test_demo_or_missing_prepared_pose_rejected_before_connection(self):
        with tempfile.TemporaryDirectory() as temp:
            factory=Mock()
            with self.assertRaisesRegex(ValueError,'真实机械臂'):
                return_prepared(execute=True,settings_path=self.settings(temp,'demo'),session_factory=factory)
            factory.assert_not_called()
            path=Path(temp)/'missing.json';path.write_text('{}')
            with self.assertRaises(ValueError):load_prepared(path)

    def test_empty_or_out_of_range_episode_rejected_before_viewer(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'meta').mkdir()
            info=root/'meta/info.json';info.write_text('{"total_episodes":0}')
            with self.assertRaisesRegex(ValueError,'0 个完整回合'):validate_episode(root,0)
            info.write_text('{"total_episodes":1}')
            self.assertEqual(validate_episode(root,0)['total_episodes'],1)
            for episode in (-1,1):
                with self.assertRaises(ValueError):validate_episode(root,episode)

if __name__=='__main__':unittest.main()
