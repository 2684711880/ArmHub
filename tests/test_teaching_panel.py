"""Teaching contracts with no devices: commands, origin, dataset admission and packet mode."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock,patch

import numpy as np
from apps.teaching_panel.controller import TeachingWorker,validate_options
from apps.teaching_panel.hardware import DemoSession,HardwareSession
from apps.teaching_panel.recorder import EpisodeStore,atomic_json,read_episode


class TeachingContracts(unittest.TestCase):
    def worker(self,directory):
        return TeachingWorker(lambda *x:None,Mock(),demo=True,settings_path=Path(directory)/'settings.json',store_base=Path(directory)/'raw')

    def test_constructing_worker_does_not_open_devices(self):
        with tempfile.TemporaryDirectory() as temp:
            worker=self.worker(temp)
            self.assertIsNone(worker.session)
            self.assertFalse(worker.allowed.is_set())
            self.assertFalse(worker.store.is_alive())

    def test_no_permission_no_drag_or_gripper_command(self):
        with tempfile.TemporaryDirectory() as temp:
            worker=self.worker(temp);worker.session=Mock();worker.mode='readonly'
            with self.assertRaisesRegex(ValueError,'允许实机动作'):worker.action('teach',None)
            with self.assertRaisesRegex(ValueError,'允许实机动作'):worker.action('close',None)
            worker.session.teach.assert_not_called();worker.session.gripper.assert_not_called()

    def test_stop_invalidates_previously_queued_commands(self):
        with tempfile.TemporaryDirectory() as temp:
            worker=self.worker(temp);worker.submit('teach')
            epoch,action,value=worker.commands.get_nowait();worker.request_stop()
            self.assertNotEqual(epoch,worker.epoch)
            self.assertTrue(worker.stop_event.is_set())

    def test_prepare_is_saved_from_stationary_feedback_without_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            worker=self.worker(temp);worker.session=DemoSession();worker.sample=worker.session.read();worker.mode='readonly'
            worker.history.extend([(0.,worker.sample['q']),(.3,worker.sample['q']),(.6,worker.sample['q'])])
            worker.action('save_prepare',None)
            saved=json.loads(worker.settings_path.read_text())['prepared']
            self.assertEqual(saved['q'],worker.sample['q'])
            self.assertEqual(worker.session.mode,'readonly')
            self.assertEqual(saved['source'],'demo')

    def test_return_requires_preview_and_rejects_drift(self):
        with tempfile.TemporaryDirectory() as temp:
            worker=self.worker(temp);worker.session=DemoSession();worker.sample=worker.session.read();worker.mode='hold';worker.permission(True)
            with self.assertRaisesRegex(ValueError,'先预览'):worker.action('return',None)
            worker.plan=dict(start=[q+.1 for q in worker.sample['q']])
            with self.assertRaisesRegex(ValueError,'起点已变化'):worker.action('return',None)
            self.assertEqual(worker.mode,'hold')

    def test_gripper_during_drag_uses_its_own_sdk_packet(self):
        session=HardwareSession.__new__(HardwareSession);session.mode='teach'
        motors=[Mock() for i in range(7)];driver=Mock()
        session.arm=SimpleNamespace(config={'gripper_bounds_rad':[0,2], 'gripper_torque_cap_nm':1},_motors=motors,_driver=driver,_moving=False)
        session.gripper(.2,.3,.3,{'q':[0]*6})
        motors[6].pos_vel_MAXtqe.assert_called_once_with(.2,.3,.3)
        for motor in motors[:6]:motor.pos_vel_MAXtqe.assert_not_called();motor.pos_vel_tqe_kp_kd.assert_not_called()
        driver.motor_send_cmd.assert_called_once()

    def test_nonfinite_gravity_is_rejected_before_any_torque_packet(self):
        session=HardwareSession.__new__(HardwareSession);session.last_send=None
        session.gravity=lambda q:[float('nan')]*6
        session.arm=SimpleNamespace(config={'gravity_compensation':{'max_velocity_rad_s':1.,'max_command_gap_s':.2}},_send_gravity=Mock())
        with self.assertRaisesRegex(RuntimeError,'模型输出无效'):session.teach_tick({'dq':[0.]*6,'q':[0.]*6})
        session.arm._send_gravity.assert_not_called()

    def test_long_running_episode_does_not_stop_until_explicit_end(self):
        import time
        with tempfile.TemporaryDirectory() as temp:
            worker=self.worker(temp);worker.session=DemoSession();worker.mode='teach'
            worker.recording=True;worker.record_started=time.monotonic()-10000;worker.next_frame=0.
            worker.camera.latest.return_value=dict(monotonic_s=time.monotonic(),sequence=1,image=np.zeros((16,16,3),np.uint8))
            worker.tick()
            self.assertTrue(worker.recording)
            self.assertIsNone(worker.ending)
            self.assertEqual(worker.store.queue.get_nowait()[0],'frame')
            worker.action('end',None) if worker.allowed.is_set() else worker.end_record()
            self.assertFalse(worker.recording)
            self.assertNotIn('duration_s',validate_options({'duration_s':.01}))

    def test_invalid_settings_rejected(self):
        for values in ({'gripper_torque':2},{'fps':0},{'return_scale':1},{'return_duration_s':float('nan')},{'repo_id':'../../invalid'}):
            with self.assertRaises(ValueError):validate_options(values)


class RecordingContracts(unittest.TestCase):
    def test_failed_and_discarded_episode_never_calls_dataset_export(self):
        with tempfile.TemporaryDirectory() as temp:
            store=EpisodeStore(lambda *x:None,base=temp)
            folder=Path(temp)/'episode';folder.mkdir()
            for outcome in ('failure','discard'):
                atomic_json(folder/'episode.json',dict(status='review',outcome='unlabelled',quality_ok=True))
                (folder/'samples.jsonl').write_text('')
                with patch('apps.teaching_panel.recorder.export_episode') as export:
                    store.handle('decide',(str(folder),outcome));export.assert_not_called()

    def test_invalid_new_episode_does_not_overwrite_previous_metadata(self):
        with tempfile.TemporaryDirectory() as temp:
            store=EpisodeStore(lambda *x:None,base=temp)
            previous=Path(temp)/'previous';previous.mkdir()
            atomic_json(previous/'episode.json',{'status':'exported','sentinel':42})
            store.folder=previous;store.meta={'status':'exported','sentinel':42}
            store.start()
            try:
                store.post('begin',dict(repo_id='../../bad',source='demo'))
                store.queue.join()
                self.assertEqual(json.loads((previous/'episode.json').read_text()),{'status':'exported','sentinel':42})
            finally:store.shutdown();store.join()

    def test_interrupted_frames_retain_raw_but_are_excluded(self):
        with tempfile.TemporaryDirectory() as temp:
            store=EpisodeStore(lambda *x:None,base=temp)
            store.handle('begin',dict(repo_id='demo/test',task='test',fps=20,source='demo'))
            for i in range(6):
                store.handle('frame',(dict(monotonic_s=i*.05,relative_s=i*.05,observation=[0.]*7,action=[0.]*7),np.zeros((16,16,3),np.uint8)))
            store.handle('end','interrupted')
            meta,rows=read_episode(store.folder)
            self.assertFalse(meta['quality_ok']);self.assertEqual(len(rows),6)
            self.assertTrue((store.folder/'images/frame-000000.png').is_file())


if __name__=='__main__':unittest.main()
