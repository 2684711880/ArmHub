from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from PyQt5 import QtWidgets
from apps.drawing_panel.app import DrawingPanel
from apps.movel.moveL_error_analysis import save_run


class DrawingPanelUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.qt=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_replay_disables_hardware_controls_without_worker(self):
        rec=dict(name='synthetic',duration_s=1.,settle_s=0.,fraction=1.,completed=True,error=None,
            tags=dict(plane='xy',shape='line_u',direction='fwd'),
            waypoints=np.array([[0.,0.,.1],[.04,0,.1]]),plan_p=np.array([[0.,0.,.1],[.04,0,.1]]),
            fb_p=np.array([[0.,0.,.1],[.039,.001,.1]]),fb_q=np.zeros((2,6)))
        with tempfile.TemporaryDirectory() as folder:
            save_run(folder,[rec],dict(source='synthetic_ui_test'))
            with patch('apps.drawing_panel.app.HardwareWorker',side_effect=AssertionError('Replay must not construct a worker')):
                panel=DrawingPanel(replay=Path(folder))
                self.assertIsNone(panel.worker)
                self.assertFalse(panel.connect_btn.isEnabled())
                self.assertTrue(all(not b.isEnabled() for b in panel.motion_buttons))
                self.assertEqual(len(panel.canvas.traces[0]['points']),2)
                panel.close()

    def test_execution_checkbox_and_draw_button_command_are_explicit(self):
        with patch('apps.drawing_panel.app.HardwareWorker') as cls:
            worker=cls.return_value;worker.submit.return_value=True;worker.is_alive.return_value=False
            panel=DrawingPanel()
            panel.receive('connected',True)
            self.assertFalse(panel.draw_fwd.isEnabled())
            panel.allow.setChecked(True)
            self.assertTrue(panel.draw_fwd.isEnabled())
            panel.draw_fwd.click()
            args=worker.submit.call_args.args
            self.assertEqual(args[0],'draw');self.assertEqual(args[2: ],('fwd',True))
            self.assertTrue(panel.busy)
            self.assertFalse(panel.draw_rev.isEnabled())
            panel.allow.setChecked(False)
            worker.request_stop.assert_called_once()
            panel.close()


if __name__=='__main__':unittest.main()
