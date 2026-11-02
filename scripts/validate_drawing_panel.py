"""Validate actual panel/worker read-only startup; never requests enable/motion."""
import json
from pathlib import Path
import sys
import time
from PyQt5 import QtCore,QtWidgets
from apps.drawing_panel.app import DrawingPanel

root=Path(__file__).resolve().parents[1]
app=QtWidgets.QApplication(sys.argv[:1])
panel=DrawingPanel(auto_connect=True);panel.show()
started=time.monotonic();report={};log_path=None;data_dir=panel.worker.directory

def inspect():
    global log_path
    if panel.connected and not panel.busy and panel.last_feedback is not None:
        log_path=panel.worker.arm.log_path
        report.update(status='passed',mode='live_hardware_read_only',
                      software_enabled=panel.worker.arm._enabled,
                      feedback_position_m=panel.last_feedback['position'],
                      session_directory=str(data_dir))
        panel.grab().save(str(root/'validation/drawing_panel/live_read_only.png'))
        timer.stop();panel.close()
    elif time.monotonic()-started>35:
        report.update(status='failed',reason='Read-only connection timed out')
        timer.stop();panel.close()

timer=QtCore.QTimer();timer.timeout.connect(inspect);timer.start(100)
app.exec_()
if log_path is not None:
    events=[json.loads(x)['event'] for x in log_path.read_text().splitlines()]
    report['motor_command_packets']=events.count('command')
    report['driver_log']=str(log_path)
    assert not report['software_enabled'] and report['motor_command_packets']==0
(root/'validation/drawing_panel/read_only_startup.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
if report.get('status')!='passed':raise SystemExit(1)
