"""GUI loop with blocked spawned workers; mocked drawing never opens a window."""
import time
import unittest
from unittest.mock import patch
import numpy as np
from apps.hole_approach import app
from apps.hole_approach.__main__ import arguments


def busy_planner(tasks,results,quit_event):
    quit_event.wait(.8)

def busy_camera(options,vision,display,quit_event):
    quit_event.wait(.8)

def busy_motor(options,calibration,status,commands,vision,tasks,results,statuses,generation,quit_event):
    statuses.put({'state':'BUSY','message':'Test-only blocked worker; no arm'})
    quit_event.wait(.8)


class UiProcessTests(unittest.TestCase):
    def run_keys(self,keys):
        a=arguments([]);a.no_gui=False
        times=[];original=app.publish_key;events=iter(keys)
        def publish(key,*args):
            times.append((key,time.monotonic()));return original(key,*args)
        with patch.object(app.cv2,'namedWindow'),patch.object(app.cv2,'resizeWindow'),patch.object(app.cv2,'imshow'),patch.object(app.cv2,'setMouseCallback'),patch.object(app.cv2,'waitKey',side_effect=lambda _:ord(next(events,'q'))),patch.object(app.cv2,'destroyAllWindows'),patch.object(app,'publish_key',side_effect=publish):
            app.run_hardware(a,np.eye(4),'test',(busy_planner,busy_camera,busy_motor))
        return times
    def test_keys_are_not_blocked_by_camera_or_ik_workers(self):
        times=self.run_keys('pt q')
        self.assertEqual([r[0] for r in times],list('pt q'))
        self.assertLess(times[-1][1]-times[0][1],.4)
    def test_overlay_frame_switch_does_not_publish_motion(self):
        times=self.run_keys('pfq')
        self.assertEqual([r[0] for r in times],list('pq'))

if __name__=='__main__':unittest.main()
