"""Real CameraWorker call sequence: frame, transient timeout, frame."""
import threading
import time
import unittest
import numpy as np
from apps.teaching_panel.camera import CameraWorker


class BriefStallCamera:
    def __init__(self):
        self.calls=0;self.timeout_seen=threading.Event();self.disconnected=False
    def connect(self):pass
    def disconnect(self):self.disconnected=True
    def async_read(self,timeout_ms=250):
        self.calls+=1
        if self.calls==2:
            self.timeout_seen.set()
            raise TimeoutError('Timed out after 250 ms. Read thread alive: True.')
        time.sleep(.01)
        return np.full((16,16,3),self.calls%255,np.uint8)


class CameraRecoveryTests(unittest.TestCase):
    def test_short_timeout_does_not_close_preview_and_frames_resume(self):
        camera=BriefStallCamera();events=[]
        worker=CameraWorker(lambda event,value:events.append((event,value)),camera_factory=lambda:camera)
        worker.start();worker.open()
        try:
            self.assertTrue(camera.timeout_seen.wait(1))
            deadline=time.monotonic()+1
            while time.monotonic()<deadline and worker.counter<3:time.sleep(.01)
            self.assertGreaterEqual(worker.counter,3,'短暂超时后应该继续收到图像')
            self.assertTrue(worker.wanted.is_set())
            self.assertFalse(camera.disconnected)
        finally:worker.shutdown();worker.join(timeout=2)
        self.assertFalse(worker.is_alive())


    def test_sustained_timeout_closes_camera_instead_of_publishing_stale_frames(self):
        camera=BriefStallCamera()
        def timeout(**kwargs):raise TimeoutError('no frames')
        camera.async_read=timeout
        events=[]
        worker=CameraWorker(lambda event,value:events.append((event,value)),camera_factory=lambda:camera,stall_timeout_s=.05)
        worker.start();worker.open()
        try:
            deadline=time.monotonic()+1
            while time.monotonic()<deadline and worker.wanted.is_set():time.sleep(.01)
            self.assertFalse(worker.wanted.is_set())
            self.assertTrue(camera.disconnected)
            self.assertIsNone(worker.latest())
            self.assertTrue(any(event=='error' for event,value in events))
        finally:worker.shutdown();worker.join(timeout=2)



if __name__=='__main__':unittest.main()
