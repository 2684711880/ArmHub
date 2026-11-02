import threading
import time
import unittest
import tempfile
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from continuous_preview import ContinuousPreview, CachedCamera


class FakeCamera:
    def __init__(self):
        self.is_connected = False
        self.connects = self.closes = self.reads = 0
        self.read_threads = set()
        self.fail = False

    def connect(self):
        if self.is_connected:
            raise RuntimeError('duplicate pipeline')
        self.connects += 1
        self.is_connected = True

    def disconnect(self):
        self.closes += 1
        self.is_connected = False

    def async_read(self, timeout_ms=250):
        self.read_threads.add(threading.current_thread().name)
        time.sleep(.005)
        if self.fail:
            raise RuntimeError('USB disconnected')
        self.reads += 1
        return np.full((4, 4, 3), self.reads % 255, dtype=np.uint8)


class PreviewTests(unittest.TestCase):
    def test_one_pipeline_keeps_preview_between_rounds_and_releases_on_exit(self):
        original = FakeCamera()
        robot = SimpleNamespace(cameras={'d435': original})
        published = threading.Event()
        preview = ContinuousPreview(robot, publish=lambda k, f: published.set())
        try:
            camera = robot.cameras['d435']
            for _ in range(3):
                camera.connect()
                self.assertEqual(camera.async_read().shape, (4, 4, 3))
                camera.disconnect()
                self.assertTrue(original.is_connected)
            published.clear()
            self.assertTrue(published.wait(.5), 'waiting phase must still publish')
            self.assertEqual(original.connects, 1)
            self.assertEqual(original.closes, 0)
            self.assertEqual(original.read_threads, {'lerobot-camera-preview'})
            image = camera.async_read()
            image[:] = 0
            self.assertTrue(camera.async_read().any(), 'consumer cannot mutate cache')
        finally:
            preview.close()
        self.assertEqual(original.closes, 1)
        self.assertIs(robot.cameras['d435'], original)
        self.assertFalse(camera.thread.is_alive())
        self.assertFalse(camera.display_thread.is_alive())

    def test_camera_failure_rejects_cached_frame_and_new_round(self):
        original = FakeCamera()
        robot = SimpleNamespace(cameras={'d435': original})
        preview = ContinuousPreview(robot, publish=lambda *args: None)
        try:
            original.fail = True
            time.sleep(.1)
            with self.assertRaisesRegex(TimeoutError, '新鲜图像'):
                preview.check_ready()
        finally:
            preview.close()

    def test_long_stall_recovers_without_restarting_program(self):
        original = FakeCamera()
        preview = ContinuousPreview(SimpleNamespace(cameras={'d435':original}), publish=lambda *args:None)
        camera = preview.cameras['d435']
        try:
            original.fail = True
            time.sleep(2.2)  # The previous worker permanently exited after two seconds.
            self.assertTrue(camera.thread.is_alive())
            with self.assertRaises(TimeoutError):
                camera.async_read(timeout_ms=10)
            original.fail = False
            self.assertEqual(camera.async_read(timeout_ms=1000).shape, (4,4,3))
            self.assertEqual(original.connects, 1)
        finally:
            preview.close()

    def test_blocked_or_failing_display_does_not_stop_capture(self):
        entered = threading.Event()
        release = threading.Event()
        original = FakeCamera()
        def publish(*args):
            entered.set()
            release.wait(2)
            raise RuntimeError('viewer failure')
        preview = ContinuousPreview(SimpleNamespace(cameras={'d435':original}), publish=publish)
        camera = preview.cameras['d435']
        try:
            self.assertTrue(entered.wait(1))
            before = camera.sequence
            time.sleep(.08)
            self.assertGreater(camera.sequence, before)
            self.assertIsNotNone(camera.async_read())
            release.set()
            time.sleep(.15)
            self.assertTrue(camera.thread.is_alive())
            self.assertIsNotNone(camera.async_read())
        finally:
            release.set()
            preview.close()

    def test_keep_return_and_wait_do_not_interrupt_capture_or_save_extra_frames(self):
        import manual_review as gates
        original = FakeCamera()
        robot = SimpleNamespace(cameras={'d435':original}, is_connected=True)
        def disconnect():
            robot.cameras['d435'].disconnect()
            robot.is_connected=False
        robot.disconnect=disconnect
        with tempfile.TemporaryDirectory() as temp:
            preview = ContinuousPreview(robot, publish=lambda *args:None, status_dir=temp)
            camera = robot.cameras['d435']
            dataset = SimpleNamespace(episode_buffer={'size':12})
            try:
                def verify_preview(**kwargs):
                    before = camera.sequence
                    time.sleep(.08)
                    self.assertGreater(camera.sequence, before)
                    self.assertTrue(original.is_connected)
                    self.assertEqual(dataset.episode_buffer['size'],12)
                def ask(prompt):
                    verify_preview()
                    return 's'
                self.assertEqual(gates.review_round(robot,dataset,{},ask=ask),'keep')
                verify_preview()  # Saving on main thread.
                gates.reset_ready(robot,prepare=verify_preview)
                verify_preview()  # Waiting to place the next block.
                self.assertEqual(original.connects,1)
                self.assertEqual(original.closes,0)
            finally:
                preview.close()
            status=json.loads((Path(temp)/'d435.json').read_text())
            self.assertTrue(status['stopped'])
            self.assertGreater(status['frames'],1)
            self.assertGreater(status['published'],0)

    def test_stale_cache_cannot_be_recorded(self):
        proxy = CachedCamera(FakeCamera(), lambda frame: None)
        proxy.frame = np.zeros((4, 4, 3), dtype=np.uint8)
        proxy.frame_time = time.monotonic() - 1
        with self.assertRaisesRegex(TimeoutError, "新鲜图像"):
            proxy.async_read(timeout_ms=5)

    def test_startup_failure_closes_camera_without_connecting_arm(self):
        original = FakeCamera()
        original.fail = True
        robot = SimpleNamespace(cameras={'d435': original})
        with self.assertRaisesRegex(TimeoutError, '新鲜图像'):
            ContinuousPreview(robot, publish=lambda *args: None)
        self.assertEqual(original.closes, 1)
        self.assertIs(robot.cameras['d435'], original)


if __name__ == '__main__':
    unittest.main()
