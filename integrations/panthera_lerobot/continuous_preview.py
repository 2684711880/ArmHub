"""One camera owner across recording, review and preparation; no arm commands."""
import json
import logging
from pathlib import Path
import threading
import time


class CachedCamera:
    def __init__(self, camera, publish, status_path=None):
        self.camera = camera
        self.publish = publish
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.frame = None
        self.frame_time = 0.0
        self.error = None
        self.thread = None
        self.display_thread = None
        self.sequence = 0
        self.published = 0
        self.display_error = None
        self.status_path = Path(status_path) if status_path else None
        self.last_status = 0.0

    def __getattr__(self, name):
        return getattr(self.camera, name)

    @property
    def is_connected(self):
        return self.camera.is_connected

    def start(self):
        self.camera.connect()
        self.thread = threading.Thread(target=self._run, name='lerobot-camera-preview', daemon=True)
        self.thread.start()
        self.async_read(timeout_ms=3000)
        self.display_thread = threading.Thread(target=self._display, name="lerobot-camera-display", daemon=True)
        self.display_thread.start()

    def _status(self, force=False):
        now = time.monotonic()
        if not self.status_path or (not force and now - self.last_status < 1):
            return
        self.last_status = now
        with self.condition:
            row = dict(unix_s=time.time(), frames=self.sequence, published=self.published,
                       frame_age_s=now-self.frame_time if self.frame_time else None,
                       read_error=str(self.error) if self.error else None,
                       display_error=self.display_error, stopped=self.stop.is_set())
        try:
            self.status_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.status_path.with_suffix('.tmp')
            temporary.write_text(json.dumps(row, ensure_ascii=False, indent=2)+'\n')
            temporary.replace(self.status_path)
        except OSError:
            logging.exception('无法写入相机状态日志')

    def _run(self):
        # Keep polling through a transient stall. A stale cache still fails async_read,
        # so recording cannot silently reuse an image while waiting for recovery.
        failures = 0
        try:
            while not self.stop.is_set():
                try:
                    frame = self.camera.async_read(timeout_ms=250).copy()
                except Exception as exc:
                    if self.stop.is_set():
                        break
                    failures += 1
                    with self.condition:
                        self.error = exc
                        self.condition.notify_all()
                    if failures == 1 or failures % 40 == 0:
                        logging.warning('D435 暂无新图像，后台继续尝试恢复：%s', exc)
                    self._status()
                    self.stop.wait(.05)
                    continue
                with self.condition:
                    self.frame = frame
                    self.frame_time = time.monotonic()
                    self.sequence += 1
                    self.error = None
                    self.condition.notify_all()
                if failures:
                    logging.info('D435 新图像已恢复')
                failures = 0
                self._status()
        finally:
            self._status(force=True)

    def _display(self):
        sequence = 0
        while not self.stop.is_set():
            with self.condition:
                self.condition.wait_for(lambda: self.stop.is_set() or self.sequence > sequence, timeout=.25)
                if self.stop.is_set():
                    break
                if self.sequence <= sequence:
                    continue
                sequence = self.sequence
                frame = self.frame.copy()
            try:
                self.publish(frame)
                with self.condition:
                    self.published = sequence
                    self.display_error = None
            except Exception as exc:
                with self.condition:
                    first_failure = self.display_error is None
                    self.display_error = str(exc)
                if first_failure:
                    logging.exception('Rerun 显示异常；相机读取继续运行')
                self.stop.wait(.1)

    def async_read(self, timeout_ms=200):
        deadline = time.monotonic() + timeout_ms / 1000
        with self.condition:
            while True:
                if self.stop.is_set():
                    raise RuntimeError('D435 预览已关闭')
                if self.error is None and self.frame is not None and time.monotonic() - self.frame_time < 0.5:
                    return self.frame.copy()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('D435 没有新鲜图像；后台仍在尝试恢复') from self.error
                self.condition.wait(remaining)

    def connect(self, **kwargs):
        # The original camera is already connected; only validate freshness.
        self.async_read(timeout_ms=1000)

    def disconnect(self):
        # The recorder releases serial between rounds, but preview keeps ownership.
        pass

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        if self.thread is not None:
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                raise RuntimeError('相机预览线程未退出，不能并发关闭相机')
        if self.display_thread is not None:
            self.display_thread.join(timeout=1)
            if self.display_thread.is_alive():
                logging.warning('Rerun 显示线程仍在等待；相机读取已停止')
        if self.camera.is_connected:
            self.camera.disconnect()
        self._status(force=True)


class ContinuousPreview:
    def __init__(self, robot, publish=None, status_dir=None):
        self.robot = robot
        self.original = dict(robot.cameras)
        self.cameras = {}
        if publish is None:
            if status_dir is None:
                status_dir = Path(__file__).resolve().parents[2]/"logs/lerobot/camera"
            import rerun as rr
            import rerun.blueprint as rrb
            rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(
                *[rrb.Spatial2DView(origin=f'observation.{key}', name=key) for key in self.original],
                rrb.TimeSeriesView(origin='/', name='关节与动作'),
            )))
            publish = lambda key, frame: rr.log(f'observation.{key}', rr.Image(frame))
        try:
            for key, camera in self.original.items():
                if getattr(camera, 'use_depth', False):
                    raise ValueError('持续预览当前仅支持 RGB 采集')
                status_path = Path(status_dir)/f"{key}.json" if status_dir else None
                proxy = CachedCamera(camera, lambda frame, key=key: publish(key, frame), status_path)
                self.cameras[key] = proxy
                proxy.start()
            robot.cameras.update(self.cameras)
            robot._armhub_continuous_preview = self
        except BaseException:
            self.close()
            raise

    def check_ready(self):
        for camera in self.cameras.values():
            camera.connect()

    def close(self):
        try:
            for camera in self.cameras.values():
                camera.close()
        finally:
            self.robot.cameras.update(self.original)
            self.robot._armhub_continuous_preview = None
