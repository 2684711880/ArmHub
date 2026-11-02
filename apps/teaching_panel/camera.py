"""Camera owns its thread; UI/control consume timestamped copies without waiting."""
import json
from pathlib import Path
import threading
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]


class CameraWorker(threading.Thread):
    def __init__(self, emit, demo=False, camera_factory=None, stall_timeout_s=2.0):
        super().__init__(name='teaching-camera',daemon=False)
        self.camera_factory=camera_factory or self.create_camera
        self.stall_timeout_s=stall_timeout_s;self.last_success=0.;self.waiting=False
        self.emit=emit; self.demo=demo; self.exit_event=threading.Event()
        self.wanted=threading.Event();self.lock=threading.Lock();self.frame=None;self.camera=None;self.counter=0

    @staticmethod
    def create_camera():
        from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
        from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
        settings=json.loads((ROOT/'integrations/panthera_lerobot/configs/local.json').read_text())['cameras']
        if set(settings)!={'d435'}:
            raise ValueError('示教窗口当前需要配置一个名为 d435 的相机')
        config={k:v for k,v in settings['d435'].items() if k!='type'}
        return RealSenseCamera(RealSenseCameraConfig(**config))

    def open(self):
        self.wanted.set();self.emit('camera_state','opening')
    def close(self):
        self.wanted.clear();self.emit('camera_state','closed')
    def shutdown(self): self.exit_event.set();self.wanted.clear()

    def latest(self):
        with self.lock:
            return self.frame  # Frames are immutable after publication.

    def run(self):
        try:
            while not self.exit_event.is_set():
                if not self.wanted.is_set():
                    if self.camera is not None:
                        self.camera.disconnect();self.camera=None
                    with self.lock:self.frame=None
                    self.exit_event.wait(.05);continue
                try:
                    if self.demo:
                        image=np.full((480,640,3),(226,234,233),np.uint8)
                        # Synthetic tabletop/box/block, clearly labelled in the UI.
                        image[75:405,45:595]=(205,185,151)
                        image[150:300,395:555]=(105,134,143)
                        x=int(200+20*np.sin(time.monotonic()))
                        image[245:300,x:x+55]=(206,100,60)
                        image[170:235,x+8:x+23]=(75,91,102)
                        image[170:235,x+33:x+48]=(75,91,102)
                    else:
                        if self.camera is None:
                            self.camera=self.camera_factory()
                            self.camera.connect()
                            self.last_success=time.monotonic();self.waiting=False
                        try:
                            image=self.camera.async_read(timeout_ms=250)
                        except TimeoutError:
                            # SDK producer waits up to 500 ms; a 250 ms consumer timeout is not a disconnect.
                            if not self.wanted.is_set() or self.exit_event.is_set():continue
                            if time.monotonic()-self.last_success>=self.stall_timeout_s:
                                raise RuntimeError(f'连续 {self.stall_timeout_s:g} 秒未收到相机图像，预览已停止；可重新打开相机')
                            if not self.waiting:
                                self.emit('camera_state','waiting')
                                self.emit('log','相机暂未返回新帧，保留连接并重试。')
                                self.waiting=True
                            self.exit_event.wait(.02)
                            continue
                        if self.waiting:self.emit('log','相机图像已恢复。')
                        self.last_success=time.monotonic();self.waiting=False
                    if not self.wanted.is_set() or self.exit_event.is_set():continue
                    now=time.monotonic();self.counter+=1
                    packet=dict(image=np.ascontiguousarray(image).copy(),monotonic_s=now,
                                sequence=self.counter,source='demo' if self.demo else 'hardware')
                    with self.lock:self.frame=packet
                    self.emit('camera_frame',packet)
                    self.exit_event.wait(1/30 if self.demo else .003)
                except Exception as exc:
                    self.emit('error',f'相机：{exc}');self.wanted.clear();self.emit('camera_state','error')
                    if self.camera is not None:
                        try:self.camera.disconnect()
                        except Exception:pass
                        self.camera=None
                    with self.lock:self.frame=None
        finally:
            if self.camera is not None:
                try:self.camera.disconnect()
                except Exception as exc:self.emit('error',f'关闭相机：{exc}')
