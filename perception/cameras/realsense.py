"""Aligned RGB-D with depth units and distortion checks; no arm control."""
import json
import time
from pathlib import Path
import numpy as np

class SnapshotCamera:
 def __init__(self,path):self.path=Path(path)
 def read(self):
  with np.load(self.path,allow_pickle=False) as f:rgb=f['rgb'].copy();depth=f['depth_m'].copy();k=f['K'].copy();meta=json.loads(str(f['metadata'].item()))
  if not 0<=time.time()-meta['wall_time']<=1.5:raise RuntimeError('RGB-D snapshot stale; run scripts/live_detect.py or use --camera-source live')
  if np.any(np.abs(meta.get('coeffs',[0]*5))>1e-8):raise RuntimeError('Nonzero camera distortion needs an undistorted RGB-D source')
  return rgb,depth,k,meta['wall_time']
 def close(self):pass
class LiveCamera:
 def __init__(self,serial=None,width=640,height=480,fps=30):
  import pyrealsense2 as rs
  self.rs=rs;self.pipe=rs.pipeline();cfg=rs.config();cfg.enable_stream(rs.stream.color,width,height,rs.format.rgb8,fps);cfg.enable_stream(rs.stream.depth,width,height,rs.format.z16,fps)
  if serial:cfg.enable_device(serial)
  self.profile=self.pipe.start(cfg);self.align=rs.align(rs.stream.color)
  try:
   for _ in range(30):self.pipe.wait_for_frames(5000)
  except Exception:
   self.pipe.stop();raise
 def read(self):
  f=self.align.process(self.pipe.wait_for_frames(5000));c=f.get_color_frame();d=f.get_depth_frame()
  if not c or not d:raise RuntimeError('Missing aligned RGB-D')
  intr=c.profile.as_video_stream_profile().intrinsics
  if np.any(np.abs(intr.coeffs)>1e-8):raise RuntimeError('Nonzero camera distortion needs undistortion')
  k=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]])
  return np.asanyarray(c.get_data()).copy(),np.asanyarray(d.get_data()).astype(float)*d.get_units(),k,time.time()
 def close(self):self.pipe.stop()
