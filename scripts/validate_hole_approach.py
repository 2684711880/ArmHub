"""历史 RGB-D 检测与离线叠加预览；不连接相机或串口。"""
import json
from pathlib import Path
import cv2
import numpy as np
from perception.detectors.depth_hole import detect_depth_hole
from apps.hole_approach.overlay import HoleOverlay

ROOT=Path(__file__).resolve().parents[1]
with np.load(ROOT/'validation/hole_approach/recorded_rgbd.npz',allow_pickle=False) as frame:
    depth=frame['depth_m'];k=frame['K'];rgb=frame['rgb']
    meta=json.loads(str(frame['metadata'].item()))
hole=detect_depth_hole(depth,k)
image=cv2.applyColorMap(np.uint8(np.clip(np.nan_to_num(depth)/1.5,0,1)*255),cv2.COLORMAP_TURBO)
cv2.ellipse(image,hole['ellipse'],(0,255,0),2)
image[-120:,-160:]=cv2.resize(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),(160,120))
cv2.putText(image,'OFFLINE RECORDED FRAME - no arm / no live camera',(8,24),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
packet={'stamp':meta['wall_time'],'K':k.tolist(),'measured_center_optical_m':hole['center'].tolist()}
preview=HoleOverlay().render(image,packet,{})
path=ROOT/'validation/hole_approach/recorded_preview.png'
assert cv2.imwrite(str(path),preview)
report={'status':'passed','source':'historical_recorded_rgbd','recorded_wall_time':meta['wall_time'],
        'center_optical_m':hole['center'].tolist(),'normal_optical':hole['normal'].tolist(),
        'diameter_m':2*hole['radius'],'plane_rms_m':hole['plane_rms'],
        'arm_connected':False,'camera_opened':False,'motion_sent':False,'preview':str(path)}
(ROOT/'validation/hole_approach/recorded_detection.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
