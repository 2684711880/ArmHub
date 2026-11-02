"""Offline replan of recorded circles. Never connects/enables hardware."""
import argparse
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import numpy as np

from armhub import create_arm
from apps.movel.moveL_pos_control import PlanningRobot,checked
from apps.movel.circle_trajectory import plan_circle_curve
from apps.movel.moveL_error_analysis import load_run
from apps.movel.moveL_trajectories import path_length

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path)
p.add_argument('--speed-mm-s',type=float,default=20.)
p.add_argument('--output',type=Path,default=Path('validation/drawing_panel/circle_timing_fixed.json'))
args=p.parse_args()
if not np.isfinite(args.speed_mm_s) or not 1<=args.speed_mm_s<=100:p.error('速度需在 1–100 mm/s')
arm=create_arm('panthera_ht','hardware')
_,records=load_run(args.source)
results=[]
for rec in records:
    if rec.get('tags',{}).get('shape')!='circle':continue
    robot=PlanningRobot(arm,rec['plan_q'][0])
    w=[dict(position=position,rotation=rec['target_rotation'].copy()) for position in rec['waypoints']]
    with redirect_stdout(io.StringIO()):
        curve=plan_circle_curve(robot,w,rec['plan_q'][0],path_length(w)/(args.speed_mm_s/1000))
        checks=checked(robot,curve,rec['plan_q'][0],.05)
    results.append(dict(name=rec['name'],recorded_duration_s=rec['duration_s'],new_duration_s=curve.duration,
                        speedup=rec['duration_s']/curve.duration,checks=checks))
assert results and arm._driver is None
report=dict(status='offline_passed_no_hardware_motion',source=str(args.source),requested_speed_mm_s=args.speed_mm_s,
            acceleration_limits_rad_s2=arm.config['motion_smoothing']['acceleration_limits_rad_s2'],
            jerk_limits_rad_s3=arm.config['motion_smoothing']['jerk_limits_rad_s3'],circles=results,
            note='Planning/FK/position/dynamics/self-collision checks only; real tracking remains to be measured')
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
y=np.arange(len(results))
a=ax.barh(y-.17,[r['recorded_duration_s'] for r in results],.32,label='Previous planned duration',color='#8193a1')
b=ax.barh(y+.17,[r['new_duration_s'] for r in results],.32,label='New planned duration (offline)',color='#078c84')
ax.bar_label(a,fmt='%.2f s',padding=3);ax.bar_label(b,fmt='%.2f s',padding=3)
ax.set_yticks(y,[r['name'] for r in results]);ax.set_xlabel('Seconds');ax.set_title('Circle timing | same acceleration and jerk limits')
ax.margins(x=.2);ax.grid(axis='x',alpha=.2);ax.legend()
fig.savefig(args.output.with_suffix('.png'),dpi=160);plt.close(fig)
print(json.dumps(dict(status=report['status'],circles=[{k:r[k] for k in ['name','recorded_duration_s','new_duration_s','speedup']} for r in results]),ensure_ascii=False,indent=2))
