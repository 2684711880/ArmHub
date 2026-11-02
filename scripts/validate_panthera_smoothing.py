"""离线复核真实 Ruckig 轨迹及模型碰撞；不连接机械臂。"""
import json
from pathlib import Path
import numpy as np

from armhub.adapters.panthera_kinematics import PantheraKinematics
from armhub.motion import JointTransition

root=Path(__file__).resolve().parents[1]
config=json.loads((root/'robots/panthera_ht/hardware/config.json').read_text())
model=PantheraKinematics(config['lower_rad'],config['upper_rad'])
# 上一次 home 预览中的实测起点，仅作为历史离线 fixture。
old=json.loads((root/'validation/hardware/panthera_ht/home_preview.json').read_text())
home_start=old['preview']['start_joints_rad']
start=(-.0464955717,0.,-.0025132741,.0263893791,-.089849554,.01759292)
target=list(model.fk(start)['position_m']);target[2]+=.01
solved=model.solve(start,target)
results=[]
for name,q,goal in [('home_historical_fixture',home_start,(0.,)*6),
                    ('move_to_z_plus_1cm_fixture',start,solved['target_joints_rad'])]:
    minimum=max(2.,max(abs(a-b) for a,b in zip(q,goal))/.1)
    curve=JointTransition(q,goal,config,speed_limit=.1,minimum_duration=minimum)
    times=np.linspace(0,curve.duration,4001)
    sampled=[curve.sample(t) for t in times]
    v=np.array([r[1] for r in sampled]);a=np.array([r[2] for r in sampled])
    jerk=np.diff(a,axis=0)/(times[1]-times[0])
    peak_v=np.max(abs(v),axis=0);peak_a=np.max(abs(a),axis=0);peak_j=np.max(abs(jerk),axis=0)
    assert np.max(peak_v)<=.10000001
    assert np.all(peak_a<=np.array(config['motion_smoothing']['acceleration_limits_rad_s2'])+1e-7)
    assert np.all(peak_j<=np.array(config['motion_smoothing']['jerk_limits_rad_s3'])+1e-6)
    assert max(abs(x) for x in sampled[0][1])<1e-9
    assert sampled[-1][0]==tuple(goal) and sampled[-1][1]==(0.,)*6
    collision=model.check_trajectory(curve,config['cartesian']['collision_step_rad'])
    results.append(dict(case=name, **curve.summary(), collision_check=collision,
                        sampled_peak_velocity_rad_s=peak_v.tolist(),
                        sampled_peak_acceleration_rad_s2=peak_a.tolist(),
                        sampled_peak_jerk_rad_s3=peak_j.tolist()))
report={'status':'passed','mode':'offline_no_hardware', 'cases':results,
        'note':'实际固件速度为上限接口；离线参考约束通过不证明实物平顺性'}
path=root/'validation/hardware/panthera_ht/smoothing_offline.json'
path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
