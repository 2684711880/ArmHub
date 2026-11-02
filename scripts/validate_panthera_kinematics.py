"""纯离线验证：官方 FK/IK、模型 TCP、自碰撞负例；无 Robot 构造。"""
import json
import math
from pathlib import Path

from armhub.adapters.panthera_kinematics import PantheraKinematics

root = Path(__file__).resolve().parents[1]
config = json.loads((root/'robots/panthera_ht/hardware/config.json').read_text())
model = PantheraKinematics(config['lower_rad'], config['upper_rad'])
results = []
for q in [(0,0,0,0,0,0),(-.042,.273,.227,.074,.036,.023)]:
    pose = model.fk(q)
    target = list(pose['position_m'])
    target[2] += .01
    plan = model.solve(q, target)
    check = model.check_path(q, plan['target_joints_rad'])
    fullq = model.np.zeros(model.model.nq)
    for name, value in zip(model.sdk.joint_names,q):
        fullq[model.model.joints[model.model.getJointId(name)].idx_q] = value
    model.pin.framesForwardKinematics(model.model,model.data,fullq)
    tcp = model.data.oMf[model.model.getFrameId('gripper_center')].translation
    frame_error = math.dist(tcp,pose['position_m'])
    assert frame_error < 1e-9, frame_error
    results.append(dict(plan, collision_check=check, model_tcp_error_m=frame_error))
rejected = []
for label,target,quat in [('unreachable',(5,5,5),None),('bad_quaternion',(.25,0,.2),(0,0,0,0))]:
    try:
        model.solve((0,)*6,target,quat)
    except ValueError:
        rejected.append(label)
    else:
        raise AssertionError(label)
rng = model.np.random.default_rng(258)
collision_q = None
for attempt in range(300):
    q = rng.uniform(model.lower, model.upper)
    try:
        model.check_path(q,q)
    except ValueError as error:
        collision_q = q.tolist()
        rejected.append('self_collision')
        collision_detail = str(error)
        break
assert collision_q is not None, '未找到碰撞负例'
report = {'mode':'offline_no_hardware','status':'passed','plans':results,
          'negative_cases_rejected':rejected,'collision_negative_joint_rad':collision_q,
          'collision_detail':collision_detail}
path = root/'validation/hardware/panthera_ht/xyz_offline.json'
path.parent.mkdir(parents=True,exist_ok=True)
path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
