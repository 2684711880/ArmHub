"""独立环境下的离线动力学验证，不构造原生硬件 Robot。"""
import json
from pathlib import Path

from armhub.adapters.panthera_gravity import PantheraGravityModel


def main():
    model = PantheraGravityModel()
    records = []
    for pose in [(0,0,0,0,0,0), (0.1,0.5,0.6,-0.2,0.3,0.4), (-0.2,1.2,1.4,0.5,-0.6,0.8)]:
        actual = model(pose)
        q = model.configuration(pose)
        gradient = []
        for index, _ in model._joints:
            upper, lower = q.copy(), q.copy()
            upper[index] += 1e-6
            lower[index] -= 1e-6
            a = model.pin.computePotentialEnergy(model.model,model.data,upper)
            b = model.pin.computePotentialEnergy(model.model,model.data,lower)
            gradient.append(float((a-b)/2e-6))
        error = max(abs(a-b) for a,b in zip(actual,gradient))
        if error > 1e-5:
            raise AssertionError(f'重力与势能梯度不一致：{error}')
        records.append({'pose_rad':pose,'gravity_nm':actual,'potential_gradient_nm':gradient,'max_error_nm':error})
    report = {'source':'offline_model_only','hardware_connected':False,
              'model_nq':model.model.nq,'model_nv':model.model.nv,'checks':records,'passed':True}
    path = Path('validation/gravity/dynamics_check.json')
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
