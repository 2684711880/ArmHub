"""只创建离线动力学模型；调用官方方法，但绝不构造硬件 Robot。"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace


class PantheraGravityModel:
    def __init__(self):
        root = Path(__file__).resolve().parents[2]
        sdk = root / 'integrations/panthera_lerobot/Panthera-HT_SDK/panthera_python'
        sys.path.insert(0, str(sdk / 'scripts'))
        try:
            import numpy as np
            import pinocchio as pin
            import yaml
            from Panthera_lib import Panthera
        except ImportError as error:
            raise RuntimeError('重力补偿需要独立动力学环境；运行 bash scripts/setup_dynamics.sh') from error
        self.np = np
        self.pin = pin
        config_path = sdk / 'robot_param/Follower.yaml'
        config = yaml.safe_load(config_path.read_text())
        self.model_path = (config_path.parent / config['urdf']['file_path']).resolve()
        self.model = pin.buildModelFromUrdf(str(self.model_path))
        self.data = self.model.createData()
        self._context = SimpleNamespace(model=self.model, data=self.data)
        self._official_gravity = Panthera.get_Gravity
        self._joints = []
        for name in [f'joint{i}' for i in range(1, 7)]:
            if not self.model.existJointName(name):
                raise RuntimeError(f'动力学模型缺少 {name}')
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise RuntimeError('只支持六个标量转动关节的 Panthera 模型')
            self._joints.append((joint.idx_q, joint.idx_v))

    def configuration(self, positions):
        positions = tuple(float(value) for value in positions)
        if len(positions) != 6 or not all(math.isfinite(value) for value in positions):
            raise ValueError('重力模型需要六个有限关节角')
        # 官方 follower 模型只有六个转动关节。
        q = self.np.zeros(self.model.nq)
        for value, (index, _) in zip(positions, self._joints):
            q[index] = value
        return q

    def __call__(self, positions):
        q = self.configuration(positions)
        gravity = self._official_gravity(self._context, q)
        result = tuple(float(gravity[index]) for _, index in self._joints)
        if not all(math.isfinite(value) for value in result):
            raise RuntimeError('官方重力模型返回非有限力矩')
        return result
