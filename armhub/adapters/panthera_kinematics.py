"""官方 FK/IK 的离线适配；从基座建模，不构造硬件 Robot。"""
from contextlib import redirect_stdout
import io
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


class PantheraKinematics:
    def __init__(self, lower, upper):
        root = Path(__file__).resolve().parents[2]
        sys.path.insert(0, str(root / 'vendor/panthera/sdk/official_python'))
        import numpy as np
        import pinocchio as pin
        from scipy.spatial.transform import Rotation
        from Panthera_lib import Panthera
        self.np, self.pin, self.Rotation = np, pin, Rotation
        self.lower, self.upper = np.array(lower), np.array(upper)
        tree = ET.parse(root / 'robots/panthera_ht/description/planning.urdf').getroot()
        # 只保留 base_link 的子树，排除仿真安装高度与地板，SDK XYZ 才是基座坐标。
        links = {'base_link'}
        while True:
            children = {j.find('child').get('link') for j in tree.findall('joint')
                        if j.find('parent').get('link') in links}
            if children <= links:
                break
            links |= children
        for item in list(tree):
            if item.tag == 'link' and item.get('name') not in links:
                tree.remove(item)
            elif item.tag == 'joint' and (item.find('parent').get('link') not in links
                                         or item.find('child').get('link') not in links):
                tree.remove(item)
        for mesh in tree.iter('mesh'):
            uri = mesh.get('filename')
            if not uri.startswith('package://armhub_sim/'):
                raise RuntimeError('存在未知模型资源引用')
            mesh.set('filename', str(root / uri.removeprefix('package://armhub_sim/')))
        self.xml = ET.tostring(tree, encoding='unicode')
        self.model = pin.buildModelFromXML(self.xml)
        self.data = self.model.createData()
        model, data = self.model, self.data
        class OfflineSDK:
            forward_kinematics = Panthera.forward_kinematics
            inverse_kinematics = Panthera.inverse_kinematics
            _inverse_kinematics_dls_single_impl = Panthera._inverse_kinematics_dls_single_impl
            _inverse_kinematics_dls_multi_init_impl = Panthera._inverse_kinematics_dls_multi_init_impl

            def get_current_pos(self):
                return self.seed.copy()

        self.sdk = OfflineSDK()
        self.sdk.model, self.sdk.data = model, data
        # LeRobot may have imported the frame-based official wrapper first.
        # Both SDK variants must refer to this model's actual gripper TCP.
        if not model.existFrame('gripper_center'):
            raise RuntimeError('运动学模型缺少 gripper_center 末端坐标系')
        self.sdk.end_effector_frame_id = model.getFrameId('gripper_center')
        self.sdk.joint_names = [f'joint{i}' for i in range(1, 7)]
        self.sdk.motor_count = 6
        self.sdk.joint_limits = {'lower': self.lower, 'upper': self.upper}
        self.geometry = None
        self.root = root

    @staticmethod
    def vector(values, length, label):
        result = tuple(float(value) for value in values)
        if len(result) != length or not all(math.isfinite(value) for value in result):
            raise ValueError(f'{label} 需要 {length} 个有限数')
        return result

    def fk(self, positions):
        q = self.vector(positions, 6, '关节角')
        result = self.sdk.forward_kinematics(q)
        return {'frame_id': 'base_link', 'tcp_frame': 'gripper_center',
                'position_m': tuple(float(v) for v in result['position']),
                'quaternion_xyzw': tuple(float(v) for v in self.Rotation.from_matrix(result['rotation']).as_quat())}

    def solve(self, positions, target, quaternion=None):
        seed = self.np.array(self.vector(positions, 6, '起点关节角'))
        target = self.vector(target, 3, '空间目标')
        before = self.fk(seed)
        desired = before['quaternion_xyzw'] if quaternion is None else self.vector(quaternion, 4, '四元数')
        if abs(math.sqrt(sum(v*v for v in desired))-1.0) > 1e-3:
            raise ValueError('四元数必须归一化')
        rotation = self.Rotation.from_quat(desired).as_matrix()
        # 仅裁剪数学初值中容许的零点噪声，不向电机发送零位命令。
        self.sdk.seed = self.np.clip(seed, self.lower, self.upper)
        output = io.StringIO()
        with redirect_stdout(output):
            solved = self.sdk.inverse_kinematics(target, rotation, self.sdk.seed,
                                                multi_init=False, eps=1e-5, max_iter=1500)
        seed_policy = 'current'
        if solved is None:
            # Official DLS aborts when an intermediate iterate crosses a limit.
            # Retry deterministic interior mathematical seeds; no hardware motion.
            candidates = []
            seeds = [self.np.array([0., .5, .5, 0., 0., 0.]),
                     self.np.array([0., 1., 1., 0., 0., 0.]),
                     (self.lower + self.upper) / 2]
            with redirect_stdout(output):
                for initial in seeds:
                    candidate = self.sdk.inverse_kinematics(
                        target, rotation, self.np.clip(initial, self.lower, self.upper),
                        multi_init=False, eps=1e-5, max_iter=1500)
                    if candidate is not None:
                        candidate = self.np.asarray(candidate)
                        if (candidate.shape == (6,) and self.np.isfinite(candidate).all()
                                and self.np.all(candidate >= self.lower)
                                and self.np.all(candidate <= self.upper)):
                            pose = self.fk(candidate)
                            if (math.dist(pose['position_m'], target) <= .001
                                    and self.orientation_error(pose['quaternion_xyzw'], desired) <= .01):
                                candidates.append(candidate)
            if candidates:
                solved = min(candidates, key=lambda q: float(self.np.linalg.norm(q-seed)))
                seed_policy = 'deterministic_interior_fallback'
        if solved is None:
            raise ValueError('官方 IK 未找到当前姿态附近的解；目标或保持姿态约束不可达。'+output.getvalue()[-300:])
        solved = self.vector(solved, 6, '逆解')
        if any(not low <= value <= high for value, low, high in zip(solved, self.lower, self.upper)):
            raise ValueError('官方 IK 解超出关节限位')
        pose = self.fk(solved)
        error = math.dist(pose['position_m'], target)
        angle = self.orientation_error(pose['quaternion_xyzw'], desired)
        if error > 0.001 or angle > 0.01:
            raise ValueError(f'IK 结果复核失败：位置误差 {error:.6f} m，姿态误差 {angle:.6f} rad')
        return {'start_pose': before, 'target_position_m': target, 'target_quaternion_xyzw': desired,
                'target_joints_rad': solved, 'ik_position_error_m': error, 'ik_orientation_error_rad': angle,
                'orientation_policy': 'hold_current' if quaternion is None else 'explicit_quaternion',
                'ik_seed_policy': seed_policy}

    @staticmethod
    def orientation_error(actual, desired):
        dot = abs(sum(a*b for a,b in zip(actual, desired)))
        return 2*math.acos(min(1.0, max(0.0, dot)))

    def check_path(self, start, goal, step_rad=0.02):
        segments = max(1, math.ceil(max(abs(a-b) for a,b in zip(start,goal))/step_rad))
        samples = (tuple(a+(b-a)*i/segments for a,b in zip(start,goal)) for i in range(segments+1))
        return self._check_samples(samples, step_rad, 'nominal_joint_interpolation_self_collision_only')

    def check_trajectory(self, trajectory, step_rad=0.02):
        return self._check_samples(trajectory.collision_samples(step_rad), step_rad,
                                   'ruckig_reference_self_collision_only')

    def prepare_collision_geometry(self):
        if self.geometry is None:
            self.geometry = self.pin.buildGeomFromUrdfString(self.model, self.xml, self.pin.GeometryType.COLLISION)
            self.geometry.addAllCollisionPairs()
            self.pin.removeCollisionPairs(self.model, self.geometry,
                str(self.root / 'robots/panthera_ht/planning/robot.srdf'))
            self.geometry_data = self.pin.GeometryData(self.geometry)
        return {'collision_pairs':len(self.geometry.collisionPairs)}

    def _check_samples(self, samples, step_rad, scope):
        self.prepare_collision_geometry()
        count = 0
        for sample, positions in enumerate(samples):
            count += 1
            q = self.np.zeros(self.model.nq)
            for name,value in zip(self.sdk.joint_names,positions):
                joint = self.model.joints[self.model.getJointId(name)]
                q[joint.idx_q] = value
            if self.pin.computeCollisions(self.model,self.data,self.geometry,self.geometry_data,q,False):
                collisions = []
                for pair, result in zip(self.geometry.collisionPairs,self.geometry_data.collisionResults):
                    if result.isCollision():
                        collisions.append((self.geometry.geometryObjects[pair.first].name,
                                           self.geometry.geometryObjects[pair.second].name))
                raise ValueError(f'参考轨迹自碰撞，样本 {sample}：{collisions[:4]}')
        return {'samples': count, 'step_rad': step_rad, 'collision_pairs_checked':len(self.geometry.collisionPairs),
                'scope':scope}
