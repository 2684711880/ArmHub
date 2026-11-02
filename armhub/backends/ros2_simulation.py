"""共享 MoveIt/Gazebo 后端。延迟导入 ROS，Mock 用户不需要 ROS 环境。"""
import json
import math
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

from ..interfaces import ArmBackend
from ..types import ArmState, MotionResult


class Ros2SimulationBackend(ArmBackend):
    def __init__(self, profile):
        super().__init__(profile)
        self.node = None
        self.context = None
        self._state = None
        self._enabled = False
        self._active_goal = None

    @property
    def capabilities(self):
        return frozenset({'state', 'joint_motion', 'trajectory_execution', 'cartesian_target', 'stop'})

    def connect(self):
        if self.node is not None:
            return
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.action import ActionClient
            from rclpy.parameter import Parameter
            from rclpy.executors import SingleThreadedExecutor
            from ament_index_python.packages import get_package_share_directory
            from sensor_msgs.msg import JointState
            from rcl_interfaces.srv import GetParameters
            from controller_manager_msgs.srv import ListControllers
            from moveit_msgs.action import MoveGroup
            from moveit_msgs.srv import GetPositionFK
            from control_msgs.action import FollowJointTrajectory
        except ImportError as error:
            raise RuntimeError('仿真后端需要 ROS 2 环境，请通过 bash scripts/with_ros.sh 运行') from error
        share = Path(get_package_share_directory('armhub_sim'))
        self.config = json.loads((share / f'robots/{self.profile.robot_id}/robot.json').read_text())
        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('armhub_client_' + uuid.uuid4().hex[:8], context=self.context,
                                     parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._subscription = self.node.create_subscription(JointState, '/joint_states', self._on_state, 10)
        self._move_group = ActionClient(self.node, MoveGroup, '/move_action')
        self._trajectory = ActionClient(self.node, FollowJointTrajectory, self.config['trajectory_action'])
        self._fk = self.node.create_client(GetPositionFK, '/compute_fk')
        try:
            description_client = self.node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
            if not description_client.wait_for_service(timeout_sec=45.0):
                raise RuntimeError('未找到仿真，请先运行 scripts/launch_sim.sh')
            description = self._wait(description_client.call_async(
                GetParameters.Request(names=['robot_description'])), 10).values[0].string_value
            root = ET.fromstring(description)
            if (root.get('name') != self.config['model_name']
                    or root.find('ros2_control/hardware/plugin') is None
                    or root.find('ros2_control/hardware/plugin').text != 'gazebo_ros2_control/GazeboSystem'):
                raise RuntimeError('运行模型或硬件插件不匹配；拒绝把仿真命令发给其他机器人')
            self._joint_limits = {
                joint.get('name'): (float(joint.find('limit').get('lower')),
                                    float(joint.find('limit').get('upper')))
                for joint in root.findall('joint')
                if joint.get('name') in self.profile.joint_names
            }
            controllers = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
            if not controllers.wait_for_service(timeout_sec=45):
                raise RuntimeError('仿真控制器管理器未启动')
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                states = self._wait(controllers.call_async(ListControllers.Request()), 10)
                active = {controller.name for controller in states.controller if controller.state == 'active'}
                if {'joint_state_broadcaster', 'arm_controller', 'gripper_controller'} <= active:
                    break
                self.executor.spin_once(timeout_sec=0.1)
            else:
                raise RuntimeError('仿真控制器未全部激活')
            for client in [self._move_group, self._trajectory]:
                if not client.wait_for_server(timeout_sec=30):
                    raise RuntimeError('MoveIt 或轨迹 action 未就绪')
            if not self._fk.wait_for_service(timeout_sec=15):
                raise RuntimeError('MoveIt FK 服务未就绪')
            self.get_state()
        except BaseException:
            self.close()
            raise

    def _on_state(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values and math.isfinite(values[name]) for name in self.profile.joint_names):
            self._state = (message, time.monotonic(), tuple(values[name] for name in self.profile.joint_names))

    def _wait(self, future, timeout):
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('等待 ROS 响应超时')
        result = future.result()
        if result is None:
            raise RuntimeError('ROS 响应为空')
        return result

    def _require_node(self):
        if self.node is None:
            raise RuntimeError('请先 connect()')

    def get_state(self):
        self._require_node()
        # 本次调用后接收完整消息；不把缓存读取时刻当成反馈时间。
        started = time.monotonic()
        deadline = started + 3
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
            if self._state is not None and self._state[1] >= started:
                message, received, positions = self._state
                now_s = self.node.get_clock().now().nanoseconds / 1e9
                stamp_s = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                if not -0.1 <= now_s - stamp_s <= 1.0:
                    continue
                return ArmState(self.profile.joint_names, positions, received, self._enabled, 'simulation')
        raise RuntimeError('没有完整且新鲜的仿真关节反馈')

    def get_pose(self):
        from moveit_msgs.srv import GetPositionFK
        self.get_state()
        request = GetPositionFK.Request()
        request.header.frame_id = self.config['base_frame']
        request.fk_link_names = [self.config['tcp_frame']]
        request.robot_state.joint_state = self._state[0]
        response = self._wait(self._fk.call_async(request), 10)
        if response.error_code.val != 1 or not response.pose_stamped:
            raise RuntimeError(f'FK 失败：{response.error_code.val}')
        pose = response.pose_stamped[0].pose
        return {'frame_id': self.config['base_frame'], 'tcp_frame': self.config['tcp_frame'],
                'position_m': (pose.position.x, pose.position.y, pose.position.z),
                'quaternion_xyzw': (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)}

    def enable(self):
        # 本客户端的软件执行许可；不调用任何实物使能服务。
        self.get_state()
        self._enabled = True

    def stop(self):
        self._require_node()
        if self._active_goal is not None:
            handle = self._active_goal
            response = self._wait(handle.cancel_goal_async(), 5)
            if not response.goals_canceling:
                raise RuntimeError('取消请求未被接受，需要检查仿真执行状态')
            result = self._wait(handle.get_result_async(), 10)
            if result.status not in [4, 5, 6]:
                raise RuntimeError('执行尚未进入终态')
            self._active_goal = None

    def close(self):
        if self.node is not None:
            try:
                self.stop()
            finally:
                self.executor.shutdown()
                self.node.destroy_node()
                self.node = None
                self.context.shutdown()
                self._enabled = False

    def _execute(self, client, goal, timeout=90):
        if not self._enabled:
            raise RuntimeError('请先显式 enable()')
        self.get_state()
        handle = self._wait(client.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError('动作目标被拒绝')
        self._active_goal = handle
        try:
            wrapped = self._wait(handle.get_result_async(), timeout)
        except BaseException:
            self.stop()
            raise
        self._active_goal = None
        if wrapped.status != 4:
            code = wrapped.result.error_code
            raise RuntimeError(f'动作未成功：status={wrapped.status}, error_code={getattr(code, "val", code)}')
        return wrapped.result

    @staticmethod
    def _vector(values, size, label):
        values = tuple(float(value) for value in values)
        if len(values) != size or not all(math.isfinite(value) for value in values):
            raise ValueError(f'{label} 需要 {size} 个有限数')
        return values

    def move_to(self, position_m, *, quaternion_xyzw=None, velocity_scale=0.15):
        from geometry_msgs.msg import Pose
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
        from shape_msgs.msg import SolidPrimitive

        self._require_node()
        xyz = self._vector(position_m, 3, '目标位置')
        if not math.isfinite(velocity_scale) or not 0 < velocity_scale <= 1:
            raise ValueError('velocity_scale 必须位于 (0, 1]')
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.config['planning_group']
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnectkConfigDefault'
        request.allowed_planning_time = 8.0
        request.num_planning_attempts = 5
        request.max_velocity_scaling_factor = float(velocity_scale)
        request.max_acceleration_scaling_factor = float(velocity_scale)
        request.start_state.is_diff = True
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.config['base_frame']
        position.link_name = self.config['tcp_frame']
        position.weight = 1.0
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        center = Pose()
        center.position.x, center.position.y, center.position.z = xyz
        center.orientation.w = 1.0
        position.constraint_region.primitives = [region]
        position.constraint_region.primitive_poses = [center]
        constraints.position_constraints = [position]
        if quaternion_xyzw is not None:
            quaternion = self._vector(quaternion_xyzw, 4, '四元数')
            if abs(math.sqrt(sum(value * value for value in quaternion)) - 1) > 1e-3:
                raise ValueError('四元数必须归一化')
            orientation = OrientationConstraint()
            orientation.header.frame_id = self.config['base_frame']
            orientation.link_name = self.config['tcp_frame']
            orientation.orientation.x, orientation.orientation.y, orientation.orientation.z, orientation.orientation.w = quaternion
            orientation.absolute_x_axis_tolerance = 0.02
            orientation.absolute_y_axis_tolerance = 0.02
            orientation.absolute_z_axis_tolerance = 0.02
            orientation.weight = 1.0
            constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        response = self._execute(self._move_group, goal)
        if response.error_code.val != 1:
            raise RuntimeError(f'MoveIt 失败：error_code={response.error_code.val}')
        actual = self.get_pose()['position_m']
        error = math.dist(actual, xyz)
        if error > 0.005:
            raise RuntimeError(f'执行反馈终点误差 {error:.6f} m 超过 0.005 m')
        return MotionResult('succeeded', 'simulation', self.get_state().positions,
                            'MoveIt 规划、Gazebo 执行及末端 FK 反馈核对完成；路径未约束为直线',
                            xyz, actual, error, len(response.planned_trajectory.joint_trajectory.points))

    def move_joints(self, positions, *, duration_s):
        from ..types import TrajectoryPoint
        return self.execute_trajectory([TrajectoryPoint(tuple(positions), duration_s)])

    def execute_trajectory(self, points):
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint

        self._require_node()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.profile.joint_names)
        previous = 0.0
        for point in points:
            values = self._vector(point.positions, len(self.profile.joint_names), '关节位置')
            for name, value in zip(self.profile.joint_names, values):
                low, high = self._joint_limits[name]
                if not low <= value <= high:
                    raise ValueError(f'{name} 超出模型限位')
            timestamp = float(point.time_from_start_s)
            if not math.isfinite(timestamp) or timestamp <= previous:
                raise ValueError('时间必须为正、有限且严格递增')
            previous = timestamp
            item = JointTrajectoryPoint(positions=list(values), velocities=[0.0] * len(values))
            total_ns = round(timestamp * 1e9)
            item.time_from_start.sec, item.time_from_start.nanosec = divmod(total_ns, 10**9)
            goal.trajectory.points.append(item)
        if not goal.trajectory.points:
            raise ValueError('轨迹不能为空')
        result = self._execute(self._trajectory, goal, timeout=previous + 30)
        if result.error_code != 0:
            raise RuntimeError(f'轨迹执行失败：{result.error_string}')
        state = self.get_state()
        if max(abs(a-b) for a, b in zip(state.positions, values)) > 0.02:
            raise RuntimeError('反馈关节终点误差超过 0.02 rad')
        return MotionResult('succeeded', 'simulation', state.positions,
                            'Gazebo 轨迹执行成功，关节反馈已核对', trajectory_points=len(goal.trajectory.points))
