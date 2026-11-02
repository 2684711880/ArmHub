"""稳定的上层入口。当前同步调用；返回成功必须表示完成而非提交。"""

from abc import ABC, abstractmethod
from collections.abc import Sequence

from .profiles import RobotProfile
from .types import ArmState, MotionResult, Pose, TrajectoryPoint, UnsupportedCapability


class ArmBackend(ABC):
    def __init__(self, profile: RobotProfile):
        self.profile = profile

    @property
    @abstractmethod
    def capabilities(self) -> frozenset[str]: ...

    @abstractmethod
    def connect(self) -> None:
        """建立连接，不自动使能、不回零、不开始重力补偿。"""

    @abstractmethod
    def close(self) -> None:
        """释放资源；具体后端必须说明运动中的关闭行为。"""

    @abstractmethod
    def get_state(self) -> ArmState: ...

    @abstractmethod
    def enable(self) -> None: ...

    @abstractmethod
    def stop(self) -> None:
        """请求受控停止；不等同于物理急停，也不默认等同于断使能。"""

    @abstractmethod
    def move_joints(self, positions: Sequence[float], *, duration_s: float) -> MotionResult: ...

    @abstractmethod
    def execute_trajectory(self, points: Sequence[TrajectoryPoint]) -> MotionResult: ...

    def move_linear(self, target: Pose, *, speed_m_s: float) -> MotionResult:
        """由当前 TCP 到目标；保持直线语义，禁止用关节点到点冒充。"""
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持 move_linear")

    def set_teach_mode(self, enabled: bool) -> None:
        """拖动示教能力；与 SDK 启动、重力补偿分别定义。"""
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持拖动示教")

    def move_to(self, position_m: Sequence[float], *, quaternion_xyzw=None,
                velocity_scale: float = 0.15) -> MotionResult:
        """末端到基座坐标系中的空间目标；路径不约束为直线；姿态可选。"""
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持 move_to")

    def request_hold(self):
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持非阻塞保持")

    def poll_hold(self,feedback=None):
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持保持反馈轮询")

    def prepare_motion_model(self):
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持运动模型预加载")

    def get_joint_feedback(self):
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持详细反馈")

    def start_reference_trajectory(self, curve, **kwargs):
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持连续参考")

    def advance_reference_trajectory(self,feedback=None):
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持连续参考")

    def command_gripper(self, position, *, speed, torque, command_guard=None):
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持夹爪控制")

    def poll_gripper(self):
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持夹爪反馈轮询")

    def home(self, *, velocity_scale: float = 1.0) -> MotionResult:
        """回到机型定义的关节 home 姿态；不重新标定编码器零点。"""
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持 home")

    def run_gravity_compensation(self, *, duration_s: float | None = None):
        """持续补偿到时限或 Ctrl+C；正常退出须由后端确认保持停止。"""
        raise UnsupportedCapability(f"{type(self).__name__} 尚不支持重力补偿")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
