"""公共数据契约。关节 rad，位置 m，时间 s；不使用厂商数据类型。"""

from dataclasses import dataclass


class UnsupportedCapability(NotImplementedError):
    """当前后端没有实现该能力，禁止静默降级。"""


@dataclass(frozen=True)
class ArmState:
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    observed_at_monotonic_s: float
    enabled: bool
    source: str  # mock / simulation / hardware，不能将 mock 当成传感器反馈


@dataclass(frozen=True)
class Pose:
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    frame_id: str
    tcp_frame: str


@dataclass(frozen=True)
class TrajectoryPoint:
    positions: tuple[float, ...]  # 严格遵循 profile.joint_names 顺序
    time_from_start_s: float


@dataclass(frozen=True)
class MotionResult:
    status: str
    source: str
    final_positions: tuple[float, ...]
    detail: str
    target_position_m: tuple[float, ...] | None = None
    actual_position_m: tuple[float, ...] | None = None
    position_error_m: float | None = None
    trajectory_points: int = 0


@dataclass(frozen=True)
class GravityResult:
    status: str
    source: str
    duration_s: float
    cycles: int
    mean_frequency_hz: float
    max_command_gap_s: float
    clipped_cycles: int
    exit_reason: str
    hold_stop_confirmed: bool


@dataclass(frozen=True)
class JointFeedback:
    """复制后的反馈值，任务层不接触厂商 Motor 或原生快照对象。"""
    ID: int
    position: float
    velocity: float
    torque: float
    fault: int
    mode: int
    num: int
    time: float
