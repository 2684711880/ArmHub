"""确定性的软件替身：瞬时更新虚拟状态，无物理、无碰撞、无实时轨迹。"""

import math
import time
from collections.abc import Sequence

from ..interfaces import ArmBackend
from ..types import ArmState, MotionResult, TrajectoryPoint


class MockBackend(ArmBackend):
    def __init__(self, profile):
        super().__init__(profile)
        self._connected = False
        self._enabled = False
        self._positions = (0.0,) * len(profile.joint_names)

    @property
    def capabilities(self):
        return frozenset({"joint_motion", "trajectory_execution", "state", "stop"})

    def connect(self):
        self._connected = True

    def close(self):
        self._enabled = False
        self._connected = False

    def _require_connection(self):
        if not self._connected:
            raise RuntimeError("请先 connect()")

    def get_state(self):
        self._require_connection()
        return ArmState(self.profile.joint_names, self._positions,
                        time.monotonic(), self._enabled, "mock")

    def enable(self):
        self._require_connection()
        self._enabled = True

    def stop(self):
        self._require_connection()
        # 同步 Mock 无在途运动；保留当前位置和使能状态。

    def move_joints(self, positions: Sequence[float], *, duration_s: float):
        return self.execute_trajectory([TrajectoryPoint(tuple(positions), duration_s)])

    def execute_trajectory(self, points: Sequence[TrajectoryPoint]):
        self._require_connection()
        if not self._enabled:
            raise RuntimeError("请先显式 enable()")
        points = tuple(points)
        if not points:
            raise ValueError("轨迹不能为空")
        previous_time = 0.0
        checked = []
        low, high = self.profile.mock_position_bounds_rad
        # 全部检查完成后才更新状态，后续坏点不会造成部分执行。
        for point in points:
            positions = tuple(float(p) for p in point.positions)
            if len(positions) != len(self.profile.joint_names):
                raise ValueError("关节数量必须与 profile.joint_names 一致")
            if any(not math.isfinite(p) or not low <= p <= high for p in positions):
                raise ValueError(f"Mock 关节值须为有限数且位于 [{low}, {high}] rad；此范围不是实物限位")
            timestamp = float(point.time_from_start_s)
            if not math.isfinite(timestamp) or timestamp <= previous_time:
                raise ValueError("轨迹时间必须有限、为正且严格递增")
            previous_time = timestamp
            checked.append(positions)
        self._positions = checked[-1]
        return MotionResult("succeeded", "mock", self._positions,
                            "仅完成虚拟状态更新；未按时间播放，未执行物理仿真或实物运动")
