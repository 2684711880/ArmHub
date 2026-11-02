"""配置只存静态事实；运行时能力由实际后端报告。"""

import json
from dataclasses import dataclass
from pathlib import Path

ROBOT_DIR = Path(__file__).parent / "robots"


@dataclass(frozen=True)
class RobotProfile:
    robot_id: str
    name: str
    joint_names: tuple[str, ...]
    mock_position_bounds_rad: tuple[float, float]
    references: dict
    status: dict


def load_profile(robot_id: str) -> RobotProfile:
    # 从白名单中查找，robot_id 不作为任意文件路径使用。
    profiles = {p.stem: p for p in ROBOT_DIR.glob("*.json")}
    if robot_id not in profiles:
        raise ValueError(f"未知机械臂 {robot_id!r}；可选：{', '.join(sorted(profiles))}")
    data = json.loads(profiles[robot_id].read_text())
    return RobotProfile(
        robot_id=data["robot_id"], name=data["name"],
        joint_names=tuple(data["joint_names"]),
        mock_position_bounds_rad=tuple(data["mock_position_bounds_rad"]),
        references=data["references"], status=data["status"],
    )
