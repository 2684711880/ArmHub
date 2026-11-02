"""接入边界占位。创建对象不导入 SDK、不打开串口/CAN、不发布 ROS 消息。"""

from ..interfaces import ArmBackend
from ..types import UnsupportedCapability


class PendingBackend(ArmBackend):
    backend_name = "pending"

    @property
    def capabilities(self):
        return frozenset()

    def _pending(self, *args, **kwargs):
        raise UnsupportedCapability(
            f"{self.profile.robot_id}/{self.backend_name} 尚未接入；"
            "当前仅为接口骨架，不连接设备。请先使用 backend='mock' 查看调用链。"
        )

    connect = _pending
    get_state = _pending
    enable = _pending
    stop = _pending
    move_joints = _pending
    execute_trajectory = _pending

    def close(self):
        pass  # 从未分配设备资源
