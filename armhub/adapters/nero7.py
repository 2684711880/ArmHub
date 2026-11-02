from ..backends.pending import PendingBackend


class NeroHardwareBackend(PendingBackend):
    """待接 pyAgxArm / 官方 ROS；Nero 实物效果尚未验证。"""

    backend_name = "hardware"
