from .adapters.nero7 import NeroHardwareBackend
from .adapters.panthera_ht import PantheraHardwareBackend
from .backends.mock import MockBackend
from .backends.ros2_simulation import Ros2SimulationBackend
from .profiles import ROBOT_DIR, load_profile

HARDWARE_ADAPTERS = {
    "panthera_ht": PantheraHardwareBackend,
    "nero7": NeroHardwareBackend,
}


def list_robots() -> list[str]:
    return sorted(path.stem for path in ROBOT_DIR.glob("*.json"))


def create_arm(robot: str, backend: str = "mock"):
    profile = load_profile(robot)
    if backend == "mock":
        return MockBackend(profile)
    if backend == "simulation":
        return Ros2SimulationBackend(profile)
    if backend == "hardware":
        adapter = HARDWARE_ADAPTERS.get(robot)
        if adapter is None:
            raise ValueError(f"{robot} 尚未注册实物适配器")
        return adapter(profile)
    raise ValueError("backend 必须是 mock / simulation / hardware")
