"""同一份任务代码切换六轴/七轴；本轮只运行 Mock。"""

import argparse
from dataclasses import asdict
import json

from armhub import create_arm


def run(robot: str) -> dict:
    with create_arm(robot, backend="mock") as arm:
        before = arm.get_state()
        # 从当前虚拟位置出发，只改变末关节；不写死 6/7 轴，不自动 home。
        target = list(before.positions)
        target[-1] += 0.05
        arm.enable()
        result = arm.move_joints(target, duration_s=2.0)
        after = arm.get_state()
        return {"robot": robot, "backend": "mock", "before": asdict(before),
                "result": asdict(result), "after": asdict(after)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="panthera_ht")
    args = parser.parse_args()
    print(json.dumps(run(args.robot), ensure_ascii=False, indent=2))
