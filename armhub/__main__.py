import argparse
from dataclasses import asdict
import json

from .factory import create_arm, list_robots


def main():
    parser = argparse.ArgumentParser(description="ArmHub 接口骨架；默认不连接任何设备")
    parser.add_argument("command", choices=["list", "info"])
    parser.add_argument("--robot", default="panthera_ht", choices=list_robots())
    parser.add_argument("--backend", default="mock", choices=["mock", "simulation", "hardware"])
    args = parser.parse_args()
    if args.command == "list":
        print("\n".join(list_robots()))
        return
    arm = create_arm(args.robot, args.backend)
    print(json.dumps({"profile": asdict(arm.profile), "backend": args.backend,
                      "adapter": type(arm).__name__,
                      "implemented_capabilities": sorted(arm.capabilities)},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
