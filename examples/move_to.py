"""同一空间目标入口；实物默认只预览，--execute 才下发。"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from armhub import create_arm


def main():
    parser = argparse.ArgumentParser(description='末端到 base_link 中的 XYZ（m）；实物默认保持当前姿态')
    parser.add_argument('--robot', required=True, choices=['panthera_ht', 'nero7'])
    parser.add_argument('--backend', choices=['simulation', 'hardware'], default='simulation')
    targets = parser.add_mutually_exclusive_group()
    targets.add_argument('--xyz', type=float, nargs=3, metavar=('X', 'Y', 'Z'))
    targets.add_argument('--delta', type=float, nargs=3, metavar=('DX', 'DY', 'DZ'), help='相对本次实测位置，沿基座坐标轴偏移')
    parser.add_argument('--quaternion', type=float, nargs=4)
    parser.add_argument('--velocity-scale', type=float, default=None)
    parser.add_argument('--state', action='store_true', help='只读取当前 TCP 位姿')
    parser.add_argument('--execute', action='store_true', help='实物明确执行；不加时只预览')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.velocity_scale is None:
        args.velocity_scale = 1.0 if args.backend == 'hardware' else 0.15
    if not args.state and args.xyz is None and args.delta is None:
        parser.error('运动需要 --xyz 或 --delta；只读使用 --state')
    report = {'robot': args.robot, 'backend': args.backend}
    arm = None
    failed = False
    try:
        with create_arm(args.robot, backend=args.backend) as arm:
            report['before'] = arm.get_pose()
            if not args.state:
                target = args.xyz if args.xyz is not None else [a+b for a,b in zip(report['before']['position_m'], args.delta)]
                report['requested_position_m'] = target
                if args.backend == 'hardware' and not args.execute:
                    report['preview'] = arm.preview_move_to(target, quaternion_xyzw=args.quaternion,
                                                          velocity_scale=args.velocity_scale)
                else:
                    arm.enable()
                    report['result'] = asdict(arm.move_to(target, quaternion_xyzw=args.quaternion,
                                                         velocity_scale=args.velocity_scale))
                    report['after'] = arm.get_pose()
            report['status'] = 'succeeded'
    except (Exception, KeyboardInterrupt) as error:
        failed = True
        report.update(status='failed', error=str(error) or type(error).__name__)
    if getattr(arm, 'log_path', None):
        report['log_file'] = str(arm.log_path)
    output = json.dumps(report, ensure_ascii=False, indent=2)
    print(output)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output + '\n')
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
