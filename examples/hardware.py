"""首次实物接入：默认只读；jog 不带 --execute 时仅显示实测起点与目标。"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

from armhub import create_arm


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['state', 'jog', 'gravity', 'home'])
    parser.add_argument('--robot', default='panthera_ht', choices=['panthera_ht', 'nero7'])
    parser.add_argument('--joint', type=int, choices=range(1, 7), default=6)
    parser.add_argument('--degrees', type=float, default=2.0)
    parser.add_argument('--velocity-scale', type=float, default=1.0, help='home 的速度比例')
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--seconds', type=float, help='gravity 的运行时长；不指定则运行到 Ctrl+C')
    parser.add_argument('--q', type=float, nargs=6, help='仅用于 gravity 离线预览的六轴姿态（rad）')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.command == 'state' and args.execute:
        parser.error('state 是只读命令，不接受 --execute')
    if args.command != 'gravity' and (args.seconds is not None or args.q is not None):
        parser.error('--seconds / --q 仅用于 gravity')
    if args.execute and args.q is not None:
        parser.error('实物补偿只使用实时反馈，不接受预览姿态 --q')
    if not math.isfinite(args.degrees) or abs(args.degrees) > 5:
        parser.error('首次接入只允许不超过 ±5°')
    report = {'robot': args.robot, 'backend': 'hardware', 'command': args.command,
              'motion_requested': args.command in ['jog', 'gravity', 'home'] and args.execute}
    arm = create_arm(args.robot, 'hardware')
    try:
        if args.command == 'gravity':
            if 'gravity_compensation' not in arm.capabilities:
                raise RuntimeError(f'{args.robot} 尚未实现重力补偿，不连接设备')
            # 依赖/模型预检查在连接实物前完成；预览完全不打开串口。
            model = arm._gravity_model()
            if not args.execute:
                q = args.q if args.q is not None else (0.0,)*6
                requested = model(q)
                caps = arm.config['gravity_compensation']['torque_caps_nm']
                report.update({'status': 'preview', 'q_source': 'offline_user_pose' if args.q else 'offline_zero_pose',
                               'q_rad': q, 'requested_gravity_nm': requested,
                               'torque_caps_nm': caps,
                               'applied_if_requested_nm': [max(-cap,min(cap,value)) for value,cap in zip(requested,caps)],
                               'note': '离线模型计算；未连接设备、未发送力矩；实物启动需 --execute'})
                output = json.dumps(report, ensure_ascii=False, indent=2)
                print(output)
                if args.report:
                    args.report.parent.mkdir(parents=True, exist_ok=True)
                    args.report.write_text(output+'\n')
                return
        if args.command == 'home' and 'home' not in arm.capabilities:
            raise RuntimeError(f'{args.robot} 尚未实现 home，不连接设备')
        with arm:
            before = arm.get_state()
            report['before'] = asdict(before)
            report['motors'] = [{'id': item.ID, 'position_rad': item.position,
                                 'velocity_rad_s': item.velocity, 'fault': item.fault,
                                 'mode': item.mode, 'counter': item.num,
                                 'device_time_unix_s': item.time} for item in arm._last_states]
            report['log_file'] = str(arm.log_path)
            if args.command == 'gravity':
                arm.enable()
                report['result'] = asdict(arm.run_gravity_compensation(duration_s=args.seconds))
                report['after'] = asdict(arm.get_state())
            if args.command == 'home':
                if args.execute:
                    arm.enable()
                    report['result'] = asdict(arm.home(velocity_scale=args.velocity_scale))
                    report['after'] = asdict(arm.get_state())
                else:
                    report['preview'] = arm.preview_home(velocity_scale=args.velocity_scale)
                    report['note'] = '仅预览 home，不发送运动命令'
            if args.command == 'jog':
                target = list(before.positions)
                target[args.joint-1] += math.radians(args.degrees)
                report['target_rad'] = target
                report['joint'] = args.joint
                report['delta_degrees'] = args.degrees
                if args.execute:
                    arm.enable()
                    report['result'] = asdict(arm.move_joints(target, duration_s=args.duration))
                    arm.stop()
                    report['stop_confirmed'] = True
                    report['after'] = asdict(arm.get_state())
                else:
                    report['note'] = '仅预览，不发送位置/速度/力矩运动命令'
            report['status'] = 'completed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        log_path = getattr(arm, 'log_path', None)
        report['log_file'] = str(log_path) if log_path else None
    output = json.dumps(report, ensure_ascii=False, indent=2)
    print(output)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output + '\n')
    if report['status'] == 'failed':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
