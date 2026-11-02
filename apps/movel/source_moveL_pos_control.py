#!/usr/bin/env python3
"""
moveL 位置控制示例（含末端运动误差分析）

在指定平面（xy / xz / yz）内依次执行直线、对角线、正方形、回字螺旋和圆形
轨迹，可按正向、反向或两个方向执行，记录编码器反馈并输出末端误差指标与可视化；
两个方向都执行时额外输出正反向对比。轨迹几何见
moveL_trajectories.py，误差定义见 moveL_error_analysis.py。

每条轨迹的流程：moveL 引入到起点（不记录）→ 记录执行轨迹并静置采样 →
moveJ 回到锚点。所有轨迹在第一次运动前先做 IK 与关节限位预检。

示例:
    python 6_moveL_pos_control.py --plane xy xz --trajectories square spiral circle
    python 6_moveL_pos_control.py --plane yz --size 0.06 --speed 0.03
    python 6_moveL_pos_control.py --trajectories circle spiral --direction rev

输出:
    <output>/run.json              运行元数据
    <output>/NN_<plane>_<traj>_<dir>.*  每条轨迹的原始数据(.npz)、逐帧误差(.csv)、诊断图(.png)
    <output>/pair_<plane>_<traj>.png    正反向对比图（--direction both）
    <output>/summary.png           各轨迹误差对比
    <output>/metrics.json          全部统计指标

已有数据可用以下命令重新出图：
    python moveL_error_analysis.py <output>
"""

import argparse
import time
from pathlib import Path

import numpy as np
from Panthera_lib import Panthera
from moveL_error_analysis import record_cartesian_motion, save_run, analyze_run
from moveL_trajectories import PLANES, SHAPES, DIRECTIONS, shape_uv, to_waypoints, path_length, preflight


def current_pose(robot):
    fk = robot.forward_kinematics()
    return np.array(fk['position']), fk['rotation']


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--plane', nargs='+', choices=sorted(PLANES), default=['xy'], help='运动平面，可多个')
    parser.add_argument('--trajectories', nargs='+', choices=SHAPES, default=list(SHAPES), help='轨迹，按给定顺序执行')
    parser.add_argument('--direction', choices=('fwd', 'rev', 'both'), default='both',
                        help='轨迹方向；both 先正向后反向，并输出正反向对比')
    parser.add_argument('--size', type=float, default=0.08, help='轨迹框边长 / 圆直径 (m)')
    parser.add_argument('--pitch', type=float, default=0.01, help='回字螺旋圈距 (m)')
    parser.add_argument('--speed', type=float, default=0.05, help='末端平均速度 (m/s)，决定每条轨迹时长')
    parser.add_argument('--anchor', type=float, nargs=3, default=[0.24, 0.0, 0.1], help='锚点位置 (m)')
    parser.add_argument('--z-min', type=float, default=0.05, help='tool_link 允许的最低高度 (m)，防止碰桌')
    parser.add_argument('--settle', type=float, default=0.5, help='每条轨迹结束后的静置采样时间 (s)')
    parser.add_argument('--output', type=Path, default=Path(time.strftime('moveL_error_%Y%m%d_%H%M%S')),
                        help='结果目录（必须不存在）')
    args = parser.parse_args()
    if not 0.02 <= args.size <= 0.15:
        parser.error('--size 需在 0.02–0.15 m')
    if not 0.005 <= args.pitch <= args.size / 4:
        parser.error('--pitch 需在 0.005 m 与 size/4 之间')
    if not 0.01 <= args.speed <= 0.1:
        parser.error('--speed 需在 0.01–0.1 m/s')
    if args.z_min < 0.0:
        parser.error('--z-min 不能低于基座平面')
    if not 0.0 <= args.settle <= 5.0:
        parser.error('--settle 需在 0–5 s')
    return args


def build_plan(robot, args, anchor_q, anchor_pos, anchor_rot):
    """生成全部轨迹并预检；任一条不通过则不运动"""
    anchor_pose = {'position': anchor_pos, 'rotation': anchor_rot}
    plan, failed = [], False
    print("\n轨迹预检（IK、关节跳变、限位余量）:")
    directions = DIRECTIONS if args.direction == 'both' else (args.direction,)
    for plane in args.plane:
        for shape in args.trajectories:
            uv = shape_uv(shape, args.size, args.pitch)
            for direction in directions:
                name = f'{plane}_{shape}_{direction}'
                waypoints = to_waypoints(anchor_pos, anchor_rot, plane, uv if direction == 'fwd' else uv[::-1])
                length = path_length(waypoints)
                check = preflight(robot, anchor_q, anchor_pose, waypoints, args.z_min)
                status = (f"✓ {check['ik_points']:4d} IK 点, 限位余量 {check['min_margin_rad']:.2f} rad"
                          if check['ok'] else f"✗ {check['reason']}")
                print(f"  {name:<18} {length*1000:7.1f} mm  {length/args.speed:5.1f} s  {status}")
                failed |= not check['ok']
                plan.append(dict(name=name, waypoints=waypoints, duration=length / args.speed,
                                 tags=dict(plane=plane, shape=shape, direction=direction)))
    return None if failed else plan


def run_plan(robot, plan, anchor_q, speed, settle, records):
    for k, item in enumerate(plan):
        print("\n" + "="*60)
        print(f"轨迹 {k+1}/{len(plan)}: {item['name']}（{item['duration']:.1f}s）")
        print("="*60)
        start = item['waypoints'][0]
        pos, _ = current_pose(robot)
        lead = np.linalg.norm(np.asarray(start['position']) - pos)
        if lead > 1e-3 and not robot.moveL(start['position'], start['rotation'],
                                           duration=max(1.0, lead / speed)):
            print("  ✗ 引入段失败")
            return False
        time.sleep(0.3)
        if not record_cartesian_motion(robot, item['name'], item['waypoints'], duration=item['duration'],
                                       settle_s=settle, records=records, tags=item['tags']):
            print(f"  ✗ {item['name']} 失败")
            return False
        robot.moveJ(anchor_q, duration=2.0, iswait=True)
    return True


def main():
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    print("="*60)
    print("MoveIt 风格笛卡尔控制示例（末端误差分析）")
    print("="*60)

    # 1. 初始化机械臂
    print("\n初始化机械臂...")
    robot = Panthera()

    # 2. 移动到锚点
    print("\n移动到初始位置...")
    ik_rot1 = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    zero_pos = [0.0] * robot.motor_count
    vel = [0.5] * robot.motor_count
    robot.Joint_Pos_Vel(zero_pos, vel, iswait=True)

    pos1 = robot.inverse_kinematics(args.anchor, ik_rot1, robot.get_current_pos())
    if pos1 is None:
        print("锚点 IK 无解")
        return
    robot.moveJ(pos1, duration=3.0, iswait=True)
    anchor_q = robot.get_current_pos()
    anchor_pos, anchor_rot = current_pose(robot)
    print(f"\n锚点位置: [{anchor_pos[0]:.4f}, {anchor_pos[1]:.4f}, {anchor_pos[2]:.4f}] m")

    # 3. 预检全部轨迹
    plan = build_plan(robot, args, anchor_q, anchor_pos, anchor_rot)
    if plan is None:
        print("\n预检未通过，未执行任何轨迹。可减小 --size、调整 --anchor 或 --z-min。")
        robot.moveJ(zero_pos, 3.0, iswait=True)
        return

    # 4. 执行并记录；中断或失败时也导出已记录的数据
    records = []
    try:
        if run_plan(robot, plan, anchor_q, args.speed, args.settle, records):
            robot.moveJ(zero_pos, 3.0, iswait=True)
    finally:
        if records:
            save_run(args.output, records, dict(
                started_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                planes=args.plane, trajectories=args.trajectories, direction=args.direction, size_m=args.size,
                pitch_m=args.pitch, speed_m_s=args.speed, z_min_m=args.z_min, anchor_m=anchor_pos.tolist(),
                eef_step_m=robot.eef_step, resample_dt_s=robot.resample_dt,
                feedback='host receive wall time of 6 arm motors, fresh replies only'))
            analyze_run(args.output)
            print(f"\n误差分析结果: {args.output.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
