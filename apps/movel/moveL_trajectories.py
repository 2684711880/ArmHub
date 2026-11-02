#!/usr/bin/env python3
"""
moveL 平面测试轨迹生成与预检

所有轨迹画在指定平面（xy / xz / yz）内、以初始位姿（锚点）为中心、边长
size 的正方形框里；平面第一轴记为 u，第二轴记为 v（竖直平面 v 为 z）。末端
姿态全程保持锚点姿态。预检会拒绝 tool_link 低于 z_min 的路径点。

默认锚点 [0.24, 0, 0.1] 处，保持该姿态时 J4（限位 ±1.6 rad）在 z≈0.16 m 附近
到达限位，因此框不能只向上延伸。

轨迹:
    line_u    沿 u 轴穿过框中心的直线
    line_v    沿 v 轴的直线
    diagonal  框的对角线
    square    框的外边
    spiral    回字螺旋：从左下角沿外框顺时针绕行、每圈内缩 pitch
    circle    以锚点为圆心、size/2 为半径的整圆，从 -v 侧逆时针一圈

方向:
    fwd       按上述顺序
    rev       顶点倒序：直线终点→起点；方形、圆改为反向绕行；螺旋从内向外
"""

import numpy as np

PLANES = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}
SHAPES = ('line_u', 'line_v', 'diagonal', 'square', 'spiral', 'circle')
DIRECTIONS = ('fwd', 'rev')
CIRCLE_SEGMENTS = 720          # 0.5° 一段；半径 40 mm 时弦高约 0.4 µm，可当作真圆
JOINT_LIMIT_MARGIN = 0.02      # rad


def shape_uv(shape, size, pitch):
    """返回平面坐标下的折线顶点 (k, 2)，单位 m，锚点为原点"""
    h = size / 2
    if shape == 'line_u':
        return np.array([[-h, 0], [h, 0]])
    if shape == 'line_v':
        return np.array([[0, -h], [0, h]])
    if shape == 'diagonal':
        return np.array([[-h, -h], [h, h]])
    if shape == 'square':
        return np.array([[-h, -h], [h, -h], [h, h], [-h, h], [-h, -h]])
    if shape == 'spiral':
        return _rect_spiral(-h, h, -h, h, pitch)
    if shape == 'circle':
        a = -np.pi / 2 + np.linspace(0, 2 * np.pi, CIRCLE_SEGMENTS + 1)
        return np.column_stack([h * np.cos(a), h * np.sin(a)])
    raise ValueError(f'未知轨迹: {shape}')


def plane_normal(plane):
    """e_u × e_v；平面坐标图中路径左侧即 normal × 切向"""
    i, j = PLANES[plane]
    return np.cross(np.eye(3)[i], np.eye(3)[j])


def _rect_spiral(x0, x1, y0, y1, pitch):
    pts = [(x0, y0)]
    while x1 - x0 > pitch / 2 and y1 - y0 > pitch / 2:
        pts += [(x1, y0), (x1, y1), (x0, y1)]
        y0 += pitch
        if y1 - y0 <= pitch / 2:
            break
        pts.append((x0, y0))
        x0, x1, y1 = x0 + pitch, x1 - pitch, y1 - pitch
    return np.array(pts)


def path_length(waypoints):
    p = np.array([w['position'] for w in waypoints])
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def preflight(robot, anchor_q, anchor_pose, waypoints, z_min):
    """
    从锚点关节角出发，对引入段和整条轨迹逐段求 IK，检查高度、跳变与关节限位

    不发送任何指令。返回 dict(ok, reason, ik_points, min_margin_rad)。
    """
    z = min(w['position'][2] for w in waypoints)
    if z < z_min:
        return dict(ok=False, reason=f'最低点 z={z:.3f} m 低于 z_min={z_min} m', ik_points=0, min_margin_rad=None)
    lower, upper = robot.joint_limits['lower'], robot.joint_limits['upper']
    q = np.asarray(anchor_q, dtype=float)
    segments = [(anchor_pose, waypoints[0])] + list(zip(waypoints[:-1], waypoints[1:]))
    count, margin = 0, np.inf
    for k, (a, b) in enumerate(segments):
        if np.linalg.norm(np.asarray(b['position']) - np.asarray(a['position'])) < 1e-6:
            continue
        traj, ok = robot._interpolate_segment(a, b, q)
        label = '引入段' if k == 0 else f'第 {k} 段'
        if not ok:
            return dict(ok=False, reason=f'{label} IK 失败或关节跳变', ik_points=count, min_margin_rad=None)
        traj = np.asarray(traj)
        per_joint = np.min(np.minimum(traj - lower, upper - traj), axis=0)
        m = float(per_joint.min())
        margin = min(margin, m)
        if m < JOINT_LIMIT_MARGIN:
            return dict(ok=False, reason=f'{label} J{np.argmin(per_joint)+1} 距限位不足 {JOINT_LIMIT_MARGIN} rad',
                        ik_points=count, min_margin_rad=m)
        count += len(traj)
        q = traj[-1]
    return dict(ok=True, reason=None, ik_points=count, min_margin_rad=margin)


def to_waypoints(anchor_pos, anchor_rot, plane, uv):
    points = np.asarray(uv, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('轨迹顶点必须为有限的 (N, 2) 数组')
    positions = np.tile(np.asarray(anchor_pos, dtype=float), (len(points), 1))
    positions[:, list(PLANES[plane])] += points
    return [dict(position=p, rotation=np.array(anchor_rot, copy=True)) for p in positions]
