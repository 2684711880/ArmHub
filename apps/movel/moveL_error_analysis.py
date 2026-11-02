#!/usr/bin/env python3
"""ArmHub moveL 编码器反馈记录、误差分析与可视化。

通过 ArmHub 的已检查连续参考执行器录制，不直接发送 SDK 指令。
位置是编码器关节角经过 SDK FK 的结果，不是外部测量真值。
主跟踪参考为接收时刻的连续计划参考；实际离散下发 q/t 另存。
路径误差与时间无关，正反向比较扣除各自规划误差。
重新出图：python -m apps.movel.moveL_error_analysis <run_dir>
"""

import argparse
import csv
import json
import os
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from .moveL_trajectories import PLANES, plane_normal

SETTLE_TOLERANCE_M = 0.0005
LAG_SEARCH_S = np.arange(0.0, 0.3001, 0.002)
def record_cartesian_motion(robot, name, waypoints, duration, settle_s, records, tags=None, curve=None,
                            command_guard=None, on_feedback=None, command_mode='reference'):
    """使用 ArmHub 的已预检参考执行器；只记录，不建立第二个 SDK 控制连接。"""
    if curve is None:
        raise ValueError('需要预检后的 ArmHub 曲线')
    times = np.linspace(0, curve.duration, max(2, int(np.ceil(curve.duration / robot.resample_dt)) + 1))
    sampled = [curve.sample(t) for t in times]
    rec = dict(name=name, duration_s=curve.duration, settle_s=settle_s, tags=dict(tags or {}),
               fraction=1.0, completed=False, error=None,
               waypoints=np.array([w['position'] for w in waypoints]),
               target_rotation=np.asarray(waypoints[-1]['rotation']),
               plan_t=times, plan_q=np.array([x[0] for x in sampled]),
               plan_v=np.array([x[1] for x in sampled]), dispatch_t=[])
    records.append(rec)
    t_fb, q_fb, dq_fb, phase, stamps_fb, cmd_q = [], [], [], [], [], []
    previous = None
    sample_t = [];speed_caps=[]
    original_record = robot.arm._record
    def capture(event, **data):
        original_record(event, **data)
        if event == 'stream_reference':
            rec['dispatch_t'].append(time.monotonic()-robot.arm._reference_execution.started)
            sample_t.append(data['elapsed_s'])
            cmd_q.append(data['position_rad'])
            speed_caps.append(data['velocity_caps_rad_s'])
    robot.arm._record = capture
    try:
        robot.arm.start_reference_trajectory(curve, expected_start=curve.sample(0)[0],
                                             command_guard=command_guard,command_mode=command_mode)
        session = robot.arm._reference_execution
        wall_start = time.time() - (time.monotonic() - session.started)
        while True:
            wait = session.next_due-time.monotonic()
            if not session.final_sent and wait > 0:
                time.sleep(wait)
            result = robot.arm.advance_reference_trajectory()
            states = session.states[:6]
            counters = tuple(x.num for x in states)
            if previous is None or all(a > b for a,b in zip(counters,previous)):
                previous = counters
                stamps = [x.time for x in states]
                t_fb.append(float(np.mean(stamps))-wall_start)
                stamps_fb.append(stamps)
                q_fb.append([x.position for x in states]);dq_fb.append([x.velocity for x in states])
                phase.append(int(t_fb[-1] >= curve.duration))
                if on_feedback is not None:
                    on_feedback(session.states)
            if result is not None:
                break
        end = time.monotonic()+settle_s
        while time.monotonic() < end:
            if command_guard is not None and not command_guard():
                raise RuntimeError('用户请求停止记录')
            all_states = robot.arm.read_feedback()
            states = all_states[:6]
            stamps = [x.time for x in states]
            t_fb.append(float(np.mean(stamps))-wall_start);stamps_fb.append(stamps)
            q_fb.append([x.position for x in states]);dq_fb.append([x.velocity for x in states]);phase.append(1)
            if on_feedback is not None:
                on_feedback(all_states)
        rec['completed'] = True
    except BaseException as exc:
        rec['error'] = f'{type(exc).__name__}: {exc}'
        if robot.arm._moving:
            robot.arm.stop()
        raise
    finally:
        robot.arm._record = original_record
        rec['command_sample_t'] = np.asarray(sample_t)
        rec['dispatch_t'] = np.asarray(rec['dispatch_t'])
        rec['dispatched_q'] = np.asarray(cmd_q).reshape(-1,6)
        rec['dispatched_speed_caps_rad_s'] = np.asarray(speed_caps).reshape(-1,6)
        rec['fb_motor_wall_t'] = np.asarray(stamps_fb).reshape(-1,6)
        rec['fb_t'] = np.asarray(t_fb)
        rec['fb_q'] = np.asarray(q_fb).reshape(-1,6)
        rec['fb_dq'] = np.asarray(dq_fb).reshape(-1,6)
        rec['fb_phase'] = np.asarray(phase,dtype=int)
        _attach_kinematics(robot,rec)
    print(f"  完成 {name}: {len(t_fb)} 帧新反馈, {session.sent} 次下发")
    return True


def _interp_rows(t, t_ref, rows):
    return np.column_stack([np.interp(t, t_ref, rows[:, j]) for j in range(rows.shape[1])])


def _fk_batch(robot, qs):
    p, rot = [], []
    for q in qs:
        fk = robot.forward_kinematics(q)
        p.append(fk['position'])
        rot.append(fk['rotation'])
    return np.asarray(p, dtype=float).reshape(-1, 3), np.asarray(rot, dtype=float).reshape(-1, 3, 3)


def _attach_kinematics(robot, rec):
    """执行结束后统一计算 FK，避免占用控制周期"""
    rec['plan_p'], rec['plan_R'] = _fk_batch(robot, rec['plan_q'])
    # 超出轨迹末端时参考保持在最后一个指令点
    rec['ref_q'] = _interp_rows(rec['fb_t'], rec['plan_t'], rec['plan_q']) if len(rec['fb_t']) else np.zeros((0, 6))
    rec['ref_p'], rec['ref_R'] = _fk_batch(robot, rec['ref_q'])
    rec['fb_p'], rec['fb_R'] = _fk_batch(robot, rec['fb_q'])


#######################
# 保存与读取
#######################
_META_KEYS = ('name', 'duration_s', 'settle_s', 'tags', 'fraction', 'completed', 'error')


def save_run(folder, records, metadata):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    motions = []
    for i, rec in enumerate(records):
        stem = f"{i+1:02d}_{_slug(rec['name'])}"
        arrays = {k: v for k, v in rec.items() if isinstance(v, np.ndarray)}
        np.savez_compressed(folder / f"{stem}.npz", **arrays)
        motions.append(dict(file=f"{stem}.npz", **{k: rec[k] for k in _META_KEYS}))
    _write_json(folder / 'run.json', dict(metadata, motions=motions))


def load_run(folder):
    folder = Path(folder)
    run = json.loads((folder / 'run.json').read_text())
    records = []
    for m in run['motions']:
        with np.load(folder / m['file']) as data:
            rec = {k: data[k] for k in data.files}
        rec.update({k: m[k] for k in _META_KEYS})
        records.append(rec)
    return run, records


def _slug(text):
    return ''.join(c if c.isalnum() else '_' for c in text).strip('_').lower()


def _write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


#######################
# 误差计算
#######################
def project_to_polyline(points, vertices):
    """
    点到折线的最近距离

    返回:
        dist: 每个点到折线的距离
        s: 投影点沿折线的弧长
        seg: 最近线段编号
        proj: 投影点坐标
    """
    points = np.atleast_2d(points)
    a, b = vertices[:-1], vertices[1:]
    ab = b - a
    length = np.linalg.norm(ab, axis=1)
    denom = np.where(length > 0, length**2, 1.0)
    u = np.clip(np.einsum('nkd,kd->nk', points[:, None, :] - a[None], ab) / denom, 0, 1)
    proj = a[None] + u[..., None] * ab[None]
    d = np.linalg.norm(points[:, None, :] - proj, axis=2)
    seg = np.argmin(d, axis=1)
    rows = np.arange(len(points))
    s = np.concatenate([[0], np.cumsum(length)])[seg] + u[rows, seg] * length[seg]
    return d[rows, seg], s, seg, proj[rows, seg]


def rotation_angle(r_ref, r_act):
    """两组旋转矩阵之间的相对旋转角（rad）"""
    rel = np.einsum('nji,njk->nik', r_ref, r_act)
    return R.from_matrix(rel).magnitude()


def _stats(values, scale=1000.0):
    """绝对值统计；默认把米换算为毫米"""
    v = np.abs(np.asarray(values, dtype=float)) * scale
    if v.size == 0:
        return None
    return dict(rmse=float(np.sqrt(np.mean(v**2))), mean=float(v.mean()),
                p95=float(np.percentile(v, 95)), max=float(v.max()))


def analyze_motion(rec):
    """计算一段运动的误差序列和统计指标"""
    t, fb_p, ref_p = rec['fb_t'], rec['fb_p'], rec['ref_p']
    vertices = rec['waypoints']
    t_end = float(rec['plan_t'][-1])
    moving = rec['fb_phase'] == 0
    series = dict(t=t, moving=moving)

    plan_dist, plan_s, _, _ = project_to_polyline(rec['plan_p'], vertices)
    series.update(plan_path_error=plan_dist, plan_s=plan_s)
    metrics = dict(name=rec['name'], command_mode=rec.get('tags',{}).get('command_mode','reference'), completed=bool(rec['completed']), error=rec['error'],
                   path_length_mm=float(np.sum(np.linalg.norm(np.diff(vertices, axis=0), axis=1)) * 1000),
                   duration_s=t_end, planning_path_error_mm=_stats(plan_dist),
                   feedback_samples=int(len(t)))
    lateness = rec['dispatch_t'] - rec.get('command_sample_t', rec['plan_t'][:len(rec['dispatch_t'])])
    if len(lateness):
        metrics['dispatch_lateness_ms'] = dict(p95=float(np.percentile(lateness, 95) * 1000),
                                               max=float(lateness.max() * 1000))
    if len(t) < 2:
        return metrics, series

    err = fb_p - ref_p
    path_dist, path_s, _, _ = project_to_polyline(fb_p, vertices)
    _, _, ref_seg, _ = project_to_polyline(ref_p, vertices)
    tangent = np.diff(vertices, axis=0)
    tangent = tangent / np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    u = tangent[ref_seg]
    along = np.einsum('nd,nd->n', err, u)
    cross = np.linalg.norm(err - along[:, None] * u, axis=1)
    orient = rotation_angle(rec['ref_R'], rec['fb_R'])
    joint_err = rec['fb_q'] - rec['ref_q']
    to_goal = np.linalg.norm(fb_p - vertices[-1], axis=1)
    to_cmd_end = np.linalg.norm(fb_p - rec['plan_p'][-1], axis=1)
    series.update(err=err, err_norm=np.linalg.norm(err, axis=1), along=along, cross=cross,
                  path_error=path_dist, path_s=path_s, orient=orient, joint_err=joint_err,
                  to_goal=to_goal, to_cmd_end=to_cmd_end)

    # 等效延迟：反馈与延迟 τ 后的指令轨迹对齐
    m = moving if moving.sum() >= 2 else np.ones_like(moving)
    rms = [np.sqrt(np.mean(np.sum((fb_p[m] - _interp_rows(t[m] - tau, rec['plan_t'], rec['plan_p']))**2, axis=1)))
           for tau in LAG_SEARCH_S]
    best = int(np.argmin(rms))
    metrics['equivalent_lag_search_upper_bound_ms'] = float(LAG_SEARCH_S[-1] * 1000)
    metrics['equivalent_lag_upper_boundary_hit'] = bool(best == len(LAG_SEARCH_S)-1)

    outside = np.nonzero(to_cmd_end > SETTLE_TOLERANCE_M)[0]
    if len(outside) == 0:
        settle = 0.0
    elif outside[-1] + 1 < len(t):
        settle = float(t[outside[-1] + 1] - t_end)
    else:
        settle = None

    final = fb_p[-1] - vertices[-1]
    at_end = np.nonzero(t <= t_end)[0]
    metrics.update(
        feedback_rate_hz=float((m.sum() - 1) / np.ptp(t[m])) if np.ptp(t[m]) > 0 else None,
        tracking_error_mm=_stats(np.linalg.norm(err[moving], axis=1)),
        along_track_error_mm=_stats(along[moving]),
        cross_track_error_mm=_stats(cross[moving]),
        path_error_mm=_stats(path_dist[moving]),
        orientation_error_deg=_stats(np.rad2deg(orient[moving]), scale=1.0),
        joint_error_max_deg=np.rad2deg(np.max(np.abs(joint_err[m]), axis=0)).tolist(),
        joint_error_rmse_deg=np.rad2deg(np.sqrt(np.mean(joint_err[m]**2, axis=0))).tolist(),
        equivalent_lag_ms=float(LAG_SEARCH_S[best] * 1000),
        residual_after_lag_rmse_mm=float(rms[best] * 1000),
        error_at_trajectory_end_mm=float(to_goal[at_end[-1]] * 1000) if len(at_end) else None,
        final_error_mm=float(np.linalg.norm(final) * 1000),
        final_error_xyz_mm=(final * 1000).tolist(),
        planning_final_error_mm=float(np.linalg.norm(rec['plan_p'][-1] - vertices[-1]) * 1000),
        servo_final_error_mm=float(to_cmd_end[-1] * 1000),
        final_orientation_error_deg=float(np.rad2deg(rotation_angle(
            rec['target_rotation'][None], rec['fb_R'][-1:])[0])),
        settle_time_s=settle, settle_tolerance_mm=SETTLE_TOLERANCE_M * 1000,
    )
    if not moving.any():
        for k in ('tracking_error_mm', 'along_track_error_mm', 'cross_track_error_mm',
                  'path_error_mm', 'orientation_error_deg'):
            metrics[k] = None
    return metrics, series


def write_motion_csv(path, rec, series):
    cols = ['t_s', 'phase'] + [f'cmd_{a}_m' for a in 'xyz'] + [f'fk_{a}_m' for a in 'xyz'] + \
        [f'err_{a}_mm' for a in 'xyz'] + ['err_norm_mm', 'along_mm', 'cross_mm', 'path_error_mm',
                                         'orientation_error_deg'] + [f'joint{j+1}_err_deg' for j in range(6)]
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i in range(len(series['t'])):
            w.writerow([series['t'][i], int(rec['fb_phase'][i]), *rec['ref_p'][i], *rec['fb_p'][i],
                        *(series['err'][i] * 1000), series['err_norm'][i] * 1000, series['along'][i] * 1000,
                        series['cross'][i] * 1000, series['path_error'][i] * 1000,
                        np.rad2deg(series['orient'][i]), *np.rad2deg(series['joint_err'][i])])


#######################
# 可视化
#######################
C_IDEAL, C_CMD, C_MEAS = '#8a8a8a', '#1f6fc5', '#e07b18'


def _pyplot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def plot_motion(path, rec, metrics, series):
    plt = _pyplot()
    t_end = float(rec['plan_t'][-1])
    origin = rec['waypoints'][0]
    mm = lambda p: (np.atleast_2d(p) - origin) * 1000
    has_fb = 'err' in series
    fig, axs = plt.subplots(2, 4, figsize=(21, 9.5), layout='constrained')

    # 路径投影到运动范围最大的两个坐标轴；第三轴偏差体现在路径误差图里
    ax = axs[0, 0]
    w, c = mm(rec['waypoints']), mm(rec['plan_p'])
    f = mm(rec['fb_p']) if has_fb else c
    plane = rec.get('tags', {}).get('plane')
    i, j = PLANES[plane] if plane in PLANES else np.sort(np.argsort(np.ptp(np.vstack([w, c]), axis=0))[-2:])
    ax.plot(w[:, i], w[:, j], '--', color=C_IDEAL, lw=1.5, label='Ideal line')
    ax.plot(c[:, i], c[:, j], color=C_CMD, lw=1.5, label='Commanded FK')
    if has_fb:
        ax.plot(f[:, i], f[:, j], color=C_MEAS, lw=1.1, label='Measured FK')
    ax.plot(w[0, i], w[0, j], 'o', color='k', ms=5, mfc='none', label='Start')
    ax.plot(w[-1, i], w[-1, j], 'x', color='k', ms=7, label='Target')
    ax.set(xlabel=f"{'XYZ'[i]} (mm, rel. start)", ylabel=f"{'XYZ'[j]} (mm, rel. start)",
           title=f"Path, {'XYZ'[i]}{'XYZ'[j]} projection", aspect='equal')
    ax.legend(fontsize=8, loc='best')

    ax = axs[0, 1]
    ax.plot(series['plan_s'] * 1000, series['plan_path_error'] * 1000, color=C_CMD, label='Commanded (planning)')
    if has_fb:
        mv = series['moving']
        ax.plot(series['path_s'][mv] * 1000, series['path_error'][mv] * 1000, '.-', ms=2, lw=0.8,
                color=C_MEAS, label='Measured (motion)')
        ax.plot(series['path_s'][~mv] * 1000, series['path_error'][~mv] * 1000, '.', ms=3,
                color='#7a3d00', label='Measured (settle)')
    for s in _corner_arc_lengths(rec['waypoints']) * 1000:
        ax.axvline(s, color=C_IDEAL, lw=0.8, ls=':')
    ax.set(xlabel='Arc length along ideal path (mm)  [dotted: corners]', ylabel='Distance to ideal path (mm)',
           title='Path error (timing-independent)')
    ax.legend(fontsize=8)

    if has_fb:
        t = series['t']
        ax = axs[0, 2]
        for k, (a, col) in enumerate(zip('XYZ', ('#c0392b', '#27ae60', '#2c6fbb'))):
            ax.plot(t, series['err'][:, k] * 1000, color=col, lw=1, label=a)
        ax.plot(t, series['err_norm'] * 1000, color='k', lw=1.4, label='|e|')
        ax.set(ylabel='Measured − commanded (mm)', title='Time-aligned tracking error')
        ax.legend(fontsize=8, ncol=4)

        ax = axs[0, 3]
        ax.plot(t, series['along'] * 1000, color='#8e44ad', label='Along path (lag < 0)')
        ax.plot(t, series['cross'] * 1000, color='#16a085', label='Cross path')
        ax.set(ylabel='Error (mm)', title='Along / cross path decomposition')
        ax.legend(fontsize=8)

        ax = axs[1, 0]
        ax.semilogy(t, np.maximum(series['to_goal'] * 1000, 1e-3), color=C_MEAS, label='|FK − target|')
        ax.semilogy(t, np.maximum(series['to_cmd_end'] * 1000, 1e-3), color=C_CMD, lw=1,
                    label='|FK − commanded end|')
        ax.axhline(SETTLE_TOLERANCE_M * 1000, color='r', ls='--', lw=1,
                   label=f'settle tol {SETTLE_TOLERANCE_M*1000:g} mm')
        ax.set(ylabel='Distance (mm)', title='Convergence to end point')
        ax.legend(fontsize=8)

        ax = axs[1, 1]
        ax.plot(t, np.rad2deg(series['orient']), color='#555', lw=1)
        ax.set(ylabel='Angle (deg)', title='Orientation error (commanded vs measured)')

        ax = axs[1, 2]
        for k in range(series['joint_err'].shape[1]):
            ax.plot(t, np.rad2deg(series['joint_err'][:, k]), lw=1, label=f'J{k+1}')
        ax.set(ylabel='Measured − commanded (deg)', title='Joint tracking error')
        ax.legend(fontsize=8, ncol=3)

        for a in (axs[0, 2], axs[0, 3], axs[1, 0], axs[1, 1], axs[1, 2]):
            _mark_phases(a, t_end, t[-1])
            a.set_xlabel('Time (s)  [shaded: settle]')

    ax = axs[1, 3]
    ax.axis('off')
    ax.text(0.0, 1.0, _metrics_text(metrics), va='top', family='monospace', fontsize=9.5,
            transform=ax.transAxes)

    for a in axs.flat:
        a.grid(alpha=0.25)
    status = 'completed' if rec['completed'] else f"ABORTED: {rec['error']}"
    fig.suptitle(f"Panthera-HT {metrics.get('command_mode','reference')} | {rec['name']} | {metrics['path_length_mm']:.0f} mm in "
                 f"{metrics['duration_s']:.2f} s | encoder FK | {status}", fontsize=13)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _corner_arc_lengths(vertices, min_turn_deg=10.0):
    """折线上转角超过阈值的顶点弧长；圆的密集顶点不会被标出"""
    d = np.diff(vertices, axis=0)
    length = np.linalg.norm(d, axis=1)
    u = d / np.maximum(length[:, None], 1e-12)
    turn = np.degrees(np.arccos(np.clip(np.einsum('kd,kd->k', u[:-1], u[1:]), -1, 1)))
    return np.cumsum(length)[:-1][turn > min_turn_deg]


def _metrics_text(m):
    fmt = lambda s: '—' if not s else f"{s['rmse']:6.3f} {s['max']:6.3f}"
    lines = ['Key metrics (mm)        RMS    Max',
             f"Planning path        {fmt(m['planning_path_error_mm'])}"]
    if m.get('tracking_error_mm'):
        settle = m['settle_time_s']
        lines += [
            f"Path (straightness)  {fmt(m['path_error_mm'])}",
            f"Tracking |e|         {fmt(m['tracking_error_mm'])}",
            f"  along path         {fmt(m['along_track_error_mm'])}",
            f"  cross path         {fmt(m['cross_track_error_mm'])}",
            '',
            f"Equivalent lag       {m['equivalent_lag_ms']:6.0f} ms",
            f"Residual after lag   {m['residual_after_lag_rmse_mm']:6.3f} RMS",
            f"Orientation max      {m['orientation_error_deg']['max']:6.3f} deg",
            f"Joint error max      {max(m['joint_error_max_deg']):6.3f} deg",
            '',
            f"Final error          {m['final_error_mm']:6.3f}",
            f"  planning end       {m['planning_final_error_mm']:6.3f}",
            f"  servo residual     {m['servo_final_error_mm']:6.3f}",
            f"Settle (to {m['settle_tolerance_mm']:g} mm)   "
            + ('   n/a' if settle is None else f"{settle:+6.2f} s"),
            '',
            f"Feedback rate        {m['feedback_rate_hz'] or 0:6.1f} Hz",
        ]
    if m.get('dispatch_lateness_ms'):
        lines.append(f"Dispatch late max    {m['dispatch_lateness_ms']['max']:6.1f} ms")
    return '\n'.join(lines)


def _mark_phases(ax, t_end, t_last):
    ax.axvline(t_end, color='k', lw=0.8, ls=':')
    if t_last > t_end:
        ax.axvspan(t_end, t_last, color='#f2e6d0', alpha=0.5, lw=0)


def plot_summary(path, all_metrics):
    plt = _pyplot()
    rows = [m for m in all_metrics if m.get('tracking_error_mm')]
    if not rows:
        return
    names = [m['name'] for m in rows]
    panels = [
        ('Path error (mm)', [('RMS', 'path_error_mm', 'rmse'), ('Max', 'path_error_mm', 'max'),
                             ('Planning max', 'planning_path_error_mm', 'max')]),
        ('Tracking error (mm)', [('RMS', 'tracking_error_mm', 'rmse'), ('Max', 'tracking_error_mm', 'max'),
                                 ('RMS after lag removed', 'residual_after_lag_rmse_mm', None)]),
        ('End point error (mm)', [('Final', 'final_error_mm', None),
                                  ('Planning end', 'planning_final_error_mm', None),
                                  ('Servo residual', 'servo_final_error_mm', None)]),
        ('Equivalent lag (ms; * = search limit)', [('Lag', 'equivalent_lag_ms', None)]),
    ]
    colors = ('#1f6fc5', '#e07b18', '#7f8c8d')
    y = np.arange(len(names))
    fig, axs = plt.subplots(1, len(panels), figsize=(18, 1.2 + 0.42 * len(names)),
                            sharey=True, layout='constrained')
    for ax, (title, series) in zip(axs, panels):
        h = 0.8 / len(series)
        for k, (label, key, sub) in enumerate(series):
            vals = [m[key][sub] if sub else m[key] for m in rows]
            bars = ax.barh(y + (k - (len(series) - 1) / 2) * h, vals, h, color=colors[k], label=label)
            labels = [f'{v:.2f}' if 'mm' in title else f"{v:.0f}{'*' if m.get('equivalent_lag_upper_boundary_hit') else ''}" for v,m in zip(vals,rows)]
            ax.bar_label(bars, labels=labels, fontsize=7, padding=2)
        ax.set_title(title, pad=24)
        ax.grid(axis='x', alpha=0.25)
        ax.margins(x=0.18)
        if len(series) > 1:
            ax.legend(fontsize=8, loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=3, frameon=False)
    axs[0].set_yticks(y, names)
    axs[0].invert_yaxis()
    fig.suptitle('Panthera-HT moveL | end-effector error summary (encoder FK)', fontsize=13)
    fig.savefig(path, dpi=140)
    plt.close(fig)


#######################
# 正反向对比
#######################
def _signed_offsets(points, vertices, normal):
    """投影到折线，返回 (弧长, 平面内侧向偏移[左正], 离面偏移)"""
    _, s, seg, proj = project_to_polyline(points, vertices)
    tangent = np.diff(vertices, axis=0)
    tangent = tangent / np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    left = np.cross(normal, tangent[seg])
    delta = points - proj
    return s, np.einsum('nd,nd->n', delta, left), np.einsum('nd,d->n', delta, normal)


def _bin_mean(s, values, edges):
    idx = np.clip(np.digitize(s, edges) - 1, 0, len(edges) - 2)
    count = np.bincount(idx, minlength=len(edges) - 1)
    total = np.bincount(idx, weights=values, minlength=len(edges) - 1)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(count > 0, total / count, np.nan)


def find_direction_pairs(records):
    """按平面、图形和可选几何标识配对；旧记录没有标识时保持兼容。"""
    index = {}
    for k, rec in enumerate(records):
        t = rec.get('tags') or {}
        if {'plane', 'shape', 'direction'} <= t.keys():
            index[(t['plane'], t['shape'], t.get('pair_id', ''), t['direction'])] = k
    return [(p, sh, index[(p, sh, group, 'fwd')], index[(p, sh, group, 'rev')])
            for (p, sh, group, d) in index if d == 'fwd' and (p, sh, group, 'rev') in index]


def analyze_direction_pair(fwd, rev, plane):
    vertices = fwd['waypoints']
    normal = plane_normal(plane)
    length = float(np.sum(np.linalg.norm(np.diff(vertices, axis=0), axis=1)))
    edges = np.linspace(0, length, max(20, int(length / 0.002)) + 1)
    out = dict(edges=edges, normal=normal)
    for key, rec in (('fwd', fwd), ('rev', rev)):
        m = rec['fb_phase'] == 0
        s, lateral, normal_off = _signed_offsets(rec['fb_p'][m], vertices, normal)
        sc, lateral_c, normal_c = _signed_offsets(rec['plan_p'], vertices, normal)
        out[key] = dict(s=s, lateral=lateral, normal=normal_off,
                        lateral_bin=_bin_mean(s, lateral, edges), normal_bin=_bin_mean(s, normal_off, edges),
                        lateral_cmd_bin=_bin_mean(sc, lateral_c, edges), normal_cmd_bin=_bin_mean(sc, normal_c, edges))
    for axis in ('lateral', 'normal'):
        f, r = out['fwd'], out['rev']
        out[f'{axis}_measured_diff'] = f[f'{axis}_bin'] - r[f'{axis}_bin']
        out[f'{axis}_planned_diff'] = f[f'{axis}_cmd_bin'] - r[f'{axis}_cmd_bin']
        out[f'{axis}_execution_diff'] = out[f'{axis}_measured_diff'] - out[f'{axis}_planned_diff']

    def stats(v):
        v = v[~np.isnan(v)] * 1000
        if not v.size:
            return None
        return dict(mean=float(v.mean()), rms=float(np.sqrt(np.mean(v**2))), max=float(np.abs(v).max()))

    metrics = dict(
        plane=plane, shape=fwd['tags']['shape'], fwd=fwd['name'], rev=rev['name'],
        compared_bins=int(np.sum(~np.isnan(out['lateral_execution_diff']))), bin_mm=float(edges[1] * 1000),
        lateral_mean_mm=dict(fwd=float(out['fwd']['lateral'].mean() * 1000),
                             rev=float(out['rev']['lateral'].mean() * 1000)),
        lateral_measured_difference_mm=stats(out['lateral_measured_diff']),
        lateral_planned_difference_mm=stats(out['lateral_planned_diff']),
        lateral_execution_difference_mm=stats(out['lateral_execution_diff']),
        out_of_plane_measured_difference_mm=stats(out['normal_measured_diff']),
        out_of_plane_execution_difference_mm=stats(out['normal_execution_diff']),
        sign_convention='lateral + = left of forward path in (u, v) plane view; out-of-plane along e_u × e_v; '
                        'differences are forward − reverse per arc-length bin; execution = measured − planned',
    )
    return metrics, out


def plot_direction_pair(path, fwd, rev, plane, metrics, out):
    plt = _pyplot()
    i, j = PLANES[plane]
    origin = fwd['waypoints'][0]
    c_fwd, c_rev = '#1f6fc5', '#c0392b'
    fig, axs = plt.subplots(1, 4, figsize=(21, 6.2), layout='constrained',
                            gridspec_kw=dict(width_ratios=[1.1, 1.4, 1.4, 0.9]))
    ax = axs[0]
    w = (fwd['waypoints'] - origin) * 1000
    ax.plot(w[:, i], w[:, j], '--', color=C_IDEAL, lw=1.3, label='Ideal')
    for rec, col, label in ((fwd, c_fwd, 'Forward'), (rev, c_rev, 'Reverse')):
        p = (rec['fb_p'][rec['fb_phase'] == 0] - origin) * 1000
        ax.plot(p[:, i], p[:, j], color=col, lw=1, label=label)
        k = len(p) // 3
        if k > 0 and k + 1 < len(p):
            ax.annotate('', xy=(p[k + 1, i], p[k + 1, j]), xytext=(p[k, i], p[k, j]),
                        arrowprops=dict(arrowstyle='-|>', color=col, lw=1.5, mutation_scale=16))
    ax.set(xlabel=f"{'XYZ'[i]} (mm, rel. forward start)", ylabel=f"{'XYZ'[j]} (mm)",
           title='Measured paths', aspect='equal')
    ax.legend(fontsize=8, loc='best')

    centers = (out['edges'][:-1] + out['edges'][1:]) / 2 * 1000
    for ax, key, title in ((axs[1], 'lateral', 'In-plane lateral offset (+ = left of forward path)'),
                           (axs[2], 'normal', 'Out-of-plane offset')):
        for d, col, label in (('fwd', c_fwd, 'Forward'), ('rev', c_rev, 'Reverse')):
            ax.plot(out[d]['s'] * 1000, out[d][key] * 1000, '.', ms=1.5, color=col, alpha=0.3)
            ax.plot(centers, out[d][f'{key}_bin'] * 1000, color=col, lw=1.3, label=f'{label} measured')
            ax.plot(centers, out[d][f'{key}_cmd_bin'] * 1000, '--', color=col, lw=1, label=f'{label} commanded')
        ax.plot(centers, out[f'{key}_execution_diff'] * 1000, color='k', lw=1.4,
                label='Execution diff (fwd − rev, cmd removed)')
        ax.axhline(0, color=C_IDEAL, lw=0.8)
        for s in _corner_arc_lengths(fwd['waypoints']) * 1000:
            ax.axvline(s, color=C_IDEAL, lw=0.8, ls=':')
        ax.set(xlabel='Arc length along forward path (mm)', ylabel='Offset (mm)', title=title)
        ax.legend(fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=3, frameon=False)
        ax.grid(alpha=0.25)
    axs[0].grid(alpha=0.25)

    mf = lambda v: '      —' if v is None else f'{v:7.3f}'
    row = lambda label, st: f"{label:<15}{mf(st and st['mean'])}{mf(st and st['rms'])}{mf(st and st['max'])}"
    text = '\n'.join([
        'fwd − rev (mm)    mean    RMS    max',
        'Lateral',
        row('  measured', metrics['lateral_measured_difference_mm']),
        row('  planned', metrics['lateral_planned_difference_mm']),
        row('  execution', metrics['lateral_execution_difference_mm']),
        'Out-of-plane',
        row('  measured', metrics['out_of_plane_measured_difference_mm']),
        row('  execution', metrics['out_of_plane_execution_difference_mm']),
        '',
        f"Lateral mean  fwd {mf(metrics['lateral_mean_mm']['fwd'])}",
        f"              rev {mf(metrics['lateral_mean_mm']['rev'])}",
        f"Bins: {metrics['compared_bins']} x {metrics['bin_mm']:.1f} mm",
        '',
        'planned: IK and spline may',
        '  leave different residuals',
        '  in the two directions.',
        'execution = measured − planned:',
        '  direction-dependent servo',
        '  error (backlash/friction/lag).',
    ])
    axs[3].axis('off')
    axs[3].text(0, 1, text, va='top', family='monospace', fontsize=9.5, transform=axs[3].transAxes)
    fig.suptitle(f"Panthera-HT moveL | {plane}_{metrics['shape']} | forward vs reverse | encoder FK", fontsize=13)
    fig.savefig(path, dpi=140)
    plt.close(fig)


#######################
# 汇总入口
#######################
def analyze_run(folder):
    """读取一次运行的全部数据，输出 metrics.json、每段 CSV/PNG 与汇总图"""
    folder = Path(folder)
    os.environ.setdefault('MPLCONFIGDIR', str(folder / '.mplcache'))
    run, records = load_run(folder)
    all_metrics = []
    for m, rec in zip(run['motions'], records):
        metrics, series = analyze_motion(rec)
        all_metrics.append(metrics)
        stem = m['file'][:-4]
        if 'err' in series:
            write_motion_csv(folder / f'{stem}.csv', rec, series)
        plot_motion(folder / f'{stem}.png', rec, metrics, series)
    pair_metrics = []
    for plane, shape, kf, kr in find_direction_pairs(records):
        fwd, rev = records[kf], records[kr]
        if np.sum(fwd['fb_phase'] == 0) < 2 or np.sum(rev['fb_phase'] == 0) < 2:
            continue
        metrics, out = analyze_direction_pair(fwd, rev, plane)
        pair_metrics.append(metrics)
        pair_id = fwd.get('tags', {}).get('pair_id', '')
        metrics['pair_id'] = pair_id
        suffix = f'_{pair_id}' if pair_id else ''
        plot_direction_pair(folder / f'pair_{plane}_{shape}{suffix}.png', fwd, rev, plane, metrics, out)
    plot_summary(folder / 'summary.png', all_metrics)
    _write_json(folder / 'metrics.json', dict(
        definition='encoder joint FK via SDK URDF; reference = continuous planned joints FK at feedback receive time; actual dispatched q/t separately stored',
        motions=all_metrics, direction_pairs=pair_metrics))
    print_summary(all_metrics)
    print_pair_summary(pair_metrics)
    return all_metrics


def print_summary(all_metrics):
    print("\n" + "="*112)
    print(f"{'运动':<18}{'路径RMS':>9}{'路径Max':>9}{'跟踪RMS':>9}{'跟踪Max':>9}"
          f"{'延迟ms':>8}{'去延迟RMS':>11}{'终点mm':>9}{'规划终点':>9}{'稳定s':>8}{'姿态Max°':>10}")
    print("-"*112)
    for m in all_metrics:
        if not m.get('tracking_error_mm'):
            print(f"{m['name']:<18}  无有效反馈")
            continue
        print(f"{m['name']:<18}{m['path_error_mm']['rmse']:>9.3f}{m['path_error_mm']['max']:>9.3f}"
              f"{m['tracking_error_mm']['rmse']:>9.3f}{m['tracking_error_mm']['max']:>9.3f}"
              f"{m['equivalent_lag_ms']:>8.0f}{m['residual_after_lag_rmse_mm']:>11.3f}"
              f"{m['final_error_mm']:>9.3f}{m['planning_final_error_mm']:>9.3f}"
              f"{'n/a' if m['settle_time_s'] is None else format(m['settle_time_s'], '.2f'):>8}"
              f"{m['orientation_error_deg']['max']:>10.3f}")
    print("="*112)


def print_pair_summary(pair_metrics):
    if not pair_metrics:
        return
    f = lambda v: '   n/a' if v is None else f'{v:7.3f}'
    g = lambda st, k: f(st[k] if st else None)
    print(f"\n正反向差值 fwd−rev (mm) {'实测侧偏均值':>12}{'规划侧偏均值':>12}{'执行侧偏均值':>12}{'执行侧偏RMS':>12}{'执行离面RMS':>12}")
    for m in pair_metrics:
        print(f"{m['plane'] + '_' + m['shape']:<22}{g(m['lateral_measured_difference_mm'], 'mean'):>12}"
              f"{g(m['lateral_planned_difference_mm'], 'mean'):>12}{g(m['lateral_execution_difference_mm'], 'mean'):>12}"
              f"{g(m['lateral_execution_difference_mm'], 'rms'):>12}{g(m['out_of_plane_execution_difference_mm'], 'rms'):>12}")


def main():
    parser = argparse.ArgumentParser(description='重新分析 moveL 误差记录')
    parser.add_argument('folder', type=Path)
    analyze_run(parser.parse_args().folder)


if __name__ == '__main__':
    main()
