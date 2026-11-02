"""Panthera J6 静态辨识采集：默认仅预览，--execute 才发送位置命令。

复用 ArmHub 原有位置控制，不启用纯力矩重力补偿，不改变模型或保护配置。
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

from armhub import create_arm
from armhub.adapters.panthera_gravity import PantheraGravityModel


ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def append_json(stream, value):
    stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')


class Observations:
    """同一离线模型给出基线力矩及 link6 局部重力；无硬件构造。"""
    def __init__(self):
        self.gravity = PantheraGravityModel()
        self.frame = self.gravity.model.getFrameId('link6')
        if self.frame >= len(self.gravity.model.frames):
            raise RuntimeError('动力学模型缺少 link6')
        joint = self.gravity.model.getJointId('joint6')
        # 此脚本的两个一阶矩特征只适用于当前模型的 J6 局部 X 转轴。
        if self.gravity.model.joints[joint].shortname() != 'JointModelRX':
            raise RuntimeError('J6 转轴与采集特征不一致，需要复核模型')

    def __call__(self, positions):
        model = self.gravity
        baseline = model(positions)
        model.pin.framesForwardKinematics(model.model, model.data, model.configuration(positions))
        rotation = model.data.oMf[self.frame].rotation
        local = rotation.T @ model.np.array([0., 0., -9.81])
        gx, gy, gz = (float(v) for v in local)
        return {'model_gravity_nm': baseline, 'gravity_link6_m_s2': [gx, gy, gz],
                'regressor_hy_hz_bias': [-gz, gy, 1.]}


def make_plan(start, j3_offsets, j6_offsets, approach_degrees):
    """先在 J6 起始角改变 J3，再双向扫描；端点有预行程保证接近方向。"""
    nodes = []
    for group, j3_offset in enumerate(j3_offsets):
        q = list(start)
        q[2] += math.radians(j3_offset)
        if group:
            previous = list(nodes[-1]['target_rad'])
            previous[5] = start[5]
            nodes.append({'kind': 'transit', 'target_rad': previous})
        nodes.append({'kind': 'transit', 'target_rad': q.copy()})
        for direction, offsets, lead in [
            ('positive', j6_offsets, j6_offsets[0] - approach_degrees),
            ('negative', list(reversed(j6_offsets)), j6_offsets[-1] + approach_degrees),
        ]:
            q[5] = start[5] + math.radians(lead)
            nodes.append({'kind': 'transit', 'target_rad': q.copy()})
            for offset in offsets:
                q[5] = start[5] + math.radians(offset)
                nodes.append({'kind': 'sample', 'target_rad': q.copy(),
                              'group': group, 'role': 'validation' if group == len(j3_offsets)-1 else 'train',
                              'direction': direction, 'j3_offset_deg': j3_offset, 'j6_offset_deg': offset,
                              'pose_id': f'g{group}_{direction}_{offset:g}'})
    return nodes


def preflight(start, nodes, config, observations, kinematics):
    """所有计划点和名义路径在任何运动命令之前完成检查。"""
    previous = tuple(start)
    features, validation = [], []
    collision_samples = 0
    for node in nodes:
        target = node['target_rad']
        for i, (value, low, high) in enumerate(zip(target, config['lower_rad'], config['upper_rad'])):
            if not low <= value <= high:
                # 与后端一致：零位噪声只允许原样保持。
                if not (value == start[i] and low-.01 <= value <= high+.01):
                    raise ValueError(f'计划 J{i+1}={math.degrees(value):.2f}° 越界；调整偏移方向/范围')
        changed = [i for i, (a, b) in enumerate(zip(previous, target)) if abs(a-b) > 1e-8]
        if len(changed) > 1 or any(i not in (2, 5) for i in changed):
            raise ValueError('计划必须逐次只改变 J3 或 J6')
        collision_samples += kinematics.check_path(previous, target)['samples']
        if node['kind'] == 'sample':
            values = observations(target)
            node['predicted_gravity_link6_m_s2'] = values['gravity_link6_m_s2']
            row = values['regressor_hy_hz_bias']
            (validation if node['role'] == 'validation' else features).append(row)
        previous = tuple(target)
    import numpy as np
    # 除以 g 仅为报告条件数，原始拟合特征仍使用 SI 单位。
    design = np.array(features) / np.array([9.81, 9.81, 1.])
    singular = np.linalg.svd(design, compute_uv=False)
    rank = int(np.linalg.matrix_rank(design))
    if rank < 3:
        raise ValueError('训练姿态不足以分离两个一阶矩与偏置；改变起始姿态或 J3/J6 扫描范围')
    condition = float(singular[0]/singular[-1])
    if condition > 1000:
        raise ValueError(f'训练激励过弱（归一化条件数 {condition:.1f}>1000），调整起始姿态或扫描范围')
    return {'collision_samples': collision_samples,
            'collision_scope': 'nominal_robot_self_collision_only; excludes camera, cables, external obstacles',
            'train_design_rank': rank, 'train_design_condition_scaled': condition,
            'train_singular_values_scaled': singular.tolist(),
            'condition_warning': condition > 100,
            'sample_poses': len(features)+len(validation), 'validation_poses': len(validation)}


def move_one_axis(arm, target, speed_rad_s, kinematics):
    """复用 <=5° 单关节接口；未运动轴始终保持计划目标，不能逐帧重设。"""
    step = min(math.radians(4.), arm.config['max_step_rad'] * .8)
    states = arm.read_feedback(record=False)
    actual = [s.position for s in states[:6]]
    changed = [i for i,(a,b) in enumerate(zip(actual,target))
               if abs(a-b) > arm.config['position_tolerance_rad']]
    if not changed:
        return
    if len(changed) != 1 or changed[0] not in (2, 5):
        raise RuntimeError('运动前非运动轴偏离计划，拒绝多关节修正')
    axis = changed[0]
    while True:
        states = arm.read_feedback(record=False)
        actual = [s.position for s in states[:6]]
        for i, (a, b) in enumerate(zip(actual, target)):
            if i != axis and abs(a-b) > arm.config['position_tolerance_rad']:
                raise RuntimeError(f'运动前 J{i+1} 偏离计划保持角度')
        delta = target[axis]-actual[axis]
        if abs(delta) <= arm.config['position_tolerance_rad']:
            break
        # 把反馈反复写回保持目标会累计量化/跟踪误差，形成目标漂移。
        # 其他轴保持固定的计划值，仅运动轴从新反馈分段前进。
        goal = [max(low,min(high,value)) for value,low,high in
                zip(target,arm.config['lower_rad'],arm.config['upper_rad'])]
        # 非运动轴原始零位噪声若略越界，使用固定物理边界目标；不写编码器零点。
        goal[axis] = actual[axis]+max(-step, min(step, delta))
        kinematics.check_path(actual, goal)
        duration = max(.5, abs(goal[axis]-actual[axis])/speed_rad_s)
        arm.move_joints(goal, duration_s=duration)


def feedback_row(states, target, observations, config, options):
    positions = [s.position for s in states[:6]]
    values = observations(positions)
    reasons = []
    if max(abs(a-b) for a, b in zip(positions, target)) > config['position_tolerance_rad']:
        reasons.append('position_error')
    near_limit = [s.ID for s, cap in zip(states[:6], config['torque_caps_nm']) if abs(s.torque) >= .95*cap]
    if near_limit:
        reasons.append('near_torque_limit')
    now = time.time()
    return {'unix_s': now, 'monotonic_s': time.monotonic(),
            'position_rad': positions, 'velocity_rad_s': [s.velocity for s in states[:6]],
            'measured_torque_nm': [s.torque for s in states[:6]],
            'j6_residual_nm': states[5].torque-values['model_gravity_nm'][5],
            'motors': [{'id': s.ID, 'position_rad': s.position, 'velocity_rad_s': s.velocity,
                        'torque_nm': s.torque, 'mode': s.mode, 'fault': s.fault,
                        'counter': s.num, 'feedback_unix_s': s.time, 'age_s': now-s.time} for s in states],
            **values, 'accepted': not reasons, 'rejection_reasons': reasons,
            'near_torque_limit_ids': near_limit}


def capture_pose(arm, node, observations, options, raw, *, clock=time):
    """以连续位置稳定判断停稳；小幅漂移重启窗口，不因单帧速度退出。"""
    begin = clock.monotonic()
    deadline = begin + options.settle_timeout + options.sample_seconds
    static_since = None
    reference = None
    collecting_since = None
    accepted = []
    total = 0
    window_id = window_restarts = discarded_samples = 0
    while clock.monotonic() < deadline:
        states = arm.read_feedback(record=False)
        row = feedback_row(states, node['target_rad'], observations, arm.config, options)
        if ('gripper_hold_raw_rad' in node and
                abs(states[6].position-node['gripper_hold_raw_rad']) > arm.config['position_tolerance_rad']):
            row['accepted'] = False
            row['rejection_reasons'].append('position_error')
        now = clock.monotonic()
        row.update({key: node[key] for key in ['pose_id', 'group', 'role', 'direction',
                                              'j3_offset_deg', 'j6_offset_deg']})
        row['target_rad'] = node['target_rad']
        if 'position_error' in row['rejection_reasons'] or row['near_torque_limit_ids']:
            row['phase'] = 'aborted'
            append_json(raw, row)
            raise RuntimeError(f"{node['pose_id']} 保持位置偏差或力矩接近限幅，停止采集")
        position_changed = (reference is not None and
                            max(abs(a-b) for a,b in zip(row['position_rad'],reference)) >= arm.config['stable_delta_rad'])
        if position_changed and collecting_since is not None:
            row['discarded_window_id'] = window_id
            row['discarded_sample_count'] = len(accepted)
            discarded_samples += len(accepted)
            accepted = []
            collecting_since = None
            window_restarts += 1
            window_id += 1
            if window_restarts <= 3 or window_restarts % 5 == 0:
                print(f"  {node['pose_id']} 位置窗口发生变化，在原位重新等待停稳（第{window_restarts}次）", flush=True)
        if reference is None or position_changed:
            static_since, reference = now, row['position_rad']
        if collecting_since is None and now-static_since >= options.settle_seconds:
            collecting_since = now
        row['window_id'] = window_id
        row['phase'] = 'settle' if collecting_since is None else 'sample'
        append_json(raw, row)
        total += 1
        if collecting_since is not None:
            accepted.append(row)
            if now-collecting_since >= options.sample_seconds and len(accepted) >= options.min_samples:
                break
        clock.sleep(options.sample_period)
    else:
        raise TimeoutError(f"{node['pose_id']} 未在期限内取得足够连续位置稳定数据（重启窗口{window_restarts}次，有效样本{len(accepted)}个）")
    import numpy as np
    keys = ['position_rad', 'velocity_rad_s', 'measured_torque_nm', 'model_gravity_nm',
            'gravity_link6_m_s2', 'regressor_hy_hz_bias', 'j6_residual_nm']
    summary = {key: node[key] for key in ['pose_id', 'group', 'role', 'direction', 'j3_offset_deg', 'j6_offset_deg']}
    summary.update({'accepted_samples': len(accepted), 'raw_samples': total,
                    'sample_window_id': window_id, 'window_restarts': window_restarts,
                    'discarded_samples': discarded_samples,
                    'actual_sample_span_s': accepted[-1]['monotonic_s']-accepted[0]['monotonic_s'],
                    'target_rad': node['target_rad']})
    for key in keys:
        values = np.array([r[key] for r in accepted])
        summary[key+'_mean'] = np.mean(values, axis=0).tolist()
        summary[key+'_std'] = np.std(values, axis=0).tolist()
    summary['position_span_rad'] = np.ptp(np.array([r['position_rad'] for r in accepted]), axis=0).tolist()
    summary['reported_velocity_peak_rad_s'] = np.max(np.abs(np.array([r['velocity_rad_s'] for r in accepted])),axis=0).tolist()
    return summary


def collect(arm, plan, observations, kinematics, options, folder):
    results = []
    status = {'status': 'running', 'motion_sent': False, 'hold_stop_confirmed': False,
              'completed_poses': 0, 'total_poses': sum(n['kind']=='sample' for n in plan)}
    write_json(folder/'summary.json', status)
    enabled = False
    try:
        with (folder/'samples.jsonl').open('x', buffering=1) as raw, (folder/'poses.jsonl').open('x', buffering=1) as poses:
            arm.enable()
            enabled = True
            # 第一个位置保持也属于运动命令；异常/中断均尝试当前位置保持。
            status['motion_sent'] = True
            arm.stop()
            for node in plan:
                move_one_axis(arm, node['target_rad'], math.radians(options.speed_deg_s), kinematics)
                if node['kind'] != 'sample':
                    continue
                print(f"[{len(results)+1}/{status['total_poses']}] {node['role']} "
                      f"J3偏移={node['j3_offset_deg']:+g}° J6偏移={node['j6_offset_deg']:+g}° "
                      f"{node['direction']}：停稳采样", flush=True)
                record = capture_pose(arm, node, observations, options, raw)
                append_json(poses, record)
                results.append(record)
                status['completed_poses'] = len(results)
                write_json(folder/'summary.json', status)
                print(f"  n={record['accepted_samples']}，J6力矩均值="
                      f"{record['measured_torque_nm_mean'][5]:+.4f} Nm，"
                      f"模型残差={record['j6_residual_nm_mean']:+.4f} Nm", flush=True)
        status['status'] = 'completed'
    except KeyboardInterrupt:
        status.update(status='interrupted', error='Ctrl+C；保留已采集数据')
        raise
    except BaseException as error:
        status.update(status='failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        try:
            if enabled:
                arm.stop()
                status['hold_stop_confirmed'] = True
                status['final_position_rad'] = [s.position for s in arm._last_states[:6]]
        except BaseException as error:
            status.update(status='stop_unconfirmed', stop_error=str(error))
            raise RuntimeError('当前位置保持未确认，请检查实物及物理急停') from error
        finally:
            status['hardware_log'] = str(arm.log_path)
            status['ended_utc'] = datetime.now(timezone.utc).isoformat()
            write_json(folder/'summary.json', status)
            if results:
                with (folder/'poses.csv').open('w', newline='') as stream:
                    fields = ['pose_id', 'role', 'direction', 'j3_offset_deg', 'j6_offset_deg',
                              'accepted_samples', 'j6_residual_nm_mean', 'j6_residual_nm_std']
                    writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
                    writer.writeheader()
                    writer.writerows(results)
    return status


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='发送位置控制命令并采集；默认只读预览')
    parser.add_argument('--offline-q', nargs=6, type=float, help='仅离线验证模型和计划，完全不连接串口（rad）')
    parser.add_argument('--j3-offsets-deg', nargs='+', type=float, default=[0., 10., 20.], help='相对启动姿态；最后一组仅验证')
    parser.add_argument('--j6-offsets-deg', nargs='+', type=float, default=[-30., -20., -10., 0., 10., 20., 30.])
    parser.add_argument('--approach-deg', type=float, default=3., help='扫描两端额外预行程（度）')
    parser.add_argument('--speed-deg-s', type=float, default=2., help='J3/J6位置控制速度上限（度/秒）')
    parser.add_argument('--sample-seconds', type=float, default=2.)
    parser.add_argument('--settle-seconds', type=float, default=.6)
    parser.add_argument('--settle-timeout', type=float, default=10.)
    parser.add_argument('--sample-period', type=float, default=.025, help='查询后的额外等待，非采样频率保证')
    parser.add_argument('--j6-origin-run', type=Path, help='沿用某次已保存实验的 J6 扫描基准，避免失败重跑时偏移累积；J3仍相对本次起点')
    parser.add_argument('--min-samples', type=int, default=20)
    parser.add_argument('--output', type=Path, help='新建的输出目录，不能覆盖已有目录')
    args = parser.parse_args(argv)
    if args.execute and args.offline_q is not None:
        parser.error('--offline-q 不能与 --execute 混用')
    numeric = [args.speed_deg_s, args.sample_seconds, args.settle_seconds, args.settle_timeout,
               args.sample_period, args.approach_deg]
    if any(not math.isfinite(v) or v <= 0 for v in numeric):
        parser.error('采集时间、速度、预行程必须为正的有限数')
    if not 1.2 <= args.speed_deg_s <= 3.:
        parser.error('速度只允许 1.2～3 度/秒，保留现有位置模式速度下限及单关节限制')
    if args.sample_seconds < 1 or args.settle_seconds < .5 or args.min_samples < 10:
        parser.error('采样至少1秒、静止窗口至少0.5秒、有效样本至少10个')
    if args.settle_timeout <= args.settle_seconds:
        parser.error('等待静止超时必须大于静止窗口')
    j3, j6 = args.j3_offsets_deg, args.j6_offsets_deg
    if (len(j3) < 3 or len(set(j3)) != len(j3) or j3[0] != 0 or
            len(j6) < 5 or sorted(set(j6)) != j6 or
            any(not math.isfinite(v) for v in j3+j6+(args.offline_q or []))):
        parser.error('J3至少3个不同偏移且首个为0；J6至少5个严格递增偏移；所有值必须有限')
    return args


def main(argv=None):
    args = parse_args(argv)
    observations = Observations()  # 依赖和模型检查先于串口连接
    arm = create_arm('panthera_ht', 'hardware')
    if math.radians(args.speed_deg_s) > arm.config['jog_max_speed_rad_s']:
        raise ValueError('速度超过当前单关节配置上限')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    folder = args.output or ROOT/'logs/j6_identification'/stamp
    folder = folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    print(f'数据目录：{folder}', flush=True)
    try:
        if args.offline_q is None:
            arm.connect()
            initial_states = arm._stable_feedback()
            start = tuple(s.position for s in initial_states[:6])
        else:
            start = tuple(args.offline_q)
        scan_origin = list(start)
        if args.j6_origin_run is not None:
            saved = json.loads((args.j6_origin_run/'metadata.json').read_text())
            if saved.get('robot') != 'panthera_ht':
                raise ValueError('J6 基准实验不是 Panthera')
            scan_origin[5] = float(saved.get('scan_origin_rad',saved['start_rad'])[5])
            if not math.isfinite(scan_origin[5]):
                raise ValueError('保存的 J6 基准角无效')
        plan = make_plan(scan_origin, args.j3_offsets_deg, args.j6_offsets_deg, args.approach_deg)
        if args.offline_q is None:
            for node in plan:
                node['gripper_hold_raw_rad'] = initial_states[6].position
        kinematics = arm._kinematics()
        report = preflight(start, plan, arm.config, observations, kinematics)
        metadata = {'schema_version': 2, 'robot': 'panthera_ht',
                    'source': 'offline_preview' if args.offline_q is not None else 'hardware',
                    'created_utc': datetime.now(timezone.utc).isoformat(),
                    'start_rad': start, 'scan_origin_rad': scan_origin,
                    'static_detection': 'continuous_position_window; reported_velocity_is_diagnostic_only',
                    'hold_target_policy': 'fixed_plan_targets; clamp_boundary_noise_only; no_feedback_reanchoring',
                    'control': 'position_hold_POS_VEL_MAX_TORQUE',
                    'gravity_world_m_s2': [0., 0., -9.81], 'payload_frame': 'link6',
                    'regressor_parameters': ['hy_kg_m', 'hz_kg_m', 'bias_nm'],
                    'regressor_equation': 'measured_tau6 - model_G6 = -gz*hy + gy*hz + bias + residual',
                    'model_gripper_coordinates': 'neutral; raw gripper motor rad is not prismatic m',
                    'torque_feedback': 'SDK motor feedback decoded as Nm; physical calibration not verified',
                    'model_sha256': hashlib.sha256(observations.gravity.model_path.read_bytes()).hexdigest(),
                    'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    'hardware_config': arm.config, 'options': {k: str(v) if isinstance(v, Path) else v for k,v in vars(args).items()},
                    'preflight': report, 'hardware_log': str(arm.log_path)}
        write_json(folder/'metadata.json', metadata)
        write_json(folder/'plan.json', plan)
        print(f'起始角度（度）：{[round(math.degrees(v), 2) for v in start]}', flush=True)
        print(f'J6扫描基准（度）：{math.degrees(scan_origin[5]):.3f}',flush=True)
        print(f'J3相对偏移：{args.j3_offsets_deg}°；J6相对偏移：{args.j6_offsets_deg}°；两端另有±{args.approach_deg}°预行程', flush=True)
        print(f"共 {report['sample_poses']} 个采样点，最后一组 J3 保留验证；训练设计矩阵秩={report['train_design_rank']}，条件数={report['train_design_condition_scaled']:.1f}", flush=True)
        if report['condition_warning']:
            print('提示：当前特征分离较弱，可能需要更大姿态跨度；会保留原始数据供解算诊断。', flush=True)
        print('碰撞检查仅覆盖现有机器人模型；相机、线缆、桌面等需核对实物空间。', flush=True)
        print('结束/Ctrl+C在最后位置保持，不回到起点、不回零、不关闭力矩。', flush=True)
        if not args.execute:
            write_json(folder/'summary.json', {'status': 'preview', 'motion_sent': False, **report})
            print('预览完成，未发送运动命令。加 --execute 才开始采集。', flush=True)
            return 0
        print('开始位置保持采集；关闭其他机械臂控制程序，采样时勿用手托住负载。', flush=True)
        result = collect(arm, plan, observations, kinematics, args, folder)
        print(f"采集完成：{result['completed_poses']} 点；保持确认={result['hold_stop_confirmed']}\n数据目录：{folder}", flush=True)
        return 0
    except BaseException as error:
        if not (folder/'summary.json').exists():
            write_json(folder/'summary.json', {'status': 'preflight_failed', 'motion_sent': False,
                                               'error': f'{type(error).__name__}: {error}'})
        print(f'采集未完成：{type(error).__name__}: {error}\n已保存数据：{folder}', flush=True)
        return 130 if isinstance(error, KeyboardInterrupt) else 1
    finally:
        arm.close()


if __name__ == '__main__':
    raise SystemExit(main())
