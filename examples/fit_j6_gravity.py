"""离线解算 J6 静态采集数据；不导入硬件接口、不修改控制配置。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def rmse(values):
    return float(np.sqrt(np.mean(np.square(values))))


def fit(x, y, train):
    """只在训练姿态拟合；每个姿态等权，不因连续帧数不同改变权重。"""
    theta, _, rank, singular = np.linalg.lstsq(x[train], y[train], rcond=None)
    if rank < x.shape[1]:
        raise ValueError('训练设计矩阵不满秩')
    prediction = x @ theta
    error = y-prediction
    return {'parameters': theta.tolist(), 'train_rank': int(rank),
            'train_condition_si': float(singular[0]/singular[-1]),
            'train_rmse_nm': rmse(error[train]), 'validation_rmse_nm': rmse(error[~train]),
            'validation_max_abs_error_nm': float(np.max(np.abs(error[~train]))),
            'baseline_validation_rmse_nm': rmse(y[~train]),
            'prediction_nm': prediction.tolist(), 'error_nm': error.tolist()}


def pair_poses(poses):
    pairs = []
    keys = sorted({(p['group'],p['j6_offset_deg']) for p in poses})
    for group, offset in keys:
        items = [p for p in poses if (p['group'],p['j6_offset_deg'])==(group,offset)]
        if len(items)!=2 or {p['direction'] for p in items}!={'positive','negative'}:
            raise ValueError(f'组{group}、J6偏移{offset} 缺少唯一的正反配对')
        positive = next(p for p in items if p['direction']=='positive')
        negative = next(p for p in items if p['direction']=='negative')
        if positive['role'] != negative['role']:
            raise ValueError('正反配对训练/验证标签不一致')
        pairs.append({'group': group, 'role': positive['role'], 'j3_offset_deg': positive['j3_offset_deg'],
                      'j6_offset_deg': offset,
                      'features': np.mean([p['regressor_hy_hz_bias_mean'] for p in items],axis=0).tolist(),
                      'mean_residual_nm': float(np.mean([p['j6_residual_nm_mean'] for p in items])),
                      'positive_nm': positive['j6_residual_nm_mean'], 'negative_nm': negative['j6_residual_nm_mean'],
                      'half_difference_nm': (positive['j6_residual_nm_mean']-negative['j6_residual_nm_mean'])/2,
                      'j6_actual_direction_difference_rad': abs(positive['position_rad_mean'][5]-negative['position_rad_mean'][5])})
    return pairs


def validate_data(folder):
    meta = json.loads((folder/'metadata.json').read_text())
    status = json.loads((folder/'summary.json').read_text())
    poses = read_jsonl(folder/'poses.jsonl')
    if meta.get('robot')!='panthera_ht' or meta.get('source')!='hardware':
        raise ValueError('需要 Panthera 实机采集数据')
    if status['status']!='completed' or not status['hold_stop_confirmed']:
        raise ValueError('采集未完整完成或未确认保持，不作为完整实验解算')
    if len(poses)!=status['total_poses'] or len({p['pose_id'] for p in poses})!=len(poses):
        raise ValueError('姿态点数或唯一性校验失败')
    expected = {n['pose_id'] for n in read_plan(folder) if n['kind']=='sample'}
    if {p['pose_id'] for p in poses} != expected:
        raise ValueError('实际采样点与计划不一致')
    raw = read_jsonl(folder/'samples.jsonl')
    by_id = {p['pose_id']: [] for p in poses}
    for row in raw:
        if row.get('pose_id') in by_id:
            by_id[row['pose_id']].append(row)
    min_span = float('inf')
    max_mean_error = 0.
    effective = []
    for pose in poses:
        window = pose.get('sample_window_id',0)
        rows = [r for r in by_id[pose['pose_id']] if r['phase']=='sample' and r['accepted'] and r.get('window_id',0)==window]
        if len(rows)!=pose['accepted_samples'] or len(rows)<meta['options']['min_samples']:
            raise ValueError(f"{pose['pose_id']} 有效窗口样本数不一致")
        span = rows[-1]['monotonic_s']-rows[0]['monotonic_s']
        min_span = min(min_span,span)
        if span+1e-6<meta['options']['sample_seconds']:
            raise ValueError('有效采样时长不足')
        for field in ['position_rad','measured_torque_nm','model_gravity_nm','regressor_hy_hz_bias','j6_residual_nm']:
            error = float(np.max(np.abs(np.mean([r[field] for r in rows],axis=0)-pose[field+'_mean'])))
            max_mean_error = max(max_mean_error,error)
            if error>1e-8:
                raise ValueError(f'{field} 原始样本与摘要不一致')
        for row in rows:
            if any(m['fault'] for m in row['motors']) or row['rejection_reasons']:
                raise ValueError('有效窗口中存在故障/拒绝样本')
            if row['near_torque_limit_ids']:
                raise ValueError('存在接近力矩限幅的有效样本')
        effective.extend(rows)
    quality = {'pose_count':len(poses),'raw_frames':len(raw),'effective_frames':len(effective),
               'minimum_sample_span_s':min_span,'summary_mean_crosscheck_max_error':max_mean_error,
               'maximum_j6_within_pose_std_nm':max(p['j6_residual_nm_std'] for p in poses),
               'max_position_span_rad':np.max([p['position_span_rad'] for p in poses],axis=0).tolist(),
               'window_restarts':sum(p['window_restarts'] for p in poses),
               'maximum_effective_feedback_age_s':max(m['age_s'] for r in effective for m in r['motors']),
               'hold_stop_confirmed':True}
    return meta, poses, quality


def read_plan(folder):
    return json.loads((folder/'plan.json').read_text())


def solve(poses):
    train = np.array([p['role']=='train' for p in poses])
    if not np.any(train) or not np.any(~train):
        raise ValueError('必须有独立训练与验证姿态')
    groups_train = {p['group'] for p in poses if p['role']=='train'}
    groups_validation = {p['group'] for p in poses if p['role']=='validation'}
    if groups_train & groups_validation:
        raise ValueError('同一 J3 组不可同时参与训练和验证')
    x = np.array([p['regressor_hy_hz_bias_mean'] for p in poses])
    y = np.array([p['j6_residual_nm_mean'] for p in poses])
    direction = np.array([1 if p['direction']=='positive' else -1 for p in poses])
    pairs = pair_poses(poses)
    xp = np.array([p['features'] for p in pairs])
    yp = np.array([p['mean_residual_nm'] for p in pairs])
    tp = np.array([p['role']=='train' for p in pairs])
    fits = {'gravity_individual':fit(x[:,:2],y,train),
            'gravity_bias_individual':fit(x,y,train),
            'gravity_bias_direction_individual':fit(np.column_stack([x,direction]),y,train),
            'gravity_paired':fit(xp[:,:2],yp,tp),
            'gravity_bias_paired':fit(xp,yp,tp)}
    half = np.array([p['half_difference_nm'] for p in pairs])
    condition = float(np.linalg.cond(x[train]/[9.81,9.81,1.]))
    preferred = np.array(fits['gravity_paired']['parameters'])
    gravity_prediction = x[:,:2]@preferred
    fits['gravity_paired']['individual_validation_rmse_nm'] = rmse((y-gravity_prediction)[~train])
    return {'train_groups':sorted(groups_train),'validation_groups':sorted(groups_validation),
            'train_poses':int(np.sum(train)),'validation_poses':int(np.sum(~train)),
            'train_condition_scaled_with_bias':condition,
            'fits':fits,'paired_data':pairs,
            'directional_half_gap_nm':{'mean':float(np.mean(half)),'min':float(np.min(half)),
                                      'max':float(np.max(half)),'std':float(np.std(half))},
            'gravity_prediction_range_nm':[float(np.min(gravity_prediction)),float(np.max(gravity_prediction))],
            'interpretation':'Effective J6 correction in SDK torque units; direction dependence is not uniquely attributable to friction; mass/3D COM not separately identifiable.'}


def plot_result(poses, report, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    pairs = report['paired_data']
    theta = np.array(report['fits']['gravity_paired']['parameters'])
    fig, axes = plt.subplots(2,3,figsize=(14,7),sharex=True,constrained_layout=True)
    for col, group in enumerate(sorted({p['group'] for p in poses})):
        for direction, marker, label in [('positive','o','Positive approach'),('negative','s','Negative approach')]:
            items = sorted([p for p in poses if p['group']==group and p['direction']==direction],key=lambda p:p['j6_offset_deg'])
            axes[0,col].errorbar([p['j6_offset_deg'] for p in items],[p['j6_residual_nm_mean'] for p in items],
                                 yerr=[p['j6_residual_nm_std'] for p in items],marker=marker,capsize=2,label=label)
        data = [p for p in pairs if p['group']==group]
        offsets = [p['j6_offset_deg'] for p in data]
        observed = np.array([p['mean_residual_nm'] for p in data])
        predicted = np.array([p['features'][:2] for p in data])@theta
        axes[0,col].plot(offsets,observed,'k.-',label='Paired mean')
        axes[0,col].plot(offsets,predicted,'r--',label='Gravity candidate (train only)')
        axes[1,col].plot(offsets,observed-predicted,'o-',label='Paired prediction error')
        axes[1,col].plot(offsets,[p['half_difference_nm'] for p in data],'s--',label='Half directional gap')
        for axis in axes[:,col]:
            axis.axhline(0,color='gray',lw=.7)
            axis.grid(alpha=.25)
        axes[0,col].set_title(f"J3 offset +{data[0]['j3_offset_deg']:g} deg ({data[0]['role']})")
        axes[1,col].set_xlabel('J6 offset (deg)')
    axes[0,0].set_ylabel('Measured torque - original G6 (Nm)')
    axes[1,0].set_ylabel('Torque (Nm)')
    axes[0,0].legend(fontsize=8)
    axes[1,0].legend(fontsize=8)
    fig.suptitle('Panthera J6: static payload identification / no controller parameters changed')
    fig.savefig(path,dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    folder = args.run.resolve()
    output = args.output or Path(__file__).resolve().parents[1]/'validation/j6_identification'/folder.name
    output.mkdir(parents=True,exist_ok=False)
    meta,poses,quality = validate_data(folder)
    report = solve(poses)
    report.update({'source_run':str(folder),'source':'offline_analysis_of_recorded_hardware',
                   'hardware_connected':False,'parameters_applied':False,'quality':quality,
                   'source_sha256':{name:hashlib.sha256((folder/name).read_bytes()).hexdigest()
                                    for name in ['metadata.json','samples.jsonl','poses.jsonl','summary.json']}})
    preferred = report['fits']['gravity_paired']
    candidate = {'status':'offline_candidate_only','enabled':False,'automatically_applied':False,
                 'source_run':str(folder),'frame':'link6','units':'kg*m conditional on SDK torque scale',
                 'hy':preferred['parameters'][0],'hz':preferred['parameters'][1],
                 'equation':'delta_G6 = -gravity_link6_z * hy + gravity_link6_y * hz',
                 'bias_applied':False,'directional_term_applied':False,
                 'hardware_compensation_accuracy':'not tested',
                 'limitations':['Not an isolated camera mass or COM estimate',
                                'Directional hysteresis remains; static approach sign is not a validated moving-friction law',
                                'Parameters describe only J6; other joints have not been identified',
                                'SDK torque physical scale not independently verified']}
    (output/'fit_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (output/'candidate_parameters.json').write_text(json.dumps(candidate,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    with (output/'paired_predictions.csv').open('w',newline='') as stream:
        fields = ['group','role','j3_offset_deg','j6_offset_deg','positive_nm','negative_nm','mean_residual_nm','half_difference_nm','gravity_prediction_nm','error_nm']
        writer = csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore')
        writer.writeheader()
        for pair,prediction,error in zip(report['paired_data'],preferred['prediction_nm'],preferred['error_nm']):
            writer.writerow({**pair,'gravity_prediction_nm':prediction,'error_nm':error})
    plot_result(poses,report,output/'diagnostics.png')
    baseline = preferred['baseline_validation_rmse_nm']
    val = preferred['validation_rmse_nm']
    directional = report['fits']['gravity_bias_direction_individual']
    text = f"""# Panthera J6 静态辨识结果

原始数据：`{folder}`。离线分析，不连接机械臂，没有应用参数。

## 数据与验证

- 完成 {quality['pose_count']} 点、{quality['effective_frames']} 帧有效数据，原始帧/摘要均值校验通过，已确认停止保持。
- 每个有效窗口至少 {quality['minimum_sample_span_s']:.3f} 秒；J6 点内最大标准差 {quality['maximum_j6_within_pose_std_nm']:.5f} Nm。
- J3 偏移 0°/10° 的 28 个点训练；20° 的 14 个点只验证。每点等权，不随机拆连续帧。
- 本轮归一化训练条件数（含偏置）{report['train_condition_scaled_with_bias']:.2f}。

## 有效重力修正候选

先将同组同角度的正反数据配对取均值，再拟合两个质量一阶矩，不把方向项或常量偏置混入重力：

```text
hy = {candidate['hy']:.9f} kg*m
hz = {candidate['hz']:.9f} kg*m
delta_G6 = -gz*hy + gy*hz
```

这里 gy/gz 为 link6 坐标系中的实际重力（m/s²）。以上物理单位以 SDK 力矩比例正确为前提。这是原模型误差与附加相机负载的有效合成修正，不是相机单独质量/三维重心。

验证集正反均值 RMSE：原模型 {baseline:.5f} Nm → 候选 {val:.5f} Nm，下降 {(1-val/baseline)*100:.1f}%。这是**配对均值**结果。对单方向保持力矩，重力候选验证 RMSE 为 {preferred['individual_validation_rmse_nm']:.5f} Nm；原始单方向验证基线为 {report['fits']['gravity_individual']['baseline_validation_rmse_nm']:.5f} Nm。

## 尚未解释的方向相关误差

正反差值的一半平均 {report['directional_half_gap_nm']['mean']:.5f} Nm，范围 {report['directional_half_gap_nm']['min']:.5f}～{report['directional_half_gap_nm']['max']:.5f} Nm。考虑常量偏置和接近方向后，方向系数 {directional['parameters'][3]:.5f} Nm，单方向验证 RMSE 为 {directional['validation_rmse_nm']:.5f} Nm。

这表明方向相关效应显著，可能涉及静摩擦、回差、线缆或驱动反馈特性；当前实验不能唯一归因，也不能把静止时的接近方向项直接当作运动摩擦补偿。重力模型仍有约 {val:.5f} Nm 的配对验证误差，不能宣称完全消除下沉/上抬。

含偏置拟合得到的 hy 会明显变化，说明小量 hy 与常量偏置的区分较弱；所有模型参数及误差保存在 `fit_report.json`。

## 接下来

先用候选只修正 J6 重力项，在现有力矩限幅和反馈保护下比较新姿态补偿前后的效果，并记录静止漂移与残余力矩。当前未修改重力控制程序；`candidate_parameters.json` 的 enabled=false。其他关节负载效应、相机独立质量、完整重心和惯量均未辨识。

![诊断曲线](diagnostics.png)
"""
    (output/'report.md').write_text(text)
    print(json.dumps({'output':str(output.resolve()),'quality':quality,'candidate':candidate,
                      'paired_validation_before_nm':baseline,'paired_validation_after_nm':val,
                      'direction_coefficient_nm':directional['parameters'][3]},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
