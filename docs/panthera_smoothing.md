# Panthera 平滑执行

`move_to` 与 `home` 默认使用 Ruckig 平滑参考，再经官方 SDK 执行。命令不变，无需额外开关。单关节 `jog` 仍保留原来的调试方式。仿真仍走 MoveIt，实物此次没有改成 MoveIt。

```bash
cd /home/a/ArmHub
bash scripts/hardware.sh home                 # 预览 Ruckig 时长与参考约束
bash scripts/hardware.sh home --execute       # 平滑回位
bash scripts/move_to.sh --robot panthera_ht --backend hardware --delta 0 0 0.01
bash scripts/move_to.sh --robot panthera_ht --backend hardware --delta 0 0 0.01 --execute
```

实物每次执行仍需 `--execute`；相对命令每次基于新的当前位置，重复会累积。Python `arm.enable(); arm.home()` / `arm.move_to(target)` 自动使用同一平滑执行器。

## 执行链

```text
当前位置新反馈 + home 关节目标／官方 IK 结果
    → Ruckig 生成六轴同步时间轨迹
    → 采样整条参考轨迹做自碰撞检查
    → 再次检查起点稳定及是否变化
    → 按真实时间取 q(t)、v(t)、a(t)，约每 50 ms 下发
    → 官方 pos_vel_MAXtqe + 七轴 motor_send_cmd
    → 新反馈核对跟踪、终点到位和稳定
```

Ruckig 从已确认静止的实测关节角起步，参考初始和终点速度、加速度为零；当前位置中的微小编码器零位偏差不直接截断清零。轨迹极值不得超出物理限位，只有已有起点噪声向模型内收敛的情况允许保留。生成位置在 `armhub/motion.py`，执行循环在 `armhub/adapters/panthera_ht.py`。实现参考旧工程 `smooth_arm_trajectory.JointTransition` 与 `ProjectPanthera.stream_reference`，运行只依赖 ArmHub 自己的文件。

## 参数与语义

`robots/panthera_ht/hardware/config.json` 的 `motion_smoothing`：

| 参数 | 当前值 | 含义 |
|---|---|---|
| `control_period_s` | 0.05 s | 目标参考下发周期 20 Hz，实际周期另行记录 |
| `acceleration_limits_rad_s2` | 六轴 0.8 | 参考加速度上限，沿用旧工程 |
| `jerk_limits_rad_s3` | 六轴 4.0 | 参考 jerk 上限，沿用旧工程 |
| `stream_min_speed_rad_s` | 0.005 | SDK 最低速度上限，沿用旧平滑执行设置 |
| `max_tracking_error_rad` | 0.15 | 实测角度与当前参考最大误差 |
| `max_cycle_gap_s` | 0.2 s | 下发间隔或单次发送耗时超限则尝试保持停止 |
| `feedback_timeout_s` | 0.1 s | 执行期间等待七轴新反馈上限 |

每轴速度上限 `velocity_limits_rad_s` 已恢复为昨天的 `[1.68, 2.0, 2.0, 1.68, 1.48, 1.48] rad/s`，Ruckig、参考检查与 SDK 下发均按各自轴限速；不恢复已删除的单次位移、角度或最长时长限制。实物 `move_to` / `home` 默认 `velocity_scale=1.0`，可显式降速；仿真命令的默认比例仍为 0.15。Ruckig 会按参考速度、加速度、jerk 约束自动延长原先时长，预览显示实际生成的 `duration_s`。这些参数约束上位机参考，不是厂商实物性能保证。

SDK 的 `pos_vel_MAXtqe` 接收位置、速度上限和力矩上限。这里的 `v(t)` 转为速度上限，**不是带符号速度前馈**，`a(t)` 不直接发给固件。固件实际是否平顺仍取决于跟踪表现。轨迹末尾明确下发最终目标，然后等待到位与稳定；不会在每个中间点停下来等待。

若调度稍晚，按当前时间取参考，不补发历史点；长时间阻塞则拒绝继续跳点。故障、反馈中断、跟踪误差超限或 Ctrl+C 会尝试基于新反馈保持；反馈不可用时明确记录 `stop_unconfirmed`。退出不等同于断使能。

碰撞检查针对生成后的 Ruckig 参考，采样时间间隔不大于 50 ms，按速度上限和角度步长进一步细分。夹爪采用模型中性开度，实物维持原始位置。没有外部障碍检查、绕障规划或精确 TCP 直线保证。

## 反馈日志与验证

预览新增 `execution_backend: ruckig_stream`、参考周期、v/a/jerk 上限和计划采样数。完成结果中 `trajectory_points` 为实际下发参考数。

逐帧日志新增：

- `stream_reference`：参考 q/v/a、跟踪误差与指令间隔。
- `stream_completed`：实际参考数、最大间隔、最大跟踪误差与终点误差。
- `stream_failure`：失败原因；停止未能确认时另有 `stop_unconfirmed`。

```bash
# 依赖已装好；新环境恢复时执行 setup_dynamics.sh（已锁定 ruckig==0.12.2）
bash scripts/with_dynamics.sh -m unittest discover -s tests -v
bash scripts/with_dynamics.sh scripts/validate_panthera_smoothing.py
```

本次 53 项离线测试通过。真实 Ruckig + 虚拟调度时钟 + 驱动替身验证了连续下发、起终速度为零、夹爪保持、最终到位、阻塞与反馈失效、跟踪误差、Ctrl+C。两条真实模型离线轨迹（历史回位起点，以及抬高 1 cm）各采样 4001 点核对参考 v/a/jerk，并通过参考轨迹自碰撞检查，报告在 `validation/hardware/panthera_ht/smoothing_offline.json`。

本次未发送实物运动指令。离线测试不能确认实物抖动已改善，实际下发周期、跟踪误差和抖动仍需用户执行后核对。
