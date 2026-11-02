# 统一重力补偿命令

## 持续补偿

保持机械臂工作区无人、急停可操作，先用手托住机械臂。退出旧控制器；补偿命令独占同一个串口。

```bash
cd /home/a/ArmHub
bash scripts/hardware.sh gravity --robot panthera_ht --execute
```

补偿运行时可以缓慢改变姿态，观察是否明显自行上抬、下沉或抖动。**Ctrl+C 会切回当时位置保持，并检查反馈；切回后不要继续强行拖动。** 程序退出不会自动回零、刹车、复位或断使能。如果反馈失效导致无法确认停止，终端会报错，不能将程序退出当作机械臂已安全停止。

需要定时结束时可选加 `--seconds N`，默认不设时限。

只做离线计算，不连接设备：

```bash
bash scripts/hardware.sh gravity --robot panthera_ht
bash scripts/hardware.sh gravity --robot panthera_ht --q 0.1 0.5 0.6 -0.2 0.3 0.4
```

预览默认使用模型零姿态，明确标记 `offline_zero_pose`，不是读取实物当前位置。`--q` 不能用于实际执行；实物计算只使用实时反馈。

## 统一接口

```python
from armhub import create_arm

with create_arm('panthera_ht', 'hardware') as arm:
    arm.enable()
    result = arm.run_gravity_compensation(duration_s=10.0)
    print(result.hold_stop_confirmed)
```

使用动力学环境运行 Python：`bash scripts/with_dynamics.sh your_script.py`。不指定 `duration_s` 时持续到 Ctrl+C。NERO 尚未接入该能力，统一接口明确返回不支持，不猜测或模拟厂商开关。

## 实现与来源

参考 `/home/a/Panthera-HT/scripts/gravity_comp_joints.py`：它是上位机根据关节角持续计算重力并发送力矩，而不是固件重力补偿开关。

本工程不构造第二个硬件 Robot。`PantheraGravityModel` 只创建离线 Pinocchio 模型，调用已导入官方 `Panthera.get_Gravity`，再由现有硬件连接下发：

```text
七电机新反馈 → 六轴角度 q
                   ↓
官方 get_Gravity(q) → G(q)
                   ↓
限幅 → SDK 五参数接口（六轴 Kp=0、Kd=0、前馈力矩=G）
                   ↓
到时 / Ctrl+C / 异常 → 当前姿态保持与反馈确认
```

模型和 `get_Gravity` 均使用 LeRobot 集成的官方 SDK：`integrations/panthera_lerobot/Panthera-HT_SDK/panthera_python/`。模型路径从其 `robot_param/Follower.yaml` 读取，与 LeRobot 使用同一份 follower URDF；不再使用 ROS 规划模型。当前该 URDF 的 `link6` 质量按用户要求设为 0.43 kg，质心和惯性参数未改动。

当前只补偿模型中的重力，不包含摩擦、科氏力、实际工具负载辨识或位置刚度。假设与原工程一样基座直立、重力方向为模型 −Z。官方 follower 模型只有六个转动关节。

位置控制和五参数力矩控制共享 SDK 的命令缓冲区，不能在同一帧混用。补偿帧只填写六轴，夹爪槽位保留协议的“未更新”标志；退出保持时才统一填写七个电机的实测位置。未加入夹爪开合动作。

## 参数与日志

参数在 `robots/panthera_ht/hardware/config.json` 的 `gravity_compensation`：

- 力矩上限 `[15, 30, 30, 15, 5, 3]` Nm。沿用旧补偿范围，并将 J6 限制在现有实物配置的 3 Nm 以内。超过时限幅且记录饱和次数；饱和时不能宣称补偿完整。
- 目标周期 5 ms；**这是配置目标，不是实测频率保证**。循环等待全部七电机的新帧，实际频率取决于反馈与操作系统调度。
- 新帧等待上限 50 ms、指令周期间隔上限 100 ms、六轴速度保护阈值 2.0 rad/s；原有关节限位、故障和新鲜度检查继续生效。速度阈值是手动拖动测试参数，不是厂商保证。
- 大约每秒打印姿态、力矩、平均循环频率和饱和次数；约 20 Hz 保存详细记录到 `logs/panthera_ht/*.jsonl`。
- 返回 `GravityResult`，记录实际运行时长、周期数、平均频率、最大指令间隔、饱和周期、退出原因以及保持停止是否确认。

沿用固件 `motor_timeout_ms=0`，进程被强杀或系统失联时没有新增的固件自动停机保证。Ctrl+C 是受程序处理的正常退出；不能用断 USB、kill -9 或关闭终端代替它。

## 依赖与验证状态

动力学环境独立放在 `.venv_dynamics/`，版本按参考工程锁定于 `deps/gravity-requirements.txt`。本机已安装；重建用 `bash scripts/setup_dynamics.sh`。

验证包括：

- 27 项离线测试通过：包含既有接口与运动测试，以及力矩限幅、六轴帧/七轴保持切换、Ctrl+C、慢周期、反馈中断、速度保护诊断和控制模式互斥。
- 三个模型姿态下，官方重力计算与势能数值梯度一致，最大误差约 `5.1e-9 Nm`；这只验证模型计算，不验证实物质量、摩擦或安装标定。
- 证据：`validation/gravity/dynamics_check.json`、`offline_zero_pose.json`、`control_tests.log`。

实现验证阶段没有启用实物补偿。随后用户进行了首次手动拖动测试，补偿运行约 4.36 秒、858 个控制周期后触发原 1.0 rad/s 速度保护，并完成退出保持确认。用户确认触发前正在手动拖动；该次记录不能证明力矩补偿精度或自主稳定性。

原日志缺失触发瞬间的具体关节速度，现已补上 `gravity_speed_guard` 事件，记录超限关节、原始速度、样本计数、反馈时间及最近 150 ms 位置/速度序列。重力补偿专用阈值调整为 2.0 rad/s，保留瞬时保护，未改动其他保护；随后用户反馈“测试通过”；该反馈说明用户已完成实物体验验证，尚无量化补偿精度报告。

补偿正常后，再单独验证关节/空间目标与轨迹执行；不自动解除现有单关节调试限制。
