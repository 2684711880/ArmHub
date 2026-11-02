# Panthera-HT 实物首次接入

2026-10-19，独立复制参考工程 `/home/a/Panthera-HT` 已验证的官方原生 SDK 及既有生命周期/反馈快照修正，在 ArmHub 中重新编译，不依赖旧工程运行环境。

## 当前结果

读取串口 `/dev/ttyACM0` 的七电机新反馈，故障码为 0；J6 相对当前姿态下发 +2°，停止保持后的实测行程约 1.8°，与目标差约 0.20°。其他机械臂关节最大位置变化约 0.108°。已通过新反馈确认当前位置保持。

当前 `position_tolerance_rad=0.01`，即约 0.573°，成功表示在该容差内到位并保持配置要求的时间，不表示精确走完 2°。本次日志也不证明所有关节精度或完整轨迹性能。

证据在 `validation/hardware/panthera_ht/`：只读检查、J6 测试报告、汇总与控制台记录；逐帧位置、速度、力矩、故障、模式、反馈计数和命令保存在报告关联的 `logs/panthera_ht/*.jsonl`。

## 使用入口

先关闭旧工程控制器、拖动示教或任何其他串口占用程序。机械臂工作区无人，物理急停可操作。

```bash
cd /home/a/ArmHub
# 只读：状态查询，不使能、不回零、不发送运动目标
bash scripts/hardware.sh state
# 只预览：以本次实测姿态为起点
bash scripts/hardware.sh jog --joint 6 --degrees 2
# 明确执行：从本次实测起点相对 +2°
bash scripts/hardware.sh jog --joint 6 --degrees 2 --duration 2 --execute
```

**每运行一次 jog 都是基于当次当前位置的相对运动，重复执行会累积位移。** 不自动回原位。若只是查看，不加 `--execute`。

设备名称变化时可设置 `ARMHUB_PANTHERA_DEVICE=/dev/serial/by-id/...`，其目标必须是已确认的 ttyACM/ttyUSB 字符设备。程序检查访问权限、现有串口占用，并用设备锁防止本工程重复连接。该 advisory lock 不能阻止启动后其他不遵守锁的程序强行打开串口，因此运行期间不要再开旧控制器。

## SDK 与调用链

```text
create_arm("panthera_ht", "hardware")
         ↓
PantheraHardwareBackend
         ↓
官方 hightorque_robot.Robot / Motor
         ↓ pos_vel_MAXtqe → motor_send_cmd
USB 串口 → CAN 板 → 七个电机
         ↓ 接收线程 + 加锁反馈快照
每轴样本计数/新鲜度/故障检查 → 到位和保持确认
```

原始官方 Python `Panthera_lib` 也复制到了 SDK 参考目录，但首次关节调试直接使用与昨天 `ProjectPanthera.Joint_Pos_Vel` 相同的官方原生位置/速度上限/力矩上限接口，不导入旧视觉应用，也不额外安装其 Pinocchio/视觉环境。没有重新实现串口/CAN 协议。

资源位置：

- `vendor/panthera/sdk/native/`：既有修正后的官方 C++ SDK 源码。
- `vendor/panthera/sdk/bindings.cpp`：既有 Python 绑定，含 `commission_safe_binding` 和反馈快照复制。
- `vendor/panthera/sdk/official_python/Panthera_lib/`：官方高级 Python API 参考。
- `vendor/panthera/sdk/python/hightorque_robot/`：本工程编译的绑定。
- `hardware_ws/build/`：本工程原生库、LCM 和 serial 库构建结果。
- `robots/panthera_ht/hardware/`：设备、电机、物理关节限制与接入参数。
- `armhub/adapters/panthera_ht.py`：ArmHub 适配逻辑。
- `deps/panthera_sdk.lock.json`：来源与文件 SHA256。

重新编译使用 `bash scripts/build_hardware.sh`。首次导入脚本依赖旧工程路径，复制完成后的编译和运行只使用 ArmHub 自己的文件。CMake 库版本与 Python 绑定元数据为 4.4.7；原生 SDK 启动横幅标为 4.6.0，两者是来源工程已有的版本字段差异。

显式回位入口为 `bash scripts/hardware.sh home --execute` / `arm.home()`：六轴回零姿态、夹爪保持；不重新标定零点。见 [回 home](panthera_home.md)。

## 本阶段明确边界

- 已开放只读状态、小幅单关节运动、当前位置保持式停止；统一重力补偿已完成软件与模型验证，用户已反馈实物测试通过，见 `gravity_compensation.md`；NERO 实物仍未接入。
- `move_joints` 单次相对当前姿态不超过 5°，最多一个关节发生显著变化；`move_to` 支持受限多关节到点，见 [实物 XYZ](panthera_xyz.md)。单关节调试 `jog` 继续使用独立的 `jog_max_speed_rad_s=0.10`；空间到点、home 与视觉任务恢复昨天的每轴速度上限，详见平滑执行说明。
- `enable()` 是软件执行许可，不发送独立固件使能命令；第一次位置控制命令的电机行为由官方协议和固件决定。
- 关节目标按实物限位检查。编码器在零位附近小于 0.01 rad 的反馈偏差仅允许保持当前实测值，不扩大主动运动范围，也不将反馈强制写为零。
- 七电机共享缓冲区，每次明确填写六轴目标，并保持夹爪的实测原始角度。本轮不提供夹爪开合，也不将原始夹爪 rad 直接当作模型 prismatic m。
- duration 用于按位移计算固件速度上限，并非已验证的精确到达时间、速度曲线或加速度保证。
- `stop()` 读取新反馈，发送当前位置保持，再核对位置和速度；不是物理急停或断使能。
- 连接及退出使用被动生命周期，不自动 home、reset、brake、清零。退出后固件可保持最后目标，**退出程序不代表电机失能**。
- 沿用已验证配置 `motor_timeout_ms=0`，没有新增固件通信失联自动停机策略。若反馈失效导致无法确认保持停止，程序会明确报错；不能据“停止发送命令”推断实物已停止，需要检查实物并使用物理急停。
- `hardware.move_to` 已实现并通过离线与实物只读预览，首次 XYZ 实物移动待用户验证。`move_linear`、连续轨迹和仿真到实物协同仍未实现。

离线验证包含故障、样本计数停滞、未使能、越界、多关节目标、全七轴目标填充和零位偏差保持，完整测试可用 `bash scripts/with_dynamics.sh -m unittest discover -s tests -v` 复现。实际硬件证据仅覆盖本页记录的首次小幅 J6 闭环。
