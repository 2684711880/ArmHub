# Panthera LeRobot 接入

当前发布的安装与测试入口见 [环境安装](../../docs/installation.md)，历史采集/训练与本次离线验证分开记录于 [发布验证](../../docs/publication.md)。下文的“本次接入”描述最初接入阶段的验证范围。

此目录把官方 Panthera LeRobot 示例接入 ArmHub 工程管理。采集、拖动示教、主从跟随和策略下发仍走官方 `Panthera` Python 类及其 SDK 函数，尚未转换成 `ArmBackend`。当前集成入口支持单从臂、可选单主臂；双臂代码仅保留作上游参考。

## 目录与来源

- `Panthera-HT_lerobot/`：官方 LeRobot 0.4.3 源码和 Panthera 插件。
- `Panthera-HT_SDK/`：官方 SDK，和上面的仓库同级。插件按官方相对目录导入 Python 包及模型。
- 新环境使用 NumPy 2.2.6 / Pinocchio 3.8.0，与 ArmHub 原有动力学环境分别安装。
- 原生 `hightorque_robot`：复用 `ArmHub/vendor/panthera/sdk/python/` 已编译的生命周期/反馈快照修正版，不安装官方二进制 wheel，也不使用旧工程的绝对路径。
- `sources.lock.json`：两个官方仓库的 URL、提交，以及补丁路径。第三方目录按现有 ArmHub vendor 习惯忽略 Git；干净机器运行 `prepare_sources.py` 下载固定提交并应用补丁，SDK 编译仍按 ArmHub 的 `build_hardware.sh` 流程完成。
- `patches/armhub-lifecycle.patch`：关闭主从臂固定准备位/自动回零，移除线程退出时的零力矩清理，退出交给本集成的保持流程；数据回放异常也进入退出处理；相机默认配置为空，帮助与解析不再自动枚举设备。保留官方函数名和主体控制算法。
- `configs/local.json`：设备与采集设置；`.runtime/`：自动生成的 SDK 参数文件。

## 环境与无设备检查

在 `/home/a/ArmHub` 运行：

```bash
bash scripts/setup_lerobot.sh
bash scripts/lerobot.sh check
bash scripts/lerobot.sh --help
```

独立 `.venv_lerobot` 使用 Python 3.10，初始安装 CPU 版 PyTorch 2.7.1，便于先验证流程；训练加速需要另外换装与本机 CUDA 匹配的 GPU 版。检查只导入库、验证插件注册与配置路径，不构造 SDK Robot，不打开机械臂或相机。

## 设备与相机配置

编辑 `integrations/panthera_lerobot/configs/local.json`：

- `follower_device` 默认 `/dev/ttyACM0`，执行前核对设备身份。
- `leader_device` 默认 `null`；使用主从模式前填写另一块控制板的实际设备路径，不能与从臂相同。
- `cameras` 已配置 `d435`：按完整设备名 `Intel RealSense D435` 匹配唯一相机，RGB 640×480 / 30 FPS，暂不录制深度。相机未连接时会明确报错；若同时连接多台同型号相机，须将 `serial_number_or_name` 改为目标相机的实际序列号。采集循环仍为 20 FPS。
- 此配置已通过离线解析；当前 RealSense 枚举为空，尚未验证实物图像。

枚举相机（只访问相机，不连接机械臂）：

```bash
bash scripts/lerobot.sh cameras
```

相机配置示例，替换为枚举到的实际序列号：

```json
"cameras": {
  "wrist": {
    "type": "intelrealsense",
    "serial_number_or_name": "实际序列号",
    "width": 640,
    "height": 480,
    "fps": 30,
    "use_depth": false
  }
}
```

## 采集、数据回放、训练、策略评估

下面的 `record`、`replay`、`evaluate` 会发送实物命令，必须显式加 `--execute`。启动前退出 ArmHub 其他实物控制器。串口检查与 advisory lock 和现有 ArmHub 共用，拒绝本集成的第二个设备连接。

```bash
# 单臂拖动示教；默认 1 回合、15 秒、20 FPS
bash scripts/lerobot.sh record --repo-id local/panthera_demo --execute
# 两台机械臂：先配置 leader_device，再加 --leader
bash scripts/lerobot.sh record --repo-id local/panthera_leader_demo --leader --execute
# 回放已经采集的数据，第 0 回合
bash scripts/lerobot.sh replay --repo-id local/panthera_demo --episode 0 --execute
# ACT 训练：数据必须已经存在；-- 后可以追加官方训练参数
bash scripts/lerobot.sh train --repo-id local/panthera_demo -- --steps=100
# 策略执行并记录评估数据
bash scripts/lerobot.sh evaluate --repo-id local/eval_panthera_act \
  --checkpoint /绝对路径/pretrained_model --execute
```

本地数据存到 `datasets/lerobot/<repo-id>/`，训练输出存到 `outputs/lerobot/panthera_act/`，两者排除 Git。重复采集换一个 repo-id，或按官方参数明确选择恢复采集；不自动覆盖数据。

## 行为与验收边界

连接之后会进入官方后台保持/补偿循环；单臂采集会切换到拖动示教模式，并非只读连接。保留官方位置+重力前馈控制与原有增益，和 ArmHub 的 Ruckig 到点模式不同。官方单臂拖动代码对夹爪使用固定 `1.6` 的目标值，后续实物验收必须核对夹爪行为和负载；本次接入未宣称这些参数已适配实物。

自动准备位、回合固定复位与自动回零已关闭，用户在采集前自行确认起始姿态。退出先停止唯一后台发送线程，再基于七电机新反馈请求当前位置保持并核对稳定，然后释放相机和串口；失败会报错。保持请求不代表电机失能或物理急停，进程结束后固件可能继续保留末次目标。

本次验证范围是依赖导入、命令参数、插件注册、SDK 路径和离线生命周期测试；未连接机械臂、未录制数据、未训练或执行策略。第一次实物回放前必须核对记录起点与当前姿态，并先做小范围示教，原始 LeRobot 回放不提供 ArmHub 的整段轨迹碰撞检查。

## 桌面抓取示教窗口

启动 `bash scripts/teaching_panel.sh`，或使用 `--demo` 无设备体验。准备位、夹爪 O/C 开合、原始回合审阅和成功回合导出流程见 [窗口说明](../../docs/teaching_panel.md)。本地补丁还统一了数据集 create/load 的 root 路径，避免重复拼接 repo-id。

官方单臂录一轮、返回已保存准备位与离线图像/曲线播放，见 [试录说明](../../docs/official_single_trial.md)。新增 `prepare` 默认只读预览，`prepare --execute` 才移动；`view --repo-id ... --episode 0` 不连接机械臂。

实机回放追加 `--gripper-log` 可记录夹爪目标、实际开度、速度与 SDK 力矩反馈，不改变运动指令。日志位于 `logs/lerobot/gripper/`，用于标定接触与保持策略，见 [夹爪诊断说明](../../docs/gripper_contact_plan.md)。

本轮蓝块位置触发加力试验可用 `replay ... --gripper-assist 0.1`，保留官方位置控制，在接近夹持开度后叠加小幅闭合前馈，张开时撤销，并自动记录日志；见 [位置加力说明](../../docs/gripper_position_assist.md)。默认回放行为不变。

50 条成功数据可用 `record --review --repo-id local/blue_to_box_clean_01 --execute`：首次回车准备；结束后 s 保留并回准备位、r 失败重录并回准备位、q 丢弃本轮退出。到位后等 n 开始，自动续采并显示当前条数。终端字母输入由独立线程读取，通过分阶段队列交给主流程。详见 [人工审阅采集](../../docs/manual_review_recording.md)。
