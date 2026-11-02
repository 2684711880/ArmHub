# ArmHub：统一机械臂接口、视觉任务与模仿学习

ArmHub 是一个独立的机器人实验工程，通过 `create_arm(robot, backend)` 区分机械臂型号和执行后端。当前包含 **Panthera-HT 六轴**和 **NERO 七轴单臂**的模型、共享 ROS 2 仿真、Panthera 实物适配、RGB-D 视觉任务，以及官方 Panthera LeRobot / ACT 集成。

| 模块 | 当前能力与边界 | 文档 |
|---|---|---|
| 统一接口 | 六/七轴 Mock、状态、关节运动、轨迹与能力查询；统一 rad / m / s / xyzw | [架构](docs/architecture.md) |
| 共享仿真 | ROS 2 Humble + Gazebo Classic + MoveIt 2；按机型切换模型、规划、执行和反馈 FK | [模型与验证](docs/models.md) |
| Panthera 实物 | 官方 SDK、反馈、关节调试、home、IK 到点、Ruckig 参考与重力补偿；不同功能的实物证据分别记录 | [实物接入](docs/panthera_hardware.md) |
| NERO 实物 | ArmHub 内尚未接入；不继承独立 Nero-7 工程的实物能力 | [能力查询](armhub/robots/nero7.json) |
| 视觉与标定 | RGB-D 洞口检测、跟踪/丢孔回收、手眼标定与绘图窗口；完整任务仍需现场验收 | [视觉任务](docs/hole_approach.md) · [手眼标定](docs/manual_handeye.md) |
| LeRobot / ACT | 官方链路的数据采集、审阅、回放、训练与评估；单独 Python 环境，尚未统一到 ArmBackend | [LeRobot](integrations/panthera_lerobot/README.md) |

## 克隆后先运行 Mock

只需 Python 3.10+ 和 Git；下面的命令不需要 ROS、SDK 或设备：

```bash
git clone https://github.com/2684711880/ArmHub.git ~/ArmHub
cd ~/ArmHub
python3 -m armhub list
python3 -m armhub info --robot panthera_ht --backend mock
python3 -m examples.joint_motion --robot panthera_ht
python3 -m examples.joint_motion --robot nero7
bash scripts/test_offline.sh core
```

Mock 只验证接口和数据，不计算物理。需要安装命令行入口时，可在自己的虚拟环境执行 `python -m pip install -e .`，再运行 `armhub list`。

## 安装与文档导航

完整的仿真、动力学、视觉和 LeRobot 安装顺序见 [环境安装](docs/installation.md)。机械臂与相机使用前，按实际设备重新核对串口、关节限制、相机序列号、手眼矩阵、工具和负载；仓库内配置是这套实验设备的参考值。

- [安装与离线验证](docs/installation.md) · [本次发布验证](docs/publication.md)
- [架构](docs/architecture.md) · [模型来源](docs/models.md) · [第三方来源与许可](THIRD_PARTY_NOTICES.md)
- [Panthera 实物](docs/panthera_hardware.md) · [重力补偿](docs/gravity_compensation.md) · [J6 辨识](docs/j6_identification.md)
- [洞口任务](docs/hole_approach.md) · [moveL](docs/movel.md) · [绘图](docs/drawing_panel.md)
- [LeRobot](integrations/panthera_lerobot/README.md) · [审阅采集](docs/manual_review_recording.md) · [策略执行](docs/policy_full_run.md)

模型、网格和带既有修正的原生 SDK 源码快照随仓库提供；LeRobot 两个上游目录由固定提交和补丁恢复。虚拟环境、构建结果、原始数据集、训练权重及日常运行日志不上传。`validation/` 保留精选历史证据，其中的绝对路径和日期用于追溯，不能作为另一台机器的运行配置。

## 发同一条空间目标命令

手动拖动与 RViz 手眼标定也已接入本工程：另一个终端运行重力补偿后，使用 `bash scripts/run_handeye.sh` 打开采集窗口。相机、FK、RViz 配置和插件均来自 ArmHub 本地，步骤见 [手动手眼标定](docs/manual_handeye.md)。

终端一启动 Panthera，默认 Gazebo 在后台计算物理、RViz 显示机械臂：

```bash
cd ~/ArmHub
bash scripts/launch_sim.sh panthera_ht
```

终端二发送目标：

```bash
cd ~/ArmHub
bash scripts/move_to.sh --robot panthera_ht --xyz 0.25 0.0 0.30
bash scripts/move_to.sh --robot panthera_ht --xyz 0.22 0.08 0.32
```

在终端一 **Ctrl+C，等整套仿真退出**，再切换机型。也可以在任意终端运行 `bash scripts/stop_sim.sh`，它只识别并停止本工程的仿真 launch：

```bash
bash scripts/launch_sim.sh nero7
```

终端二使用同一个程序和同一组 XYZ：

```bash
bash scripts/move_to.sh --robot nero7 --xyz 0.25 0.0 0.30
bash scripts/move_to.sh --robot nero7 --xyz 0.22 0.08 0.32
```

这里 XYZ 单位为米，参考系是各自的 **base_link**；不是世界坐标。Panthera 控制 `gripper_center`，NERO 控制 `gripper_base`，这些是当前模型定义的末端参考点，不代表已标定的实物工具 TCP。只提供 XYZ 时，姿态由规划器选择；若业务需要姿态，追加 `--quaternion QX QY QZ QW`，必须归一化。

此命令是 `move_to`：**规划一条可执行路径到指定点，路径不要求是直线**。`move_linear` 仍明确标记未实现。

可选参数：

```bash
# 同时显示 Gazebo 窗口；默认只显示 RViz
bash scripts/launch_sim.sh nero7 gui:=true
# 无界面验证
bash scripts/launch_sim.sh nero7 rviz:=false gui:=false
# 只读仿真末端状态
bash scripts/move_to.sh --robot nero7 --state
# 降速并保存结果
bash scripts/move_to.sh --robot nero7 --xyz 0.25 0 0.30 --velocity-scale 0.1 --report logs/move.json
```

当前切换方式是重启同一个仿真入口；尚未制作机型下拉选择器。两种模型不会同时出现在同一个场景。重复启动会拒绝占用已有 ArmHub Gazebo 端口。

## 统一 Python 调用

通过 `bash scripts/with_ros.sh python3 ...` 加载本工程环境：

```python
from armhub import create_arm

with create_arm(robot="panthera_ht", backend="simulation") as arm:
    arm.enable()  # 本客户端的软件执行许可，不是实物使能
    result = arm.move_to((0.25, 0.0, 0.30))
    print(result.position_error_m)
```

切换为 `robot="nero7"`，任务代码不变。`move_to` 会等待 MoveIt/Gazebo 执行完成，并读取关节反馈，通过 FK 核对末端位置；超出 5 mm 会报错，不把提交成功当作到位成功。

## 资源与代码位置

```text
ArmHub/
├── armhub/
│   ├── interfaces.py             # 上层统一接口
│   ├── types.py                  # 状态、轨迹、执行结果
│   ├── factory.py                # robot + backend 选择
│   ├── robots/*.json             # 轻量注册信息、能力接入状态
│   ├── backends/
│   │   ├── mock.py               # 软件替身
│   │   └── ros2_simulation.py    # 两种机械臂共用的 MoveIt/仿真后端
│   └── adapters/                 # Panthera SDK 适配；NERO 实物待接入
├── robots/
│   ├── panthera_ht/
│   └── nero7/
│       ├── robot.json            # 规划组、基座、TCP、关节映射
│       ├── description/          # 规划和仿真模型，同源几何
│       ├── planning/             # SRDF、IK、限制、控制器映射
│       └── simulation/           # ros2_control 控制器
├── vendor/                       # 独立复制的官方模型/网格及参考配置
│   ├── panthera/
│   └── agilex/
├── deps/models.lock.json         # 来源提交、文件 SHA256
├── ros_ws/src/armhub_sim/        # 共享 launch、RViz、物理世界
├── scripts/                     # 构建、启动、命令、验收
├── examples/move_to.py           # 同一个空间目标示例
├── integrations/panthera_lerobot/ # 官方 LeRobot、配置、固定版本和补丁
├── apps/                        # 视觉、绘图与示教窗口
├── experiments/                 # 运动、辨识和标定实验
├── tests/                       # 离线契约、控制逻辑与集成测试
└── validation/                  # 分机型的实际仿真验收结果
```

Panthera 官方 SDK 及旧工程已有的生命周期/反馈修正已独立迁入 `vendor/panthera/sdk/` 并重新编译；包装代码在 `armhub/adapters/panthera_ht.py`。NERO 实物仍拒绝连接。均不沿用旧工程的自动 home 动作。

## Panthera 实物入口

已完成七电机只读反馈与首次小幅 J6 实物运动及保持停止确认。重力补偿已由用户实物测试通过。实物空间到点已接入官方 IK/SDK，并通过离线与实物只读预览；首次 XYZ 实物移动待用户核对，完整轨迹尚未验收。

```bash
cd ~/ArmHub
bash scripts/hardware.sh state
# 只预览本次当前位置 +2°，不发送运动命令
bash scripts/hardware.sh jog --joint 6 --degrees 2
# --execute 才执行；每次都是基于当次当前位置，重复运行会累积位移
bash scripts/hardware.sh jog --joint 6 --degrees 2 --duration 2 --execute
# 重力补偿持续到 Ctrl+C，然后切回当前位置保持
bash scripts/hardware.sh gravity --robot panthera_ht --execute
```

实物空间目标也使用同一入口，默认只预览，添加 `--execute` 才执行。先从抬高 1 cm 开始：

```bash
bash scripts/move_to.sh --robot panthera_ht --backend hardware --delta 0 0 0.01
bash scripts/move_to.sh --robot panthera_ht --backend hardware --delta 0 0 0.01 --execute
```

实物默认保持当前末端姿态；命令、函数与验证边界见 [实物 XYZ 到点](docs/panthera_xyz.md)。

退出后固件可保留最后目标，不等同于电机失能。完整边界、配置和实测结果见 [Panthera 实物接入](docs/panthera_hardware.md)。重编译 SDK 使用 `bash scripts/build_hardware.sh`。

回到参考工程定义的六轴零姿态、夹爪保持：

```bash
bash scripts/hardware.sh home            # 只预览
bash scripts/hardware.sh home --execute  # 执行并等待到位
```

统一函数为 `arm.home()`，不重新标定编码器零点，见 [回 home](docs/panthera_home.md)。`home` 与实物 `move_to` 默认使用 Ruckig 连续参考，见 [平滑执行](docs/panthera_smoothing.md)。

重力补偿的实现、依赖、操作与证据边界见 [统一重力补偿命令](docs/gravity_compensation.md)。

J6 连杆上的相机负载可用 `bash scripts/collect_j6_gravity.sh` 只读预览，追加 `--execute` 后以位置保持采集静态实测力矩；默认 J3 偏移 0/+10/+20°、J6 双向扫描 ±30°（两端另有 3°预行程），不自动应用辨识结果。数据与操作说明见 [J6 静态辨识采集](docs/j6_identification.md)。

## 构建与验证

先按 [环境安装](docs/installation.md) 完成所需模块的依赖。修改源模型、规划/控制配置生成器或 launch 后：

```bash
cd ~/ArmHub
bash scripts/build_sim.sh
bash scripts/test_offline.sh core
# 完整回归需要已安装 LeRobot 环境和重编译 SDK
bash scripts/test_offline.sh full
```

生成器 `scripts/prepare_sim_models.py` 产生 `robots/*` 下模型与配置，**重复构建会重新生成这些文件**。调整安装场景、初始状态、IK 或限制生成规则时，应编辑生成器；修改 launch/RViz 文件则在 `ros_ws/src/armhub_sim/` 中编辑。

`vendor/` 的模型、SDK 源码及编译依赖快照已纳入 Git，`deps/models.lock.json` 和 `deps/panthera_sdk.lock.json` 记录来源与哈希。干净克隆无需旧 Panthera-HT / Nero-7 工程；`import_models.py` 和 `import_panthera_sdk.py` 仅保留为历史导入工具，不属于首次安装步骤。生成的 Python SDK 扩展不上传，必须在本机重新编译。

启动对应机型后执行完整仿真验收：

```bash
bash scripts/with_ros.sh python3 scripts/validate_sim.py --robot panthera_ht
# 切换模型后
bash scripts/with_ros.sh python3 scripts/validate_sim.py --robot nero7
```

验收包括两个相同 XYZ 目标、不可达目标拒绝、碰撞场景拒绝和移除障碍后恢复；JSON 保存于 `validation/<机型>/acceptance.json`。

软件环境固定使用 ROS 域 88、localhost DDS 与 Gazebo 端口 11488，与参考工程隔离。关节直接轨迹接口 `execute_trajectory` 检查模型限位并发送控制器，**它本身不做 MoveIt 碰撞规划**；需要规划到空间点时使用 `move_to`。

来源和适配说明见 [模型来源](docs/models.md)，总体边界见 [架构](docs/architecture.md)。仿真验收与 [Panthera 首次实物验收](docs/panthera_hardware.md) 分别记录；两者都不证明接触动力学或工具标定。

## 摄像头与洞口任务

相机、深度识别和目标滤波位于 `perception/`，界面与任务状态机位于 `apps/hole_approach/`，手眼标定位于 `calibration/panthera_ht/wrist_camera/`。电机侧使用一个 ArmHub 会话，与 `home` / XYZ 共用连续参考执行器。

```bash
bash scripts/hole_approach.sh --camera-only # 只看相机与检测，不连接机械臂
bash scripts/hole_approach.sh               # 相机与机械臂只读预览；p 规划
bash scripts/hole_approach.sh --execute     # 允许按键 g/t/r/c/o；启动不自动运动
bash scripts/hole_approach.sh --dry-run     # 合成场景离线跑两段，无设备
```

`g` 锁定点击时目标；`t` 跟踪；空格保持；`r` 回位；`c/o` 夹爪；`q` 退出。当前默认 20 cm 对准间距、18 cm 连续前送、4 cm 杆尖。导入的是当前参考工程的手眼求解结果（待独立实测验证），默认不再使用旧的左右 50 mm 补偿。完整用法与验证边界见 [洞口应用](docs/hole_approach.md)。

Panthera 实物速度上限已配置为六轴 `[1.68, 2.0, 2.0, 1.68, 1.48, 1.48]` rad/s，视觉丢孔回收为 0.6 rad/s。平滑执行保留，实物 home／XYZ 默认速度比例为 1.0，可用 `--velocity-scale` 降速。配置修改后退出并重启程序，无需重新编译。

## moveL 轨迹与误差分析

上传的平面轨迹实验已接入 ArmHub：`bash scripts/movel.sh` 默认只读预检，`--execute` 才执行。支持直线、对角线、正方形、螺旋和圆的正反向记录与出图。先从 20 mm 直线开始；参数、自动引入/回锚点动作和数据定义见 [moveL 实验](docs/movel.md)。

## 夹笔与绘图窗口

```bash
bash scripts/drawing_panel.sh
```

窗口集成 Home、可编辑绘图准备位、开合夹爪、基座 Z 升降 1 cm、直线/方形/圆/螺旋正反向、速度设置、实时编码器 FK 轨迹与自动数据/图片导出。启动不运动，先只读连接，再显式允许执行；用“记为绘图起点”确定落笔位置。详细流程和证据边界见 [绘图窗口](docs/drawing_panel.md)。

## Panthera LeRobot 数据采集与模仿学习

官方 LeRobot 工程和配套 SDK 已加入 `integrations/panthera_lerobot/`，保留官方控制链，使用独立 `.venv_lerobot`；原生 SDK 复用本工程生命周期修正版。采集、数据回放、ACT 训练和策略评估入口统一为 `bash scripts/lerobot.sh`。启动不自动移动到固定准备位，退出不自动回零。

```bash
bash scripts/setup_lerobot.sh   # 首次安装 / 恢复固定版本上游源码
bash scripts/lerobot.sh check  # 离线检查，不打开设备
```

设备、相机配置、实物执行命令与验证边界见 [LeRobot 接入](integrations/panthera_lerobot/README.md)。

### 桌面抓取示教窗口

```bash
bash scripts/teaching_panel.sh          # 启动不连接设备
bash scripts/teaching_panel.sh --demo   # 合成图像与反馈，不访问设备
```

窗口支持拖动后保存固定准备位、显式预览/返回准备位、夹爪 O/C 开合、逐回合录制、图像与曲线回看，以及成功回合导出 LeRobot 视频数据集。流程与验证边界见 [桌面抓取示教窗口](docs/teaching_panel.md)。
