# 环境安装与离线验证

## 支持范围

Mock 与模型检查使用 Python 3.10+，不需要 ROS 或设备。仿真、Panthera 原生 SDK 和手眼插件的当前构建目标为 **Ubuntu 22.04 x86_64、系统 Python 3.10、ROS 2 Humble、Gazebo Classic 11、MoveIt 2**。脚本会加载 `/opt/ros/humble/setup.bash`；其他操作系统、Python ABI 和 ROS 发行版尚未验证。

本文命令在克隆目录执行。历史文档、来源锁定和验证日志可能保留原工作站 `/home/a/ArmHub` 路径；换机器时使用实际克隆路径。原始数据集、模型权重和运行日志不随 Git 分发。

## 1. Mock：无需安装第三方依赖

```bash
git clone https://github.com/2684711880/ArmHub.git ~/ArmHub
cd ~/ArmHub
python3 -m armhub list
python3 -m examples.joint_motion --robot panthera_ht
python3 -m examples.joint_motion --robot nero7
bash scripts/test_offline.sh core
```

核心检查覆盖七项接口契约和四项模型检查：关节映射、TCP、网格引用、规划/物理模型几何和来源哈希。Mock 不验证动力学、碰撞效果或实物性能。

## 2. ROS 仿真

先在 Ubuntu 22.04 配好 ROS 2 Humble 软件源和环境；以下是工程依赖，不包含操作系统与 ROS 本身的安装流程：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git python3-dev python3-venv python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  python3-numpy python3-scipy python3-opencv python3-pyqt5 python3-matplotlib \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-moveit ros-humble-xacro ros-humble-rviz2 \
  ros-humble-robot-state-publisher libyaml-cpp-dev libserialport-dev
bash scripts/build_sim.sh
bash scripts/launch_sim.sh panthera_ht
```

另一个终端：

```bash
cd ~/ArmHub
bash scripts/move_to.sh --robot panthera_ht --xyz 0.25 0.0 0.30
```

切换到 NERO 前退出当前仿真，或运行 `bash scripts/stop_sim.sh`，再用 `bash scripts/launch_sim.sh nero7`。仿真使用 ROS 域 88、本机 DDS 和 Gazebo 端口 11488。完整运行验收命令见 [README](../README.md#构建与验证)。

`vendor/` 随 Git 保存必需的模型、网格及来源许可；不需要安装旧 Panthera-HT / Nero-7 工程。`scripts/import_models.py` 是历史导入工具，不是干净机器的安装入口。`build_sim.sh` 会按生成器重新生成模型和配置。

## 3. Panthera SDK、动力学与视觉

```bash
bash scripts/build_hardware.sh
bash scripts/setup_dynamics.sh
# 需要 RGB-D 应用时再安装：
bash scripts/setup_vision.sh
```

原生 SDK 从本仓库的 `vendor/panthera/sdk/native/` 和绑定源码重新编译，不下载预编译控制扩展，不连接设备。仓库中的 libserialport 编译依赖快照仅适用于 Linux x86_64，其来源许可位于 `vendor/panthera/sdk/deps/usr/share/doc/`。

`.venv_dynamics` 使用 NumPy 1.26 / Pinocchio 2.7，并继承系统 OpenCV、Qt 和 Matplotlib。针对动力学/辨识模块运行测试，例如：

```bash
bash scripts/with_dynamics.sh -m unittest discover -s tests -p 'test_j6_collection.py' -v
```

`hardware.sh state` 会访问实物串口；安装和发布验证阶段不运行它。实物操作、设备身份和保持语义见 [实物接入](panthera_hardware.md)。

## 4. LeRobot / ACT 独立环境

先完成 SDK 编译，再运行：

```bash
bash scripts/setup_lerobot.sh
bash scripts/lerobot.sh check
bash scripts/test_offline.sh full
```

`setup_lerobot.sh` 用固定提交下载两份官方源码，核对补丁 SHA256，应用生命周期补丁，再安装 `.venv_lerobot`。下载需要能访问 GitHub 和 Python/PyTorch 包源。源码版本见 `integrations/panthera_lerobot/sources.lock.json`，本地覆盖见 `patches/armhub-lifecycle.patch`。

该环境使用 NumPy 2.2.6 / Pinocchio 3.8，与动力学环境分开；完整测试应通过 `test_offline.sh full` 运行，不要在 NumPy 1 环境运行所有 LeRobot 测试。Matplotlib 3.9.4 用于完整回归中的绘图导出。`check` 只导入依赖、解码配置和检查模型，不打开机械臂或相机。

默认安装 CPU PyTorch 2.7.1；GPU 训练需要另行安装与机器匹配的 PyTorch / torchvision。数据、权重和本地 GPU 环境不会通过本仓库自动恢复，采集与训练步骤见 [LeRobot 接入](../integrations/panthera_lerobot/README.md)。

## 5. 可选 RViz 手眼插件

```bash
# rosdep 已初始化后，按包清单安装额外依赖
rosdep install --from-paths calibration_ws/src --ignore-src -r -y --rosdistro humble
bash scripts/build_handeye.sh
```

插件源码和 Humble 适配补丁在 `calibration_ws/src/`；来源提交记录在 `calibration_ws/sources.json`。相机标识和手眼结果必须按本机安装重新标定，不能直接采用实验快照，见 [手动标定](manual_handeye.md)。

## 发布内容与验证边界

- 提交：工程代码、机器人配置、必要来源快照、测试、文档和精选历史验证结果。
- 排除：虚拟环境、SDK 生成扩展、ROS/CMake 构建和安装目录、上游 LeRobot 下载目录、原始数据、训练权重、日常日志及编辑器状态。
- `ci/github-actions-offline.yml` 提供无需设备的 GitHub Actions 核心检查模板；当前 GitHub 登录缺少 `workflow` 权限，因此本次不启用自动运行。具备对应权限后，将其放到 `.github/workflows/offline.yml` 即可启用；该任务不安装 ROS、连接机械臂或训练模型。
- 完整离线测试证明软件回归通过；现场运动、相机可见性、抓取成功率和标定精度须独立验收。
