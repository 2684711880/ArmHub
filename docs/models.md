# 模型与 MoveIt 导入记录

## 来源

- Panthera：HighTorque-Robotics/Panthera-HT_ROS2，提交 `afcdc2148ca36f44e94e870dc8ad351c5e217360`，来自已存在的本地参考工程。复制官方 description 的 xacro/网格与 MoveIt 参考配置，保留 `vendor/panthera/LICENSE`。
- NERO：agilexrobotics/agx_arm_sim，提交 `f8cd8b147c75d59e14f90fb0646770eefa268ed0`。使用官方七轴单臂 `nero_gripper_d435.urdf` 和 nero 网格、单臂 SRDF。保留 `vendor/agilex/licenses/0_LICENSE`。来源工作树中 description 的 CMakeLists 存在旧改动，本工程没有复制或构建该 CMakeLists；完整来源状态与资源哈希见 `deps/models.lock.json`。

模型导入阶段没有涉及实物控制。后续 Panthera 官方 SDK 已独立迁入并做首次小幅实物验收，见 `panthera_hardware.md`；旧工程的其他实物证据不转移为本工程验收。

## 本工程适配

生成规则在 `scripts/prepare_sim_models.py`，源快照不直接修改：

1. Panthera 从官方 xacro 展开；NERO 从官方已展开模型去除相机及其固定子树。保留两台机械臂与夹爪。
2. 两种机器人独立固定在 world 高度 0.75 m；规划模型含与 Gazebo 地面一致的 z=0 碰撞地板。空间命令相对各自 base_link，因此不需要在命令中加 0.75 m。
3. 网格引用统一到 ArmHub 自有包；启动时将 URI 解析为本工程安装目录，Gazebo 不依赖旧工程的资源路径。
4. 规划与仿真使用同源的连杆、关节、惯量、视觉和网格碰撞几何。仿真模型额外添加 GazeboSystem、位置控制接口与控制器插件。
5. 两种机械臂统一规划组名为 `arm`。Panthera TCP 为官方 `gripper_center`；NERO 为官方 `gripper_base`。NERO 关节命名改为官方单臂 `joint1…joint7`，不再沿用骨架阶段的旧左臂前缀。
6. 同臂碰撞排除继承各自官方 SRDF 中仍存在的连杆对；该配置是仿真起点，尚未进行实物碰撞边界测量。
7. KDL IK、OMPL RRTConnect 与统一轨迹控制器映射由项目配置生成。真实几何关节位置/速度限制来自模型；加速度上限与 0.15 速度/加速度缩放为本工程仿真参数，不代表厂商硬件能力保证。

## 验证内容与限制

- 离线检查全部网格本地可解析、关节映射及 TCP 存在、规划/物理几何一致、来源快照哈希一致。
- 运行时校验当前 robot_description 的机器人名与 GazeboSystem 插件，禁止以另一台机型配置向当前机器人执行命令。
- `move_to` 使用 MoveGroup action 做规划与执行，成功后基于 Gazebo joint_states 做 FK，检查末端到点误差不超过 5 mm。FK 使用相同模型，因此是关节反馈下的运动学结果，不是独立视觉或运动捕捉测量。
- 两个通用目标为 `(0.25, 0, 0.30)` 与 `(0.22, 0.08, 0.32)` m；验收还包含不可达目标、碰撞起点拒绝、移除障碍后恢复。只给位置时末端姿态不固定。
- `move_linear`、实物同步、力控与拖动示教尚未实现；Panthera 实物 SDK 当前仅完成状态与小幅单关节接入。
- 初次切换测试观察到当前安装 MoveIt 在关闭析构阶段段错误，Gazebo 和其他进程仍已退出；新机型可以启动。运行期间规划执行通过与关闭稳定性分开记录，退出问题尚未修复。

## 后续 SDK 接入位置

官方 SDK 源码放 `vendor/<厂商>/sdk/`，版本登记在独立 SDK 锁定文件；本工程转换与反馈逻辑放 `armhub/adapters/<机型>.py`。模型快照与 SDK 可以分别更新，但升级后必须重新核对关节顺序、限位、零位、TCP 和反馈语义。
