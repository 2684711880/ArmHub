# 架构边界与接入顺序

## 两个独立选择

`robot` 选择机械臂，`backend` 选择执行环境：

```text
robot:    panthera_ht / nero7 / 后续机型
backend:  mock / simulation / hardware
```

当前两种机型共用 Mock 和 ROS 2 仿真后端；Panthera 实物适配器已接入状态与小幅单关节运动，NERO 实物仍明确拒绝连接。构造对象不打开设备，不自动使能、回零或开启重力补偿。

## 一条空间命令的完整链

```text
examples/move_to.py
       ↓ create_arm(robot, backend="simulation")
Ros2SimulationBackend.move_to(XYZ)
       ↓ 按 robots/<机型>/robot.json 选规划组、基座、末端
MoveGroup action：位置约束 + 可选姿态约束
       ↓ KDL IK + OMPL 规划 + 时间参数化
MoveIt 轨迹执行管理器
       ↓ FollowJointTrajectory
arm_controller → GazeboSystem → 仿真机械臂
       ↓ joint_states
GetPositionFK → 核对末端误差 → MotionResult
```

六轴/七轴差异由模型和配置表达，共享后端不按品牌重复实现运动逻辑。用户发送相同 XYZ 时，两种机械臂可以采用不同的姿态和关节路径。

## 各层职责

| 层 | 文件位置 | 职责 |
|---|---|---|
| 任务 | `examples/`、未来应用目录 | 组合运动目标，不依赖 SDK 类型 |
| 统一接口 | `armhub/interfaces.py`、`types.py` | 单位、命令、反馈和结果语义 |
| 注册入口 | `armhub/factory.py`、`armhub/robots/` | 选择机型与后端，展示接入状态 |
| 机器人资源 | `robots/<机型>/` | 模型、SRDF、限制、IK、控制器和运行映射 |
| 共享仿真后端 | `armhub/backends/ros2_simulation.py` | MoveIt/轨迹 action、仿真反馈与到位检查 |
| 厂商实物适配器 | `armhub/adapters/` | 将统一接口转为官方 SDK 命令、反馈、停止和故障 |
| 官方资源 | `vendor/` | 第三方模型与 Panthera 官方 SDK 来源快照 |

目前使用普通注册表。接入第二、第三台后再决定是否需要插件自动发现，不提前做复杂调度和插件系统。

## 已实现的契约

- 关节 rad、位置 m、时间 s、四元数 xyzw。
- 关节数组严格按 `profile.joint_names` 排列。
- `move_to` 的位置参考各机型 `base_link`，TCP 由机器人配置选定。仿真未指定姿态时允许规划器选择姿态；Panthera 实物默认保持当前姿态。
- `move_to` 不是直线运动；`move_linear` 明确拒绝执行，不做静默替换。
- Mock 返回仅表示虚拟状态已更新，不计算物理或按时间播放。
- 仿真 `connect()` 检查运行模型名与 GazeboSystem，等待控制器与服务就绪；`get_state()` 等待完整新消息，检查仿真时间戳。
- 仿真 `enable()` 只是本客户端的软件执行许可。
- 运动同步等待终态；MoveIt 或控制器失败就报错。空间运动成功后根据关节反馈做 FK，检查目标位置误差不超过 5 mm。
- 直接轨迹接口检查数量、有限数、模型位置限制和时间递增，核对终点关节误差；它不执行碰撞规划。需要避障规划的空间目标使用 `move_to`。
- 仿真 `stop()` 取消当前客户端持有的 action 并等待终态；Panthera 实物 `stop()` 发送当前实测姿态保持并核对反馈。均不是物理急停。尚未提供跨线程并发调用或外部停止命令服务。

重力补偿通过可选 `run_gravity_compensation()` 统一；Panthera 已实现官方重力模型与原生力矩接口循环，待实物测试，NERO 明确不支持。拖动示教、通用力矩控制仍逐台核对能力后再接，不把“SDK 初始化”当成这些能力的等价操作。

## 仿真描述的唯一发布者

物理模型含 GazeboSystem 插件，规划模型不含物理插件。两者共用几何，但不能竞争同一个 `robot_description` 发布源。

`robot_state_publisher` 是物理描述的唯一发布者；`move_group` 配置 `publish_robot_description=false`，自己通过参数读取规划描述。Gazebo 从物理描述生成模型。此次导入验证发现并修复了两者同时发布造成的启动竞争，验收必须包含控制器激活与真实反馈，不能只看模型生成成功。

## 已完成与后续

1. Mock 骨架：已完成，统一接口与六/七轴示例可运行。
2. 独立模型与共享 MoveIt 仿真：已实现；运行验收证据按机型保存到 `validation/`，包含成功与拒绝场景。
3. Panthera 实物接入：已审计并独立编译既有 SDK，七电机新反馈、软件执行许可、小幅 J6 运动与保持停止通过首次验证。下发 +2°、按当前容差测得停止后约 +1.8°，详细证据见 `docs/panthera_hardware.md`。空间目标已接入官方 IK/SDK，通过离线及实物只读预览，首次实物 XYZ 移动待核对；完整轨迹仍未开放。
4. NERO 实物接入：官方 SDK/ROS 已有参考，实际表现未知，保留未验证状态。
5. 仿真到实物：两端独立验证后增加协调层，检查型号、关节映射、初始实测状态、工具及版本；先仿真预演，再明确触发实物执行。不将任意 joint_states 订阅直接变为实物驱动。

官方目标点 API 与按相同位置/时间点执行轨迹是两种不同能力。实物适配器须说明是否保持完整轨迹语义，不能因统一函数名就宣称仿真实物一致。

## 新机械臂接入动作

新增官方资源与版本记录 → 新增 `robots/<机型>/` 模型/配置 → 新增轻量注册信息 → 共享仿真后端验收 → 实现并注册官方 SDK 适配器 → 独立实物验收。

第三台优先复用共享 launch 和 MoveIt 后端；厂商差异留在资源配置和实物适配器。若模型需要额外物理插件，先扩展配置入口，不复制整套应用程序。

Panthera 实物 `move_to` / `home` 现共用 Ruckig 平滑参考执行器（`armhub/motion.py`），约每 50 ms 通过官方 SDK 下发，并核对跟踪与到位反馈；不等同于 MoveIt 实物规划或外部任意轨迹接口。见 `docs/panthera_smoothing.md`。

视觉上层已落在 `perception/`、`apps/hole_approach/` 与 `calibration/`。应用通过 `armhub/task_runtime.py` 持有一个后端；相机、IK、UI 与电机拥有者独立。`start_reference_trajectory` / `advance_reference_trajectory` 支持任务逐 tick 推进；普通阻塞 `home` / `move_to` 同样使用 `armhub/execution.py`，没有第二套串口发送器。当前完整视觉任务只接入 Panthera 实物，默认只读，详见 `docs/hole_approach.md`。
