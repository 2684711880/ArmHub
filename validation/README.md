# 仿真验收记录

2026-10-19，在独立 ArmHub 工程中运行，未连接或控制实物。

| 机型 | 成功运动次数 | 成功动作最大末端终点误差 | 不可达拒绝 | 碰撞拒绝及移除后恢复 |
|---|---:|---:|---|---|
| Panthera-HT 六轴 | 3 | 0.989 mm | 通过 | 通过 |
| NERO 七轴单臂 | 3 | 1.004 mm | 通过 | 通过 |

两种机型都通过同一个 `arm.move_to()` 执行 `(0.25, 0, 0.30)`、`(0.22, 0.08, 0.32)`，以及清除障碍后的第一个目标。坐标单位 m、相对各自 base_link。姿态不固定，轨迹未约束为直线。

误差由 Gazebo 关节反馈经同源模型 FK 计算，不是独立传感器测量；规划成功之外也检查了执行结果和终点反馈。

- [Panthera 完整验收](panthera_ht/acceptance.json)
- [NERO 完整验收](nero7/acceptance.json)
- [NERO 运行时错误机型连接拒绝](nero7/identity_rejection.json)
- [Panthera 修正唯一描述发布源后、带 RViz 重启的运动复验](panthera_ht/final_launch_smoke.json)：误差 0.504 mm。
- [汇总](summary.json)
- [11 项离线测试](offline-tests.log)

控制器和规划器运行日志分别保存于各机型的 `launch.log`。当前安装 MoveIt 在关闭阶段曾发生段错误；Gazebo 与其余进程退出后可切换机型。该问题尚未修复，不将运动通过表述为退出稳定性通过。
