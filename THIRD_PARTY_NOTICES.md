# 第三方来源与许可

ArmHub 含第三方资源快照及本地适配。各目录中的原始许可和版权声明继续适用；本文件不以统一许可证替代它们。

| 资源 | 上游或来源 | 固定版本 / 记录 | 许可文件 |
|---|---|---|---|
| Panthera 模型、网格、MoveIt 参考 | [HighTorque-Robotics/Panthera-HT_ROS2](https://github.com/HighTorque-Robotics/Panthera-HT_ROS2) | `afcdc2148ca36f44e94e870dc8ad351c5e217360`，`deps/models.lock.json` | `vendor/panthera/LICENSE`，MIT |
| NERO 模型、网格、MoveIt 参考 | [agilexrobotics/agx_arm_sim](https://github.com/agilexrobotics/agx_arm_sim) | `f8cd8b147c75d59e14f90fb0646770eefa268ed0`，`deps/models.lock.json` | `vendor/agilex/licenses/0_LICENSE`，沿用来源工作树保留的 MIT 文件 |
| Panthera 原生 SDK 与绑定 | 本地 Panthera-HT 参考工程中已修正的官方 SDK | `deps/panthera_sdk.lock.json`；绑定/库元数据 4.4.7，启动横幅 4.6.0 | `vendor/panthera/sdk/native/LICENSE`，Apache-2.0 声明 |
| LCM | 原生 SDK 携带的源码 | SDK 锁定文件逐文件记录 | `vendor/panthera/sdk/native/third_party/lcm/COPYING` 及各子目录许可 |
| serial_cmake | 原生 SDK 携带的源码 | 同上 | 保留源文件中的许可声明 |
| pybind11 头文件 / CMake | 原参考工程安装包快照 | 2.13.6；SDK 锁定文件记录头文件哈希 | `vendor/panthera/sdk/build_deps/pybind11/LICENSE`，BSD |
| libserialport 头文件与库依赖快照 | 原参考工程的 Debian/Ubuntu 包解包结果 | SDK 锁定文件记录哈希，`deps/usr/` | `vendor/panthera/sdk/deps/usr/share/doc/libserialport-dev/copyright` 和 `libserialport0/copyright`，LGPL-3.0+ |
| moveit_calibration / moveit_visual_tools | [MoveIt](https://github.com/moveit) | `calibration_ws/sources.json`，各目录 `imported.patch` | 各源码目录 `LICENSE`，BSD |
| Panthera LeRobot 与配套 SDK | [Panthera-HT_lerobot](https://github.com/HighTorque-Robotics/Panthera-HT_lerobot)、[Panthera-HT_SDK](https://github.com/HighTorque-Robotics/Panthera-HT_SDK) | `integrations/panthera_lerobot/sources.lock.json` | 按固定提交下载时保留上游许可证；上游目录不重复提交 |

NERO 的来源工作树带有 description CMakeLists 的历史修改；本工程模型导入未使用该 CMakeLists。许可文件如实按来源保存，不能将其中的作者字段推断为全部厂商模型的版权归属。

原生 SDK 保留既有被动连接/退出、反馈快照加锁及绑定修正，`reference_commission.patch` 保存参考补丁。它不是未经修改的上游发行包，也不是同名 SDK 最新版本；更新后需重新验证。

视觉任务和状态机来自用户的 Panthera-HT 工程，迁移记录在 `deps/hole_approach_sources.json`；moveL 轨迹与分析来自用户提供文件，记录在 `deps/movel_sources.json`。锁定文件中的旧本地路径仅用于追溯。
