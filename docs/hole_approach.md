# 摄像头与洞口任务

昨天的视觉应用已迁入 ArmHub。相机、深度洞口检测、目标滤波、坐标叠加和任务状态机复用参考实现；电机侧改为一个 ArmHub 会话，应用不再创建 `ProjectPanthera` 或直接访问串口。参考工程保持原样，来源与原始 SHA256 记录在 `deps/hole_approach_sources.json`，运行不依赖原工程目录。

## 启动

当前环境的 OpenCV、RealSense、Pinocchio 与 Ruckig 已可用，新环境安装使用 `bash scripts/setup_vision.sh`；环境继承系统的 `python3-opencv` / `python3-yaml`。

```bash
cd /home/a/ArmHub
# 只枚举相机，不打开机械臂
bash scripts/hole_approach.sh --probe
# 相机画面与深度洞口检测，不连接机械臂；q 退出
bash scripts/hole_approach.sh --camera-only
# 相机 + 实测机械臂位姿 + 界面，默认禁止运动，可按 p 预览
bash scripts/hole_approach.sh
# 明确允许交互控制；启动时不自动 home、运动或开合夹爪
bash scripts/hole_approach.sh --execute
```

先退出旧工程控制器、重力补偿和其他相机占用程序。机械臂 USB 路径仍用 `ARMHUB_PANTHERA_DEVICE` 选择；指定相机用 `--camera-serial SERIAL`。`--camera-only --no-gui` 读取一帧、打印检测结果后退出。交互界面可调整窗口大小；`--no-gui` 可用终端键入命令并回车，空格保持也可输入 `s`。

| 按键 | 功能 |
|---|---|
| `p` | 三帧确认与静止反馈后预览两段目标，不运动 |
| `g` | 锁定点击时可靠观测，执行对准和连续前送 |
| `t` | 开关目标跟踪，短时丢孔预测，预测失效后回 home；已开始的前送先完成 |
| 空格 | 取消旧规划／轨迹并请求实测位置保持 |
| `r` | 六轴回配置中的 home，夹爪保持 |
| `c` / `o` | 闭合／张开夹爪；只表示原始位置目标，不保证夹稳 |
| `f` | 相机／基座坐标显示切换，本地显示操作，不发运动命令 |
| `u` / `a` | 打开工具 TCP／板面偏移调节，再 APPLY；取消旧规划，修改仅在本次会话生效 |
| `q` | 结束；运动中尝试保持，不自动回位或断使能 |

默认只读时 `g/t/r/c/o` 会明确拒绝；使用 `--execute` 启动才允许这些动作。可选 `--execute --execute-once` 在三帧可靠观测后自动执行一次，不是默认启动行为。`--plan-only` 自动预览后退出。

## 文件与配置位置

```text
perception/cameras/realsense.py             相机、深度单位、对齐、内参、帧时间
perception/detectors/depth_hole.py          仅深度的板面／洞口几何识别
perception/tracking/hole_estimator.py       目标滤波、短时预测和离群值拒绝
apps/hole_approach/app.py                   UI、队列、四进程与生命周期
apps/hole_approach/controller.py            GO／跟踪／前送／回位状态机
apps/hole_approach/planning.py              对准、前送几何与路径检查
apps/hole_approach/config.yaml              任务与相机参数
calibration/panthera_ht/wrist_camera/       手眼矩阵、标定状态、原始求解证据
robots/panthera_ht/tools/rod.json           杆尖相对夹爪的 TCP 配置
armhub/tools.py                            工具坐标变换
armhub/trajectory.py                       可重规划 Ruckig、连续前送曲线
armhub/task_runtime.py                     任务使用的 ArmHub 会话与离线运动学
armhub/execution.py                        共用非阻塞参考执行器
```

物理限位、速度、加速度、jerk 与力矩限制只从 Panthera 的 `hardware/config.json` 读取；任务配置不复制一套更高的电机上限。六轴参考速度上限已恢复为昨天的 `[1.68, 2.0, 2.0, 1.68, 1.48, 1.48] rad/s`，丢孔回收速度恢复为 `0.6 rad/s`；加速度、jerk 和下发周期保持不变。

任务默认沿用对准距孔 0.20 m、前送 0.18 m、最终距孔平面 0.02 m。工具默认在 `gripper_center` 的 +X 方向增加 0.04 m。可用 `--align-clearance`、`--feed-length`、`--rod-length` 覆盖；固定修改应写入任务／工具配置文件。

相机外参仍相对 `gripper_center`，不会因为加了杆尖 TCP 再叠加一次工具长度。相机与工具分别配置。当前手眼矩阵来自原工程的 15 组 TSAI 求解，状态保留为 **`calibrated_pending_independent_measurement`**，并复制了求解 JSON 和独立角点试验记录。不是已独立验证精度的标定。当前参考工程已经移除旧的左右各 50 mm 经验补偿，本次默认补偿为零。相机换装或安装关系改变不能直接复用旧外参。

## 执行与状态

```text
相机进程 → 深度检测 → 最新观测队列
                             ↓
UI → 带代次的用户意图 → 任务／电机拥有者 ↔ 独立 IK 进程
                             ↓
                    ArmTaskRuntime
                             ↓
       ArmHub start_reference_trajectory / advance_reference_trajectory
                             ↓
              共用 ReferenceExecution → Panthera 官方 SDK
```

界面不等待相机、IK 或到位。规划结果必须匹配任务代次；HOLD、新指令、TCP APPLY 会使旧结果失效，发送前还会再次核对代次。GO 按点击时的可靠观测执行，不因洞口随后移动而改目标。跟踪和 GO 分开定义。

前送由密集 IK 几何结点拟合成一条五次样条，再按参考速度、加速度和 jerk 约束调整时间。结点不是到位停靠点；执行器按真实时间采样。前送期间丢孔或旧的异步跟踪结果不会打断这段动作，机械臂反馈故障、跟踪误差、HOLD 或新用户指令仍能中止。对准和前送的阶段边界保留终点反馈确认，不宣称跨阶段速度完全连续。

`home`、普通 XYZ 到点和视觉任务使用同一底层参考执行器。这里只对 Panthera 实物开放完整任务；感知模块可复用，NERO 需要自己的实物适配、运动学提供器、工具和相机标定后再接入，不能仅改名称就直接运动。

规划会对生成的对准与前送轨迹检查模型自碰撞；实时替换的轨迹再次检查起点、限位、参考 v/a/jerk 和碰撞。当前碰撞模型没有桌面、洞口板、额外杆体等外部几何，不代表整套场景已避障。SDK 速度仍为固件速度上限，实物动态性能与最终精度需要实测。

夹爪命令通过同一个 ArmHub 后端填写全部七轴缓冲区，六轴保持实测位置，夹爪使用原始 rad 目标；没有将原始角度当成 URDF 开度。后台继续核对位置、机械臂保持与超时，夹爪到位也不等同于物体夹稳。

## 验证与当前边界

```bash
# 不打开相机／串口，非零幅度对准 + 前送 18 cm
bash scripts/hole_approach.sh --dry-run --report validation/hole_approach/dry_run.json
# 历史深度图检测与叠加渲染，不将历史图当实时帧
bash scripts/with_dynamics.sh scripts/validate_hole_approach.py
# 完整离线回归
bash scripts/with_dynamics.sh -m unittest discover -s tests -v
```

离线场景是合成的可达几何，包含两个关节约 0.20 rad 的对准运动与 90 个前送几何结点；恢复速度后的前送参考时长约 3.09 s（此前 0.10 rad/s 配置下约 7.59 s），采样最大直线误差约 0.001 mm。它检验数学链条，不代表真实定位精度。历史 RGB-D 检测出的洞口直径约 90.0 mm，板面残差约 1.45 mm。结果和预览在 `validation/hole_approach/`。

首次迁移验收时的设备检查（不是当前连接状态）：RealSense 枚举为空；只读机械臂会话未收到七轴完整新反馈，已退出，关联日志无运动指令。因此实时画面、视觉引导运动、夹爪与实物平顺性均未在此次迁移中验收。接好相机和机械臂后，先用 `--camera-only`、默认只读 UI 和 `p` 核对，再运行交互执行。新框架不会自动启动实物动作。

速度恢复后需要退出并重启应用，运行中的进程不会自动重载配置。恢复验证包含 106 项离线测试；记录见 `validation/hole_approach/speed_restore.json`。

## 追踪启动等待修复

启动参考轨迹时，之前会重新执行两次约 0.5 秒的静止观察，已收到的相机帧在此期间超过 0.7 秒有效期，追踪状态机随后立即 HOLD。修复后复用持续反馈中已经建立的静止判定；观测间隔超过反馈新鲜度上限（0.15 秒），或任意轴／夹爪位置变化达到 0.003 rad，就清除稳定结论，冷启动仍需完整稳定窗口和新样本计数。

驱动替身复现中，预检由约 1.10 秒降到 0.05 秒，第一帧参考正常下发；该数字是离线虚拟时钟测量，不是实物延迟保证。111 项测试通过，包括冷启动、断档、位置变化重新确认稳定，以及追踪首帧不再因等待过期。

界面显示 Track ON/OFF、Vision、连续确认帧数、相机帧年龄和状态原因。每次任务另存 `logs/hole_approach/events/*.jsonl`，包含实际收到的按键、状态转移、帧年龄和失败原因，启动时终端打印路径。修复需退出并重新启动应用，再按 `t`；默认只读启动仍不会执行追踪动作，交互执行用 `--execute`。


## 追踪执行链修复（2026-10-19 第二轮）

第一轮只解决了重复等待静止，未覆盖实际执行中的全部问题。用户随后运行的 `logs/hole_approach/events/20261019T051624_425767Z.jsonl` 记录了首次参考启动后约 3.44 秒的相机帧年龄，以及后续 `轨迹替换检查超时，已请求当前位置保持`；此后应用进入 PAUSED、关闭追踪，图像观测仍继续更新。该日志证明追踪曾进入执行，并非按键没有收到。

逐项对照 `/home/a/Panthera-HT/scripts/depth_hole_controller.py` 与 `scripts/python_sdk_runtime.py`：昨天的参考下发复用 `states=self.states`，电机循环内不构造碰撞网格；旧默认接近动作使用 SDK moveJ，平滑分支才逐周期下发参考。ArmHub 保留已要求加入的 Ruckig 平滑，修正以下执行差异：

- 在 INITIALIZING 阶段加载后端碰撞几何，再允许控制，避免首次运动时加载网格阻塞反馈循环。目标更新仍逐条预检完整参考轨迹的自碰撞。
- 已发完参考、正在保持最终目标等待到位时，替换目标不再因最后一次下发超过 0.2 秒误报断流；未发完的活动轨迹仍受下发间隔保护，最终保持仍检查新反馈、故障、跟踪误差和到位超时。
- 控制器与执行器共享生成曲线时的起始时刻，预检不再悄悄重置参考时间轴；过期时间轴会被拒绝。
- 每 tick 复用本轮已获取的反馈，重新校验计数、新鲜度、故障与限位；没有合格新反馈时仍主动查询，减少重复串口等待。

实际离线模型的几何预热与预热后轨迹检查时间、实际日志错误事件保存于 `validation/hole_approach/tracking_chain_fix.json`。这些时间只适用于记录的样本与本机，不代表实时保证。`tests_tracking_chain_fix.txt` 记录 116 项测试通过，其中包含真实 Controller → ArmTaskRuntime → Panthera 后端 → Ruckig 的目标更新回归（电机传输与碰撞模型使用替身），以及故障反馈拒发测试。`tracking_handoff_before_fix.txt` 保留修复前可重现的错误。

本轮未执行实物运动，仍需重启应用后检验现场追踪。退出旧进程，在 ArmHub 运行 `bash scripts/hole_approach.sh --execute`，界面就绪后按 `t`。日志中的 PAUSED 必须根据状态原因处理，不能视作仍在追踪。


## 丢洞回收恢复（2026-10-19）

保留昨天的追踪策略：相机仍有新画面但洞口丢失时，短暂遮挡先允许有限预测；预测不可用后进入 RECOVER，沿关节空间朝 home 收回，途中发现合格洞口立即保持，不必走完整段回零；连续三帧确认后重新规划追踪。到 home 仍看不到洞则 SEARCH_HOME 等待。相机断流或反馈不同步仍保持，不盲目回收。已经开始的 FEED_CONTINUOUS 先执行完，再按洞口状态恢复搜索；GO 继续使用点击时锁定的目标。

实际日志 `logs/hole_approach/events/20261019T052747_862543Z.jsonl` 显示 FEED_CONTINUOUS → TRACK_HOLD → RECOVER，随后报 `参考起始时刻已过期，拒绝采用旧时间轴` 并进入 PAUSED，关闭追踪。不是恢复分支缺失，而是静止启动的预检等待让上次新增的时间轴检查拒绝了回收。

修正：静止且曲线初始速度、加速度为零时，预检结束后开始曲线计时，并将实际起始时刻返回控制器；运动中的连续接续保持原时间轴，过期或下发断档仍拒绝。没有增加超时阈值或跳过静止、碰撞、反馈检查。

`tests/test_tracking_recovery.py` 覆盖预检超过 0.2 秒后仍能从静止开始回收、运动中断档仍拒绝、丢洞回收和三帧再捕获、相机断流保持、连续进给完成后回收。完整回归输出见 `validation/hole_approach/tests_tracking_recovery_fix.txt`。实物需退出旧程序后重新运行 `bash scripts/hole_approach.sh --execute`，按 `t` 验证；本轮未执行实物运动。


## 保持与到位执行链修正（2026-10-19 第三轮）

实际任务日志 `20261019T053246_542535Z.jsonl`：回收过程中看见洞口，进入保持，但三秒内未满足停止判据，报“当前姿态保持指令已发出，但停止反馈未确认”，之后 PAUSED。迁移遗漏在于：昨天 `Controller.hold()` 调用 `moveJ(..., iswait=False)` 后返回；ArmHub 却在同一电机循环阻塞调用 `stop()`，等待速度反馈连续低于 0.05 rad/s。原始记录中 J3 的位置已稳定，但速度反馈反复跳动，阻塞期间相机观测也无法消费。直接增加超时不能解决这种执行方式差异。

现在任务层 `request_hold()` 下发一次当前位置保持，`poll_hold(feedback)` 逐 tick 确认。相机、反馈和观测继续更新；保持待确认期间保留追踪意图，不启动回收、规划执行或回零；确认后按最新洞口状态继续。回零按键在运动中触发保持时进入 HOME_WAIT，确认后才执行。CLI 的 `stop()` 仍阻塞等待同一套保持确认。

保持确认使用七轴的新反馈、故障与限位校验、目标误差不超过配置到位容差，并要求位置在配置稳定幅度内持续一个到位稳定窗口；原始速度仍检查有限性并记录峰值，不再用不稳定的单帧速度阈值判停。持续位移或漂移、断流和故障仍不能被确认停止。此判据是位置保持确认，不是独立传感器对物理零速度的证明。

另一个日志问题是回零：曲线结束时 J2 约有 0.041 rad 滞后，最终目标的固件速度上限却降成 0.005 rad/s，五秒后仍剩约 0.019 rad，触发到位超时。SDK 的 MAXtqe velocity 是速度上限，不是有符号速度前馈；最终目标现在使用配置 `hold_speed_rad_s=0.02` 的到位速度下限，并仍受各轴轨迹速度上限约束。曲线进行中保持原平滑参考，最大跟踪误差和超时保护保留。任务层移除重复到位窗口，统一使用后端已确认的完成结果；后端超时也会被应用记录和处理。

证据：`tests/fixtures/tracking_reacquire_hold.json` 保存失败时实际保持请求后三秒的反馈片段；`test_tracking_hold.py` 回放它，并覆盖连续两轮丢洞→回收→重获→保持→确认→重新接近、运动中回零等待保持、持续漂移拒绝、最终目标滞后到位。完整 127 项离线测试通过，结果与实际错误摘要在 `validation/hole_approach/tests_tracking_hold_fix.txt` 和 `tracking_hold_fix.json`。这些证据证明所记录的软件故障可复现且已通过回归，本轮未操作实物，不能据此宣称现场追踪验收完成。
