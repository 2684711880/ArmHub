# Panthera 实物空间到点

统一函数为 `arm.move_to((x, y, z))`。本实现参考昨天 `/home/a/Panthera-HT/scripts/python_sdk_hole_approach.py` 的 `solve_plan` 和 `python_sdk_runtime.py` 的 `ProjectPanthera.Joint_Pos_Vel`：调用本工程内复制的官方 FK/IK，再调用同一官方原生位置／速度上限／力矩上限接口。运行不依赖旧工程，不启动视觉、回零或重力补偿。

## 直接测试

先退出重力补偿及旧工程控制器，确保机械臂周围有运动空间。

```bash
cd /home/a/ArmHub
# 当前实测关节角经 FK 得到 TCP 位姿，只读
bash scripts/move_to.sh --robot panthera_ht --backend hardware --state
# 预览：本次实测位置沿 base_link 的 Z 轴抬高 1 cm，不下发运动
bash scripts/move_to.sh --robot panthera_ht --backend hardware --delta 0 0 0.01
# 执行：以执行时的新反馈为起点，抬高 1 cm
bash scripts/move_to.sh --robot panthera_ht --backend hardware --delta 0 0 0.01 --execute
```

`--delta` 每次按当前位置计算，重复执行会累积位移。预览输出的 `target_position_m` 可直接填入绝对目标，避免重复相对移动：

```bash
# 将 X Y Z 替换为预览中的三个数；单位 m
bash scripts/move_to.sh --robot panthera_ht --backend hardware --xyz X Y Z --execute
```

不加 `--execute` 时实物只预览；原来的仿真命令默认仍直接执行。`--report logs/xyz.json` 可保存结果；实物每次连接另存逐帧日志，输出包含 `log_file`。

Python 调用使用本工程动力学环境，例如 `bash scripts/with_dynamics.sh your_script.py`：

```python
from armhub import create_arm

with create_arm('panthera_ht', backend='hardware') as arm:
    pose = arm.get_pose()
    target = list(pose['position_m'])
    target[2] += 0.01
    print(arm.preview_move_to(target))  # 只预览
    arm.enable()                       # 软件执行许可，不回零
    result = arm.move_to(target)        # 此函数直接执行并等待反馈到位
    print(result)
```

## 行为与边界

- XYZ 参考 `base_link`，TCP 为模型 `gripper_center`（官方 joint6 后 +X 0.165 m），不包含另装工具的未标定偏移，也不包含仿真底座 0.75 m 高度。
- 实物省略四元数时保持当前姿态；仿真省略时允许 MoveIt 选择姿态。需要相同目标姿态时两边都显式传 `--quaternion QX QY QZ QW`。
- 官方近邻 IK 从当前角度求解，不使用随机远处解；失败会拒绝，不自动换姿态。空间路径为关节点到点，TCP 不保证直线。
- 下发前复核限位、FK 残差、起点变化，并检查生成后的 Ruckig 参考轨迹自碰撞。夹爪几何按模型中性开度检查，实物夹爪原始角度保持。**没有桌面／外部障碍物碰撞检查；固件实际运动也不保证沿理想插值。**
- 不额外限制单次 TCP 平移距离、单轴变化角度、末端姿态变化角度或规划最长时间；目标仍须通过 IK、物理关节限位和自碰撞检查。当前六轴速度上限为 `[1.68, 2.0, 2.0, 1.68, 1.48, 1.48] rad/s`，默认 `velocity_scale=1.0`，规划时长至少 2 s；这都是本工程接入参数，不是厂商性能保证。位置到位容差 5 mm、姿态 0.03 rad，并要求关节到位与反馈稳定。修改位置在 `robots/panthera_ht/hardware/config.json` 的 `cartesian`。
- 正常完成后保持最终目标；异常或 Ctrl+C 尝试基于新反馈保持停止。程序退出不代表断使能。
- NERO 仍仅支持仿真空间到点；外部任意连续轨迹接口、直线运动和仿真实时下发实物尚未实现。

`move_to` 和 `home` 默认共用 Ruckig 平滑执行器，约每 50 ms 下发连续参考；命令不变。参数与反馈见 [平滑执行](panthera_smoothing.md)。

## 验证证据

`validation/hardware/panthera_ht/xyz_offline.json`：两个起点保持姿态抬高 1 cm，官方 IK 位置残差约 0.010 mm；官方 FK 与模型 TCP 一致；不可达、非法四元数、自碰撞负例拒绝。复现：

```bash
bash scripts/with_dynamics.sh scripts/validate_panthera_kinematics.py
bash scripts/with_dynamics.sh -m unittest discover -s tests -v
```

`xyz_preview.json` 为实际串口新反馈的只读预览：当前位置约 `(0.24655, 0.00546, 0.17429)` m，目标 Z 为 `0.18429` m，最大关节变化约 2.22°，IK 残差约 0.0099 mm，17 对几何的 3 个名义路径样本通过，`motion_sent=false`。这些结果不证明实物 XYZ 到位精度；此功能的首次实物移动留给用户执行与核对。
