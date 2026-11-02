# Panthera 回 home

`home` 与昨天工程 `depth_hole_controller.py` 中的 `r` 键一致：六轴关节角回到 `[0, 0, 0, 0, 0, 0]` rad，夹爪保持本次实测原始位置。它是关节运动，不是 TCP 移到基座原点，不是编码器零点校准；不会调用固件 reset 或 set_reset_zero。

## 命令

先退出重力补偿和其他控制程序，确认回位空间畅通。

```bash
cd /home/a/ArmHub
# 只预览，显示当前角度、目标、关节变化量、计算时长和自碰撞检查结果
bash scripts/hardware.sh home
# 执行并等待六轴到位与稳定反馈
bash scripts/hardware.sh home --execute
# 可降低每轴参考速度比例，默认 1.0 使用恢复后的机型上限
bash scripts/hardware.sh home --velocity-scale 0.05 --execute
```

可追加 `--report logs/home.json` 保存报告；逐帧日志路径见 `log_file`。不自动回位，只有显式调用才回 home。无需重新编译 SDK。

Python 入口（通过 `scripts/with_dynamics.sh` 运行）：

```python
from armhub import create_arm

with create_arm('panthera_ht', backend='hardware') as arm:
    print(arm.preview_home())
    arm.enable()
    result = arm.home()
    print(result)
```

六轴速度上限为 `[1.68, 2.0, 2.0, 1.68, 1.48, 1.48] rad/s`，与昨天工程一致；平滑参考依旧约每 50 ms 下发。

目标保存在 `robots/panthera_ht/hardware/config.json` 的 `home_joints_rad`，当前沿用昨天工程的六轴零姿态。每次执行重新读取稳定反馈，按最大关节位移和速度上限计算时长，没有单次角度或最长时长限制；规划后起点发生变化会拒绝发送。回位默认使用 Ruckig 平滑参考，经官方 `pos_vel_MAXtqe` 按时间连续填写七轴命令，六轴回位、夹爪保持；完成判据与现有关节控制一致（关节误差 ≤0.01 rad，持续稳定 0.3 s），不是精确为零。

检查生成后 Ruckig 参考轨迹的自碰撞，不检查桌面与其他外部障碍，固件实际路径不保证严格沿插值。若路径自碰撞会拒绝，不自行绕行。异常或 Ctrl+C 时尝试读取新反馈并保持当前位置；退出不代表断使能。NERO 实物尚未开放 home。

具体平滑参数、日志与验证边界见 [平滑执行](panthera_smoothing.md)。

## 验证

`tests/test_panthera_home.py` 使用驱动替身验证无使能拒绝、预览不发指令、六轴回位与夹爪保持、长距离自动计时、非法目标和速度、碰撞拒绝、重力补偿互斥。实际串口预览保存到 `validation/hardware/panthera_ht/home_preview.json`。本次没有执行实物回位，回位效果待用户核对。
