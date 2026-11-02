# 官方单臂试录与回看

退出其他机械臂与相机程序，在 `/home/a/ArmHub` 运行。官方采集仍使用官方的位置示教和弱夹爪回弹控制，没有改为手掰完全自由模式。

## 返回此前保存的准备位

读取 `integrations/panthera_lerobot/configs/teaching.json` 中 `source=hardware` 的准备位，沿用示教窗口的回位速度、最短时长和可选 TCP Z 下限。原文件不改写。

```bash
# 连接机械臂，只读反馈并预览路径，不发送运动命令：
bash scripts/lerobot.sh prepare
# 确认实际空间的路径可通行后执行；此命令会移动机械臂：
bash scripts/lerobot.sh prepare --execute
```

回位复用现有 HardwareSession、Ruckig、限位、自碰撞、起点漂移、反馈及跟踪检查。六轴运动期间保持当前夹爪开度；六轴到位后，按 teaching.json 的 gripper_open/gripper_speed/gripper_torque 张开夹爪，再根据新反馈确认到位并关闭连接。当前配置为 1.6 rad、0.3 rad/s、0.3 Nm 上限。模型不包含桌面、物块、线缆等外部障碍。Ctrl+C 请求退出并沿用既有当前位置保持流程，退出不表示电机失能。

## 只录一轮

```bash
bash scripts/lerobot.sh record --repo-id local/blue_pick_trial_01 --execute -- \
  --display_data=true \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=inf \
  --dataset.reset_time_s=0 \
  --dataset.single_task="pick up the blue block"
```

启动后会连接 D435 和机械臂、进入官方示教采集。`inf` 取消按时间结束；录制完成按右方向键结束本轮，等待保存和视频编码结束后终端提示符返回。不要通过强制杀进程结束录制。只有一轮时不需要复位阶段。左方向键用于重录，Esc 结束采集。

每次新试录换一个新的数据集名；失败创建的空目录也不覆盖或删除。前次 `local/blue_pick_official_01` 在添加本流程时元数据为 0 个完整回合，保留原样。

## 回看图像与动作曲线

```bash
bash scripts/lerobot.sh view --repo-id local/blue_pick_trial_01 --episode 0
```

此命令只读取本地 LeRobot 数据集，不导入或连接机械臂控制器。Rerun 底部选择 `episode` 时间轴，点击播放或拖动，查看 D435 视频以及七维实测位置和动作曲线。也可不打开查看器，保存可再次打开的文件：

```bash
bash scripts/lerobot.sh view --repo-id local/blue_pick_trial_01 --episode 0 \
  --save logs/lerobot/blue_pick_trial_01.rrd
```

`replay --execute` 是另一条会驱动真实机械臂的官方动作复现入口，不等同于上述离线回看。本次先验证录制完整和图像动作对应，不自动执行实机回放。

## 软件验收

`tests/test_lerobot_trial.py` 覆盖保存位来源、只读预览、显式执行、预检拒绝、反馈完成、异常退出与空回合拒绝。离线视频回看另用合成回合导出 `.rrd` 验证，不等同于真实机械臂试录或实机返回验收。
