> 历史受限试跑说明：应用户要求，evaluate 已恢复官方策略执行链，当前不自动安装本文的 policy_guard。当前完整运行命令见 docs/policy_full_run.md。

# ACT 首次实物受限试跑

用户执行硬件，离线验证不代表实物验收。先退出现有采集或示教程序，保持训练时的相机、盒子位置，清空机械臂运动区域。

```bash
cd /home/a/ArmHub
bash scripts/lerobot.sh prepare --execute
```

准备位完成后摆好蓝块。下一条命令加载模型后会自动开始，最多运行一轮5秒：

```bash
bash scripts/lerobot.sh evaluate \
  --checkpoint /home/a/ArmHub/outputs/lerobot/blue_to_box_act_v1/checkpoints/last/pretrained_model \
  --repo-id local/eval_blue_to_box_01 \
  --policy.device=cuda \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=5 \
  --dataset.reset_time_s=0 \
  --execute
```

此试跑验证方向与跟踪，不要求完成抓取。评估数据与20条训练数据分开。eval_blue_to_box_01若已存在，下次换eval_blue_to_box_02，避免覆盖。

当前 evaluate 自动安装 policy_guard：有限数、七维完整性、机型位置范围、首帧准备位、鲜活七电机反馈及故障、速度和力矩反馈检查；拒绝六轴预测相对实测超过0.3rad的跳变，目标变化率与SDK速度限制0.15rad/s；实际目标到实测跟踪误差不超过0.1rad；六轴目标相对起点不超过0.25rad；检查实测到目标、前目标到新目标的插值自碰撞及TCP基座坐标Z>=0.05m。控制间隔超过0.3s后拒绝下一帧。异常锁定后不再发送策略动作。退出沿既有反馈保持链处理，不自动回零或开爪。

首次评估采用SDK pos_vel_MAXtqe模式，替换官方策略直接PD包，六轴沿机型力矩上限，夹爪0.3 SDK Nm。不是完整50帧预测轨迹验证，也不包含加速度/jerk约束、桌面/盒子/人的碰撞模型或硬件急停保证；TCP高度以基座而非实测桌面为参考。自碰撞检查采用现有名义夹爪几何。仅用位置夹持，不叠加replay抓取辅助。不能将该试跑视为完整任务验收。

日志位于logs/lerobot/policy/trial_*.jsonl。Ctrl+C触发程序退出保持；断电可能失去支撑并下坠，程序停止不等于硬件急停。

速度尖峰处理：0.6～1.2 rad/s 时冻结上一目标；同一电机连续至少3帧且超过0.6持续至少0.12秒才退出，恢复到阈值内即清除计时。超过1.2 rad/s立即退出。故障码、非有限反馈和力矩超限立即退出。每帧及异常日志保存七电机位置、速度、力矩、故障码、反馈计数和时间。此调整没有确认旧现场报错的具体触发项，需用新日志复核。
