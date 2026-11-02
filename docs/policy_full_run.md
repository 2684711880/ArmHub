# 完整实物策略运行

用户要求撤销新增首次试跑保护层。evaluate 已恢复官方 Panthera send_action：PD位置跟踪及重力前馈，不再使用 policy_guard 的 pos_vel_MAXtqe 模式、0.15rad/s限速、±0.25rad活动范围、0.6/1.2rad/s速度检查、目标增量检查、自碰撞及TCP高度检查。既有ArmHub反馈身份/时效/故障检查、串口独占和退出保持保留。没有实物执行验证。

先退出现有控制程序，回训练准备位并摆好蓝块：

```bash
cd /home/a/ArmHub
bash scripts/lerobot.sh prepare --execute
```

保持相机、桌面与盒子配置，下一命令模型加载完成后自动开始：

```bash
bash scripts/lerobot.sh evaluate \
  --checkpoint /home/a/ArmHub/outputs/lerobot/blue_to_box_act_v1/checkpoints/last/pretrained_model \
  --repo-id local/eval_blue_to_box_03 \
  --policy.device=cuda \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=inf \
  --dataset.reset_time_s=0 \
  --display_data=true \
  --execute
```

无自动时限；放好物块后按右方向键结束这轮。Esc或Ctrl+C退出。需要焦点在终端而非Rerun。模型没有成功终止判断，任务完成后也会继续输出动作，必须人工结束。此轮评估录像与训练数据分开保存；目录已有则换下一个编号。位置夹爪未叠加replay力矩辅助。
