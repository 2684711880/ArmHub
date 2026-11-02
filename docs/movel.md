# Panthera moveL 平面轨迹实验

入口为 `bash scripts/movel.sh`。三份上传文件已接入 `apps/movel/`：轨迹几何和误差分析来自上传版本，位置控制入口改为 ArmHub。上传的轨迹文件缺少 `to_waypoints`，已补齐。原始上传文件仍在 Downloads，旧位置控制源码另存为 `source_moveL_pos_control.py`，仅供对照，不作为启动入口。

## 实机运行

```bash
cd /home/a/ArmHub
# 实时反馈只读预览；默认以当前 TCP 和姿态为锚点
bash scripts/movel.sh --trajectories line_u --direction fwd --size .02 --pitch .005 --speed .01
# 指定原示例锚点及姿态，预检后执行 20 mm 直线
bash scripts/movel.sh --trajectories line_u --direction fwd --size .02 --pitch .005 --speed .01 --anchor .24 0 .1 --execute
# 原示例 XY 六种轨迹，正向和反向
bash scripts/movel.sh --anchor .24 0 .1 --execute
# 选择平面和形状
bash scripts/movel.sh --plane xy xz --trajectories square circle --direction both --anchor .24 0 .1 --execute
# 无硬件离线预检，不打开串口
bash scripts/movel.sh --anchor .24 0 .1 --q 0 0 0 0 0 0
# 重新出图
bash scripts/with_dynamics.sh -m apps.movel.moveL_error_analysis /绝对路径/结果目录
```

不带 `--execute` 只读预览；`--q` 为离线模式，与 `--execute` 互斥。`--size` 默认 .08 m、`--speed` 默认 .05 m/s、`--pitch` 默认 .01 m。指定 .02 m 小框时必须同时使用 `--pitch .005`。输出默认在 `logs/movel/`，也可 `--output` 指定不存在的目录。

启动不回零。指定 `--anchor` 才从实测起点转移到该锚点（原示例姿态）；每条曲线执行前沿笛卡尔路径引入起点，记录曲线，结束后按关节参考回锚点，最后保持在锚点。Ctrl+C/错误沿用 ArmHub 当前位置保持确认。夹爪保持原位置。

## 执行链与预检

`moveL_pos_control → 官方 SDK 数学 FK/IK（不构造硬件 Panthera）→ 五次关节样条 → ArmHub CheckedCurve / ReferenceExecution → 官方原生 SDK → 七电机新反馈`。

所有锚点转移、引入段、实验曲线和返回锚点段在使能前检查位置/速度/加速度/jerk、自碰撞及 TCP 最低高度；执行器在每段运动前复核当前起点、反馈新鲜度、故障、夹爪保持和跟踪偏差。失败不自动改参数重试。

几何生成保留上传版本；IK 改用 1e-5 精度，五次样条端点速度和加速度为零，按现有速度/加速度/jerk 限制延长时长。**因此不是原版三次样条和 10 ms 下发的复现**；ArmHub 当前控制周期 50 ms，实际时长见 `preview.json`，不是简单的路径长除速度。矩形角点经过关节样条后可能有微小偏离，规划路径误差会单独给出。

TCP 按官方数学层定义为 joint6 后沿局部 X 方向 0.165 m。高度限制仅检查该 TCP 的基座 Z 坐标；自碰撞检查不包含桌面、外部障碍和未建模工具。

## 结果与定义

- `preview.json`：起点转移、引入、曲线与返回段的时长/检查信息。
- `preflight.log`：全部预检输出。
- `run.json`：本次配置、状态和错误。
- `NN_*.npz`：计划 q/v、真实下发 q、取样/下发时间、各关节接收时间与反馈。
- `NN_*.csv/png`、`summary.png`、`metrics.json`、`pair_*.png`：逐帧误差、统计与正反向对比。

位置均由编码器关节角经 FK 计算，**不是外部测量的末端绝对精度**。主跟踪参考为反馈接收时刻的连续计划曲线；实际离散下发目标与时间另存，50 ms 下发保持、串口接收延迟和电机不同步均会影响结果。`dispatch_lateness_ms` 在此适配中表示本周期取样到下发日志的耗时，不表示相对于固定序号时间点的累计迟到。理论曲线时长超过 300 ms 的滞后会超出原分析器搜索窗。

已记录后续失败或中断时仍保存已记录的数据；引入/返回段仅在后端 JSONL 留有控制日志，不算实验轨迹。

## 装笔并画 4 cm 正方形

```bash
cd /home/a/ArmHub
bash scripts/gripper_open.sh --execute
# 放笔，然后关闭；默认动作速度 .3 rad/s，动作力矩上限 .3 Nm
bash scripts/gripper_close.sh --execute
# 确认笔夹稳、笔尖轻触水平纸面，并核对 +X/+Y 方向的 4 cm 空间后预览
bash scripts/draw_square.sh
# 确认准备好后才执行
bash scripts/draw_square.sh --execute
```

两个夹爪脚本不带 `--execute` 均只读预览。可用 `--position` 指定原始电机角度；默认打开 1.6 rad、关闭 0 rad。关爪未到目标且反馈位置持续稳定时，视为可能接触物体，切换到实测位置保持；不宣称已夹稳。保持仍沿用后端配置的夹爪力矩上限 1 Nm，动作限矩不代表接触力已标定。

画线入口使用 `--square-from-current`：当前位置为首个角点，只走四边，不从中心引入、不返回中心、不回零、不自动降低笔尖。默认 XY 平面、边长 .04 m、平均请求速度 .02 m/s。保持末端姿态时，固定安装的笔尖相对 TCP 偏移不改变方形边长，但纸面高度与实际笔尖偏移必须现场确认。这里是位置控制，没有笔尖接触力/柔顺控制。

可以覆盖参数，例如 `bash scripts/draw_square.sh --size .08 --execute`。单平面、单方向、只允许 square，不能搭配 `--anchor`。
