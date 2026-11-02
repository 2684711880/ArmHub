# 标记左上角 15 cm 对准 — 独立绝对验证实验

验证对象：`validation/handeye/solved_111.json` 解算出的手眼外参
`T_gripper_center → handeye_color_optical`（TSAI，t = [-68.98, 34.21, 154.09] mm）。

标记：ID 23，DICT_6X6_250，**黑方块边长 100.0 mm 已确认**。

---

## 1. 为什么要做这个实验

board scatter（4.12 mm rms / 5.97 mm max）只检验**一致性**：15 个姿态推算出的板位姿是否彼此相同。
若外参有一个**共模误差**（例如整体平移 20 mm），所有姿态会推算出"同样错"的板位姿，散点照样很小。
LOO 最大 8.02 mm 也只是样本内重采样，不是绝对测量。

本实验测**绝对精度**：用外参算出「夹爪尖端应该在哪」，把臂开过去，再用**不经过相机**的量具量实际位置。
量具与相机无关，共模误差藏不住。

---

## 2. 目标点定义（关键，容易算错）

### 2.1 标记坐标系原点在**左下角**，不是中心

采样用的 MoveIt 插件走 `cv::aruco::estimatePoseBoard` + `GridBoard`。
OpenCV 4.5.4 的 1×1 GridBoard `objPoints`（已实测打印）：

| objPoints | 板坐标系 | 物理角 |
|---|---|---|
| 0 | (0, 0.1, 0) | **左上角 ← 目标** |
| 1 | (0.1, 0.1, 0) | 右上角 |
| 2 | (0.1, 0, 0) | 右下角 |
| 3 | (0, 0, 0) | 左下角 = **原点** |

所以 YAML 里的 `object_wrt_sensor` 是 `T_camera_board`，板原点在**黑方块左下角**，
+X 向右，+Y 向上，+Z 出板面朝外。

**目标点 = 板坐标系 (0, 0.100, 0) = 黑方块左上角。**

> 若误按 `estimatePoseSingleMarkers` 的"中心原点"算，目标会偏 **70.7 mm**，实验直接作废。

### 2.2 本脚本的检测约定

`scripts/handeye_core.py::observe_marker` 用 `solvePnP` + 中心原点。两套约定指向同一物理点：

| 角 | 中心系 (s=0.100) | 板系 (插件) |
|---|---|---|
| 左上 | (-0.050, 0.050, 0) | (0, 0.100, 0) |
| 右上 | (0.050, 0.050, 0) | (0.100, 0.100, 0) |
| 右下 | (0.050, -0.050, 0) | (0.100, 0, 0) |
| 左下 | (-0.050, -0.050, 0) | (0, 0, 0) |

外参 X 只连 gripper 和 camera optical，与标记坐标系无关，所以用哪个检测器都行，
只要角点坐标跟着约定换。脚本默认 `--corner top-left`，并把目标角画红圈存在 `detect.png` 里，
**先看图确认是黑方块左上角再执行**。

---

## 3. 器材与准备

1. **标记**：ID 23，贴平贴实在墙上（不能弯）。
   **量黑方块边长 = 100.0 mm**，不含白色静区。已经确认就再复核一次并记入记录表。
2. **物理标记目标角**：在黑方块左上角旁贴一小块胶带做记号（不要盖黑方块）。
3. **可测量的夹爪尖端**：`gripper_center` 是 URDF 虚拟系，没法直接量。
   装一根尖锐指针/杆，用游标卡尺量 **杆尖相对 gripper_center 的偏移**（通常沿 +X 一段），
   记为 `--rod-length`。这一步顺带完成待办的杆尖 TCP 标定。
4. **量具**：钢直尺或游标卡尺（读数 0.5–1 mm）；可选 150 mm 量块；可选侧视手机照片（画面放一把尺）。
5. **机械臂抬到展开锚点**（J2/J3 不在收拢位）。不要用收拢姿态做本实验。
6. 清空臂到墙之间的工作空间，急停可达。

---

## 4. 测量原理

脚本把**杆尖**送到：黑方块左上角 + 板法向外 `standoff`（本次 **150 mm**）。

到位后量两个数：

| 量 | 含义 | 期望 |
|---|---|---|
| `d_plane` | 杆尖 → **标记平面** | 150 mm |
| `d_corner` | 杆尖 → **已标记的那个角** | 150 mm |

反算（`d_corner` 是空间直角三角形斜边：一直角边 `d_plane`，另一直角边是面内横向偏差 e）：

```
深度(轴向)误差 = d_plane - 150 mm
横向误差 e     = sqrt(d_corner² - d_plane²)
```

可选侧视照片：杆尖和那个角同框，尺子同平面，独立复核。

**这套量具不经过相机**，所以能证伪共模误差。

---

## 5. 步骤

### 5.0 准备
量边长 → 标记目标角 → 量杆尖偏移 → 抬臂展开 → 清工作空间。

### 5.1 观测 + 解算预览（不动）

相机采图（二选一）：
- 快照（默认）：另开一个终端跑 `scripts/live_detect.py`，或 `scripts/capture_wrist_rgbd.py`
- 实时：加 `--camera-source live`

```bash
cd /home/a/Panthera-HT
./scripts/run_marker_corner_align.sh \
  --extrinsic validation/handeye/solved_111.json \
  --rod-length 0.04 --standoff 0.15
```

脚本输出：
- 目标角的图像像素 + `validation/handeye/corner_validation/<时间>_topleft/detect.png`（红圈）
- 目标 base XYZ / RPY、目标关节角、关节增量、IK 残差
- `plan.json`

**检查：**
1. `detect.png` 红圈是否落在黑方块左上角
2. 关节增量是否合理（最大增量、J2/J3 是否仍 ≥ 0）
3. 离墙距离是否安全

### 5.2 执行

```bash
./scripts/run_marker_corner_align.sh \
  --extrinsic validation/handeye/solved_111.json \
  --rod-length 0.04 --standoff 0.15 \
  --execute
```

输入 `GO` 才动。默认 ≤ 0.25 rad/s，手放急停。
（预览和执行分两次跑，中间不要动标记和相机。）

### 5.3 物理测量
量 `d_plane`、`d_corner`。

### 5.4 记录

```bash
./scripts/run_marker_corner_align.sh --record --run-id 1 \
  --d-plane-mm 98.5 --d-corner-mm 101.2 --note "J1=0.2 start"
```

脚本反算深度/横向误差并追加到 `validation/handeye/corner_validation/measurements.csv`，
多跑几次会自动打印均值和最大偏差。

### 5.5 重复 5 次

每次换一个**不同的臂型**观测（不要只动腕），同一目标角。
5 次的**均值**和**标准差**分开看——这是本实验最重要的一步。

---

## 6. 判读

| 现象 | 结论 | 处置 |
|---|---|---|
| 横向 e 小、深度也小 | 外参绝对精度够 | 可考虑采用（走文档状态更新流程） |
| e 一致偏大，5 次标准差小 | **系统性偏差** | 可修正，但记成经验补偿，不得冒充第一性原理结果 |
| e 5 次散得开 | **FK / 机械回差 / 联轴器** | 改外参没用，先查机械与关节零位 |
| 深度偏、e 小 | 沿视线方向错 | 标记尺寸已确认 100 mm → 优先查相机内参 / 焦距 |
| 位置都对、姿态明显歪 | 外参旋转部分错 | 重采样，增大旋转激励 |

**通过标准**（由孔任务间隙决定，这里是暂定诊断阈值）：
- 横向 e ≤ 3 mm 且深度误差 ≤ 3 mm
- 5 次最大偏差 ≤ 5 mm
- 参考：LOO 目前是 4.98 mm rms / 8.02 mm max，所以本实验若做到 3 mm 以内，
  说明 LOO 的偏差来自样本内 FK 噪声，外参本身没那么差。

---

## 7. 硬约束

- 验证通过前 **不写** `hardware_ws/config/wrist_camera.yaml`，**不撤** `target_compensation.yaml` 的 50 mm 经验补偿
- J2/J3 下限 0 rad，不得为负；不要在收拢姿态单独动腕部
- 重力补偿与硬件桥互斥，都抢 `/dev/ttyACM0`，同时只能跑一个
- 运动前先预览；执行时手放急停
- Ctrl+C 重力补偿会掉电/松刹车 — 先托住臂

---

## 8. 产出物

```
validation/handeye/corner_validation/
  <时间>_topleft/
    detect.png        标注图，红圈 = 目标角
    plan.json         目标位姿、关节角、IK 残差、检测位姿
  measurements.csv    每次物理测量一行
```
