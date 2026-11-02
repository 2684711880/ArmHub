# Panthera 腕部相机标定

`hand_eye.json` 当前采用用户在 2026-10-22 采集的 17 组 D405 标定结果（`handeye1.yaml`、`pose1.launch.py`），绑定序列号 `262622274345`，状态为 `calibrated_pending_independent_measurement`。样本内位置散布 RMS 5.63 mm、最大 10.45 mm；仍待角点实物验证。此前官方矩阵备份在 `evidence/hand_eye_official_before_handeye1.json`。

坐标定义为 gripper_center → handeye_color_optical（米）。官方标定例程的 FK 使用 joint6 加 X 方向 0.165 m 的工具偏移，与 ArmHub gripper_center 一致，因此矩阵无需附加平移。相机内参仍须从本台 D405 实时读取。

旧安装的 15 组 TSAI 参数备份在 `evidence/hand_eye_before_official_d405.json`，历史求解与验证记录仍保留。任务和工具参数分别位于 `apps/hole_approach/config.yaml` 与 `robots/panthera_ht/tools/rod.json`，不混入手眼矩阵。
