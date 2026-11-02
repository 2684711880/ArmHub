# D405 自动手眼标定

使用实际 D405 `262622274345`，DICT_6X6_250 的 ID 23，黑色方块边长 0.100 m（不含白边）。固定标记，采集期间不要移动标记、基座、相机支架或工具。

先退出重力补偿、示教窗口和其他控制程序。手动准备展开姿态，让整个码在画面中有余量。J1–J3 固定在起点，仅 J4–J6 在各自起点 ±10° 内采样。起点必须在配置限位内；不会自动去准备位或放宽关节限位。预检进行全部15姿态路径的自碰撞检查和旋转激励检查，不涵盖桌面、标定板和相机线等外部障碍。

```bash
cd /home/a/ArmHub
# 相机、实际反馈和路径检查，不发送运动目标
bash scripts/auto_handeye.sh --marker-size 0.10
# 自动运动、采集与求解
bash scripts/auto_handeye.sh --marker-size 0.10 --amplitude-deg 10 --speed 0.04 --execute
```

`--marker-id` 默认23，`--serial` 默认上述D405，`--amplitude-deg` 范围6..15度，`--speed` 范围0.005..0.06 rad/s。建议首先使用默认值。运动中保持急停可达。Ctrl+C/异常请求当前位置保持，不自动回起点；退出不代表失能。

15个不同姿态中12组参与拟合，3组留出验证。每次到位后等待稳定，再用前后实际反馈包围5张新图；拒绝关节移动、码位姿跳变和差的重投影。使用官方SDK FK，TCP为gripper_center，即link6沿+X偏移165 mm，与官方TCP一致。运动复用ArmHub现有SDKMoveJBridge与反馈保护。

D405彩色流的inverse_brown_conrady通过RealSense的deproject接口转换角点为归一化视线，然后使用无畸变PnP；不把逆畸变参数当作普通OpenCV畸变系数。

输出在 `calibration/panthera_ht/wrist_camera/sessions/<北京时间>/`：

- `plan.json`：姿态与各段路径检查。
- `camera.json`、`preview.png`：内参与首次观测。
- `pose_XX_0.png` 到 `pose_XX_4.png`：每姿态5张原图。
- `samples.json`：实际反馈、FK、码位姿、帧时间和静止稳定性记录，逐姿态保存。
- `calibration_result.json`：T_tcp_camera、XYZ毫米、RPY角度、各方法结果和验证指标。
- 失败时保存 `failure.json`，已采样数据保留。

PARK/TSAI/HORAUD按训练集一致性选取；留出验证不参与选方法。暂定留出最大位置偏差3 mm、最大角度偏差2°。这检验跨姿态一致性，不代替实物角点绝对精度测量。结果不自动覆盖现用hand_eye.json。

重新求解已保存采样（无需设备）：

```bash
bash scripts/auto_handeye.sh --solve /绝对路径/会话目录/samples.json
```

纯离线包络/激励预览：`bash scripts/auto_handeye.sh --dry-run`。默认使用合成起点，不证明实物路径通过。
