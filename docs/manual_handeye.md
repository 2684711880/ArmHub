# 手动拖动与 RViz 手眼标定

全部入口在 ArmHub。终端1运行重力补偿；终端2只打开D405、读取实时反馈日志并发布FK/图像给RViz。标定端不构造机械臂连接、不发送运动命令。无自动运动或回准备位。

## 启动

```bash
cd /home/a/ArmHub
# 终端1：使用官方SDK重力计算和follower模型；link6当前为0.43 kg
bash scripts/hardware.sh gravity --robot panthera_ht --execute
```

```bash
cd /home/a/ArmHub
# 终端2：手动采样窗口
bash scripts/run_handeye.sh
```

相机为D405 `262622274345`。标记为DICT_6X6_250的ID 23，黑色边长0.100 m，不含白边。保持标记固定，手动拖动相机，停稳后点击Take Sample。采约15组不同姿态，包含多个方向的旋转，然后Calibrate。主动保存采样和结果到 `calibration/panthera_ht/wrist_camera/sessions/`；窗口崩溃可能丢失未保存的采样。新结果不自动写入现用hand_eye.json。

Frames Selection：

| 字段 | 坐标系 |
|---|---|
| Sensor | handeye_color_optical |
| Object | handeye_target |
| End-effector | gripper_center |
| Robot base | base_link |

安装类型为Eye-in-hand。`gripper_center`是link6沿+X方向165 mm的TCP，和官方SDK一致。D405逆Brown畸变经RealSense SDK反投影映射，发布去畸变图像与对应零畸变CameraInfo。

每次启动的日志独立保存到 `validation/handeye/<北京时间>_<PID>/`。采集端只接受150 ms内的新gravity_tick，补偿停止后不继续发布旧反馈。RViz启动在ROS_DOMAIN_ID=77、仅本机网络。ArmHub重力补偿本身不依赖ROS域。

## 依赖

插件源码位于 `calibration_ws/src/`，源仓库版本与已有Humble兼容修改记录在 `calibration_ws/sources.json` 及各源码目录的 `imported.patch`。本地构建：

```bash
bash scripts/build_handeye.sh
```

运行使用ArmHub的 `.venv_dynamics`（NumPy 1、RealSense、ROS Python），不再依赖旧工程虚拟环境、ROS模型或标定插件安装目录。日志锁仍使用同一用户会话锁，避免旧窗口和新窗口同时打开相机。

此次迁移未修复此前RViz的 `std::length_error` 崩溃；如果重现，保留本次独立日志进行定位。跨姿态一致性和独立角点绝对精度均需检查后，才能采用结果。
