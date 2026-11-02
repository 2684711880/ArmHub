# GitHub 发布记录

本次整理日期：2026-11-02。发布范围是 `/home/a/ArmHub`，独立于 Nero-7 工程。

## 内容

- 保留统一接口、Mock、共享仿真、Panthera SDK 适配、视觉应用、绘图、辨识、手眼标定和 LeRobot 集成。
- 将必需的模型/网格、原生 SDK 源码及编译依赖快照纳入版本管理，保留来源锁定和第三方许可。
- 新增新机器安装说明、无需设备的核心测试入口和 CI 模板。当前 GitHub 登录缺少 `workflow` 权限，模板保存在 `ci/github-actions-offline.yml`，未启用自动运行。
- LeRobot 上游目录继续通过固定提交加补丁恢复；虚拟环境、数据集、权重、日常日志、生成扩展和构建目录不提交。
- 既有配置和标定作为参考，未修改控制器行为或执行实物运动。

## 软件验证

本次离线验证的具体结果见 [summary.json](../validation/publication/summary.json)：

| 检查 | 结果 |
|---|---|
| 完整软件回归 | 255 项通过，0 失败、0 错误 |
| 干净检出核心检查 | 11 项通过 |
| 干净检出原生 SDK 构建与导入 | 通过，`commission_safe_binding=True` |
| 干净检出 ROS 仿真包构建 | 通过，1 个包 |
| 来源文件 SHA256 | 80 个模型文件和 968 个 SDK/依赖文件全部匹配 |
| LeRobot 导入、配置与重力检查 | 通过，不打开设备 |
| Python 依赖与 shell 语法 | `pip check` 通过，工程 shell 脚本语法通过 |

干净检出来自本次暂存内容，未复制原工作树的构建目录或虚拟环境；使用的是本机已安装的 ROS/CMake/系统依赖，不能据此宣称全新操作系统已完成端到端安装。未启动 Gazebo 运行验收或 RViz，也未连接设备。

核心测试无需 ROS 或设备；完整测试需要 LeRobot 环境和重新编译的 SDK。早期将所有测试放在动力学环境执行会缺少 LeRobot 包；完整入口已统一为 `bash scripts/test_offline.sh full`，并补齐其 Matplotlib 依赖。

历史 `validation/` 中的采集日志、图像和报告按原验证范围解释；部分报告引用未上传的原始日志或合成数据，不代表这些原始数据随仓库分发，也不能将历史离线通过解释为全部现场任务通过。

## 已有蓝块数据与 ACT 模型

整理时重新读取本地数据与 checkpoint 元数据：

- `local/blue_to_box_clean_01`：20 回合、4142 帧、20 FPS，机器人类型 `panthera_follower`。
- `blue_to_box_act_v1`：checkpoint 训练步数为 20000；ACT 使用 `chunk_size=50`、`n_action_steps=10`、单帧观测、ResNet18。
- 数据和 checkpoint 均保留在本机，未上传；本次没有重新训练、推理或测量抓取成功率。

这说明本机已有采集和训练产物，不证明策略在新相机位置或新任务条件下成功。评估当前仍使用官方 `send_action` 链，未恢复已撤销的 policy_guard；详见 [当前策略执行](policy_full_run.md)。
