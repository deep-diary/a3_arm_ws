# F149 — LeRobot 原生 A3 follower robot 类 + 采集全链路（A0）【P0】


- **说明：** reBot 的 `rebot_b601_dm/rs_follower` 已原生合入 HuggingFace LeRobot（calibrate / teleoperate / record / dataset-viz / replay）。A3 需自写 7-DOF follower robot 类（`L1_joint`..`L7_joint`，复用 `a3_lerobot_config/config/a3_robot.yaml`），经 ROS 2 后端对接标准栈（**不得**与硬件接口抢 CAN），用 PS4（F16）替代 StarArm102 leader；打通标定、采集、可视化、回放。
- **验收标准：**
  1. `lerobot-calibrate` 可完成 A3 关节标定，标定文件可跨机复用
  2. PS4 遥操作下 `lerobot-record` 稳定采集 ≥50 episode，`/joint_states` 与相机帧时间同步
  3. `lerobot-dataset-viz`/`lerobot-replay` 可查看与回放数据集（回放经 FJT/编排层落地，受门控约束）
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A0/Step 1；F16（PS4 遥操作）、F21（编排层 AI 模式）、F142（观测帧率）；[B601 LeRobot Wiki](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/)
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
