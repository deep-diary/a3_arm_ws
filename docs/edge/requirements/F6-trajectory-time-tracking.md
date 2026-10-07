# F6 — 轨迹时间跟踪（C2）


- **说明：** 执行层接收多点 `JointTrajectory` 后，按各点 `time_from_start` 插值，再向电机/仿真下发；禁止仅取 `points.front()` 作为整轨目标
- **验收标准：**
  1. 含 ≥2 个航点、时长约 2 s 的轨迹，中间采样时刻关节角位于首末点之间（非阶跃到首点）
  2. 轨迹结束后稳定在末点姿态
  3. `enable_power_sequence_gate` 为真且 gate 关闭时拒绝执行
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C2；CloudEdge mock/ESP32 插值语义对齐
- **状态：** `implemented`（仿真验收，见 [WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
