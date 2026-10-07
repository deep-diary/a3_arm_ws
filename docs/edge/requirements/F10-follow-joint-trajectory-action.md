# F10 — FollowJointTrajectory Action（C1）


- **说明：** 提供 `/arm_controller/follow_joint_trajectory` Action Server，将 goal 转到执行层轨迹话题并回报 feedback/result；尊重 gate 与 `control_mode` 互斥
- **验收标准：**
  1. `ros2 action send_goal` 多点轨迹可成功结束
  2. gate 关闭或 `ZERO_TORQUE`/`SERVO`/`GRAVITY_COMP` 时拒绝 goal
  3. cancel 中止执行
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- **状态：** `implemented`（仿真）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
