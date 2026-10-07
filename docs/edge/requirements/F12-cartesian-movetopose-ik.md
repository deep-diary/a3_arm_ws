# F12 — 笛卡尔 MoveToPose / IK


- **说明：** Pinocchio IK 服务 `/a3/move_to_pose_ik`；可选 Action `/a3/move_to_pose` 解算后经 FJT/轨迹执行
- **验收标准：**
  1. 给定可达位姿返回关节解
  2. 无解/奇异返回失败
  3. Action 路径可驱动仿真到位
- **关联：** CONTROL_ROADMAP L3/C1
- **状态：** `implemented`（仿真）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
