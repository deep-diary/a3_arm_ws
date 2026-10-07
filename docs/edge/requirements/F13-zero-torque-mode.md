# F13 — 零力矩模式（C5）


- **说明：** `/a3/zero_torque/start|stop`；MIT `kp` 极小 + 可配 `kd` + 重力 FF；`control_mode=ZERO_TORQUE`；与轨迹/Servo 互斥
- **验收标准：**
  1. 模式可脚本切换并恢复增益
  2. 进入时拒绝新轨迹 Action
- **关联：** [shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（motor_protocol 服务；真机手感板测中）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
