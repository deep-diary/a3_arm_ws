# F14 — MoveIt Servo（C4）


- **说明：** 接入 `servo_config.yaml`；笛卡尔速度 → 关节轨迹；`control_mode=SERVO`；命令超时停机
- **验收标准：**
  1. Twist 命令引起关节连续变化（仿真）
  2. 超时后停止
- **关联：** CONTROL_ROADMAP C4
- **状态：** `implemented`（`servo.launch.py` + mode bridge；依赖 `moveit_servo`；**真机电机入环见 F65**——2026-09-21 前真机在 SERVO 模式互锁丢弃 servo 轨迹）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
