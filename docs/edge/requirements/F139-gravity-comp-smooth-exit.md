# F139 — 重力补偿平滑退出（MIT ramp-out，消除模式切换 clack/jerk）【P0】


- **说明：** 退出重力补偿/自由拖动、与位置模式交接时，MIT `kp/kd/tau` 不做瞬时切换，按可配时长（如 0.3–0.5 s）斜坡退出并同步重锚当前位姿，消除「咔哒」声与冲击。对标社区 fork `rebotarm_monitor_ros2` 的 gravity compensation smooth stop；补齐 C3/F89b 遗留的「退出 ramp-out」真机待办。
- **验收标准：**
  1. 重力补偿/自由拖动 → 位置模式交接过程中关节无可见阶跃、无 clack 声响（真机）
  2. ramp 时长可配；ramp 中途触发急停时立即失能，不等待斜坡走完
  3. vcan 仿真可断言切换前后指令增益/力矩连续（逐 tick 变化量有界）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C3；F89b（STRICT 控制器切换）；F66（使能沿重锚/软起步）；[rebotarm_monitor_ros2](https://github.com/danieldoradotalaveron-rb/rebotarm_monitor_ros2)
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
