# F15 — JTC 兼容样条插值（C2 增强）


- **说明：** 执行层按航点字段自动选用线性 / 三次 Hermite / 五次样条（对齐 ros2_control JTC）；`effort` 始终线性；参数 `trajectory_interpolation_method: auto|linear|cubic|quintic`
- **验收标准：**
  1. 仅 `positions` 轨迹行为与线性回归一致；Wave A 脚本仍 PASS
  2. 带非零 `velocities` 时选用三次，航点处速度连续
  3. 带 `accelerations` 时选用五次
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C2
- **状态：** `implemented`（仿真；真机板测中）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
