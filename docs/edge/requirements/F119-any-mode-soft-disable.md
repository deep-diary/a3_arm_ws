# F119 — 任意模式软失能（R3：先退出当前模式 → 回 idle → 保电失能）


- **说明：** R3 语义本应是软失能（先回 idle 再失能），代码已实现 `_disable_cb` → `_safe_park_then_disable`（F40/F113）。**真实缺口**：`_mode ∈ BLOCKED_MODES{SERVO, ZERO_TORQUE, GRAVITY_COMP}` 或 busy state ∈ {TEACH, SERVO, AI} 时 `_disable_cb` 直接拒绝——servo/示教测试完按 R3 无反应。本需求把失能前置为**先安全退出锁定模式**：`STATE_TEACH` → 复用 `_stop_teach_cb`（退出 ZERO_TORQUE，F54 自动存档为接受副作用）；`mode ∈ BLOCKED_MODES` → SERVO→`/servo_node/stop_servo`（已核验真实 Trigger server）/ ZERO_TORQUE→`/a3/zero_torque/stop` / GRAVITY_COMP→`/a3/gravity_compensation/stop`；随后等待 `_mode` 离开 BLOCKED_MODES 至多 `disable_mode_exit_timeout_s=2.0`（servo 桥 0.5 s 后回 IDLE），超时给出可执行拒绝文案（不改状态守护安全）。busy 拒绝集由 `{INIT, TEACH, SERVO, AI}` 缩为 `{INIT, AI}`，`BLOCKED_MODES` 拒绝保留为防御断言。**不用** `/servo_node/pause_servo`（语义=重锚定测量，非停 servo）。
- **验收标准：**
  1. READY 下 disable → 状态序含 SAFE_PARK（回 idle）→ DISABLED 终态、无错
  2. SERVO 下（start_servo + twist 后）disable → `_mode` 在 2 s 内出 SERVO → SAFE_PARK → DISABLED；重使能后 start_servo 仍可用
  3. TEACH 下 disable → TEACH 退出 + mode→IDLE → SAFE_PARK → DISABLED；F54 自动存档触发
  4. 重力补偿下 disable → 出 GRAVITY_COMP → SAFE_PARK → DISABLED
  5. 负例：持续按 twist 使 SERVO 不消 → 2 s 超时拒绝 + 可执行文案 + 状态机不崩
- **关联：** F40/LL-077（失能语义）、F12（desired 保持）、F54（示教自动存档）、F53（组合拒绝语义）、F113（idle 点位）；[SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（2026-09-26 仿真验收；真机回归待上电）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
