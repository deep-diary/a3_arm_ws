# LL-133 — R3 软失能已实现但被 BLOCKED_MODES/busy 拒绝是语义缺口：servo 测试完按 R3 无反应

> **日期：** 2026-09-26
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble

## 现象

servo 测试（`moveit_servo` 跑着，`control_mode=SERVO`）完想停，按 R3（失能）**无反应**。`_disable_cb` 对 `mode ∈ BLOCKED_MODES{SERVO, ZERO_TORQUE, GRAVITY_COMP}` 或 busy state（INIT/TEACH/SERVO/AI）直接拒绝。

## 根因

软失能本身已实现（F40/F113：`_disable_cb` → `_safe_park_then_disable` → SAFE_PARK 回 idle → `/a3/motor/reset` → DISABLED），但入口被模式/状态互锁卡在门外——只认识 `READY`（idle 位）这一个入口，用户从任何功能模式按 R3 都被静默拒。这是**语义缺口**：失能被当成「只能从静止态发起」，而操作员预期是「任何模式都能软失能」。

## 正确做法 / 规避

- F119 前序（在 busy/mode 拒绝**之前**）：`STATE_TEACH` → 结束示教（F54 自动存档是接受的副作用）；`SERVO` → `/servo_node/stop_servo`；`ZERO_TORQUE` → `/a3/zero_torque/stop`；`GRAVITY_COMP` → `/a3/gravity_compensation/stop`；等 `_mode` 退出阻塞模式（`disable_mode_exit_timeout_s: 2.0`）再走 F40 safe-park。
- 超时（如操作员仍按着死人开关）→ **可执行拒绝**（「松开手柄再试，或直接 /a3/motor/reset」），不改状态，不静默。
- 不用 `pause_servo`（语义是重锚定测量）；急停（X 长按）/`/a3/motor/reset` 直达链路保持绕过 prelude 即时生效。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py` `_disable_cb`（L2113-2211）
- `docs/edge/REQUIREMENTS.md` F119
- `docs/shared/SAFETY.md`「失能保护（F40）」F119 段

## 附录（2026-09-26 F119 battery 发现）— R3 后重使能的 monitor 时序

- **arm_monitor 在 sim 栈（edge_web_sim / edge_teleop_full_sim）同样生效**：订阅 FJT/servo 轨迹、调真实 `/a3/motor/{stop,reset}`（sim 电机真被关），UNEXPECTED_DISABLE 闩锁（`unexpected_disable_sustain_s: 0.5` < 编排层本地兜底 1.0，LL-039 的持续窗设计）→ TRIGGERED → arm_controller watchdog 消费（`unexpected_disable_guard: true`）→ DISABLED。报告 fire ~0.6 s after 电机失能，恢复 OK ~2.1 s after 进入 DISABLED。
- **潜在真实竞态**：`_safe_park_then_disable` 的 `/a3/motor/reset` 会触发 monitor 闩锁；操作员秒级后重使能，sim 压缩到 ~50 ms——重使能若落在 monitor 报告窗内，watchdog 在 READY 上消费 TRIGGERED → 使能后又 DISABLED（F119 S5 原崩路径）。真机手速不会踩到，但**自动化快速 repeat R3→enable 会**；验收用场景间 3.0 s settle（闩锁在「已 DISABLED」期排空）规避。若要在编排层加「R3 后短冷却」需评估对阵式重使能（急停恢复）的延迟成本，见 F120 议题。