# F40 — 失能保护（disable → 自动回 idle（原 home，F113）→ 失能）


- **说明：** `/a3/arm/disable` 不再是「无条件直接失能」——不在 idle（原 home，F113 改名、点位值不变）容差内时先平滑回 idle 再失能，防止 ready 位直接掉臂。新增参数：`disable_home_pose_name: "idle"`（F113 起，原 `"home"`）、`disable_home_tol_rad: 0.15`、`disable_home_duration_s: 3.0`、`disable_home_confirm_s: 0.5`、`disable_park_timeout_s: 8.0`。新辅助 `_at_home()`（全关节 |q−home| ≤ tol）与 `_safe_park_then_disable()`（同步阻塞：发布 home 轨迹抢占活跃轨迹——执行层 OnTrajectory 天然支持替换，无需排队 → `SAFE_PARK`（期间拒绝新运动指令）→ 轮询连续 `confirm_s` 收敛 → reset → `DISABLED`）。服务语义（同步阻塞返回，`success=true ⟺ 已失能`）：READY/TRAJ 容差内 → 直达 reset → DISABLED；容差外 → safe park → reset → DISABLED（message 含耗时）；IDLE → 直达 reset；DISABLED/COOLING → 幂等不动电机；FAULT → 紧急直达 reset；INIT/TEACH/SERVO/AI → 拒绝 busy；SAFE_PARK → 拒绝「already safe parking」。park 超时 → **FAULT 不 reset**（保持使能、停在半途，人工介入）；reset 被 gate 拒 → park 前 `success=false` + 原文 + "(stop power sequence first)"，park 完成后被拒 → 回 READY（已在 home 位，安全）；`_have_js==False` → 直达 reset + WARN（保持旧行为）。`/a3/motor/reset` 直达保留作紧急失能。
- **验收标准：**
  1. 容差外 disable：`READY → SAFE_PARK`（`_traj_done_at` 清零防旧 TRAJ 时间戳误回 READY）→ 收敛连续 0.5 s → reset → `DISABLED`，最终位姿全部在 home ±0.15 rad 内
  2. 容差内 disable：跳过 park 直达 reset；park 超时：FAULT 且电机保持使能、不 reset
  3. disable 后 7 电机 mode_status=0；DISABLED 下运动命令被拒（`_can_move` 拒绝表，F45）
- **关联：** F45（SAFE_PARK/DISABLED 状态）、F41（park 轨迹复用统一插值）；[shared/SAFETY.md](../shared/SAFETY.md)（失能保护）；[LL-026](../../lessons_learned/LL-026-txstats-window-and-torque-latch-release.md)（park 被残留力矩 latch 钉死风险）
- **状态：** `completed`（2026-09-13 真机验收：READY 位 disable → `[state change] READY -> SAFE_PARK (safe park -> home (3.0s, 150 pts))` → `disable -> success=True 'safe park -> disabled (0.9s)'` → `SAFE_PARK -> DISABLED`；最终位姿 [-0.0044, -0.0006, -0.0171, +0.3336, +0.0125, +0.0056, -0.0002] 全 ≈ home、7/7 失能。实测 0.9 s 即返回——tol 判据在 park 中途即满足、提前 reset、重力把臂荡回平衡位，符合设计）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
