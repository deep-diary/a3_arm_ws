# F186 — 反馈 stale 时「冻结保持」而不是软下垂（恢复无回弹冲击）

- **说明：** F81 的 freeze-hold 在真机上表现成「**整臂发软、带阻尼下垂**」，通讯恢复后
  **瞬间弹回原位**，冲击很大（2026-10-24 L6 接头接触不良反复复现，最终导致断电）。

  根因在 `write()` 的 stale 分支：

  | | 旧实现 | 后果 |
  |---|---|---|
  | 刚度 | 全局 `kp_`/`kd_`（默认 80/2） | 与每关节 kp（100~150）不一致，弱 |
  | 前馈 | **零**（`t_ff = 0`） | 重力无人抵消 → 稳态下垂 `τ_g/kp`（数度） |
  | 目标 | 每拍取**实测**位姿 | 命令跟着下垂走，永远追不上 |

  而恢复后正常路径用「每关节 kp + 重力前馈 + 控制器旧目标」→ 把下垂的那几度一次性拉回，
  就是用户感受到的"弹回"。

- **验收标准：**
  1. stale 期间**仍在发控制帧**（不静默、不丢帧），且每关节 kp/kd 与健康期**完全一致**（不退化成软保持）
  2. stale 期间命令位姿**锁存不变**（故障边沿的控制器命令），力矩位**仍带重力前馈**
  3. 反馈恢复后 kp 在 `freeze_recover_ramp_s`（默认 0.4 s）内从 0 斜坡回额定 → 真正被重力带走过一段时是**平滑拉回**
  4. `freeze_recover_ramp_s=0` 可关闭斜坡；参数非法值 clamp 到 [0, 3] s

- **实现：**
  - `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`
    - stale 分支不再自行拼帧，而是**锁存** `joints_[i].cmd_pos/cmd_eff` 后**落到正常帧构建路径**
      （因此自动获得每关节 kp/kd、重力/摩擦前馈、effort/position 分模式行为）
    - 新增 `hold_active_/hold_pos_/hold_eff_`、`recover_ramp_*`；`write()` 的 `period` 用于把
      `freeze_recover_ramp_s` 换算成拍数
    - 新硬件参数 `freeze_recover_ramp_s`（默认 0.4）
  - 验收脚本：`scripts/a3_test/f186_freeze_hold_acceptance.py`（vcan + `vcan_motor_sim.py --silence-file`）

- **vcan 实测（2026-10-24，vcan9 + 模拟器，arm 断电）：**

  | 判据 | 结果 |
  |---|---|
  | 健康期 6 台控制帧齐备 | PASS（kp=100 每关节一致）|
  | stale 后仍持续下发 | PASS（6/6）|
  | stale 后 kp 与健康期一致 | PASS（100.0 vs 100.0）|
  | stale 后位置字段冻结 | PASS（max Δ = 0.0000）|
  | stale 后力矩位仍带前馈 | PASS（m3 = 37278，中性 32767）|
  | 恢复后 kp 回额定 | PASS |
  | 恢复斜坡中间值 | 采到 241 个（0 < kp < 99）|

  注意：vcan 模拟器在位置模式下不建模重力，因此**"不下垂"这一条只能靠真机复核**；
  本脚本证明的是"保持帧的刚度/前馈/目标与健康期一致"这一充分条件。

- **已知边界：**
  - 总线**整段**失效（线断/接口 down）时任何软件都发不出帧，电机按自身超时走下坡；
    本需求只保证"恢复时平滑拉回"，不保证"断线期间不下垂"。
  - `arm_controller` 在 stale→fresh 边沿只清门禁，JTC 的目标仍是故障前那一点；
    若期间确实下垂较多，靠 kp 斜坡平滑拉回（而不是重整目标）。

- **关联：** F81（单电机反馈断线保护）、F83（启动软起步）、[LL-184](../../lessons_learned/LL-184-freeze-hold-soft-droop-and-snap-back.md)、
  `docs/shared/SAFETY.md`（F81 条款 3）

- **状态：** `implemented`（2026-10-24；vcan 全判据 PASS，真机"不下垂"待上电复核）
