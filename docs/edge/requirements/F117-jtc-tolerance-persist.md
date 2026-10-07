# F117 — JTC 跟踪容差 0.30/0.05 临时定型（持久化实测热设组合；语义=卡滞检测非防撞）


- **说明：** 真机事件（2026-09-25 goto 回 idle 中止）：JTC 报 `Position Error: 0.3073 > 0.300000` 触发 `PATH_TOLERANCE_VIOLATED`。三项独立 bag 证据（controller_state / 原始 /joint_states / error==恰好指令距离）表明**根因不是容差过窄，而是 CAN 反馈降级导致机械臂零跟踪**：/joint_states 掉到 ~12.3 Hz（标称 50）、max gap 0.392 s、491 s 内 1255 个 >200 ms 空洞，末尾 `CAN write failed: Resource temporarily unavailable`。0.30/0.05 只是运行时 `ros2 param set` 的 hot-set，**未持久化，重启即回退** 0.15/0.03。本需求把实测跑过的组合 `trajectory: 0.30 / goal: 0.05` 写入 `el_a3_controllers.yaml`（L1–L6）。**语义澄清：JTC 轨迹容差是卡滞/零跟踪检测器，不是防撞**——防撞靠 URDF 限位 + MoveIt 碰撞 + `safety_limits.py` 限速 ± 电源门禁 + F107 力矩门禁 + F110/F48 通道门禁；0.30 仍 << F81 卡滞界 0.5 rad，零跟踪下 0.30 也会中止（error 会冲到关节折叠幅）。**TEMPORARY**：回收紧 trigger = CAN 健康恢复后重跑 f117 验收在 0.15 配置下全绿。
- **验收标准：**
  1. `el_a3_controllers.yaml` 六个 arm joint（L1–L6）加载生效 `trajectory: 0.30 / goal: 0.05`（重启级持久，非 param set）；注释块标明 TEMPORARY 原因、非防撞语义、回收紧条件
  2. vcan `f117_jtc_tolerance_deg_acceptance.py`（`--alpha 0.0088` 注入滞后 ≈0.20 rad ∈ (0.15, 0.30)）：idle↔ready 双向 FJT error 0，无 `PATH_TOLERANCE_VIOLATED`
  3. 同 alpha 下 hot-set `trajectory: 0.15` → FJT 中止（`PATH_TOLERANCE_VIOLATED`）——证明配置生效、0.30 是覆盖降级滞后区的必要放宽
  4. `--alpha 0.002`（插值跟随滞后 ≈0.9 rad）→ 0.30 下也中止——证明 0.30 不是无限容差
- **关联：** F97/F112（容差配置演进）、F81（卡滞看门狗）、F110/F48（通道门禁）、F107（力矩门禁）；[LL-131](../lessons_learned/LL-131-jtc-tolerance-not-anti-collision.md)、[SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（2026-09-26 yaml 持久化 + 仿真验收；CAN 反馈根因另行持续跟踪）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
