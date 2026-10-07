# F29 — 互锁模式回收（/a3/control_mode 发布缺陷修复，真机实测）


- **说明：** 真机 2026-09-06 实测发现：`a3_arm_controller` 只在轨迹开始时发布 `TRAJ_RUNNING`，轨迹结束（`_back_to_ready`）与节点启动均不发布非阻塞模式；`/a3/control_mode` 为 VOLATILE 事件型话题，无周期性兜底 → 夹爪力控互锁（F25/F26 的 `GRIPPER_FORCE` 互锁表）在真机跑过任意轨迹后**永久锁存 `TRAJ_RUNNING`**，力控指令永远被拒（仿真闭环不经过该节点，故 F28 未暴露）。修复：① 轨迹结束发布 `READY`；② 节点启动即发布 `IDLE`，并 2 s 后重发一次（启动首条发布可能早于订阅发现、被静默丢弃）。
- **验收标准：**
  1. 真机跑完任意轨迹（`goto` / `set_joint_positions` / `playback`）后，立即发 `/a3/gripper/command {mode: force}` 成功（无需重启任何节点）
  2. `a3_arm_controller` 重启后，无需其他操作，夹爪力控指令成功（2 s 重发覆盖发现窗口）
  3. 夹爪互锁恢复全程无人工干预，仿真闭环回归不退化
- **关联：** F25（力控）、F26（互锁表）、[shared/SAFETY.md](../shared/SAFETY.md)（`GRIPPER_FORCE` 互锁）；[LL-009](../../lessons_learned/LL-009-arm-control-mode-no-release.md)
- **状态：** `implemented`（2026-09-06 真机泡棉力控测试验证：跑完 set_joints 轨迹后力控 0.3 Nm 直接成功）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
