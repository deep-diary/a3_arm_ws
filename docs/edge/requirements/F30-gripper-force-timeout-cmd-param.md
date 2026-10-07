# F30 — 夹爪力控超时以命令参数为准（真机实测）


- **说明：** 真机 2026-09-06 实测发现：`GripperCommand.timeout_s` 传入 `_start_force` 后被丢弃，`_tick_force` 恒用配置 `grasp_timeout_s=5.0` → 软物体（泡棉）0.3 Nm 力环尚未收敛即误报 `ERR_GRASP_TIMEOUT` 进 FAULT。修复：每次夹取把命令 `timeout_s` 存入 `_force_timeout_s`（缺省/非正回落配置值），超时判断改用它。
- **验收标准：**
  1. 真机泡棉 0.3/0.5/0.7/1.0 Nm 四档均在 `timeout_s: 12.0` 内 `GRASPED`，稳态力矩在目标 ±10% 内
  2. 不传 `timeout_s` 时回落配置 `grasp_timeout_s`（5.0 s）行为不变
  3. 超时进 FAULT 后，下一次指令（release/force）清除错误码（F25 原语义不变）
- **关联：** F25（力环）、F28（真机回归）；[LL-010](../../lessons_learned/LL-010-gripper-force-timeout-ignored.md)
- **状态：** `implemented`（2026-09-06 真机泡棉四档力控实测通过）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
