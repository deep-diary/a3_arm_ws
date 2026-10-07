# F114 — PS4 一键保存当前位姿为命名点位（L2 短按，保存到包内 named_poses.yaml）


- **说明：** 手柄示教/拖动到目标位后，希望能一键把当前位姿存为命名点位，之后可 `goto` 回去。三处改动：
  1. **`/a3/arm/save_named_pose`（F39）写入目标从 `~/.a3/poses.yaml` 改为包内 `a3_description/config/named_poses.yaml`**（F113 弃用用户层覆盖后的收敛）；写入后同步更新 `self._poses` 运行时即时生效；同名覆盖仍支持。
  2. **时间戳命名**：手柄无法输入文本，`name` 为空时自动命名 `snap_YYYYMMDD_HHMMSS`；服务仍支持显式 `name`（Web/CLI 调用不受限）。
  3. **L2 短按绑定**：`a3_teleop_ps4` 新增 `save_named_pose` action（无参数，调编排层服务，位置留空 = 当前位姿），`default.yaml` 绑定 `l2` 短按；L2 此前预留未绑（LL-052 只提 touchpad 蓝牙无事件，L2 可用）。
- **验收标准：**
  1. 真机/仿真 READY 或示教拖动中按 L2 短按 → 服务返回 `success=true`，`message` 含自动名 `snap_*`；`named_poses.yaml` 追加对应点位且值 = 当前 7 关节读数
  2. 保存后立即 `goto_named_pose {pose_name: <返回的名>}` 可执行（运行时 `_poses` 已合并）
  3. 显式 `name: idle/ready/zero` 调用仍覆盖内置点（与 F39 原语义一致）；`positions` 显式给定时长度≠7 报错
  4. L2 短按绑定不影响 R2 夹爪力控（扳机轴）与其他键
- **关联：** F39（save_named_pose 服务）、F113（点位收敛包内）、F60/F64（PS4 映射）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（arm 服务表）
- **状态：** `implemented`（2026-09-26 落地；真机回归待上电）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
