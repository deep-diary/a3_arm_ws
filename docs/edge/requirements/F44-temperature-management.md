# F44 — 温度管理（warn / protect / COOLING）


- **说明：** 参数 `temp_protect_enabled: true`、`temp_warn_c: 90.0`、`temp_protect_c: 95.0`（2026-09-13 由 65 上调——官方电机自带 130°C 兜底，65 使 ready 位保位发热几分钟即误触，LL-023）、`temp_hysteresis_c: 5.0`。warn（≥90，fresh 门控）：置 `temp_warn` + WARN 日志 + `arm_temp_warn` 遥测，不动状态机。protect（≥95 任一 fresh 关节）：READY/TRAJ → 复用 F40 流程 safe park → **COOLING**（message 带关节与温度）；IDLE/DISABLED/COOLING → 直接 COOLING；SAFE_PARK 进行中不打断；reset 被 gate 拒 → **FAULT**(overtemp reset refused)（温度保护不可放弃）。COOLING 下全 fresh 关节 < protect−hysteresis 时**自动转 DISABLED**（实时反映已降温、灯效红闪→橙）。重新使能：COOLING 下 `_enable_cb`/`_init_cb` 先查全 fresh 关节 < protect−hysteresis 才放行；无 fresh 关节不阻碍 + WARN。顺带电机故障监视：fault_mask≠0（含固件过温锁存 bit3）→ reset 广播 + FAULT。`ArmStatus.msg` 追加 `float64[] temperatures`、`bool temp_warn`；MQTT scalar rule fields 扩展 `temp_warn` → `arm_temp_warn`。
- **验收标准：**
  1. 超保护阈 → 自动回 home → 失能 → COOLING；降温至保护阈−迟滞前 enable 被拒
  2. warn 级仅告警不打断运动
  3. 无反馈（fresh=false）时温度判读不生效——温度=0.0 不是 NaN，断连不得被误判「已冷却」放行使能（LL-011 教训）
- **关联：** F40（复用 safe park）、F45（COOLING 态）；[shared/SAFETY.md](../shared/SAFETY.md)（温度策略）；LL-023（阈值放宽）
- **状态：** `completed`（2026-09-13 保护路径真机触发并复现；2026-10-05 增补：COOLING 降温后自动转 DISABLED（灯效红闪→橙）+ COOLING 边沿触发黑匣子，真机验证通过）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
