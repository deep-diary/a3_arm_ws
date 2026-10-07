# F130 — R3 safe-park 容忍 idle 物理残差：稳态无进展判定（独立噪声速度门）


- **说明：** 用户真机反馈：home 按 R3 回 idle，MoveIt 执行成功后紫灯持续 ~10–12s → 红白闪烁 FAULT、未失能，需再按 R3；且时好时坏。段内插桩实测根因两层：①idle 要求 L2/L3≈0，真机自然下垂 + 机械止挡致两关节存在 **~0.03 rad（~1.8°）物理残差**，MoveIt 轨迹与一次自动纠偏均压不到 `disable_home_settle_tol_rad=0.02`（err 全程恒 0.030–0.035）；②初版稳态判定的静止条件误用严格 `settle_vel=0.05`，idle 保持时速度反馈噪声/微调常达 0.05–0.12，使需连续 0.8s 的 plateau 计时反复清零——噪声恰好连续低则成功（~11s），凑不够则超时。修复：新增独立 `disable_park_plateau_vel_rad_s=0.15`（容忍噪声，真正运动速度远大于此）；plateau 完整条件=已发纠偏 + err≤`disable_park_residual_tol_rad=0.08`（且≤硬地板 `disable_park_residual_hard_floor_rad=0.12`）+ speed≤0.15 + 持续 `disable_park_plateau_s=0.8` 位置改善≤`disable_park_no_progress_rad=0.01`。残差超硬地板或仍在运动 → 维持原 FAULT 保持使能，不放松安全。
- **验收标准：**
  1. 真机 home→R3：紫灯 **~4s**（MoveIt 1s + 纠偏后 ~0.8s plateau）后直接失能，无红白闪烁、无需二次 R3，连续 2 次稳定
  2. 残差失能日志点名 worst 关节与 err（`idle physically unreachable, settle at residual err=… worst=…`）
  3. 残差 >0.12 rad（明显未到位/被严重遮挡）或仍在运动时依旧 FAULT 保持使能，不强行失能
  4. 正常到位（err≤0.02）路径零回归，严格 settle 判据不变
- **关联：** F40（失能保护）、F113（home→idle）、F75（位置/速度双落定 LL-077）、F107（门禁）、[SAFETY.md](../shared/SAFETY.md)（残差失能口径）
- **状态：** `implemented`（2026-10-01 真机 2 次验收通过，~4.2s 失能 err=0.033/0.039 worst=L3）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
