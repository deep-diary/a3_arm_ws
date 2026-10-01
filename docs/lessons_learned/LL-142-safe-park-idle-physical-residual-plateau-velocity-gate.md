# LL-142 — R3 safe-park 到 idle 偶发超时 FAULT：物理残差 + plateau 速度门误用严格 settle_vel

> **日期：** 2026-10-01  
> **产品线：** Edge  
> **环境：** RK3588 LubanCat + ROS 2 Humble，idle=[≈0,0,0,0.335,0.012,0,0]（竖直收拢）

## 现象

home 位按 R3 回 idle，MoveIt 报告执行成功后紫色灯持续 ~10–12s，随后灯带红白闪烁、
臂**未失能**（state=FAULT: safe park timeout: not settled at idle, still enabled），
再按一次 R3 才失能。行为不稳定：有时超时 FAULT，有时又侥幸成功。

## 根因（两层，均由段内插桩实测确认）

1. **idle 在真机物理不可达**：idle 要求 L2/L3≈0，但自然下垂 + 机械止挡使这两个关节
   存在 **~0.03 rad（~1.8°）残差**。MoveIt 轨迹 + 一次自动纠偏（corrective）都压不到
   `disable_home_settle_tol_rad=0.02` 内——err 全程恒定 0.030–0.035，是物理受限，非控制问题。
2. **plateau 速度门阈值错误（偶发的直接原因）**：为"物理够不到"设计的稳态无进展判定，
   其"静止"条件借用了严格的 `settle_vel=0.05`。真机 idle 保持时速度反馈噪声/控制器微调
   常在 **0.05–0.12** 跳动 → plateau 计时（需连续 0.8s）被反复清零十几次：
   - 噪声恰好连续低 0.8s → 成功失能（但 ~11s）
   - 凑不够 → 超时 FAULT。这就是"时好时坏"。

## 正确做法 / 规避

1. **反馈噪声阈值与严格落定阈值必须分开**：新增 `disable_park_plateau_vel_rad_s=0.15`
   专给 plateau 用（容忍 idle 保持噪声；真正运动速度远大于此），严格 `settle_vel=0.05`
   仍用于正常到位判定。修复后紫灯 11s → **~4.2s**，两次真机稳定失能、不再偶发。
2. plateau 完整条件（物理受限才放行，非放松安全）：已发纠偏 + err≤0.08（硬地板 0.12）
   + speed≤0.15 + 持续 0.8s 位置无改善（≤0.01）。残差超硬地板或仍在运动 → 仍 FAULT 保持使能。
3. 复用 LL-140 方法：先段内插桩每 0.25s 记录 err/speed/plateau 计时，用数据区分
   "物理够不到"与"判据太严"，不要只改容差或臆断。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_safe_park_then_disable` plateau 段）
- `src/a3_description/config/named_poses.yaml`（idle 点）
- `docs/edge/REQUIREMENTS.md` F130；`docs/shared/SAFETY.md`（残差失能口径）
