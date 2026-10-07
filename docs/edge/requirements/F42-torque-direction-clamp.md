# F42 — 力矩方向钳位（执行层 latch，碰撞保护）


- **说明：** 执行层（`motor_protocol_node`，权威，插在 ClampMotorCommand 之后）按**方向性判据**做碰撞保护：反馈力矩 |τ| ≥ 阈值（`torque_protection_limit_nm: [5,5,5,3,3,3,3]`，RS00 5 / EL05 3，与 LL-024 codec 量程同源）→ trip 并 latch τ 符号；冻结 = 把目标钉在反馈位（τ>0 → target=min(target, fb)；τ<0 → target=max(target, fb)，MIT 约定 τ≈kp×(target−actual)）；释放 = (mapped−fb)×latch_sign < 0 且 |mapped−fb| > `release_margin`(0.02 rad)——即「增矩方向冻结、反向放行」（无 latch 会形成 0→3 Nm 周期极限环）。kp≤0.01（zero_torque/播种）、反馈不新鲜时清 latch 不钳位；**收到新轨迹时清空全部 latch**（新轨迹 = 新意图；保护不减弱——阻力仍在时钳位会在一个 tick 内按反馈力矩重新 trip）。WARN 日志关键字 `F42 torque clamp trip: motor=%u tau=%.2f limit=%.2f fb=%.4f`（1000 ms 节流）。与 refresh 流天然兼容：refresh 持有钳位后的 `last_commanded_mit_rad_` → 轨迹结束后自动保持冻结位；zero_torque/stop 重锚定不冲突。不做「持续超限 → FAULT」升级（执行层无状态机；编排层可观测 `mtq_L{n}` 扩展）。
- **验收标准：**
  1. 手扶顶住关节（反馈力矩 ≥ 阈值）时该关节目标冻结在反馈位、不再朝阻力方向推进；反向目标放行
  2. 阻力消失后关节继续跟踪轨迹；新轨迹不受残留 latch 影响
  3. kp≤0.01（zero_torque 拖动）或反馈不新鲜时不钳位
- **关联：** [shared/SAFETY.md](../shared/SAFETY.md)（力矩方向钳位）；LL-024（阈值按型号）；固件 0x700B 硬钳兜底
- **状态：** `completed`（2026-09-13 真机验收：L4 手扶受控 trip −3.01 Nm 冻结在反馈位、力矩塌陷后释放继续走完；L6 自然 trip −3.01 同语义（WARN 日志两条与 ~/.a3/stats/torque_stats.yaml 时间戳吻合）。**期间发现并修复 LL-026 缺陷**——慢速跟踪滞后仅 ~0.006 rad < release margin 0.02，残留 latch 把新轨迹钉死（回程 L4/L6 各只动 ~0.03 rad 即停、日志无新 trip）→ 新轨迹清 latch 修复后同一回程完整走完（L4→0.1605、L6→+0.0002））


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
