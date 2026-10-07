# F33 — 夹爪力控真机行为修正（接触位移门 / 默认超时 / 释放柔顺 / 零力矩直开 / 力环目标位置遥测）


- **说明：** 用户真机实测（2026-09-06，泡棉）发现 4 个问题并逐一修正：
  1. **目标 0.3 Nm 实际仅 0.13 Nm、卡在 42% 开合**：两个叠加根因——(a) 0 位硬止位静置力矩 ≈0.16 Nm ≥ 接触阈值 max(0.1, 0.35×0.3)=0.105，力控起步即误判「已接触」，跳过恒速软闭合段，PI 从 e≈0.14 缓慢爬升；(b) web 力控按键不传 timeout，落到默认 `grasp_timeout_s: 5.0`，「grasp timeout > 5.0s」FAULT 冻结力环（LL-013）。修正：新增 `contact_min_travel_rad: 0.1` 接触判定位移门（离开力控起点 ≥0.1 rad 后力矩阈值判定才生效）；默认 `grasp_timeout_s` 5.0→15.0。
  2. **释放过快**（旧 1.08→0 rad 用位置增益 80/2 + 0.8s 轨迹）：新增释放柔顺参数 `release_duration_s: 1.5`、`release_kp: 30.0`、`release_kd: 5.0`，释放轨迹用专用低增益 + 较高阻尼，轨迹结束 +0.3s 后一次性 timer 自动恢复位置增益 80/2（重复释放先取消旧 timer）。
  3. **缺零力矩硬逻辑**：force 模式 `torque_nm<=0 且 preset 为空` → 直接 `_release()` 全开，跳过 PI 与力矩校验（返回 `"force 0 -> full open"`）；`_tick_force` 加安全网：`_target_torque<=0` 立即转释放。注意：此改动使「force 无参数用默认 0.6 Nm」路径失效（web 永远显式传 torque>0），按用户规格接受。
  4. **力控期间目标位置遥测**：`_tick_force` 每 tick 把 PI 输出更新到 `target_position`（`_target_pos_commanded=True`），前端力控期间可观察目标位置逐步变大的趋势。
- **验收标准：**
  1. sim 闭环（`./scripts/a3_test/a3_test.sh gripper`）回归全 PASS（该套件自身用 `-p grasp_timeout_s:=6.0` 覆盖，不受 15s 默认影响）
  2. 真机（泡棉）：目标 0.3 Nm 时力环持续推进，实际力矩稳定在 0.3±10% 且 GRASPED；0.5 Nm 同样
  3. 真机：force 0 → 3s 内夹爪回 0 位（归一化 position→1.0），状态非 FAULT，释放无冲击（峰值速度明显低于旧版）
  4. 释放后 gains 在 `release_duration_s + 0.3s` 内恢复 position_mode_kp/kd
  5. 力控期间 `gripper_status.target_position` 随 PI 输出实时变化（不再停留命令陈旧值）
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（GripperStatus 字段语义）；[lessons_learned/LL-013](../lessons_learned/LL-013-gripper-hardstop-torque-false-contact.md)；F24/F25/F26（力控基线）、F31（target_position 字段）、F34（web 路径阶梯验收）
- **状态：** `completed`（2026-09-06 真机 F34 阶梯全 PASS：0.3Nm actual=0.281、0.5Nm actual=0.508 均 GRASPED；力控期间 target_position 随 PI 输出逐步变大；force 0 硬逻辑 3s 内回 0 位无 FAULT；释放柔顺生效。重抓超硬限问题经位移门基准改 q_open 修复；期间暴露的 F32 插值流缺陷由电机调试会话修复（LL-014/LL-015））


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
