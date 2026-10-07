# F66 — 使能意图安全（gate 关作废全部意图 / 任意使能沿强制重锚 / 恢复死锁解锁）


- **说明：** 2026-09-22 真机第二次甩断 L6/L7（LL-039 同类复发）：X 长按硬急停由 `power_sequence_node` 直发裸 CAN disable，L3 上电由 EnableInit 直发裸 CAN enable（`SendCmdAll(0x03)`），**两条路径都绕过 `/a3/motor/enable` 服务**——F51 的「新鲜反馈重锚 + 软起步」只挂在服务路径；且真机三份 `control_gains.yaml` 把 `enable_mode_rising_smoothing` 置 false。人工搬臂回 home 后裸使能，执行层 200 Hz refresh 立刻以陈旧目标（事故值 L2=1.0839 rad）+ kp=80 续发，臂被甩回失能前位姿。其后看门狗在使能逐电机传播窗口内误报 UNEXPECTED_DISABLE（READY 后 19 ms），编排层跳 DISABLED 而 gate 仍 Running、电机仍锁存使能（LL-042），再按 L3 的 enable 服务又被 F32 拒绝 → 橙灯死锁。修复：
  1. **gate 关闭沿 = 全部运动意图作废**（`motor_protocol_node::OnPowerGate` 新增 `InvalidateAllMotionIntent()`）：清活动插值轨迹、servo 单点缓存、全部 7 电机 `last_commanded_mit_rad_`→NaN、置 `hold_suppressed_`、`has_latest_input_`=false（LL-040 回退表同源副本）、清 enable ramp；gate 关期间 refresh 只发零增益保活帧。
  2. **使能模式上升沿无条件 F51 重锚**（反馈处理，0/未知→使能态）：以当前新鲜反馈重锚目标、清回退缓存、重置平滑锚点、启动 kp/kd 软起步；陈旧目标偏差超 `enable_reanchor_tolerance_rad`(0.15) 打 ERROR。不再受 `enable_mode_rising_smoothing` 开关控制，服务使能与裸 CAN 使能同等保护；桥启动时电机已锁存 mode=2（LL-042）同样在首帧反馈沿重锚。
  3. **refresh 防甩兜底**（新参数 `stale_target_snap_guard_rad: 0.25`）：无活动轨迹/servo、电机使能反馈新鲜，但有限目标与实测偏差 >0.25 rad 且无软起步在身 → 拒绝拉拽，重锚实测位并启动软起步（正常保持残差 <0.02 rad）。
  4. **F32 恢复通道**：gate Running + `control_mode=IDLE` + 无活动轨迹时允许 `/a3/motor/enable`（command=1，走完整 F51 重锚+软起步），解锁「编排层 DISABLED/橙灯 但电源序列仍 Running」的 L3 恢复；reset/set_zero/save_param 仍严格拒绝。
  5. **电源序列使能帧补发**：EnableInit 保持期内每 50 ms 重发 0x03（幂等），消除与 EPScan 参数写同窗口竞争导致的漏帧/逐电机使能空洞。
  6. **看门狗使能建立宽限**：`none/partial→all` 使能沿（partial 也认）后 `hold_rebaseline_grace_s`(2.0 s) 窗口内豁免 UNEXPECTED_DISABLE；真实失能在宽限 + 0.5 s sustain 后仍必捕获。
- **验收标准：**
  1. `scripts/a3_test/f66_gate_enable_regression.py`（真 motor_protocol_node + mock CAN，精确复刻事故时序：旧位姿建目标→gate 关+裸失能→搬回 home→带外裸使能→gate 重开）通过：重开后所有 MIT 帧目标 ≤0.10 rad 贴 home、kp 从 0 软起步、mock 关节不被甩向旧位姿；旧二进制同脚本必失败（已验证：目标 1.090、首帧 kp=80、mock 被甩到 1.090 = 真阳性）
  2. `./scripts/a3_test/a3_test.sh incident` 全套通过（F51/F66/F50 看门狗/F52）
  3. PS4 全新栈 F62 46/0（含 S9「gate 仍开 L3 恢复 READY」）
  4. **真机待验（L6/L7 机械修复后）**：X 失能→人工挪臂→L3，臂不跳回旧位姿，电机日志每个电机出现一条 `F66 enable rising edge ... 重锚`；gate 关日志出现 `F66 motion intent invalidated`；看门狗不再在使能瞬间报 UNEXPECTED_DISABLE
- **关联：** F51（使能安全三要素，本需求把覆盖范围从服务路径扩到全部使能路径）、F32（gate 互锁恢复语义）、F60/F65（L3 幂等）、F58（stop 重力保持语义，回归判据同步更新）；[shared/SAFETY.md](../shared/SAFETY.md)；LL-039（首起甩臂事故）、LL-040（回退表缓存）、LL-042（电机锁存最后命令）、LL-043（启动禁满增益）、LL-070（本次事故，真机复测后补录）
- **状态：** `implemented-pending-hw`（2026-09-22 代码 + 编译 + mock-CAN 真桥回归 + 46/0 仿真；待 L6/L7 机械修复后真机复测）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
