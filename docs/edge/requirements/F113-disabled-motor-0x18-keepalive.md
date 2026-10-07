# F113 — 失能态电机 0x18 主动上报保活（R3 后 RViz 实时 + L3 立即使能；修复 reset 后总线静默）


- **说明：** 真机复盘（2026-09-26，`/tmp/a3_real_stack2.log` + `/tmp/a3_arm_monitor.log`）：R3 失能 → 插件 `on_deactivate` 发 BuildResetFrame 后总线彻底静默（candump 0 帧）→ `/joint_states` 冻结在最后解码的固件"无有效位置"满量程标记 ±12.49635 → RViz 不再更新；再按 L3 → `on_activate` 的 reset-all 后 500 ms 内凑不齐 7/7 `has_feedback`（`DecodeFeedback`（codec:103）只认 0x02/0x18，reset 应答不可靠）→ abort「only X/Y motors answered」，L3 永久失效。实证（`/tmp/a3_0x18_on.py`，对已 reset 的 1–7 补发 `kMotorCmdActiveReport=0x18` ON）：7/7 立即以实测 ~101 Hz 持续回流（2 s 各 200+ 帧），总线复活，L3 立即使能、RViz 恢复实时；零扭矩、臂不动。0x18 为手册纯遥测通道（数据域同 0x02），power_sequence 原设计即保活意图（cpp:437「不下发上报 OFF：shutdown 后 RViz 仍依赖 0x18」），但统一 can 栈其 `/can_tx_frames` 使能帧被 launch 断言自动丢弃，0x18 从未在真机栈真正打开（唯一 0x18 支持是 RX 侧 filter，socketcan_transport.cpp:48）
- **设计（插件 on_activate/on_deactivate 两处补发，零协议/零新节点改动）：**
  1. `on_deactivate`：在 disarm CAN 超时 → 3× 零增益刷新 → BuildResetFrame 循环**之后**，对 7 台补发 `ProtocolCodec::BuildActiveReportFrame(bus_, motor_id, true)`（复用 codec:225 现成帧）→ reset 电机在 coast 态持续主动上报，RViz 恒实时
  2. `on_activate`：在 reset-all 循环**之后**、500 ms `has_feedback` 等待之前，同样补发 0x18-ON ×7 → 首启 / 任意历史态下 7/7 必然在窗口内成立，不再赌 reset 应答
  3. 使能成功后**不补发 0x18-OFF**（持续流）：使能态叠加 ~9% 恒定总线负载（叠加命令应答最坏 ~20% of 1 Mbps，仍远未饱和），换取「插件重载 / 进程崩溃电机照常上报、RViz 恒实时」的冗余；正是 power_sequence 原设计语义。若日后要省带宽，可在 on_activate 末尾补 OFF，本轮不做
- **验收标准（仿真，vcan0 插件 + 顶真机断电；脚本 `scripts/a3_test/f113_keepalive_acceptance.py`，隔离域 ROS_DOMAIN_ID=113）：**
  1. **帧级 A**：vcan 栈使能后调 `/a3/arm/disable`，candump 过滤 0x18（ID 高 5 位）断言 7 台均收到 `(0x18<<24)|(0xFD<<8)|motor_id` 的 ON 帧（data[6]=1），且**后续持续流**不停发（≥1 s 窗口每台 ≥ 若干帧）
  2. **帧级 B**：再调 `/a3/arm/enable`，断言 on_activate 同样补发 0x18-ON ×7（首启路径同样有 0x18，不再依赖 reset 应答）
  3. **闭环 C**：enable → disable → enable 循环 ≥ 3 次全部成功、FSM READY（旧故障形态 enable-after-disable abort 消失在回归中）；disable 态 `/joint_states` 持续更新（帧率不归零、位置不复现 ±12.49635 冻结标记）
  4. **回归 D**：vcan sim 注入「不响应 0x18」的 motor 静默 → disable 后再 enable 必须复现 abort（坐实修复前故障形态），恢复后 C 重跑 3/3 转绿
- **关联：** F83（on_activate 的 has_feedback 门，本需求补 0x18 使其必然成立）、F81/LL-083（失能态反馈丢失 freeze-hold）、F111（真机电源节点 /can_tx_frames 被统一栈丢弃是 0x18 从未打开的根因）、F86（on_deactivate 首步 disarm CAN 超时，保活流不与之冲突）
- **状态：** `completed`（2026-09-26，vcan 验收 42/42：`scripts/a3_test/f113_keepalive_acceptance.py` ROS_DOMAIN_ID=113。P1–P3 同一栈 enable/disable ×3 全部成功，on_activate/on_deactivate 各补发 0x18-ON ×7（count=7），READY 态 86–88 fps / DISABLED 态 46–47 fps 持续流，disable 下 /joint_states 持续更新（帧率不归零）且位置不复现 ±12.49635 冻结标记；P4 注入「m4 静默」→ enable 必败（日志 `activate aborted: only 6/7`，hardware active 请求被 `state 3 rejected`），FSM 停在 IDLE；P5 清除注入、fresh 栈恢复 7/7 → READY，m4 恢复 0x18 流 193 帧）。真机断电后与 F110/F111 一并实测确认

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
