# F53 — 编排层「不罢工」补齐：指令×状态×模式 全组合显式拒绝（F40 零力矩 disable 死路径）


- **说明：** 2026-09-16 零力矩手感测试暴露链路缺口：`/a3/arm/disable`（F40 park 路径）在 `mode==ZERO_TORQUE`（外部 `/a3/zero_torque/start` 直打执行层，编排层状态仍 READY）下**不拒绝**——safe park 的回家轨迹被执行层 OnTrajectory **静默丢弃**（`zero_torque_active_ || SERVO`），臂在悬浮中干等 ~4–5 s 后看门狗 `FOLLOW_STUCK → stop → 3s → reset` 阶梯把臂中途切断，编排层误报「safe park aborted: 电机带外失能」→ DISABLED；且此路径下 `reset` 会**撤掉重力补偿让臂垂落**，reset/set_zero/enable 都**不退出** `zero_torque_active_`（重使能后 kp 仍 0、臂保持悬浮，「伪成功」）。原则收束为：**编排层任何指令×状态×模式组合不得静默接受或含糊失败——要么执行，要么 `success=false` + 消息给出可执行的下一步**。修复 5 处（`arm_controller.py`）：
  1. `_disable_cb`（主缺口）：`self._mode in BLOCKED_MODES`（ZERO_TORQUE/GRAVITY_COMP）时拒绝，消息提示 `先 /a3/zero_torque/stop 恢复闭环再 disable`；紧急失能 `/a3/motor/reset`（但注明此后需 zero_torque/stop 才能正常重使能）
  2. `_init_cb`：拒绝组合补 `SAFE_PARK` busy 态 + `BLOCKED_MODES`（零力矩下 set_zero 打碎零位帧；enable 不退出 zero_torque → init 伪成功）
  3. `_enable_cb`：补 `BLOCKED_MODES` 拒绝（使能=重锚当前位但 kp 仍 0，臂继续悬浮 → 伪成功）
  4. `_can_move`：拒绝表补 `FAULT`（此前静默接受轨迹、臂不动——电机已复位关断）
  5. `_enter_ai_cb`：补 `BLOCKED_MODES` 拒绝（AI 发的轨迹会被执行层丢弃，静默无动作）
- **验收标准：**
  1. 零力矩下 `/a3/arm/disable`、`enable`、`init` → `success=false` 且消息含 `先 /a3/zero_torque/stop …`（不再走进 park→看门狗切臂死路径）
  2. `/a3/zero_torque/stop` 后再 disable/enable/init 全部正常（退出后执行层恢复 kp 锚定当前位）
  3. `GRAVITY_COMP` 模式同 1（进入该模式的 set_mode 路径同样拒绝）
  4. FAULT 态下 goto/move_to/playback/jog 被拒（消息含 state）；AI 态零力矩下 enter_ai 被拒
  5. 常规路径零回归：READY 态 goto/move_to/回放、READY/TRAJ 带外失能、IDLE 直达 disable、SAFE_PARK 拒绝、F44 降温门禁行为不变；现有 `scripts/a3_test/a3_test.sh` 用例不受影响
- **配套文档：** [docs/edge/STATE_MACHINE.md](STATE_MACHINE.md)（11 态 stateDiagram + 指令×状态矩阵 + 跨层模式丢弃矩阵）；中文消息改为「动作/原因/下一步」三段式，全部服务拒绝路径均给出 `/a3/zero_torque/stop` 或 `/a3/motor/reset` 逃生
- **关联：** F40（disable park 是死路径入口）、F45（11 态状态机）、F51（使能重锚/模式共享；本需求堵住其「模式不清零」的组合缺口）、F50（看门狗阶梯是死路径的误报源）、[LL-045](../../lessons_learned/LL-045-disable-in-zero-torque-dead-path.md)
- **状态：** `implemented`（2026-09-16，代码 5 处 + 文档；构建验证通过；**真机重启后生效**——修复时臂处于零力矩测试中，未重启正在跑的旧栈，待用户跑完退出流程后手动重启加载）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
