# LL-045 — 零力矩下 disable 走死路径：park 轨迹被执行层静默丢弃 + 看门狗阶梯误切

> **日期：** 2026-09-16
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ 5J 档真机 + 零力矩手感测试

## 现象

零力矩手感测试中（`/a3/zero_torque/start` **直打执行层**，编排层状态毫不知情仍停在 READY、`self._mode="ZERO_TORQUE"`），此时调 `/a3/arm/disable`：
1. 编排层 F40 正常进入 safe park——发布 home 轨迹；
2. **执行层 OnTrajectory 静默丢弃**（`zero_torque_active_ || last_control_mode_=="SERVO"` → "Ignore trajectory"），臂继续悬浮、实际不动；
3. 看门狗运动窗内期望(home) vs 实际(原位) 偏差超 0.25 rad → `FOLLOW_STUCK → /a3/motor/stop → 3 s → reset`，把臂在 SAFE_PARK 中途切断；
4. 编排层误报「safe park aborted: 电机带外失能」→ DISABLED。

更糟的连环坑：自觉绕路的人直接调 `/a3/motor/reset`（紧急失能）——**reset/set_zero/enable 都不清执行层 `zero_torque_active_` 标志**（只清 hold_suppressed/last_commanded/enable_ramp）→ 后续 enable 是「伪成功」：F51 重锚当前位但 kp 仍 0，臂继续保持悬浮。且 reset 在此路径上**撤掉重力补偿**，臂在重力下垂落。真正是该路径唯一正确出口：`/a3/zero_torque/stop`（恢复 kp、目标重锚到当前反馈位）。

## 根因

三层各守一关、彼此缺链路：
- **执行层**：`zero_torque_active_` 是内部有效态，只有 zero_torque/stop 或 gate-close 沿能清除；enable/reset/set_zero 都不退出；
- **编排层**：`_disable_cb` 只查 state 查 BLOCKED_MODES（docstring 写了拒绝意图，代码 2026-09-16 前根本没查 `self._mode`）——对 mode 维度的共享态（`/a3/control_mode` 双发布方）无校验；
- **看门狗**：FOLLOW_STUCK 阶梯不知道 SAFE_PARK 轨迹会被执行层吞掉，正常处置反而把臂砍了。

判据：**编排层任何「我要发轨迹/依赖臂动的指令」都须同时校验 state × mode 两个维度；mode（ZERO_TORQUE/GRAVITY_COMP/SERVO）下执行层吞轨迹 = 编排层所有安全回退（park/停到哪）全部失效。**

## 正确做法 / 规避

F53 修复（`arm_controller.py` 五处）——编排层指令×状态×模式全组合要么执行、要么 `success=false` + 消息给出可执行下一步：
1. `_disable_cb`：`self._mode in BLOCKED_MODES` → 拒绝，「先 /a3/zero_torque/stop 恢复闭环再 disable；紧急失能 /a3/motor/reset（此后需 zero_torque/stop 才能正常重使能）」；
2. `_init_cb`：补 SAFE_PARK busy + BLOCKED_MODES（set_zero 打碎零位帧；enable 不退出 zero_torque）；
3. `_enable_cb`：补 BLOCKED_MODES（重锚但 kp 0 → 伪成功）；
4. `_can_move`：拒绝表补 FAULT（此前静默接受轨迹、臂不动）；
5. `_enter_ai_cb`：补 BLOCKED_MODES（AI 轨迹被吞）。

口诀：**模式不清零可以接受（执行层就地权），但编排层必须知道并拒绝在「模式吞轨迹」下做任何依赖臂动的还原。**

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_disable_cb`/`_init_cb`/`_enable_cb`/`_can_move`/`_enter_ai_cb`）
- `src/a3_can_bridge/src/motor_protocol_node.cpp`（zero_torque 服务 409-463、OnTrajectory 丢弃 857-860、enable/reset 不清标志 1515-1654）
- `docs/edge/STATE_MACHINE.md`（指令×状态×模式矩阵）
- [LL-039](LL-039-teach-exit-reanchor-false-trip-enable-snap.md)（同族：使能/模式边界的假象与三层链路）；[LL-044](LL-044-disable-park-3s-too-fast.md)（看门狗误伤 park 的另一例）。