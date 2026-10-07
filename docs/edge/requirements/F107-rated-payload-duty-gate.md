# F107 — 额定负载与占空比静态门禁（rated payload 1.5 kg + duty-cycle；工业机器人额定参数对标）


- **说明：** 当前产品没有「额定负载」概念：URDF 重力模型不含末端负载，夹持 1.5 kg 工件后起重力矩可能超过电机连续额定；也没有工作制（占空比）约束，连续 jog 导致电机热积累只能等 F84 温度硬门禁事后介入。工业机器人交付必须公布并在软件中强制 rated payload / duty cycle。依据电机厂规格（EDULITE `el_a3_sdk/docs/电机通信协议汇总.md`：RS00（L1–L3）连续 5 N·m / 峰值 14 N·m，EL05（L4–L7）连续 1.8 N·m / 峰值 6 N·m），用既有 pinocchio 模型在**使能前、运动前**做静态力矩校核，不达标即拒绝；滚动窗口统计运动占空比，超限拒绝新运动直到窗口腾出。
- **设计：**
  1. **负载参数**：FSM 参数 `payload_mass_kg`（默认 0.0）、`payload_com_m`（double[3]，负载质心相对 `gripper_link` 系的偏移，默认零）、`rated_payload_kg`（默认 1.5）。新增 a3_msgs 服务 `/a3/arm/set_payload`（`SetPayload.srv`：mass_kg + com_m → success/message）：质量区间校验（0 ≤ m ≤ rated_payload_kg，超限直接拒绝）；若当前已使能，同时要求当前位姿静态可行，否则拒绝变更（保持旧值）。负载以「固定关节挂载点质量」形式在 pinocchio 建模：在 `robot_description` 末尾注入 payload link + fixed joint（Python pinocchio 不暴露 `appendBodyToJoint`），改负载即重建模型
  2. **静态力矩门禁**：参数 `joint_rated_torque`（默认 `[5,5,5,1.8,1.8,1.8,1.8]`）、`static_torque_margin_ratio`（默认 0.8）。`pin.rnea(q,0,0)` 求重力矩，任一关节 |τ| > rated × margin 即判该位姿不可行。拦截点：
     - **使能**：`_enable_cb` 在 `_check_positions_in_limits` 通过后加当前位姿静态校核（不通过时保持 IDLE，错误消息明示超额定关节与数值）
     - **运动前**：所有实际下发轨迹在 dispatch 前逐点校核（两点 fallback / playback / safe-park / 纠偏 / L7）；move_group 自动执行路径在发送 goal 前按当前位姿→目标的关节空间密集采样（≥21 点）预校核（MoveIt 路径无关节力矩规划，采样校核 + margin 是静态门禁的工程近似，写入 SAFETY.md）
  3. **占空比门禁**：参数 `duty_gate_enabled`（默认 true）、`duty_window_s`（默认 600.0）、`duty_max_ratio`（默认 0.8）。每次 dispatch 记录运动段时长；新运动前裁剪滚动窗口并求和，已用比例（含本次预估）超限即拒绝，消息给出预计冷却秒数。该门是 F84 真实温度门之外的前馈热保护（仿真/温度传感器未覆盖工况下兜底）
- **改动：** 新增 `a3_msgs/srv/SetPayload.srv`；`a3_arm_controller/arm_controller.py`（参数 + 负载模型 + 三类拦截 + set_payload 服务）；`docs/shared/SAFETY.md`、`docs/shared/TOPIC_CONTRACT.md`、`docs/edge/QUICKSTART.md` 同步
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f107_payload_duty_acceptance.py`）：**
  1. mock 栈使能（零负载）：默认全部既有运动路径不受影响（回归）
  2. 运动到低力矩位姿后 `/a3/arm/set_payload` 设 1.5 kg：服务成功；朝低力矩目标的运动成功；朝高重力位姿（URDF 零位方向）的运动被静态门禁拒绝，消息含超额定关节/力矩值
  3. `set_payload` 质量 > rated_payload_kg 被拒；已使能时在高重力位姿设负载被拒且旧负载不变
  4. 占空比：测试参数缩短窗口（duty_window_s≈20 s、duty_max_ratio≈0.2），连续运动在预算耗尽后被拒（消息含 duty/冷却）；缩小 duty_window_s 使旧记录出窗后恢复可运动
  5. 结束后无残留进程 / 隔离域 107 无活节点，退出码 0
- **关联：** F44/F84（温度门是占空比门的事后硬保护）、F73/F89（pinocchio 重力模型同源）、F83（使能编排）、F88（两点轨迹 dispatch 集中）、F106（commissioning 默认零负载跑通）
- **状态：** `completed`（2026-09-24，mock 验收 33/33：`scripts/a3_test/f107_payload_duty_acceptance.py`。P1 零负载使能+jog 回归；P2 LOW 位形设 1.5 kg 成功、低力矩 L1 转动放行、朝零位运动被静态门拒（L3 −7.02 N·m）；P3 2.0 kg 越界拒绝且旧值不回滚（门禁仍报 1.5 kg 力矩）、高重力位形设负载被 current pose 门拒绝；P4 窗口 20 s/比例 0.2 下 13×0.3 s 后第 14 次被占空比门拒（4s motion、wait 9s），窗口缩至 1 s 旧记录出窗即恢复；结束无残留节点。期间发现并修复冷却计算 bug，见 LL-125；建模/调参坑见 LL-123/LL-124）。真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
