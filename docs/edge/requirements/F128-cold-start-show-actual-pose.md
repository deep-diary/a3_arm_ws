# F128 — 冷启动未使能即显实际姿态（on_configure 布防 0x18 纯遥测；L3 使能逻辑不变）


- **说明：** 用户真机反馈（2026-10-01）：`a3_stack.sh start -f --rviz` 起栈后、未按 L3 前，RViz 显示的是 URDF `initial_value` 的默认 home 姿态（L2=0.785/L3=−0.785），看不到机械臂实际位置；按下 L3 后才突然跳到真实姿态。根因：硬件组件以 `inactive` 启动（`el_a3_controllers.yaml` `hardware_components_initial_state`，LL-086），插件 `on_configure` 只打开 CAN、起 `RxLoop`，**不发任何 CAN 帧**；MIT 固件只在收到命令帧后回反馈，而 0x18 主动上报仅在 `on_activate`（L3）/`on_deactivate`（R3）布防（F113/F123）——冷启动到首次 L3 之间总线静默，`hw_pos` 一直保持 URDF 播种值，JSB（仅 claim state interface，不激活硬件）把该默认值发上 `/joint_states`。`power_sequence_node` 启动时本有 `OnBootActiveReportOnce`（Idle 态布防 0x18，日志明写 "for RViz /joint_states_feedback"），但 F111 统一栈里其 `/can_tx_frames` 无消费者，帧为死信。期望：起栈后**不使能电机**即可在 RViz 实时看到真实姿态（与实物不符就不按 L3，避免误使能），L3 维持当前"开门禁 + 使能 + 7/7 门 + 软起步"逻辑不变。
- **设计（仅插件 `on_configure` 补一处布防，零协议/零新节点）：**
  1. `A3MITHardwareInterface::on_configure` 末尾（CAN open + `RxLoop` 启动后）对 7 台电机先写 0x7026 EPScan n=19（100 ms/10 Hz，F123 周期），再发 `BuildActiveReportFrame(..., true)` ×7（各两轮幂等补发，防上电忙线丢单帧）；**不发 reset / enable / MIT 力矩帧**，电机保持上电 coast 态。
  2. 反馈链路复用现成通路：`RxLoop` 无条件解码 0x18（数据域同 0x02）→ `hw_pos` 状态接口裸指针 → active JSB → `/joint_states` → robot_state_publisher / RViz / MQTT 遥测；与 R3 失能后保活（F113）完全同构，只是把布防时机提前到冷启动。
  3. L3 `on_activate` 编排**一字不动**：reset-all（擦掉 0x7026）→ 0x18-ON → 500 ms 7/7 `has_feedback` 门 → clear-fault/run-mode/限幅/超时/EPScan/Enable 厂商编排 → 重锚 + 软起步。`on_deactivate` 的 EPScan+0x18 序列与新逻辑抽同一 helper。
- **验收标准：**
  1. vcan0 插件栈（`edge_ros2_control_vcan.launch.py` + `vcan_motor_sim.py`，隔离域）：fresh 起栈、从未 enable，candump 可见 0x7026 写 + 0x18-ON ×7，随后 0x18 流 ≈10 Hz/电机；`/joint_states` 持续更新且位置等于 sim 实测位（非 URDF initial_value 常量），`/a3/motor/states` 各关节 `has_feedback=true, enabled=false`
  2. 真机：`a3_stack.sh start --rviz` 后 **DISABLED/IDLE 态 RViz 即显示机械臂实际姿态**，手动轻搬关节（coast）RViz 跟随；电机无力矩/无抱闸、24V 行为与改动前一致
  3. L3 使能链路零回归：7/7 门通过 → READY、软起步正常；enable→R3 disable→enable 循环 ≥3 次成功，失能态 RViz 仍实时
  4. mock 栈（GenericSystem 不加载本插件）与 legacy 栈零影响
- **关联：** F113（0x18 keepalive，本需求补其缺失的冷启动时机）、F123（EPScan 10 Hz，三处布防同周期）、F111（power_sequence `/can_tx_frames` 死信 = 根因）、F78（插件 owner）、LL-086（硬件 inactive 启动）
- **状态：** `implemented`（vcan/真机验收待补）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
