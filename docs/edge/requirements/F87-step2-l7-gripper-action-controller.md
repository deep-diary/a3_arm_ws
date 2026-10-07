# F87（第二步） — L7 走标准 GripperActionController（per-goal max_effort；退役夹爪节点死服务依赖）


- **说明：** 第一步把固件 `0x700B` 收进使能编排后，Python 夹爪节点（F24/F25/F26）的两条外部依赖在统一栈下都是死的：`/a3/motor/set_param`（旧 `a3_can_bridge` 服务，无人启动，力矩上限写不进去 → 静默重试）和 `/mit_gains_cmd`（旧执行层增益话题，无人订阅 → 力/位增益切换全部空发）。节点内部还自维护 50 Hz 力环状态机、接触/打滑判定和归一化开合，均与 ros2_controllers 的标准实现重复。第二步按工业路径替换：L7 控制器改为 `gripper_controllers` 的 `effort_controllers/GripperActionController`（ROS 2 Humble 官方包 `ros-humble-gripper-controllers`），对外是标准 `control_msgs/action/GripperCommand`；力控由控制器内置 PID（位置/速度误差 → effort 命令）完成，per-goal `max_effort` 在控制器内逐目标钳位，固件 `0x700B`（第一步）仍是最终硬件钳位——双层限力。position GAC 变体只透传位置命令、本身不执行力上限，故不采用。
- **改动：**
  1. `el_a3_controllers.yaml`：`gripper_controller` 类型改为 `effort_controllers/GripperActionController`，参数 `joint: L7_joint`、`goal_tolerance`、`max_effort`（默认值）、`action_monitor_rate`、`allow_stalling/stall_velocity_threshold/stall_timeout`，`gains.L7_joint: {p, i, d, i_clamp}`；控制器要求 L7 同时导出 effort 命令接口与 position/velocity 状态接口（确认 `A3MITHardwareInterface` 已对 L7 导出 effort+position command）。
  2. 编排层：不再把 7 关节 FJT 中的 L7 投到 `/gripper_controller/follow_joint_trajectory`（该 action 已不存在）；L7 段转成标准 GripperCommand goal（position + max_effort）经 ActionClient 下发；纯位姿路径（goto/move_to/park）的 L7 同步段同样走 gripper action 并等待结果。
  3. PS4：开合/切换/半程改发 GripperCommand goal（position）；R2 力控扳机改发带 `max_effort` 的 goal（扳机行程映射为 max_effort，档位语义保留），删除对 `a3_msgs/GripperCommand` 服务的调用与重试状态机。
  4. MQTT：现有 `gripper_*` op 改走标准 action；`gripper_set_max_torque` 的固件写不再可用（0x700B 由使能编排统一布防、易失寄存器不接受运行期单路改写），语义改为「本次会话 per-goal max_effort 上限」或明确返回不可用（不保留静默失败）。
  5. 产品 launch 默认不再启动 Python 夹爪节点；节点代码保留（不删文件），死服务依赖在本提交内从默认路径摘除。
  6. 验收脚本 `scripts/a3_test/f87b_gripper_action_acceptance.py`（编号沿用任务序号 F87 第二步，脚本独立命名）：vcan 上断言标准 action 存在、位置 goal 收敛、带 max_effort 的抓取 goal 产生的 effort 命令不超过上限（raw Type-1 帧 t_ff/kp 解析 + sim 接触注入）、stall 检测、固件 0x700B 仍是最终钳位、FSM/PS4 路径无对死服务的调用。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f87b_gripper_action_acceptance.py`）：**
  1. `gripper_controller` 加载为 `effort_controllers/GripperActionController`，`/gripper_controller/gripper_cmd`（标准 `control_msgs/GripperCommand` action）可连；无 `/gripper_controller/follow_joint_trajectory`
  2. 位置 goal（开/合）在 goal_tolerance 内收敛、返回 `reached_goal=true`；L7 行程与标定一致（开 0.0、合 ≈1.79 rad）
  3. 力控 goal：sim 注入接触后，raw Type-1 帧解析出的实际输出 effort（t_ff 与 kp×err 合成）在稳态不超过该 goal 的 `max_effort`（±5%）；换不同 max_effort 重复验证；控制器 stall 判定按 `stall_velocity_threshold/stall_timeout` 触发并正常保持
  4. 双层限力：固件 `0x700B`（第一步编排）仍为 6 Nm；控制器命令即使配置错误也不会超过固件钳位（sim 固件钳位生效证据）
  5. 默认 launch 不启动 `a3_gripper_controller` 节点；栈内无对 `/a3/motor/set_param`、`/mit_gains_cmd` 的请求（日志/话题级证据）；FSM goto/park/disable 中的 L7 段经标准 action 执行且全链路回归通过
- **关联：** F87 第一步（固件 0x700B 布防，本项的最终硬件钳位）、F24/F25/F26（Python 夹爪节点力/位/配置能力，由标准控制器取代）、F78（统一产品栈）、F81（反馈失鲜看门狗对 L7 同样有效）
- **状态：** 仿真验收通过（vcan `f87b_gripper_action_acceptance.py` 23/23，2026-09-23：控制器类型/状态、gripper_cmd 可连、FJT action 已消失；自由空间开/合收敛 reached_goal=true（1.779 / −0.016 rad）；接触注入后 0.5/1.0 Nm 两档 raw Type-1 稳态 effort 中位 0.500/1.000（±5%），stall 契约 stalled=true/reached_goal=false（allow_stalling→succeeded）并持续保持；max_effort=20 被固件/插件钳在 6.000 Nm；0x700B 读回 6.0 Nm；Python 夹爪节点与死服务端点全部缺席；混合模式 arm FJT+L7 action、FSM goto ready、safe park→DISABLED 全链路回归）。另：F32 `/a3/motor/*` 九个旧 can_bridge 调试 op 在统一栈下无服务端，本次一并从 MQTT 下行白名单摘除（改为显式 unknown op，不再静默挂死）。真机验收待上电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
