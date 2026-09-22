# LL-090 — GripperActionController：stall 成功的 result 是 reached_goal=false；effort 变体必须配 gains

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble（ros-humble-gripper-controllers 2.54.0）

## 现象

F87 第二步首次验收，接触注入（sim 硬限位）后 GAC 的 goal 正常结束，但断言 `reached_goal=true` 失败：实际 result 是 `stalled=true, reached_goal=false`。另外 `ros2 control list_controllers` 的输出默认带 ANSI 颜色码（`\x1b[96m...`），按旧版 `name[type] state` 格式写的解析正则全部落空（rc=0 但解析为空，极易误判为栈异常）。

## 根因

1. Humble GAC 实现（`gripper_action_controller_impl.hpp:181`）对 stall 的契约：停滞后 `reached_goal=false, stalled=true`；`allow_stalling=true` 时调用 `setSucceeded`（不是 aborted）。即「成功」与「reached_goal」不是一回事——stall-succeeded 的目标永远 reached_goal=false。直觉上「allow_stalling → 成功 → reached=true」是错的。
2. effort 变体与 position 变体不同：position 变体只透传位置命令；effort 变体（`EffortHardwareInterfaceAdapter`）要求 `gains.<joint>.{p,i,d,i_clamp}` 参数存在，PID 输出再被 per-goal `max_effort` 钳位。漏配 gains → on_configure 报错，漏配 max_effort → 钳位为 0（默认 0.0），夹爪不动且无运行期错误。
3. `ros2 control` CLI 在 tty/子进程下都可能输出颜色，列分隔是多空格而非旧版的方括号。

## 正确做法 / 规避

- 验收/调用 GAC 时按三态判定：`reached_goal=true`（到位）；`stalled=true` 且 goal succeeded（接触顶住、力保持，这是力控抓取的**正常成功路径**，不要当失败）；aborted（`allow_stalling=false` 时停滞才是真失败）。
- effort GAC 配置五件事缺一不可：`joint`、`gains.<joint>.p/i/d/i_clamp`、`max_effort`（默认值）、`goal_tolerance`、`allow_stalling + stall_velocity_threshold + stall_timeout`。
- 解析 `ros2 control list_controllers`：先去 ANSI（`re.sub(r"\x1b\[[0-9;]*m", "", s)`），再按空白分列取 `name / type / state`。
- Python：`rclpy.action` 需显式 `import rclpy.action`，否则 `rclpy.action.ActionClient` AttributeError。
- 力控验收方法：sim 侧 per-motor dynamics override（`vcan_motor_sim.py --dynamics-file`，gravity_nm + stop_at），从 raw Type-1 帧 CAN-id 位 8–23 解 torque_ff，断言稳态中位 ≈ 目标 max_effort（±5%）。

## 相关路径

- `/opt/ros/humble/include/gripper_action_controller/gripper_controllers/gripper_action_controller_impl.hpp`
- `src/a3_description/config/el_a3_controllers.yaml`（gripper_controller 块，CRLF）
- `scripts/a3_test/f87b_gripper_action_acceptance.py`
