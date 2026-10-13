# F168 — 产品栈原生 6 关节档（拆掉 L7 夹爪电机）


- **说明：** 硬件拆掉 L7 夹爪电机（can1 上只剩电机 1..6）后，产品栈（`a3_bringup.launch.py hardware:=can`）此前**只有 7 关节档**：URDF ros2_control 声明 L7，硬件插件 `on_activate` 逐电机校验反馈（`activate aborted: only 6/7 motors answered`）→ 整臂无法使能；`a3_stack.sh` 的电机探测也写死 7/7 直接中止。F52 的降级档（`motor_map_5j` / `arm_controller_6j` 等）是**旧 can_bridge 栈**机制（F78 起 deprecated），不含 MoveIt/JTC/弹琴所需链路。本需求给产品栈加**原生 6J 档**：xacro `include_gripper:=false` 不生成 L7_joint/gripper_link 与 L7 ros2_control；配套 6J SRDF / controllers / power_sequence motor_map；编排层只覆盖 `joint_names`+`switch_controllers`；`a3_stack.sh` 探测后**自动选档**（7/7→7J，1..6 在线缺 7→6J）。**7J 档为默认，行为不得改变。**
- **验收标准：**
  1. 6J 起栈（`./scripts/a3_stack.sh start --rviz` 探到缺 L7，或 `--6j`）：`/joint_states` 恰好 `L1..L6`（无 `L7_joint`）；`arm_controller ready: joints=6`；`power_sequence F52 档位：6 关节`；`zero_torque configured: 6 joints`
  2. 7J 档回归不受影响：不传 `include_gripper`（或 `true`）时 `/joint_states` 7 关节、`arm_controller joints=7`，与改动前逐字一致
  3. `a3_stack.sh` 探测：7/7 在线→7J；1..6 全在线且电机 7 缺失→自动切 6J 并告警；缺其它关节→仍中止
  4. 6J 下 MoveIt（arm 组 L1–L6）、JTC、弹琴（F162）/巡游/示教回放链路可用（MoveIt 规划组不含 L7，无需改动）
- **关联：** F52（缺电机降级档，旧栈机制）、F78（统一 bringup 单入口）、F162（弹琴 app，6 关节臂为主）、[LL-157](../../lessons_learned/LL-157-product-6j-profile-include-gripper.md)（`PythonExpression` 三元 + 拼接优先级踩坑）
- **实现：**
  - `src/a3_description/urdf/el_a3.urdf.xacro`：新增 `include_gripper`（默认 `true`）；`false` 时不生成 `L7_joint`/`gripper_link`，并透传给 `el_a3_ros2_control` 宏
  - `src/a3_description/urdf/el_a3_ros2_control.xacro`：宏加 `include_gripper` 形参；`false` 时不声明 L7 关节
  - `src/a3_description/config/el_a3_controllers_6j.yaml`：无 `gripper_controller`；`zero_torque_controller` 去 L7
  - `src/a3_moveit_config/config/el_a3_6j.srdf`：去 gripper/arm_with_gripper 组、open/close 组状态、end_effector 元素与 gripper_link 碰撞对
  - `src/a3_can_bridge/config/motor_map_6j.yaml`：power_sequence 的 `joint_names`/`motor_ids_by_index` 只含 1..6
  - `src/a3_bringup/launch/a3_bringup.launch.py`：新增 `include_gripper` 启动参数，`OpaqueFunction` 在建 LaunchDescription 时解析为布尔值 → 选 xacro/SRDF/controllers/arm_controller 覆盖/power_sequence motor_map，并按需跳 `gripper_controller` spawner
  - `scripts/a3_stack.sh`：探测接受 7/7 或 1..6（缺 7）；自动选档并传 `include_gripper`；新增 `--6j` 强制档；起栈后自检 `/joint_states` 关节数
- **状态：** `implemented`（2026-10-13；mock 6J/7J 双档 + 真机 6J 起栈验证：`/joint_states`=6、`arm_controller joints=6`、`power_sequence F52 档位：6 关节`）
- **未覆盖项（有意）：** 未加 6J 版 `moveit_controllers.yaml`（仍列 `gripper_controller`，MoveIt 启动时记录一条 "Added GripperCommand controller" 但不下发，无功能影响）；`zero_torque_controller` 的 `tau_scale` 仍读 7 值标定文件，6J 下值数不匹配回退为 1.0（L1–L6 标定值本就≈1）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
