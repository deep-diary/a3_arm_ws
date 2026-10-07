# F98 — L7 夹爪限位与实测标定统一（URDF / ros2_control / MoveIt 三处 [0.0, 1.78]；堵住模型与实物 12% 偏差）


- **说明：** L7 夹爪 2026-09-13 在新电机上完成标定（`src/a3_gripper_controller/config/gripper_config.yaml`）：全开位设零、闭合为正，实测机械止位 **1.7825 rad**，运行钳位取 **1.78 rad**（不顶止位、无驻留力矩）。但三处模型限位仍是 reBot 参考值 **±1.5708**：
  1. `src/a3_description/urdf/el_a3.urdf.xacro:279`（L7 joint limit lower/upper）
  2. `src/a3_description/urdf/el_a3_ros2_control.xacro`（L7 position command_interface min/max）
  3. `src/a3_moveit_config/config/joint_limits.yaml:59-66`（MoveIt L7 min/max_position）

  工业现场不能接受「控制器模型与实际机构不一致」：MoveIt 对夹爪的任何规划/展示最多只能闭合到 1.57 rad（比实际行程少 0.21 rad ≈ 12%，夹爪永远合不拢到模型可知的全行程）；负方向在机械上无对应行程（开位即零位），模型却允许 -1.57 rad 的位置指令，存在反向驱动、扯线风险。修复取**与标定钳位一致的单一真值 [0.0, 1.78]**（can_bridge 侧 `control_gains.yaml` 早已是 [0, 1.8]，本次不改）。L5/L6 的 ±1.5708 是真实关节限位，不动。
- **改动（仅配置，零代码逻辑）：**
  1. `el_a3.urdf.xacro` L7 `<limit>`：`lower="0.0" upper="1.78"`
  2. `el_a3_ros2_control.xacro` L7 position command_interface：`min 0.0 / max 1.78`
  3. `joint_limits.yaml` L7：`min_position 0.0 / max_position 1.78`
  4. 新增 `scripts/a3_test/f98_l7_limits_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f98_l7_limits_acceptance.py`）：**
  1. **模型一致性**：解析 xacro 展开后的 URDF、ros2_control xacro、MoveIt joint_limits.yaml，L7 三处 lower=0.0、upper=1.78（且 L5/L6 仍为 ±1.5708 未误伤）
  2. **全行程可达**：vcan 栈激活标准 `gripper_controller`（effort_controllers/GripperActionController），GripperCommand position=1.78 → `/joint_states` L7 在时限内到达 ≥1.65 rad（超过旧上限 1.5708，证明全行程打通）
  3. **回零**：GripperCommand position=0.0 → L7 回到 ≤0.05 rad
- **关联：** F87（GripperActionController 标准执行后端）、L7 零点标定记录、[shared/SAFETY.md](../shared/SAFETY.md)；a3_can_bridge `control_gains.yaml` 的 L7 [0, 1.8] 已是同方向约束
- **状态：** `completed`（2026-09-23，vcan 验收 13/13：`scripts/a3_test/f98_l7_limits_acceptance.py`。A 模型一致性 8/8：xacro 展开后 URDF L7=[0,1.78]、ros2_control position 接口 [0,1.78]、MoveIt joint_limits L7=[0,1.78]，L5/L6 ±1.5708 未误伤；B GripperCommand 1.78 reached_goal，实测 pos=1.770（越过旧上限 1.5708，全行程打通）；C GripperCommand 0.0 回到 pos=0.009。f87b 回归 7/7。多限位源同步踩坑见 LL-110）。真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
