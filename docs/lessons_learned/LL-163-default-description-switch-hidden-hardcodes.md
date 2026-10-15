# LL-163 — 换默认描述形态：除 `a3_bringup.launch.py` 外还有 3 处硬编码会漏（RViz 仍显示夹爪 / F52 档位配对报错）

> **日期：** 2026-10-15
> **产品线：** Edge
> **环境：** WSL2 Ubuntu 22.04 + ROS 2 Humble（`a3_stack.sh start --sim --rviz`）

## 现象

F174 把默认描述从 7 轴 `a3_description` 换成 6 轴 + 连杆 `a3_description_6j`
（系统级配置 `a3_robot_model/config/robot_model.yaml`），`a3_bringup.launch.py`（真机/mock 统一入口）
已改完并 mock 验证 `joints=6`。但**仿真栈**（`a3_stack.sh start --sim`，走
`edge_teleop_full_sim.launch.py` → `edge_web_sim.launch.py`）里：

1. RViz 仍显示**带夹爪**的 7 轴模型；
2. 日志报 `F52 档位配对不一致：arm_controller 配了 7 关节 [...L7_joint]，但 /joint_states 上报 6 关节`。

## 根因

仿真链路**不复用** `a3_bringup.launch.py` 的选档逻辑，另有 3 处独立硬编码点（改了一处不等于改全）：

1. `edge_web_sim.launch.py` / `edge_teleop_full_sim.launch.py` 的
   **`robot_description_semantic` 写死 `el_a3.srdf`（7 轴）**，与 6 关节 URDF 不匹配；
   RViz 里的夹爪视觉来自 **`/robot_description`**（这两个 launch 自己 `open(urdf)` 读描述），
   而它们当时读的还是旧包 → 显示夹爪。
2. `sim_motor_node`（仿真执行层）默认 `joint_names` 为写死的 7 关节 `JOINTS`，
   与 6 关节描述/编排层不一致（且 `_on_feedback_timer` 里 `self._effort[L7_IDX]` 在 6 关节下越界）。
3. 仿真栈用 `a3_arm_controller/launch/arm_controller.launch.py` 起编排层，它只加载
   `arm_controller.yaml`（`joint_names` 含 L7、`switch_controllers` 含 `gripper_controller`），
   **不像 `a3_bringup.launch.py` 那样注入 joint_names/switch_controllers 覆盖** → F52 校验报错。

## 正确做法 / 规避

- 换形态（7 轴 ↔ 6 轴 + 连杆）时，**按"哪个入口起编排层/发布描述"逐条排查**，别只改产品入口：
  | 链路 | 入口 | 描述来源 | 编排层来源 |
  |---|---|---|---|
  | 真机/mock | `a3_bringup.launch.py` | `robot_model.xacro_path()` | 内联 `fsm` 节点（注入覆盖） |
  | 仿真 | `edge_teleop_full_sim` → `edge_web_sim` | `robot_model.urdf_path()` | `a3_arm_controller/launch/arm_controller.launch.py` |
- 这几处**统一读 `a3_robot_model`**：URDF/xacro、`srdf`、`controllers`、`motor_map`、`joint_names`、
  `include_gripper`；`arm_controller.launch.py` 也要注入 `joint_names` / `switch_controllers`
  （和无夹爪时 `named_poses_pkg`）。
- 排查快速定位：`grep -rn '"el_a3.srdf"' src/` 与 `grep -rn "L7\|JOINTS" src/a3_bringup/a3_bringup/sim_motor_node.py`；
  仿真起栈后三查：`/joint_states` 关节数、`arm_controller ready: joints=N`、`gravity_torque ... nq=N`。
- RViz 只认 `/robot_description`（URDF）与 TF；SRDF 不匹配**不会**画出错模型，但会让 MoveIt/Servo 报错。

## 相关路径

- `src/a3_bringup/launch/edge_web_sim.launch.py`、`edge_teleop_full_sim.launch.py`（SRDF、`use_gripper` 默认）
- `src/a3_bringup/a3_bringup/sim_motor_node.py`（`joint_names` 默认、`tau_L7` 打印越界）
- `src/a3_arm_controller/launch/arm_controller.launch.py`（注入 joint_names / switch_controllers）
- `src/a3_robot_model/config/robot_model.yaml`（唯一档位事实源）
- 需求：[F174](../edge/requirements/F174-a3-description-6j-linkage-package.md)；相关：[LL-157](LL-157-product-6j-profile-include-gripper.md)（档位切换坑）
