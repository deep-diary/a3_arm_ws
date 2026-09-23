# LL-110 — 关节重标定后必须同步全部「限位源」：URDF / ros2_control / MoveIt 三处

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 lubancat, ROS 2 Humble, xacro, MoveIt 2

## 现象

L7 夹爪 2026-09-13 在新电机上完成标定（全开设零、闭合为正，机械止位 1.7825 rad，运行钳位 1.78），但模型侧三处仍沿用 reBot 参考值 ±1.5708：

1. `src/a3_description/urdf/el_a3.urdf.xacro` L7 `<limit lower/upper>`
2. `src/a3_description/urdf/el_a3_ros2_control.xacro` L7 position `command_interface` min/max
3. `src/a3_moveit_config/config/joint_limits.yaml` L7 min/max_position

后果：MoveIt 对夹爪的规划/展示最多闭合到 1.57 rad（实际行程少 12%，夹爪永远合不到模型可知的全行程）；模型还允许 -1.57 rad 的负向位置指令，而开位即零位，负向无机械对应行程。

## 根因

参考仓库（EDULITE_A3）里的 ±1.5708 是**参考机构的默认值，不是本机标定真值**。重标定只更新了夹爪自身配置（gripper_config.yaml）和 can_bridge 的 `joint_cmd_*_rad`（该侧 L7 早已是 [0, 1.8]），没有人负责把标定结果反向同步到描述/规划层。

## 正确做法 / 规避

- 任何关节（尤其更换电机、重新设零后）完成标定，必须把限位同步到**三个源**：URDF joint `<limit>`、ros2_control `command_interface` min/max、MoveIt `joint_limits.yaml`；三处以同一标定值为准。
- 改后重编涉及包（a3_description / a3_moveit_config），并用解析模型的静态验收兜底（f98 脚本 A 段即此用途），防止只改一处。
- 注意区分同值的不同关节：L5/L6 的 ±1.5708 是真实限位，改 L7 时不要误伤——静态验收同时断言 L5/L6 不变。
- 操作钳位（1.78）与物理止位（1.7825）刻意留 0.0025 rad 余量；模型限位取钳位值，避免规划路径顶止位产生驻留力矩。

## 相关路径

- `scripts/a3_test/f98_l7_limits_acceptance.py`
- `src/a3_gripper_controller/config/gripper_config.yaml`
- `src/a3_can_bridge/config/control_gains.yaml`（L7 [0, 1.8]）
