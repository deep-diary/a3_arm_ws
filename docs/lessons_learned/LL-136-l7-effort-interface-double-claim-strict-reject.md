# LL-136 — L7 effort 接口被 gripper / zero_torque 双 claim，STRICT switch 必需原子 swap

> **日期：** 2026-09-27
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble + ros2_control（controller_manager STRICT）

## 现象

F127 想把示教（`ZERO_TORQUE` 自由拖动）期间夹爪 L7 一并释放：把 `zero_torque_controller.joints` 从 L1–L6 扩展到 L1–L7。但标准栈里 `gripper_controller`（GripperActionController，claim `[L7_joint/effort]`）与 `zero_torque_controller` 扩展后都 claim `[L7_joint/effort]`。在 Humble 的 `hardware_interface` STRICT 策略下，`switch_controller` 请求「zero_torque active 时 gripper 仍 active」会被**整体拒绝**（interface 已被一个 active controller 占用，STRICT 不抢占），导致自由拖动无法进入。

## 根因

1. ros2_control 的命令接口（command interface）在某 controller active 时即被该 controller 占用；同一接口被两个 active controller 声明违反 STRICT 唯一性，切换请求整批失败。
2. `_cm_freedrive_switch` 之前只换 `arm_controller ↔ zero_torque`（两者接口互不重叠），没涉及 L7，所以在加 L7 前这条规则从未被触发——**接口互斥是「控制器集合 × 接口集合」的整体性质，不是单控制器属性**。
3. INACTIVE 状态无法区分「被我们停用」与「本就没配置」，退出时不能靠 re-query 决定要不要恢复 gripper——必须用自己维护的标志位。

## 正确做法 / 规避

- 涉及共享命令接口的控制器切换，必须构造**一个原子 swap 请求**：enter = `deactivate [arm_controller, gripper_controller] + activate [zero_torque]`，exit 反向；绝不能分两次调用（第一次会直接被 STRICT 拒绝）。
- 进入前用 `/controller_manager/list_controllers` 探测目标 controller 是否真的 active（配置里可能存在也可能不带 g），按探测结果动态组 deactivate 列表；退出按进入时是否真带 g 的标志恢复。
- GravityCompensationController（rnea）按关节名映射，扩展 joints 列表无 count/order 假设，但必须同时满足「与其共享接口的所有 active controller 全部 deactivate」。
- 验证手法：`list_controllers` 断言 swap 后的 active/inactive 集合；真机栈与 `a3_bringup hardware:=mock` 同验证（F127 验收）。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_cm_freedrive_switch`）
- `src/a3_description/config/el_a3_controllers.yaml`（zero_torque_controller.joints）