# LL-093 — launch 参数 dict 的点号键会被写成扁平参数名（插件覆盖要用 ParameterFile）；mock GenericSystem 拒绝 effort 接口

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588 lubancat + ROS 2 Humble + ros2_control 2.54.0 / mock_components GenericSystem

## 现象

F88 需要让同一套产品栈在 `hardware:=mock` 与 `hardware:=can` 下使用**不同的夹爪控制器插件**（mock 用 position 版 GAC，真机用 effort 版 GAC）。最初尝试在 controller_manager 的 `parameters=[{"gripper_controller.gains.L7_joint.p": ..., ...}]` 这类点号键 dict 里做条件覆盖，启动后参数根本没进到目标节点命名空间；而让 mock 栈直接加载 effort 版 GAC 时，控制器 claim 接口失败、栈起不来。

## 根因

两个独立事实：

1. **launch 对 dict 参数做序列化时点号键不被当作嵌套结构**：`{"a.b.c": v}` 写成的是名为 `a.b.c` 的**扁平参数**，不是 `a → b → c` 的嵌套 YAML；而 controller 节点要的是 `gripper_controller` 命名空间下的嵌套参数。dict 形式无法可靠表达跨命名空间的整段覆盖，也无法按硬件模式切换插件类型字段。
2. **Humble `mock_components/GenericSystem::prepare_command_mode_switch` 只接受 position（及 velocity）接口**，见 `generic_system.cpp`：claim effort 命令接口直接抛错（`prepare_command_mode_switch` failure）。effort 版 `effort_controllers/GripperActionController` 在 mock 栈永远无法激活。真机 MIT 电机走 effort（Type-1 torque_ff），position 版又不能用于真机。

## 正确做法 / 规避

1. 条件化整段控制器配置用两份 YAML + launch 的 **`ParameterFile(path, allow_substs=True)`**，路径用 `PythonExpression` 按硬件模式选文件；`parameters=` 列表中放在基础 yaml 之后做覆盖。dict 只用于少量确定无嵌套歧义的标量覆盖。
2. 接受「mock 与真机命令接口不同」这一现实并在入口处显式切换：mock→`position_controllers/GripperActionController`，can→`effort_controllers/GripperActionController`；产品拓扑、话题名、action 名保持一致，上层 FSM 不感知差异。
3. 写/改 ParameterFile 后记得路径来自 install/share，YAML 改动要重编（symlink-install 下 launch 本身也要重编 a3_bringup，见 LL-074）。
4. 排查「参数覆盖没生效」：先 `ros2 param get <node> <full.name>` 看实际值，别信 launch 文件里的写法。

## 相关路径

- `src/a3_bringup/launch/a3_bringup.launch.py`（gripper_plugin_file ParameterFile 切换）
- `src/a3_description/config/gripper_position_plugin.yaml`、`gripper_effort_plugin.yaml`
- 相关：LL-079（嵌套参数名被静默忽略同源）、LL-090（GAC effort 变体配置契约）
