# LL-111 — ros2_control 控制器参数挂在各控制器自己的节点上，不在 /controller_manager 命名空间下

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 lubancat, ROS 2 Humble, ros2_control / ros2_controllers

## 现象

F99 验收需要确认 JTC 的 `cmd_timeout` 已生效，直觉地查了控制器管理器：

```bash
ros2 param get /controller_manager arm_controller.cmd_timeout
# Parameter not set

ros2 param list /controller_manager | grep arm_controller
# arm_controller.type      # 只有 type，没有任何控制器参数
```

参数「查不到」，但行为测试表明它实际生效了（日志无 configure 警告、超时确实触发）。一度怀疑参数没加载或被 YAML 结构写错。

## 根因

ros2_control 里每个**已激活的控制器拥有自己的 ROS 节点**，节点名就是控制器名（`/arm_controller`、`/gripper_controller`、`/joint_state_broadcaster`……）。控制器的全部自有参数（`joints`、`cmd_timeout`、`constraints.*`、`command_interfaces` 等）声明在**控制器自己的节点**上，对应 YAML 里与 `controller_manager:` 平级的独立顶层块：

```yaml
controller_manager:
  ros__parameters:
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController   # 只有 type 在这

arm_controller:            # ← 控制器自己的参数块
  ros__parameters:
    cmd_timeout: 2.0
```

`/controller_manager` 节点上只有 `<controller_name>.type`。`ros2 param get /controller_manager arm_controller.cmd_timeout` 这种「管理器 + 点分前缀」路径是不存在的命名空间臆测，CLI 返回 `Parameter not set`，再用宽松正则 `[0-9.]+` 解析就会匹配到句点本身（`could not convert string to float: '.'`）。

## 正确做法 / 规避

- 查控制器参数直接对控制器节点：
  ```bash
  ros2 param get /arm_controller cmd_timeout      # Double value is: 2.0
  ros2 param list /arm_controller                 # 该控制器全部参数
  ```
- 前置条件是控制器已被加载（spawn）——未加载的控制器没有节点，参数自然不存在；查参数前先确认 `ros2 control list_controllers`。
- 解析 `ros2 param get` 文本要用带类型前缀的严格正则（`Double value is:? ([0-9]+\.[0-9]+)`），不要用裸 `[0-9.]+`，`Parameter not set.` 里的句点会被误匹配。
- 排查「参数没生效」的顺序：先确认查的是控制器节点而不是 manager → 再看 configure 期 WARN（如 JTC 的 `Command timeout must be higher than goal_time tolerance`，参数会被**静默置 0**）→ 最后才怀疑 YAML。

## 相关路径

- `src/a3_description/config/el_a3_controllers.yaml`
- `scripts/a3_test/f99_jtc_cmd_timeout_acceptance.py`（read_cmd_timeout）
