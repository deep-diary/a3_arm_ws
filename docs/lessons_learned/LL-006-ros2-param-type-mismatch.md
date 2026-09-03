# LL-006 — ROS 2 参数：declare_parameter 默认值类型必须与 params-file 一致

> **日期：** 2026-09-03
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble

## 现象

`ros2 launch a3_gripper_controller gripper_controller.launch.py` 启动即崩溃：

```text
rclpy.exceptions.InvalidParameterTypeException: Trying to set parameter
'torque_limit_param_id' to '28683' of type 'INTEGER', expecting type 'DOUBLE'
[gripper_controller-1]: process has died [pid ..., exit code 1]
```

## 根因

节点代码里 `_DEFAULTS` 用 `float(PARAM_TORQUE_LIMIT)`（`28683.0`，DOUBLE）声明了
`torque_limit_param_id` 的默认值；而 `gripper_config.yaml` 里写的是整数 `28683`
（INTEGER）。rclpy 在合并 `--params-file` 覆盖值时会做严格类型检查：声明类型 DOUBLE，
被 INTEGER 覆盖 → 抛 `InvalidParameterTypeException`，节点直接退出。

该参数语义是「固件参数索引（0x700B）」，本来就该是整数；后面代码也用
`int(self._p("torque_limit_param_id"))` 消费，说明默认值声明成 float 是笔误。

## 正确做法 / 规避

1. 声明默认值时类型与 YAML 保持一致：索引/ID 用 `int(...)`，力矩/系数用 `float(...)`。
   本例改成 `int(PARAM_TORQUE_LIMIT)` 即可。
2. 排查「launch 起得来、`ros2 run` 却正常」的差异：`ros2 run` 不带 params-file 时用代码默认值
   （无覆盖不报错），`ros2 launch` 带 params-file 才暴露类型冲突。
3. 写新 Python 节点时，`_DEFAULTS` 逐项对照 YAML 的标量类型（int/float/bool/str/数组）。

## 相关路径

- `src/a3_gripper_controller/a3_gripper_controller/gripper_controller_node.py`（`_DEFAULTS`）
- `src/a3_gripper_controller/config/gripper_config.yaml`（`torque_limit_param_id: 28683`）
