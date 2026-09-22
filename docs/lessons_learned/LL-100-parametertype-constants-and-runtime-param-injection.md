# LL-100 — rclpy 参数类型常量在 ParameterType 上；用运行时改参注入验证原子切换失败路径

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble

## 现象

F89b 验收脚本经 `set_parameters_atomically` 给 FSM 注入坏控制器名，构造 `ParameterValue` 时：

```python
ParameterValue(type=ParameterValue.PARAMETER_STRING, ...)
# AttributeError: type object 'ParameterValue' has no attribute 'PARAMETER_STRING'
```

## 根因

`rcl_interfaces/msg/ParameterValue` 消息本身只有 `type`（uint8）+ 各类型值字段；`PARAMETER_NOT_SET/PARAMETER_STRING/...` 常量定义在**独立的 `ParameterType.msg`** 上（Python 侧 `from rcl_interfaces.msg import ParameterType`），不挂在 ParameterValue 类上。

## 正确做法 / 规避

- 类型常量一律走 `ParameterType.PARAMETER_STRING`（DOUBLE=3, STRING=4, DOUBLE_ARRAY=11 等）；或直接写字面 uint8 值。
- **验证「原子切换失败安全」无需起第二个栈**：F89b 标准 4 要求退出切换被拒后 FSM 保持 TEACH。做法是在同一运行栈上经
  `/a3_arm_controller/set_parameters_atomically` 把 `freedrive_arm_controller` 临时改成不存在的名字（参数未声明 read-only，运行时可改，FSM 每次切换时现读参数），STRICT 请求即整体被拒；验完再改回。比起一套「没装某控制器」的 bringup 变体（要加 launch 开关、重启、重新发现）快一个数量级，且测的是同一条真实代码路径。
- 改参后必须经 `get_parameters` **读回确认**（set 返回 successful 不等于已生效到你假设的值，LL-096 同类教训）。

## 相关路径

- `scripts/a3_test/f89b_freedrive_switch_acceptance.py`（set_fsm_string_param：set→readback）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（_cm_freedrive_switch）
