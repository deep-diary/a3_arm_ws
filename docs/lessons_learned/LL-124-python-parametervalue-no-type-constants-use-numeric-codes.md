# LL-124 — 生成的 rcl_interfaces Python ParameterValue 没有类型常量，type 字段直接填数字码

> **日期：** 2026-09-24  
> **产品线：** Edge  
> **环境：** RK3588 (lubancat) Ubuntu 22.04 + ROS 2 Humble（rclpy，Python 3.10）

## 现象

通过 `rcl_interfaces/srv/SetParameters` 运行时调参，构造 `ParameterValue` 时按 C++ / 文档习惯写 `ParameterValue.PARAM_BOOL` → `AttributeError`；换成 `ParameterValue.PARAMETER_BOOL` 同样不存在。

## 根因

rosidl 生成的 Python 消息类**不导出 `ParameterType` 枚举常量**。`dir(ParameterValue)` 只有 `SLOT_TYPES`、各值槽（bool_value/double_value/...）和 `type`，没有任何 `PARAM_*` 常量。枚举定义在 `rcl_interfaces/msg/parameter_type.py`（`ParameterType` 消息），但 `ParameterValue` 类上不附带。

## 正确做法 / 规避

`type` 直接填数字码（见 rcl_interfaces/ParameterType.msg）：

```python
pv = ParameterValue()
if isinstance(value, bool):
    pv.type = 1          # PARAMETER_BOOL
    pv.bool_value = value
else:
    pv.type = 3          # PARAMETER_DOUBLE
    pv.double_value = float(value)
```

常用码：1 bool / 2 int64 / 3 double / 4 string / 5 byte_array / 6 bool_array / 7 int64_array / 8 double_array / 9 string_array；0 = NOT_SET。

注意 bool 必须先于 int 判断（`isinstance(True, int)` 为真）。

## 相关路径

- `scripts/a3_test/f107_payload_duty_acceptance.py`（`set_param`）
