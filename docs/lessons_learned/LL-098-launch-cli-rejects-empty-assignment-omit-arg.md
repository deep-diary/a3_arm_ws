# LL-098 — ros2 launch CLI 拒绝空值 name:=，要走默认就省略参数

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble

## 现象

F89 验收脚本启动栈时无条件拼接 `gravity_scales_file:=`（值为空字符串），ros2 launch 直接拒绝：

```
malformed launch argument 'gravity_scales_file:='
```

## 根因

launch CLI 的 `name:=value` 语法要求 value 非空；空字符串不是「使用默认值」的信号，而是语法错误。

## 正确做法 / 规避

- 需要默认值时**整个参数都不要传**（Python 里按条件 append argv），不要传空串。
- 用脚本/ProcessGroup 拼 argv 时：`if value: argv.append(f"name:={value}")`。
- 顺带注意：launch 文件内给 ros2_control_node 构造「可选 ParameterFile」时，空路径要在 launch 侧 PythonExpression 里替换为已有的 controllers yaml，不能把空路径塞进参数文件列表。

## 相关路径

- `scripts/a3_test/f89_gravity_scale_acceptance.py`（start_stack）
- `src/a3_bringup/launch/a3_bringup.launch.py`
