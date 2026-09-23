# LL-109 — 验收脚本 in-process 驱动节点必须在 rclpy.init() 前自设 ROS_DOMAIN_ID / RMW

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 lubancat, ROS 2 Humble, Cyclone DDS, vcan

## 现象

F97 验收脚本（spawn 子进程起 vcan sim + 产品栈，自身进程内跑 rclpy 节点做订阅/FJT action）连续两次报 `no /joint_states in 40 s`，而 `/tmp/f97_stack.log` 证明栈完全健康：JSB 已 `Configured and activated`，仿真在发 CAN 反馈。

## 根因

子进程通过 `Popen(..., env=make_env())` 拿到 `ROS_DOMAIN_ID=97` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，但**这些环境变量不影响发起 Popen 的驱动进程本身**。驱动节点继承 shell 默认环境：

1. RMW 与栈不一致（shell 默认 FastRTPS vs 子进程 Cyclone）；
2. `ROS_DOMAIN_ID` 不一致（shell 默认域 vs 域 97）。

两者任一不一致，DDS 发现都无法配对，节点表现为「栈全绿但什么都收不到」。f89 脚本在 main 开头自设 `ROS_DOMAIN_ID`，本次新脚本漏抄了这一步。

## 正确做法 / 规避

任何「脚本内 rclpy 节点 + spawn 隔离域子进程」的验收驱动，在 `rclpy.init()` **之前**、启动任何子进程之前，对当前进程显式设置：

```python
os.environ["ROS_DOMAIN_ID"] = DOMAIN
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
os.environ["PYTHONNOUSERSITE"] = "1"
```

排查顺序：先确认 RMW 一致，再确认 domain 一致；栈日志健康但节点无发现时，直接怀疑这两点，不要去查栈。

另：`builtin_interfaces/Duration` 的 `sec` 字段只接受 `int`，传浮点秒须拆成 `sec=int(t), nanosec=int((t % 1) * 1e9)`。

## 相关路径

- `scripts/a3_test/f97_jtc_tolerance_acceptance.py`
- `scripts/a3_test/f89_gravity_scale_acceptance.py`（正确范例）
