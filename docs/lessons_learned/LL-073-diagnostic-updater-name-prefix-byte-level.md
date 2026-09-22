# LL-073 — diagnostic_updater 三连坑：组件名被自动加节点名前缀 / level 是 byte 字段（rclpy 收为 bytes）/ 验收 harness 自身进程漏设 ROS_DOMAIN_ID

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，diagnostic_updater 4.0.7 / diagnostic_msgs
> **关联：** [[LL-072-ros2-control-mock-jsb-order-jtc-single-point.md]]、F71

## 现象

F71 给 `a3_arm_monitor` 增量加标准 `/diagnostics`（官方 `diagnostic_updater`），全自动验收脚本连续踩三个坑：

1. `add("a3_monitor: Monitor", fn)` 期望组件名 `a3_monitor: Monitor`，实际发出 `a3_arm_monitor: a3_monitor: Monitor`——节点名被加了两次。
2. 组件名修正后，level 比较 `level == 0/1/2` 全部失败，打印出来是 `b'\x00'`、`b'\x02'`。
3. 首次运行时 harness 节点的发布/订阅跟 monitor 完全不通：monitor 收在 domain 59，喂数进程在默认 domain。

## 根因 / 规避

### 1. Updater.add() 的任务名自动加节点名前缀

diagnostic_updater 的 `Updater` 会把每个 task 名字统一加上节点名前缀（`<node_name>: <task_name>`）。**add() 只给裸组件名**：`add("Monitor", ...)` → 最终 `a3_arm_monitor: Monitor`。想要全限定名时先算好节点名，不要手填前缀，节点一重命名立刻错位。

### 2. DiagnosticStatus.level 是 `byte` 不是 `uint8`

`diagnostic_msgs/DiagnosticStatus` 里 `byte level`，在 ROS 2 的 Python 绑定中 `byte` 字段反序列化为**单元素 bytes**（`uint8` 才是 int）。比较前归一化：

```python
lvl = x[0] if isinstance(x, (bytes, bytearray)) else x
```

常量 `DiagnosticStatus.OK/WARN/ERROR/STALE`（0/1/2/3）是 int，可直接比。命令行 `ros2 topic echo` 看到的也是 `"\x02"` 这种 YAML，别当成字符串异常。

### 3. subprocess harness：ROS_DOMAIN_ID 必须在 harness 自身进程也设置

只给被起的 monitor 子进程 env 里塞 `ROS_DOMAIN_ID=59` 不够——harness 自己的 rclpy 节点仍在默认 domain，发布的喂数话题 monitor 收不到。**模块顶部、rclpy.init() 之前** `os.environ.setdefault("ROS_DOMAIN_ID", DOMAIN)`，子进程 env 再显式传一份。凡是「脚本起节点 + 脚本自身也是节点」的自包含验收都有这个坑。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_monitor_node.py`（裸名 add、level 映射，判定/处置逻辑零改动）
- `scripts/a3_test/f71_monitor_diagnostics_acceptance.py`（lvl() 归一化 + 模块顶 setdefault domain）
