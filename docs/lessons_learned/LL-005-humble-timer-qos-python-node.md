# LL-005 — Humble Python 节点：`create_timer` 无 `oneshot`；`/joint_states` 需 best-effort 订阅

> **日期：** 2026-09-02
> **产品线：** Edge
> **环境：** RK3588 / WSL2 Ubuntu 22.04 + ROS 2 Humble

## 现象

新增 `a3_arm_controller` 编排节点（Python + `MultiThreadedExecutor`）时踩到两个坑：

1. `ros2 service call /a3/arm/goto_named_pose` 卡住不回，节点进程直接退出，日志报：

```text
TypeError: Node.create_timer() got an unexpected keyword argument 'oneshot'
```

2. 节点订阅 `/joint_states`（`motor_protocol_node` 发布）后始终收不到消息，启动时打印：

```text
New publisher discovered on topic '/joint_states', offering incompatible QoS.
No messages will be received from it. Last incompatible policy: RELIABILITY
```

## 根因

1. `oneshot` 是 rclpy **新版本**（Iron 及以后）的 `create_timer` 关键字参数；**Humble 的 `Node.create_timer(period, callback)` 只有两个参数**，没有一次性定时器。传 `oneshot=True` 直接抛 `TypeError`，且发生在服务回调线程里，导致响应不返回、异常在 `executor.spin()` 里被抛出使节点崩溃。

2. `/joint_states` 由 `motor_protocol_node` 用 `rclcpp::SensorDataQoS()`（**best-effort**）发布；默认 `create_subscription` 是 **reliable**。best-effort 发布者 + reliable 订阅者 QoS 不兼容，订阅收不到任何消息。

## 正确做法 / 规避

1. **不要用 `oneshot`**。需要「延时切回」时用时间戳判断：记录 `self._traj_done_at = time.monotonic() + delay`，在周期定时器（如状态发布定时器）里检查 `time.monotonic() >= self._traj_done_at` 后执行并复位。

2. 订阅 `/joint_states`（及其他 best-effort 话题）时显式用 best-effort QoS：

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
js_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
self.create_subscription(JointState, "/joint_states", cb, js_qos)
```

   锁存话题（如 `/power_sequence/gate_open`、`/power_sequence/state`，`transient_local` 发布）订阅侧也要带 `DurabilityPolicy.TRANSIENT_LOCAL`，否则晚启动节点拿不到当前值。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_schedule_back_to_ready` / QoS 订阅）
- `src/a3_can_bridge/src/motor_protocol_node.cpp`（`SensorDataQoS` 发布 `/joint_states`）
- `src/a3_can_bridge/src/power_sequence_node.cpp`（`transient_local` 发布 gate/state）
