# LL-149 — rclpy ActionClient 高频抢占 goal 竞态：多线程清 _goal_handles 抛 KeyError 打崩节点

> **日期：** 2026-10-07
> **产品线：** Edge
> **环境：** RK3588 LubanCat-4 + ROS 2 Humble + MultiThreadedExecutor

## 现象

弹琴示教回放（L7 夹爪按录制时间戳 50 Hz 步进下发 GripperCommand）时，编排节点
`a3_arm_controller` 崩溃退出（exit code 1），日志：

```
[arm_controller-14] [WARN] [a3_arm_controller]: GripperCommand 未到位: ...
[arm_controller-14] Traceback (most recent call last):
  File ".../rclpy/action/client.py", line 356, in execute
      del self._goal_handles[goal_uuid]
  KeyError: b'...'
[ERROR] [arm_controller-14]: process has died ...
```

后果：`/a3/arm_status` 停止发布（10 Hz 停了），DS4 灯带停在最后一次状态（紫色=TRAJ）
不再恢复；R3 软失能也因编排节点已死而无响应。

## 根因

编排节点用 `MultiThreadedExecutor` + `ReentrantCallbackGroup`，夹爪 `ActionClient`
也挂在这个可重入组上。L7 时序步进以 50 Hz **抢占式**下发 goal（GAC 新 goal 抢占旧
goal），同一时刻多个 goal 的状态事件在多个 executor 线程并发处理，rclpy `ActionClient`
的 `execute()` 里 `del self._goal_handles[goal_uuid]` 前有 `if goal_uuid in` 检查、但检查与
删除之间无锁，被另一线程抢先删掉 → `KeyError`。这是 rclpy 非线程安全的 action client
在高频抢占 + 多线程并发下的竞态，不是业务逻辑错误。

## 正确做法 / 规避

给「高频抢占式下发」的 action client 单独挂一个 `MutuallyExclusiveCallbackGroup`，
让它的 goal 响应/结果/状态处理串行化，消除多线程并发访问 `_goal_handles` 的竞态：

```python
self._cb_group = ReentrantCallbackGroup()          # 主回调组仍可重入（避免服务回调内同步等待死锁）
self._gripper_cb_group = MutuallyExclusiveCallbackGroup()  # 夹爪高频抢占专用，串行化

self._gripper_cli = ActionClient(self, GripperCommand, "…",
                                 callback_group=self._gripper_cb_group)
```

注意：只给「高频抢占」的客户端换互斥组即可；主回调组保持 Reentrant（否则服务回调内
`_wait_future` 同步等待会死锁，LL-126 同源）。低频 action（FJT/MoveIt/Sequence）无此问题。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`
- `install/.../site-packages/a3_arm_controller/arm_controller.py`（`colcon build` 后生效）
