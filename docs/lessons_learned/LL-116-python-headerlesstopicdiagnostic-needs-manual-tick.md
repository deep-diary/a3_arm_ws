# LL-116 — Python 绑定 HeaderlessTopicDiagnostic 的内部订阅不投递，必须自建订阅并手动 `tick()`

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 / Ubuntu 22.04 / ROS 2 Humble；ros-humble-diagnostic-updater；Cyclone DDS

## 现象

F104 初版按 C++/文档直觉使用 `HeaderlessTopicDiagnostic`：构造时传入话题名，以为它内部会自行订阅该话题、收到消息自动计数：

```python
task = HeaderlessTopicDiagnostic(
    topic, self.updater,
    FrequencyStatusParam({"min": lo, "max": hi}, tolerance, window_size))
```

话题在 50 Hz 正常发布，但 `/diagnostics` 该任务恒定 ERROR「No events recorded」，任何速率下都一样。

## 根因 / 知识点

1. `HeaderlessTopicDiagnostic` 构造参数确实是话题名（任务发布为 `<节点名>: <话题名> topic status`），C++ 版本内部会创建自己的订阅；但**本机 Humble 的 Python 绑定里，这个内部订阅不投递事件**，不手动喂数据就永远零事件。
2. 该类暴露 `tick()` 方法（另有 `clear_window`/`getName`/`run`/`addTask`），由外部订阅回调调用即可正常驱动滚动窗频率统计。
3. 手动 tick 后行为完全正确：50 Hz（40–60 Hz 带）→ OK「Desired frequency met」；10 Hz → WARN「Frequency too low.」；停发整窗 → ERROR「No events recorded.」；恢复 → OK。

## 正确做法 / 规避

- Python 侧用 `HeaderlessTopicDiagnostic` 时，**自己建订阅、在回调里手动 `task.tick()`**：

```python
task = HeaderlessTopicDiagnostic(topic, updater, FrequencyStatusParam(...))
node.create_subscription(
    msg_type, topic, lambda msg, t=topic: tasks[t].tick(), best_effort_qos)
```

- 话题类型可运行时发现（`get_publishers_info_by_topic` → `importlib.import_module(f"{pkg}.msg")`），话题暂不存在周期重试。
- 排障口诀：hztest 任务恒报「No events recorded」而话题确实在发 → 先怀疑绑定层没喂 tick，而不是调频率窗参数。

## 相关路径

- `src/a3_bringup/a3_bringup/topic_rate_monitor_node.py`（自建 BEST_EFFORT 订阅 + 手动 tick）
- `scripts/a3_test/f104_topic_rate_acceptance.py`
- `docs/edge/REQUIREMENTS.md` F104
- 关联：[[LL-115-diagnostic-updater-task-name-prefixed-by-node-name]]
