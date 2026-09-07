# LL-016 — rclpy Humble SIGINT 与 WaitSet 创建竞态：节点 Ctrl-C 退出码 1

> **日期：** 2026-09-07
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ ROS 2 Humble + rclpy（Python 节点）

## 现象

`a3_mqtt_bridge`（Python 节点）每次被 launch 清理 / Ctrl-C 结束时，进程以 **exit code 1** 挂掉，日志末尾是：

```
File ".../rclpy/executors.py", line 780, in wait_for_ready_callbacks
    return next(self._cb_iter)
File ".../rclpy/executors.py", line 653, in _wait_for_ready_callbacks
    wait_set = _rclpy.WaitSet(
rclpy._rclpy_pybind11.RCLError: failed to initialize wait set: the given context is not valid,
either rcl_init() was not called or rcl_shutdown() was called., at ./src/rcl/wait.c:130
```

业务功能全部正常（遥测/指令都通），只在退出瞬间报错——但 exit code 1 会让 launch 测试套件误判、CI 红灯。

## 根因

SIGINT 到达时，rclpy 的信号处理器直接调用 `rclpy.shutdown()` 使全局 context 失效；此时 executor 可能正卡在 `_wait_for_ready_callbacks` 里创建下一个 `WaitSet`——context 已失效，`_rclpy.WaitSet(...)` 抛 `RCLError`。`rclpy.spin()` 只捕获 `KeyboardInterrupt` / `ExternalShutdownException`，`RCLError` 一路穿透 `main()` 的 except 子句，进程以 1 退出。

这是一个**时序竞态**：executor 越空闲（回调越少），退出瞬间落在 WaitSet 创建窗口的概率越高。F35 把 MQTT 发布解耦到独立线程后 executor 更空闲，竞态窗口变大，从「偶发」变成「每次必现」。

## 正确做法 / 规避

在 `main()` 的 `rclpy.spin(node)` 外层把 `RCLError` 视为正常退出：

```python
try:  # Humble: RCLError 只在私有编译模块暴露（SIGINT 竞态兜底）
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:
    RCLError = None

try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
except Exception as exc:
    if RCLError is not None and isinstance(exc, RCLError):
        pass  # SIGINT 已触发 rclpy.shutdown()，视为正常退出
    else:
        raise
```

要点：

- Humble 里 `RCLError` **不在** `rclpy.exceptions` 公共 API 中，只能从 `rclpy._rclpy_pybind11` 私有模块导入——必须带 `ImportError` 兜底（其他发行版可能没有该符号），且不能因私有导入失败而 crash。
- 只放过 `RCLError` 这一个类型，其余异常照常 re-raise，别用裸 `except Exception: pass` 吞掉真 bug。
- 若节点还带自己的清理线程（如发布线程），`destroy_node()` 里先置 `_closing` 标志再 join，避免线程在日志器销毁后仍调 `get_logger()`。

## 相关路径

- `src/a3_mqtt_bridge/a3_mqtt_bridge/ros2mqtt_bridge.py`（`main()` 与 `destroy_node()`）
- `src/a3_gripper_controller/a3_gripper_controller/gripper_controller_node.py`（`main()`；真机 hw 测试同样触发 `ExternalShutdownException` 变体，2026-09-07 已修）
- 注意：`a3_bringup`/`a3_teleop_ps4`/`a3_arm_controller` 下仍有多个 Python 节点只有裸 `except KeyboardInterrupt`（如 `ps4_mapper.py`、`sim_motor_node.py`、`arm_controller.py`）。当前它们退出瞬间 executor 多在忙、竞态窗口小未复现；后续改到这些节点时顺手按本条目补齐 `(KeyboardInterrupt, ExternalShutdownException)` + `RCLError` 兜底。
