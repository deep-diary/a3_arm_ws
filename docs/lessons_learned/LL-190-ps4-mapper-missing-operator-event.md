# LL-190 — ps4_mapper 启动即崩：调用了不存在的 `notify_operator_event`

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble

## 现象

`a3_stack.sh restart` 后栈"起来了"（launch 还在），但手柄完全没反应；
`/tmp/a3_real_stack.log` 中 `ps4_mapper` 进程 exit code 1：

```
AttributeError: 'ActionExecutor' object has no attribute 'notify_operator_event'
... File "ps4_mapper.py", line 129, in __init__
    self._publish_mode_state(self._mode)
```

## 根因

模式框架在 `Ps4Mapper.__init__` 里就发布一次当前模式并顺带发操作员事件
（灯带/震动），但 `ActionExecutor` 侧只加了调用点、没加方法本体（回滚重建时漏了）。
`__init__` 抛异常 → 节点直接退出，launch 不报错，表现为"手柄全死"。

## 正确做法 / 规避

1. 重建/合并代码后，除了 `colcon build` 通过，还要核对**运行期日志**
   （`grep -E 'process has died|Traceback' /tmp/a3_real_stack.log`）。
2. `ps4_mapper` 正常应打印 `ps4_mapper ready ... mode=normal`；没有这行就是启动失败。
3. `ActionExecutor` 必须有 `_event_pub = create_publisher(String, "/a3/arm/operator_event")`
   和 `notify_operator_event(event)`（ds4_feedback_node 消费 `mode_changed:<label>:<n>`）。

## 相关路径

- `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`
- `src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py`
- `src/a3_teleop_ps4/a3_teleop_ps4/ds4_feedback_node.py`
