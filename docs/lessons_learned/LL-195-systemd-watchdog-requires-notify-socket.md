# LL-195 — 非 systemd 启动时 `systemd_watchdog_feed` 必崩（NOTIFY_SOCKET 为空）

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble（`a3_stack.sh` 手动 setsid 起栈）

## 现象

每次手动起栈，日志固定出现：

```
[systemd_watchdog_feed-19] AttributeError: 'NoneType' object has no attribute 'sendto'
[ERROR] process has died [pid ... exit code 1] ... systemd_watchdog_feed
```

栈其余节点正常，但日志常驻一条 ERROR，容易被误认为栈不健康。

## 根因

`_on_js` 收到首帧 `/joint_states` 时**无条件** `_notify(b"READY=1")`；非 systemd 启动没有
`NOTIFY_SOCKET`，`self.sock` 为 `None`。`_notify` 只捕获 `OSError`，`AttributeError` 未被捕获。

## 正确做法 / 规避

- `_notify` 开头 `if self.sock is None: return`；
- `READY=1` 发送前也判断 `self.sock is not None`；
- 只有 systemd `Type=notify` 场景才需要喂 watchdog，手动起栈应静默跳过。

## 相关路径

- `src/a3_bringup/a3_bringup/systemd_watchdog_feed_node.py`
