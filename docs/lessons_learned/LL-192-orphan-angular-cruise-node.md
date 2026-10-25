# LL-192 — 旧版 `a3_angular_cruise` 成孤儿：同名服务被两个节点抢

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble

## 现象

新写 `a3_bringup/angular_cruise` 后，`ros2 node list` 同时出现：

```
/a3_angular_cruise      ← 旧版（14:38 启动，一直没被杀）
/angular_cruise         ← 新版
```

`ros2 service list` 只有一个 `/a3/demo/angular_cruise`，但调用可能落到**旧实现**上
（行为/参数与新代码不一致），排查时非常容易被误导。

## 根因

`a3_stack.sh` 的孤儿兜底名单 `NODE_PATTERN` / `CHECK_PATTERN` 里**没有
`angular_cruise`**。旧节点是上一版（回滚前）随栈起的；之后多次 `restart` 都只杀名单内
进程，它一直活着。ROS 2 允许多个节点提供同名服务，客户端按发现顺序挑一个 →
"改了代码却没生效"。

## 正确做法 / 规避

1. 新增常驻节点时**同步把可执行名加进 `NODE_PATTERN` 与 `CHECK_PATTERN`**
   （这次补的是 `a3_bringup/angular_cruise` / `angular_cruise`），否则每次重启都会累积。
2. 改完节点先 `ros2 node list | grep <name>` 确认只有**一个**实例，再调服务。
3. 发现行为与代码不符时，按 PID 看 `lstart`（启动时间）——老时间是孤儿，直接 TERM/KILL。

## 相关路径

- `scripts/a3_stack.sh`（`NODE_PATTERN` / `CHECK_PATTERN`）
- `src/a3_bringup/a3_bringup/angular_cruise_node.py`
