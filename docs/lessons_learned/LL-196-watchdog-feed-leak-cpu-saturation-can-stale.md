# LL-196：`systemd_watchdog_feed` 不在停机名单 → 每次重栈漏一个，最终把 CPU 打满、CAN 反馈掉帧

- **产品线：** Edge
- **日期：** 2026-10-26
- **关联：** [LL-167](LL-167-servo-node-orphans-after-restart.md)（孤儿累积同类问题）、[LL-180](LL-180-servo-move-jerk-cpu-starvation-feedback-stale.md)（CPU 挤占 → 反馈 stale）、[LL-189](LL-189-dual-stack-rumble-l3-dead-and-cpu-starvation.md)

## 现象

人脸跟踪期间反复出现瞬时 `feedback stale`（0.1–0.3 s，自动恢复）、`CAN write failed: Resource temporarily unavailable`、
画面帧率从 24 掉到 **10–15 fps**，偶发"臂猛动一下"。
`uptime` 显示 **load average 28**、`%Cpu 86 us / 2.5 id`（8 核）。

## 根因

`ps -eo pid,etimes,pcpu,args` 里发现 **9 个 `systemd_watchdog_feed`**（最早跑了 13 小时），
每个 20–30% CPU，合计约 2 个核。这个节点由 `a3_bringup.launch.py` 起，
但 **`a3_stack.sh` 的 `NODE_PATTERN` 里没有它** → 每次 `stop/restart` 只杀别的节点，它一直活着。

CPU 打满的直接后果（不是"CAN 硬件坏了"）：

1. `ros2_control_node` 的 200 Hz 控制环 + SocketCAN 读写线程被饿 → 内核 CAN RX 缓冲溢出 → 反馈帧丢 → `stale`
2. TX 线程来不及排空 → **发送队列满** → `CAN write failed: Resource temporarily unavailable`
3. `servo_target_node` 的 `dt_step` 抖动（被夹到 0.5–2×）+ IK 迟到 → 命令台阶 → "猛动一下"

> 判据：**短时（0.1–0.3 s）自恢复的 stale + TX 队列满 + 帧率掉一半**，先查 CPU/孤儿，
> 不要先怀疑接头。断电造成的 stale 是**永久**的（age 一直涨、所有温度读 0）。

## 处理

```bash
ps -eo pid,etimes,pcpu,args | grep systemd_watchdog_feed   # 找孤儿
kill -TERM <除当前栈以外的 PID>                            # 保留当前栈那个
```

实测：idle 2.5% → **16%**，load 28 → 20，画面 **10–15 fps → 21–23 fps**，stale 消失。

## 永久修复

`scripts/a3_stack.sh` 的 `NODE_PATTERN` 追加 `a3_bringup/systemd_watchdog_feed`
（本次已加）。新增任何 launch 节点时，**必须同步加进停机名单**，否则就是下一个泄漏源。

## 附带发现

- 当前整栈仍然很重（瞬时）：`arm_controller` 72%、`ps4_mapper` 58%、`vision_display_mux` 46%、
  `gravity_torque` 44%、`ee_pose_publisher` 44%、`angular_cruise` 43%、`usb_cam` 38%、`topic_rate` 33%。
  跟踪验收建议精简栈（不带 `--rviz`），或至少停掉 demo 用的 `angular_cruise`。
- `/arm_controller/joint_trajectory` 有 **2 个发布者**：`a3_servo_target` 与 `ps4_mapper`（`actions.py` 的动作路径）。
  平时 mapper 不发，一旦某个动作触发就是"双源抢同一个 JTC 目标" → 必然顿挫，排查抖动时要查它。

> **2026-10-26 再次复现（F196 新增 `joint_state_relay`）：** 新节点同样漏在 `NODE_PATTERN` 外，
> 重启后残留两个 relay → `/joint_states_lite` 实测 100 Hz（两路 50 Hz 叠加）、CPU 翻倍。
> 已把 `joint_state_relay` 加进 `NODE_PATTERN`/`CHECK_PATTERN`。**新增任何 launch 节点都必须
> 同步加停机名单**，否则重启一次漏一个。

## 相关路径

- `scripts/a3_stack.sh`（`NODE_PATTERN`）
- `src/a3_bringup/launch/a3_bringup.launch.py`（`systemd_watchdog_feed`、`angular_cruise` 节点）
- `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`（`arm_jtc_topic`）
