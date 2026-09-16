# LL-050 — `ros2 service call` CLI 首次调用有 ~3 s DDS discovery 开销：时序断言要用纯 rclpy 驱动，别用 bash 测 start→stop 时长

> **日期：** 2026-09-16
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble + domain-55 隔离模拟栈

## 现象

验收 F54「误触发示教阈值」：`start_teach` → sleep 0.04 → `stop_teach`，期望样本数 ~2（50 Hz 下 < 阈值 10）触发 skip。但 `stop_teach` 响应却报 `recorded 151 samples; auto-saved latest.yaml`，latest.yaml 还被覆盖了——0.04 s 出 151 个样本，物理上不可能（≈3.0 s @ 50 Hz）。

## 根因

- bash harness 里每个 `ros2 service call` 都**冷启动一个 CLI 进程**；首个调用要在自身进程内做 DDS discovery / service 解析 / RMW 初始化，实测**墙钟 ≈ 3 s**（`start_teach` 那次返回就被拖住）；
- shell 的 `sleep 0.04` 是紧跟 `>` 重定向上一步**返回之后**起算的，所以实际 start→stop 间隔 ≈ 3 s，是「正常录制」而非误触发；
- 用 `--once` echo `/joint_states` 循环探测状态再跑 service，同样受 CLI 冷启抖动影响，可靠性差。

## 正确做法 / 规避

**验收任何「时序/阈值/次数」语义，用纯 rclpy 一句脚本驱动**：同一进程内 `call_async` start_teach → `time.monotonic()` 打点 → `sleep(0.05)` → `call_async` stop_teach，实测 `start 墙钟 0.018 s / stop 墙钟 0.021 s / 记录 3 samples` → 正确触发 `auto-save skipped (3 samples < 10, keep previous latest)`。

万不得已用 bash CLI 时：先发一次**预热调用**（如一次无关的 `/a3/arm_status` 读取或一个无副作用的 service call）把 discovery 开销吃在与测量无关的调用上，再打点测量。

**教训：bash 里 `ros2 service call` 不是即时 RPC，首次/冷进程有秒级 DDS 开销；拿它做 start→stop 时间预算的断言 = 用尺子量头发。时序语义的验收一律 rclpy 单进程打点。**

## 关联

- [F54](../edge/REQUIREMENTS.md)（误触发阈值验收标准 #4）
- [LL-046](LL-046-playback-ramp-2p5s-too-fast.md)（同类：时序口径，视频取证优于 shell 秒表）