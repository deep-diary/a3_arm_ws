# LL-113 — 服务看门狗喂狗不能只看 /joint_states 帧新鲜：JSB 在 write 冻结后持续发末帧

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 lubancat, ROS 2 Humble, systemd 249, vcan

## 现象

F101 验收检查 4（杀掉 vcan_motor_sim → Type=notify 单元须在 25 s 内重启）首次实现时一直不触发：喂狗节点持续收到 `/joint_states`，持续发 `WATCHDOG=1`，systemd 认为服务健康。

## 根因

ros2_control 的 joint_state_broadcaster 发布的是 hardware interface 的**状态缓存**，不直接感知 CAN/电机死活。F81 freeze-hold 语义下，write() 在反馈超时时冻结在末位置、read() 不报错，于是：

- `/joint_states` 以 50 Hz 正常发布，数值是冻结的 last-known 状态；
- 单靠「帧到达时间」判活，会把「电机静默 + 冻结保持」误判为健康。

## 正确做法 / 规避

1. 喂狗条件必须双因子：`/joint_states` 帧新鲜 **且** 硬件接口 latched 话题 `/a3/hardware/feedback_stale` 非 true（插件检测单电机反馈停更时置位）。帧新鲜抓进程 hang，stale 标志抓死电机/冻结保持。
2. systemd `[Manager]` 段（RuntimeWatchdogSec 等）只在 **daemon-reexec** 后读取，daemon-reload 不生效——setup 脚本里必须 `systemctl daemon-reexec`。
3. 重启延迟 = `WatchdogSec` + `RestartSec`：实测 WatchdogSec=10/RestartSec=2 → 杀 sim 后 12.4 s NRestarts+1，验收窗口据此设。
4. 新增/修改 entry point 与 launch 后必须重编 a3_bringup；用旧 install 跑验收出现「行为与代码不符」时先怀疑构建未更新，不要怀疑设计。
5. DesignWare 看门狗 SETTIMEOUT(10) 内核向上取整，`systemctl show RuntimeWatchdogUSec` 回报 11s，10s/11s 均算通过。

## 相关路径

- src/a3_bringup/a3_bringup/systemd_watchdog_feed_node.py
- scripts/setup/setup_watchdog.sh
- scripts/a3_test/f101_watchdog_acceptance.py
- 关联 [[LL-112-cm-sched-fifo-on-rt-thread-not-process]]
