# LL-054 — 反复 SIGKILL rclpy 注入进程会污染本机 FastDDS 运行时，新 participant 的 reliable publish 永久卡死

> **日期：** 2026-09-17
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble（默认 FastDDS） + 真机主栈在跑 + 多域（0/42/50/60）隔离冒烟

## 现象

隔离验证目标 ghost 注入通道（`/a3/display_target_joint_states`）：publisher 发布 JointState 到 `/tf` 生成 `target/` 帧。**同一段注入代码，首次在干净域（42）A/B 两进程对照：7/8 条全部送达（首条因发现延迟丢失属正常）。** 但在反复 `kill -9` 掉注入器进程之后，新起的任何注入器进程——无论是原域、无 rsp 的域、还是**全新 ROS_DOMAIN_ID 的干净域**——都卡死在**第一次 publish 之后的循环迭代**（`pulse 0.0` 打了，第二个循环体不进），日志无 traceback、进程 `Ssl`、主线程 `futex_wait_queue`；`timeout` 默认 SIGTERM 都杀不掉（被 python 的信号处理挂着，要 `kill -9`）。

同一时刻真机主栈 `/joint_states` 仍 50 Hz 正常发布——它早已建立的连接不受影响；受挫的只有**新起的 participant**。

## 根因

- 真机主栈一直在跑，**每 `kill -9` 一个 rclpy 进程，它的 FastDDS participant 都不会发 DISPOSE 善后**（进程直接没了），在 discovery 图里留下 **幽灵 participant / 幽灵 writer**；
- 幽灵不清（liveliness 默认 infinite，不会自动驱逐），后续任何新 participant 与该 topic 匹配时要去**与死 writer 建立可靠会话**，reliable 发布等 ack 等不到 → writer 发送队列/重发线程耗尽 → `publish()` 永久阻塞；
- 传染性：多域反复注入调试把幽灵数量累积到某个阈值后，**连全新域的新 participant 也开始卡**（本机 DDS 资源表已污染）；
- 与 QoS 无关：RELIABLE、BEST_EFFORT 两种注入都复现；与 rsp_target 在场无关（无 rsp 的纯域也复现）。

## 正确做法 / 规避

1. **调试注入器这类长循环进程，结束时用正常 `rclpy.shutdown()` 优雅退出，不要 `kill -9`**（更不要 `pkill -f`——它会把命令行里含同样字符串的当前 shell 也杀掉，自杀两次的教训）；确有残留要清理时，`ps -eo pid,args | awk` 取 PID 再精确 kill。
2. **被污染后最可靠的恢复 = 重启全部 ROS 进程**（真机栈或模拟栈整体重启，discovery 内存态归零）；清 `/dev/shm` 下的 `fastrtps_*` 残留段（属已死 participant）可一并做。
3. 「注入 → `/tf` 出 ghost 帧」这类**端到端链路验收，选一次性干净环境**（全新 ROS_DOMAIN_ID，只起被测两端），先跑通再谈别的；验收脚本里禁止在观测窗口内反复起/杀同 topic 的进程。
4. `timeout` 杀卡死 python 要带 `-k`（如 `timeout -k 2 20 …`）否则 SIGTERM 被 python 挂着不死。

**教训：SIGKILL rclpy 长循环进程留下的 FastDDS 幽灵会污染整台机器的后续 publish，症状伪装成「代码不通」；长循环注入进程要优雅退出，污染后重启整栈，别在同一跑栈里反复起杀同 topic 进程。**

**复测确认（2026-09-17，原幽灵注入进程均已死后数小时）：** 全新 `ROS_DOMAIN_ID=42` 干净环境、对端无任何发布者，**全新 participant 仍卡死在第一次 RELIABLE `publish()`**——脉冲停在 `pub ok` 后一帧或一帧不出，同域 `ros2 topic echo` 6 s 收不到任何消息。说明污染不止来自活跃幽灵会话，而是**固化在本机 DDS 共享内存/资源表**（`/dev/shm` 下 `fastrtps_*` 段），幽灵进程消失也不会自愈。唯一恢复手段不变：`rm -f /dev/shm/fastrtps_*` + 重启全部 ROS 进程（或整机重启）。注意：复测注入器本身也是 `timeout -k` SIGKILL 掉的（exit 137）——**每次复测又给本机新增幽灵，栈重启前不要再跑任何 inject/echo 复测**。

## 关联

- 目标 ghost 双模型注入通道（`el_a3_dual_view.rviz` + `a3_bringup.launch.py` rsp_target）
- [LL-050](LL-050-ros2-cli-discovery-overhead-timing.md)（同类 DDS 环境坑：discovery 开销/冷启时序）