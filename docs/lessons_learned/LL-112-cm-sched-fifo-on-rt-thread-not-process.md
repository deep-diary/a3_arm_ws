# LL-112 — controller_manager 的 SCHED_FIFO 设在专用 RT 线程上而非进程；PAM limits 对 systemd 服务不生效

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 lubancat, Linux 6.1.84（标准 SMP，非 PREEMPT_RT）, ROS 2 Humble, ros2_control

## 现象

F100 验收要确认 controller_manager 跑在 SCHED_FIFO 下。部署 realtime 组 + pam_limits 后，栈日志明确打印：

```
Spawning controller_manager RT thread with scheduler priority: 50
Successful set up FIFO RT scheduling policy with priority 50.
```

但按「查进程调度策略」的直觉执行 `chrt -p <ros2_control_node pid>`，得到的却是：

```
当前的调度策略: SCHED_OTHER
pid ... 的当前调度优先级: 0
```

一度怀疑 sched_setscheduler 失败或权限没传到栈进程。

## 根因

controller_manager **不修改进程级调度策略**：它在启动时 `std::thread` spawn 一个专用 **RT 线程**，只对该线程（`pthread_setschedparam` / sched_setscheduler with pid=tid）设置 `SCHED_FIFO`、优先级 50。进程主线程及其余线程保持 `SCHED_OTHER`。所以：

- `chrt -p <pid>`（= 查主线程）永远显示 SCHED_OTHER，即使 RT 配置完全成功；
- 正确观测面是线程：`/proc/<pid>/task/*` 逐个 `chrt -p <tid>`，找到 `SCHED_FIFO` / 优先级 50 的那个线程。

部署侧同时踩了第二个坑：往 `/etc/security/limits.d/` 装好 `@realtime rtprio/memlock` 后，当前 SSH 会话里 `ulimit -r` 仍是 0。原因有二：

1. **pam_limits 只在 PAM 会话建立时求值**——已登录会话、`sudo` 提权都不会重新加载；必须重新登录/SSH 重连，或用 `su - <user>` 开新登录会话验证。
2. **PAM limits 对 systemd 服务完全不生效**（systemd 不走 PAM 会话）。`a3-arm.service` 必须在单元里直接写 `LimitRTPRIO=99`、`LimitMEMLOCK=infinity`，否则开机栈依旧拿不到 RT 权限。

## 正确做法 / 规避

- 验证 ros2_control 实时性：先在日志里找 `Successful set up FIFO RT scheduling policy with priority 50.`；再扫描线程而非进程：
  ```bash
  pid=$(pgrep -f ros2_control_node | head -1)
  for t in /proc/$pid/task/*; do chrt -p "${t##*/}"; done | grep SCHED_FIFO
  ```
  RT 线程在 spawn controllers 前后才出现，轮询要留几秒窗口。
- PAM 侧部署（交互/登录）：`realtime` 组 + `/etc/security/limits.d/99-a3-realtime.conf`，用幂等脚本 `scripts/setup/setup_realtime.sh`；验证用 `su - <user> -c 'ulimit -r; ulimit -l'`（期望 99 / unlimited），不要在老会话里查。
- systemd 侧部署：单元内 `LimitRTPRIO=` / `LimitMEMLOCK=`，与 PAM 配置是两条独立路径，缺一不可。
- 标准 SMP（非 PREEMPT_RT）内核上 SCHED_FIFO 已保证控制线程优先于一切 SCHED_OTHER 负载，是不换内核的工业基线；但 SCHED_FIFO 线程内若发生缺页/锁等待仍会阻塞，故 `memlock unlimited` 必须同时给。

## 相关路径

- `scripts/setup/setup_realtime.sh`
- `systemd/a3-arm.service`
- `scripts/a3_test/f100_realtime_scheduling_acceptance.py`（find_fifo_thread）
