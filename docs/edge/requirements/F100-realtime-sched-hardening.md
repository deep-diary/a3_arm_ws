# F100 — 实时调度权限硬化（controller_manager 真正跑上 SCHED_FIFO；堵住 200 Hz 控制环被普通负载抢占）


- **说明：** 每次起栈，`controller_manager` 都打印 `Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted)`——当前会话 `ulimit -r = 0`，CM 只能以 `SCHED_OTHER` 普通分时策略运行，200 Hz 的 read/update/write 控制环不被任何实时性保护：RK3588 上编译、录包、MQTT 突发等普通负载即可抢占控制线程造成抖动（轨迹跟踪容差 F97 在真机上可能误触发）。这是 ros2_control 官方文档明确要求的部署项（[real-time setup](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)）。标准做法两部分：
  1. **交互式/登录会话**：建 `realtime` 组并把运行用户加入；`/etc/security/limits.d/99-a3-realtime.conf` 给该组 `rtprio 99` + `memlock unlimited`（PAM pam_limits 在登录时生效）。
  2. **systemd 产品单元**：PAM limits **对 systemd 服务不生效**，必须在 `a3-arm.service` 直接配 `LimitRTPRIO=99` 与 `LimitMEMLOCK=infinity`。
  内核为标准 SMP（非 PREEMPT_RT）：SCHED_FIFO 已保证控制线程优先于任何普通用户负载（含本仓仿真/录包），是不换内核下的工业标准基线；升级 PREEMPT_RT 内核属于需现场确认的独立项，本项不含。
- **改动：**
  1. 新增 `scripts/setup/setup_realtime.sh`（幂等：建组/加用户/装 limits.d；需 sudo）
  2. `systemd/a3-arm.service`：`LimitRTPRIO=99`、`LimitMEMLOCK=infinity`
  3. 新增 `scripts/a3_test/f100_realtime_scheduling_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f100_realtime_scheduling_acceptance.py`）：**
  1. **PAM 会话权限**：setup 脚本执行后，新登录会话（`su - <user>`）`ulimit -r` = 99、`ulimit -l` = unlimited
  2. **无降级警告**：该登录会话起栈后，栈日志**无** `Could not enable FIFO RT scheduling policy`
  3. **策略生效**：controller_manager 的 **RT 线程**（非进程主线程）调度策略为 `SCHED_FIFO`、优先级 50——CM 在起栈时 spawn 专用 RT 线程并只对该线程 `sched_setscheduler`，主线程仍是 SCHED_OTHER；故须扫描 `/proc/<ros2_control_node pid>/task/*` 逐个 `chrt -p <tid>`（栈日志同步出现 `Spawning controller_manager RT thread with scheduler priority: 50` + `Successful set up FIFO RT scheduling policy with priority 50.`）
  4. **功能无回归**：enable → 标准两点 action 轨迹仍 SUCCESSFUL
- **关联：** F93（systemd 产品单元）、F70/F72（200 Hz 标准控制栈）、F97（跟踪容差依赖确定性周期）
- **状态：** `completed`（2026-09-24，vcan 验收 5/5）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
