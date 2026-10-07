# F101 — 看门狗硬化（硬件 watchdog + systemd 服务 watchdog 双层；控制环挂死/整机死锁确定性恢复）


- **说明：** 当前两层「挂死」场景均无确定性恢复：
  1. **整机/内核挂死**：RK3588 板载 Synopsys DesignWare 看门狗（`/dev/watchdog`，支持 SETTIMEOUT/MAGICCLOSE/KEEPALIVE）虽已随内核加载，但 systemd `RuntimeWatchdogSec=0`——没人喂狗，硬件超时不会复位整机。工业现场（无人值守 web 闭环）必须启用：systemd 以约半超时间隔喂狗，内核/调度/中断彻底死锁时硬复位。
  2. **控制栈挂死**：`a3-arm.service` 无 `WatchdogSec`——200 Hz 控制环死锁、`/joint_states` 停止刷新（F81 管「反馈陈旧后 read 报错」，但管不了进程整体 hang/死循环）时，服务永远停在僵尸式 active 态。标准做法（systemd sd_notify 协议）：单元改 `Type=notify` + `WatchdogSec=`，栈内一个极简喂狗节点订阅 `/joint_states` 与 `/a3/hardware/feedback_stale`，**仅当反馈帧新鲜且硬件未报 stale** 才经 `$NOTIFY_SOCKET` 发 `WATCHDOG=1`，并在首帧反馈时发 `READY=1`；反馈停更/冻结后停止喂狗，systemd 在超时后确定性重启整栈。配合 F86 电机侧 CAN 超时，重启窗口期电机自行失能，不会失控。
  超时取值：硬件看门狗 `RuntimeWatchdogSec=10`（实测 SETTIMEOUT(10) 实际 11 s，喂狗周期约 5 s）；服务 `WatchdogSec=15`（喂狗周期 2 s、反馈陈旧阈值 3 s）。PREEMPT_RT 与本项正交，不含。
- **改动：**
  1. 新增 `scripts/setup/setup_watchdog.sh`（幂等：装 `/etc/systemd/system.conf.d/99-a3-watchdog.conf` + daemon-reexec；需 sudo）
  2. 新增 `a3_bringup/systemd_watchdog_feed_node.py`（无 NOTIFY_SOCKET 时自动空转），接入 `a3_bringup.launch.py` 与 setup.py entry。喂狗条件：`/joint_states` 帧新鲜 **且** 硬件接口 latched 话题 `/a3/hardware/feedback_stale` 非 true——电机静默时 JSB 仍持续发布冻结的末帧（F81 freeze-hold 不返回 read ERROR），单靠帧到达会误判存活
  3. `systemd/a3-arm.service`：`Type=notify`、`NotifyAccess=all`、`WatchdogSec=15`
  4. 新增 `scripts/a3_test/f101_watchdog_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f101_watchdog_acceptance.py`）：**
  1. **硬件看门狗启用**：setup 脚本执行后 `systemctl show -p RuntimeWatchdogUSec` = 10 s（实测 DesignWare 将 SETTIMEOUT 向上取整，`show` 回报实际值 **11s**，10s/11s 均判通过）
  2. **服务就绪**：临时测试单元 `a3-f101test.service`（vcan101，`Type=notify`/`WatchdogSec=10`）在 vcan sim 存在时 30 s 内进入 active（READY=1 经首帧 /joint_states 发出），NRestarts=0
  3. **功能无回归**：enable → 标准两点 action 轨迹仍 SUCCESSFUL
  4. **挂死确定性重启**：杀掉 vcan_motor_sim（反馈停更、喂狗停止）后，单元在 25 s 内被 systemd 自动重启（NRestarts ≥ 1）
- **关联：** F93（systemd 产品单元）、F100（RT 调度）、F81（反馈陈旧 read 看门狗）、F86（电机侧 CAN 超时兜底重启窗口）、F90（黑匣子在被杀前留痕）
- **状态：** `completed`（2026-09-24，仿真验收 8/8：RuntimeWatchdogUSec 实测 11 s；临时 notify 单元 30 s 内 active、NRestarts=0；两点 action SUCCESSFUL；杀 vcan_motor_sim 后 **12.4 s** 内 NRestarts 0→1，= WatchdogSec 10 + RestartSec 2，与理论一致）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
