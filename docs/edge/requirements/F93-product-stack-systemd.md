# F93 — 产品栈 systemd 托管（版本化开机单元 + 崩溃自动重启；默认禁用，显式启用）


- **说明：** 现状产品栈靠登录后手工 `ros2 launch`：进程崩溃无人拉起、断电恢复不会自启、日志只在终端。工业现场标配是 init 系统托管：单元文件随仓库版本化、安装到 `/etc/systemd/system/`、输出进 journald；**默认 disabled**——未经现场人工确认不得让机械臂开机自动使能，由运维显式 `systemctl enable`。
  - `After=can-up.service network-online.target` + `Wants=can-up.service`：CAN 接口先行就绪，网络就绪供 MQTT/EMQX
  - `Restart=on-failure` + `RestartSec=5` + `StartLimitIntervalSec=60` / `StartLimitBurst=3`：崩溃自动拉起；60 s 内连续快速失败 3 次即停止重启（故障状态下避免反复使能），转人工处置
  - `KillSignal=SIGINT` + `TimeoutStopSec=20` + `KillMode=mixed`：SIGINT 给 launch 主进程优雅关停全栈，超时再 SIGKILL 残留
  - `Environment` 固化 `PYTHONNOUSERSITE=1`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（LL-104）；支持 `EnvironmentFile=-/etc/default/a3-arm` 现场覆盖而不改单元
- **改动：**
  1. 新增 `systemd/a3-arm.service`、`systemd/a3-arm.default`（现场环境文件范例）
  2. 新增 `scripts/setup/install_a3_service.sh`：拷贝单元 + daemon-reload；**不 enable、不 start**
  3. 新增 `scripts/a3_test/f93_systemd_acceptance.py`：mock 模式全自动验收（需 sudo，密码经 `A3_SUDO_PASS` + askpass；`hardware:=mock` 不触碰 CAN）
- **验收标准（仿真；机械臂保持断电）：**
  1. `systemd-analyze verify systemd/a3-arm.service` 退出码 0、无 error
  2. 安装后 `systemctl is-enabled a3-arm` = `disabled`、`is-active` = `inactive`
  3. 临时 mock drop-in 启动：90 s 内 `/joint_states` 恢复且 3 s 窗口 ≥100 条（实测 mock 200 Hz）；调 `/a3/arm/enable`（mock 安全）后 arm_controller / gripper_controller / joint_state_broadcaster 均 active
  4. `systemctl kill -s SIGKILL a3-arm` 模拟崩溃 → 自动重启，重启后 120 s 内 joint_states 恢复、NRestarts ≥ 1
  5. `systemctl stop` 后 10 s 内无残留 controller_manager / ros2 launch 进程；删除临时 drop-in；最终状态 disabled/inactive（全程未 enable）
- **关联：** `systemd/can-up.service`；对标工业现场开机自启 + 进程看门狗；LL-104（Cyclone 固化）、LL-105（systemd 249 键位 / Humble echo 无 --timeout / ANSI 解析）。真机急停/断电链路在 systemd 下的优雅关停需通电时补验
- **状态：** `completed`（2026-09-23，mock 全链路验收 17/17：verify 干净、安装保持 disabled/inactive、起栈后 joint_states 3 s 收到 600 条、enable 后三控制器 active、SIGKILL 后 NRestarts 0→1 且 120 s 内恢复、stop 无残留；LL-105）。真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
