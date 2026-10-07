# F90 — 故障触发有界 rosbag2 黑匣子（snapshot-mode 循环缓冲，FAULT/COOLING 边沿自动落盘）


- **说明：** 工业控制器标配故障黑匣子（事件前后一段历史自动留存供复盘）。不手搓录制器：ROS 2 官方 `rosbag2_transport` 提供 snapshot mode——常驻进程只把消息留在固定大小的内存循环缓冲（不落盘、无磁盘增长），被调 `/rosbag2_recorder/snapshot`（**服务类型 `rosbag2_interfaces/srv/Snapshot`，不是 std_srvs/Trigger**）时把当前缓冲写成分片 bag（每次触发一个独立分片文件，天然有界）。产品 bringup 默认经 `ros2 bag record --snapshot-mode` 起该录制器（`use_rosbag` 可关），FSM 在**进入 FAULT/COOLING 的边沿**（电机故障/安全停车超时/过热保护等所有故障入口都汇聚于 `_set_state`，含过温保护走 COOLING 的路径）异步触发一次 snapshot：fire-and-forget，不阻塞故障处置路径；录制器不在（use_rosbag:=false / 尚未发现）则静默跳过。录制话题最小集：`/joint_states`、`/a3/arm_status`、`/a3/control_mode`、`/diagnostics`、`/diagnostics_toplevel_state`、`/arm_controller/joint_trajectory`。
- **改动：**
  1. `a3_bringup.launch.py` 增参 `use_rosbag`（默认 true）、`bag_dir`（默认 `~/.a3/blackbox`）；`ExecuteProcess` 起 `ros2 bag record --snapshot-mode --max-cache-size 33554432 --max-bag-size 67108864 --storage mcap -o <bag_dir>/blackbox_<启动时间戳>`（时间戳在 launch 生成期取本地时间，每次启动唯一目录）。
  2. `arm_controller.py` 新增 `rosbag2_interfaces/srv/Snapshot` 客户端 `/rosbag2_recorder/snapshot`；`_set_state` 检测到旧态≠FAULT、新态=FAULT 且 `service_is_ready()` 时 `call_async`（响应回调仅记日志），不满足就绪条件直接跳过。
- **验收标准（仿真；断电；脚本 `scripts/a3_test/f90_blackbox_acceptance.py`，vcan90 标准栈，ROS_DOMAIN_ID=90）：**
  1. 启动后 `~/.a3/blackbox` 下出现本次启动的 mcap 目录；故障前目录内无消息分片（snapshot 未触发，磁盘不增长）
  2. enable→READY 后经 `/tmp/f84_health.json` 注入电机故障（motor 5 fault=4）：FSM 进入 FAULT；无需任何手工调用，新分片自动出现且包含 FAULT 前/后的 `/joint_states`、`/a3/arm_status` 等录制话题消息（`ros2 bag info` 可读，消息数 > 0，时长跨故障时刻）
  3. 有界性：循环缓冲 32 MiB / 分片 64 MiB 参数生效（bag info / 文件大小验证）；重复故障每次只多一个分片
  4. `use_rosbag:=false` 时无录制器进程，FSM 照常进 FAULT，不报错不阻塞
- **关联：** F44/F84（电机故障→FAULT）、F81（冻结保持，事件源之一）、F82（诊断话题）；对标工业控制器事件黑匣子
- **状态：** `implemented`（2026-09-23 仿真 17/17 全绿；2026-10-05 真机验收通过：过温保护触发 COOLING 边沿自动 snapshot 落盘 `blackbox_20261005_160138_0.mcap` 28.6MB，日志 `blackbox snapshot flushed` 确认）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
