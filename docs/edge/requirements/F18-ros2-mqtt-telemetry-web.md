# F18 — ROS2→MQTT 遥测上报与 Web 实时展示（跨仓）


- **说明：** 在 RK3588 上新增 `a3_mqtt_bridge` 包，订阅 A3 应用话题（精选白名单，YAML 可配），把每条消息展平为 `points`，经 MQTT（EMQX `192.168.3.73`）上报 `deep-trace/HOME-DEMO/RK3588/{device/info,device/status,telemetry}`。deep-trace 前端（外部仓库 `deep-trace`，分支 `rk3588`）以静态 seed 预置 `rk3588` 设备，详情页展示系统信息 + ROS 节点卡片 + 话题/信号两级下拉 + 带 dataZoom 缩放的实时曲线；预留 MQTT `cmd` 下行骨架用于后续双向交互。
- **验收标准：**
  1. 起 bridge 后 `mosquitto_sub -h 192.168.3.73 -t 'deep-trace/HOME-DEMO/RK3588/#' -v` 能观察到 `device/info`、`device/status`、`telemetry`
  2. `telemetry` 载荷 `points` 键与设备 YAML `points` 对齐，`ts` 为 ISO8601
  3. 浏览器 `/homes/HOME-DEMO/devices` 可见 `rk3588` 设备；详情页系统信息卡有值、节点卡片在线
  4. 选 `motor_protocol_node → /joint_states → pos_L1..L7` 曲线实时刷新且可 dataZoom 缩放
  5. bridge 断连可重连；`device/info` 以 retained 发布，重启后前端仍可取到板级信息
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；外部前端仓库 `deep-trace`（本机 `/home/cat/deep-trace`，分支 `rk3588`）
- **状态：** `in progress`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
