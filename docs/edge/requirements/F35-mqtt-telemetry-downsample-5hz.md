# F35 — MQTT 遥测降采样 5 Hz（全局）


- **说明：** 桥接节点按源话题速率发布全量遥测（叠加最坏 ~140–180 Hz，每条为 ~60 点 JSON），前端浏览器处理不过来，且 cmd（QoS 1）PUBACK 5 s 超时。改造 `a3_mqtt_bridge`：新增全局配置 `telemetry_min_interval_sec`（默认 0.2 = 5 Hz 上限），各话题回调只更新 `_points_cache` 置脏，由 0.05 s flusher 定时统一发布完整 points（最新值胜出）；发布路径解耦为有界队列（256）+ 独立发布线程，JSON 串行化与 paho publish 全部移出单线程 executor——慢 socket 不再阻塞遥测回调、cmd 分发与服务回执；telemetry 队列满可丢（最新值胜出），`cmd_result`/`device/status`/`device/info` 永不丢、不受节流；paho 线程内发布点（on_connect info、未知 op 回执）经队列后天然线程安全。
- **验收标准：**
  1. 生产栈遥测实测 ≤5 Hz（10 s 计数 ≈50 条），points 仍为全量聚合
  2. `a3_test.sh mqtt_cmd`（含 6 个 gripper op）与 `telemetry` 套件仍 PASS；cmd 下发后 `cmd_result` ~1 s 内到达
  3. web 端 cmd 发布（QoS 1）不再触发 5 s PUBACK 超时（前端零改动）
  4. broker 断连/慢 socket 期间 executor 不卡死：状态/回执重连后正常恢复
- **关联：** F18（MQTT 遥测桥）、F23（cmd 契约）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（遥测速率契约）；`a3_mqtt_bridge/config/bridge.yaml`
- **状态：** `implemented`（2026-09-07 真机实测：连续运动期 12 s 计 55 条 ≈4.6 Hz、间隔中位 0.232 s ≈5 Hz 上限；`a3_test.sh mqtt_cmd` 18/18、`telemetry` 6/6 PASS，cmd_result 均 <1 s 回达；顺带修复 SIGINT 退出竞态 exit code 1，见 LL-016。web 端 PUBACK 不再超时由前端观察复验）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
