# F102 — MQTT 通信链路健康入标准诊断（断链可观测、10 s 升级 ERROR；Comms 聚合分组；env 覆盖 broker）


- **说明：** web 闭环是产品运行模式，但 MQTT 链路健康只存在于 bridge 节点内部（`_mqtt_connected`）与日志：EMQX 不可达 / 网线脱落 / broker 宕机时，外部系统（监控看板、`/diagnostics_agg` 现场状态、黑匣子）无从感知——上行遥测静默停更，节点不退出（设计为自动重连）。工业现场要求通信通道作为标准设备状态上报：
  1. bridge 接入 `diagnostic_updater`，发布任务 `ros2mqtt_bridge: MQTT link`：连接正常 OK；断连起 10 s 内 WARN，超过 **10 s** ERROR（level 随 `on_connect`/`on_disconnect` 实时更新），消息含 broker host:port 与断连持续时长。
  2. 新增 latched（transient_local）话题 `/a3/comms/mqtt_connected`（std_msgs/Bool），连接态翻转即发布，供其他节点/F101 式看门狗消费。
  3. aggregator 配置新增 `Comms` 分组（startswith `ros2mqtt_bridge:`），链路 ERROR 进入 `/diagnostics_toplevel_state`。
  4. broker 地址支持环境变量覆盖（部署灵活性，验收用）：`A3_MQTT_HOST` / `A3_MQTT_PORT` 覆盖 config/bridge.yaml。
  - 下行安全语义不变：断连期间下行指令天然不可达；所有下行运动都是服务触发的有限轨迹，链断后臂保持末位，无失控路径。
- **改动：**
  1. `src/a3_mqtt_bridge/a3_mqtt_bridge/ros2mqtt_bridge.py`：diagnostic_updater + latched 话题 + 连接态时间戳 + env 覆盖
  2. `src/a3_bringup/config/diagnostics.yaml`：Comms 分析器
  3. 新增 `scripts/a3_test/f102_mqtt_link_health_acceptance.py`（本地 mosquitto broker，隔离域 102）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f102_mqtt_link_health_acceptance.py`）：**
  1. broker 在线：15 s 内 `/a3/comms/mqtt_connected` = true，`MQTT link` 诊断 OK
  2. 杀掉 broker：15 s 内 latched = false；断连满 10 s 后诊断 level = ERROR（共 ≤ 20 s）
  3. broker 重启：30 s 内 latched 恢复 true、诊断恢复 OK
  4. aggregator（以产品 diagnostics.yaml 启动）输出 `/A3/Comms/ros2mqtt_bridge: MQTT link`，断连时该组 ERROR
- **关联：** F90（黑匣子）、F96（主机诊断）、F82（aggregator）、F101（看门狗双层）
- **状态：** `completed`（2026-09-24，仿真验收 8/8：连接 0.7 s 后 latched=true、诊断 OK；aggregator `/A3/Comms/...` 0.4 s 内 OK；杀 broker 后 0.1 s latched=false、断连 10.5 s 诊断升 ERROR、aggregator 组 0.5 s 内同步 ERROR；broker 重启后 4.1 s latched 恢复 true、0.8 s 诊断+聚合双 OK）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
