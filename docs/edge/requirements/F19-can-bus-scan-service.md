# F19 — 总线扫描服务（通信类型 0 范围探测）


- **说明：** 在 `motor_protocol_node` 暴露 `/a3/motor/scan` 服务，对 `[id_min, id_max]` 内每个 CAN_ID 发送通信类型 0 获取设备 ID 探测帧（只发不等，对齐固件 `sendGetDeviceIdProbes`）。应答帧（cmd=0 且 bit0-7=0xFE，motor_id=bit8-15，data 为 8 字节大端 MCU UID）经 `OnRxFrame` 解析后发布到 `/a3/motor/device_id`，用于验证机械臂 7 关节电机是否全部在线、以及 `SET_CAN_ID` 改号后复核。
- **验收标准：**
  1. `ros2 service call /a3/motor/scan "{id_min: 1, id_max: 127, bus: 0}"` 返回 `sent=127`
  2. 扫描期间 `ros2 topic echo /a3/motor/device_id` 能列出总线在线电机的 motor_id 与 UID
  3. 配合 `SET_CAN_ID` 改号后重新扫描，在线 ID 列表随之变化
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) L0；`a3_can_bridge`
- **状态：** `implemented`（服务 + 编解码；真机板测见 F17）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
