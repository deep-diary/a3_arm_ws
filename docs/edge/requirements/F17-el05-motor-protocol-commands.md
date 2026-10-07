# F17 — EL05 电机协议命令集补齐（L0 增强）


- **说明：** 对照 EL05 电机说明书（外置固件 `main/boards/deep-dog/motor/protocol_motor.*`）补齐 `motor_protocol_node` 除 MIT 运控（通信类型 1）与反馈解析（类型 2/24）之外的命令编解码，并在节点暴露 ROS 服务。覆盖：获取设备 ID（类型 0，含 MCU UID 应答）、使能（3）、失能（4）、设零（6）、**设置电机 CAN_ID（类型 7，立即生效）**、软件版本（0x17）、写参数（18，通用 float 与 raw）、主动上报开关（24）与上报周期换算（EPScan_time）。
- **验收标准：**
  1. 各命令帧 CAN ID 与数据域与固件 `buildCanId` / `buildMitControlCanId` / `setCanId` / `setMotorParameter` 一致（可用 `cansend` 回读对照）
  2. `SET_CAN_ID`（类型 7）：`ID = (7<<24) | (new_id<<16) | (0xFD<<8) | current_id`，data 全 0
  3. 服务 `/a3/motor/{enable,reset,set_zero,get_device_id,request_version,set_can_id,set_param}` 可被 `ros2 service call` 正常调用
  4. `OnRxFrame` 能解析设备 ID 应答与软件版本应答，发布到 `/a3/motor/device_id` / `/a3/motor/version`
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) L0；`a3_can_bridge`
- **状态：** `implemented`（编解码 + 服务；真机板测待办）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
