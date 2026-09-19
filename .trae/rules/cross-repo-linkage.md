<!-- cursor-meta: description=跨仓联动：deep-trace Web 仓 / xiaozhi-esp32 固件仓 / a3_arm_vendor 软件克隆 / MQTT 契约一致性约束 | alwaysApply=true -->

# 跨仓联动路径

本仓（a3_arm_ws：ROS 2 全栈 + 双产品线契约）与下列仓库联动开发。涉及 MQTT Topic/JSON、
话题契约、遥测/指令协议时，主动查阅对应仓库，保持契约一致。

## 路径一览

| 角色 | 路径 | 说明 |
|---|---|---|
| Web 前端/后端（deep-trace） | RK3588 板 `/home/cat/deep-trace`，分支 `rk3588` | 遥测 Web 展示；WSL 开发机通常**没有**该仓 |
| CloudEdge 固件（xiaozhi-esp32） | 外置仓库，路径随机器变化，先探测（见 `iot-firmware-path.md`） | ESP32-S3 micro-ROS，板级 `main/boards/deep-dog/` |
| reBot 软件克隆 | 兄弟目录 `../a3_arm_vendor/` | `reBotArm_control_py`、`reBotArmController_ROS2` |
| 生产执行层来源 | trotbot 派生 | `a3_can_bridge`、`a3_teleop_ps4` |
| 参考开源项目 | 见 `reference-first.md` | reBot-DevArm / EDULITE_A3 等 |

## 契约来源

- 本仓需求：`docs/edge/REQUIREMENTS.md`（Edge 主线）、`docs/cloud_edge/REQUIREMENTS.md`（CloudEdge）
- MQTT 契约：`src/a3_mqtt_bridge/config/bridge.yaml` ↔ deep-trace `edge/device_firmware/rk3588/config/HOME-DEMO.RK3588.yaml`（`points[].code` 两侧必须一致，见 `mqtt-contract-coupling.md`）
- 话题/关节契约：`docs/shared/TOPIC_CONTRACT.md`（`L1_joint`..`L7_joint`，L7 为夹爪）
- 控制对标：`docs/shared/CONTROL_ROADMAP.md`

## 变更约束

- 修改协议字段或 Topic 时：先改本仓需求文档（`docs/edge/REQUIREMENTS.md` / `docs/cloud_edge/REQUIREMENTS.md`），再同步修订 deep-trace / 固件侧文档或实现。
- 不要假设其他仓库在当前 workspace 可写；跨仓改动应明确说明目标路径。
- CloudEdge 真实 CAN↔micro-ROS 桥接在 xiaozhi-esp32 固件实现，替换模拟器时契约保持不变。
- Web 展示（前端/后端）只改 deep-trace（RK3588 板上）；本仓只做 ROS 侧与契约文档。
