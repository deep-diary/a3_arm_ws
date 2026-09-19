<!-- cursor-meta: description=MQTT 契约耦合：a3_mqtt_bridge 信号 code 与 deep-trace 设备 YAML 两端耦合，改契约两端同改 | alwaysApply=true -->

# MQTT 契约耦合

ROS 侧 `a3_mqtt_bridge` 与外部 Web 仓 deep-trace 存在**直接契约耦合**：
桥的 `bridge.yaml` 信号 code 必须与 deep-trace 设备 YAML 的 `points[].code` 完全一致，
否则遥测在 Web 端无法显示、指令无法触发。

## 适用范围

修改以下内容时本规则生效：`src/a3_mqtt_bridge/config/bridge.yaml`、`src/a3_mqtt_bridge/**`、
deep-trace `edge/device_firmware/rk3588/config/HOME-DEMO.RK3588.yaml`、
deep-trace 前端 `frontend/src/views/iot/components/Rk3588HubPanel.vue` / `frontend/src/composables/useRk3588Mqtt.js`。

## 耦合点

| ROS 侧（a3_arm_ws） | 消费方（deep-trace） | 说明 |
|---|---|---|
| `src/a3_mqtt_bridge/config/bridge.yaml`（信号 code / topic / op 白名单） | `HOME-DEMO.RK3588.yaml`（`points[].code`） | 两侧 code 必须对齐，否则 telemetry 无曲线、cmd 无响应 |
| 话题前缀 `deep-trace/HOME-DEMO/RK3588/` | 前端订阅同一前缀（info / status / telemetry / cmd / cmd_result） | 前缀/op 命名一致性 |
| op 白名单（如 `set_joints` → `/a3/arm/set_joint_positions`） | 前端 `ArmJointSliders.vue` 等下发的 op 与参数 | 指令契约 |

## 改动检查

修改 MQTT 契约时，务必同步检查：

1. `src/a3_mqtt_bridge/config/bridge.yaml` 与 deep-trace 设备 YAML `points[].code` 是否对齐
2. `docs/shared/TOPIC_CONTRACT.md` 是否已更新
3. 前端展示（`Rk3588HubPanel.vue` / `useRk3588Mqtt.js`）是否受影响
4. 联调顺序：deep-trace 先 `python manage.py load_device_config` 合入 YAML，再起 `a3_mqtt_bridge`

## 禁止

- 不要只改 ROS 侧 YAML 而忽略 deep-trace 设备 YAML（会导致 telemetry 无法展示）
- 不要在本仓直接改 deep-trace 的前端/后端代码（外部仓，在 RK3588 板上修改）
