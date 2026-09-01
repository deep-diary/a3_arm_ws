# a3_mqtt_bridge

ROS2 → MQTT 遥测桥接节点：把 A3 应用话题（精选白名单）展平为 `points` 上报到 EMQX，
供 deep-trace 前端实时展示。需求见 [docs/edge/REQUIREMENTS.md](../../docs/edge/REQUIREMENTS.md) F18。

## 依赖

```bash
pip install --user paho-mqtt psutil pyyaml
```

（`psutil` 用于系统信息/CPU/内存/温度；`paho-mqtt` 与 `pyyaml` 为运行必需。）

> 本机 `.bashrc` 若设了 `PYTHONNOUSERSITE=1`，`pip --user` 安装的包不会被加载。
> 用 `ros2 launch` 启动时 launch 文件已自动 `PYTHONNOUSERSITE=""`；若用
> `ros2 run` 直启，请先 `unset PYTHONNOUSERSITE`。

## 构建

```bash
cd ~/a3_arm_ws
colcon build --packages-select a3_mqtt_bridge
source install/setup.bash
```

## 运行

```bash
ros2 run a3_mqtt_bridge ros2mqtt_bridge
# 或
ros2 launch a3_mqtt_bridge bridge.launch.py
```

Broker / 白名单在 [config/bridge.yaml](config/bridge.yaml) 配置。

## MQTT 话题（默认前缀 `deep-trace/HOME-DEMO/RK3588`）

| 话题 | 方向 | 说明 |
|------|------|------|
| `/device/info` | 发布（retained） | 板级信息 + 节点/话题/信号目录 |
| `/device/status` | 发布（retained，~1 Hz） | 心跳 + CPU/内存/温度 + 运行中节点 |
| `/telemetry` | 发布（每条 ROS 消息） | 展平后的 `points` |
| `/cmd` | 订阅（预留） | 双向交互骨架，暂未处理 |

## telemetry 载荷

```json
{
  "home_code": "HOME-DEMO",
  "work_unit_code": "RK3588",
  "device_program": "rk3588",
  "ts": "2026-09-01T22:00:00+08:00",
  "points": {
    "pos_L1": 0.12,
    "control_mode": "TRAJ_RUNNING",
    "gate_open": true
  }
}
```

## 验证

```bash
# 终端 A：观察上报
mosquitto_sub -h 192.168.3.73 -t 'deep-trace/HOME-DEMO/RK3588/#' -v

# 终端 B：确认 ROS 侧在发
ros2 topic hz /joint_states
```
