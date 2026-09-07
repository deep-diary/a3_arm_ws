# LL-008 — paho-mqtt 2.x 回调签名不匹配导致断线后崩溃、无法重连

> **日期：** 2026-09-06
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ ROS 2 Humble + paho-mqtt 2.x（`~/.local`）

## 现象

`ros2mqtt_bridge` 正常运行期间，一旦 broker 断连（换网络 / broker 重启 / 超时），日志出现：

```
File ".../paho/mqtt/client.py", line 4383, in _do_on_disconnect
    on_disconnect(
TypeError: Ros2MqttBridge._setup_mqtt.<locals>.on_disconnect() takes from 3 to 4 positional arguments but 5 were given
```

之后 MQTT 线程崩溃：遥测停止上报，且**不会自动重连**（进程看似还活着，但已从 MQTT 消失）。

## 根因

paho-mqtt **2.x** 在 `CallbackAPIVersion.VERSION2` 下，`on_disconnect` 回调签名是 5 参数：

```python
on_disconnect(client, userdata, disconnect_flags, reason_code, properties)
```

而回调只定义了 4 个形参 `(client, userdata, rc, properties=None)`。连接正常时此回调不被调用，问题只在**断线那一刻**暴露——属于「平时不报、断网必炸」的坑。

## 正确做法 / 规避

paho-mqtt 2.x 的 VERSION2 回调签名（与 1.x 不同）：

- `on_connect(client, userdata, flags, reason_code, properties)`
- `on_disconnect(client, userdata, disconnect_flags, reason_code, properties)`

回调定义直接按 VERSION2 写全 5 个形参（本仓已修：`on_disconnect(client, userdata, disconnect_flags=None, rc=None, properties=None)`，兼容 1.x 的 3 参数调用）。另注意 2.x 断线重连需要 `reconnect_delay_set()`（本仓已设 1~30s）。

**验证方法：** 起 bridge 后重启/断连 broker（或临时 `sudo iptables -A OUTPUT -p tcp --dport 1883 -j DROP` 再删除），观察日志应出现 `mqtt disconnected rc=...` 而非 TypeError，且 broker 恢复后自动重连并恢复遥测。

## 相关路径

- `src/a3_mqtt_bridge/a3_mqtt_bridge/ros2mqtt_bridge.py`（`_setup_mqtt` 内回调定义）
- `src/a3_mqtt_bridge/launch/bridge.launch.py`（`SetEnvironmentVariable("PYTHONNOUSERSITE","")` 加载 `~/.local` 的 paho-mqtt）
