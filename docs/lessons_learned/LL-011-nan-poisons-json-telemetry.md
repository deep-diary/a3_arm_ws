# LL-011 — NaN 毒化 JSON：joint_states 的 NaN 速度经 json.dumps 产出非法 JSON，前端整包丢弃

> **日期：** 2026-09-06
> **产品线：** Edge / 共用
> **环境：** RK3588 + ROS 2 Humble + EMQX + 浏览器 mqtt.js（Vue 3 前端）

## 现象

浏览器端机械臂页面所有 telemetry 衍生 UI（夹爪状态/模式、arm 编排状态、关节数据、曲线）全部显示「—」，只有系统信息正常；页面报文计数仍在增长。板侧用 Python paho 订阅验证遥测「一切正常」，误判为前端问题。

## 根因

- `/joint_states`（`sensor_msgs/JointState`）对未连接电机的关节 L1–L6 发布 `.nan` 速度。
- 桥接节点把所有信号合并进 points 缓存后，用 Python `json.dumps` 默认 `allow_nan=True` 直接输出 `"vel_L1": NaN`——**NaN 不是合法 JSON 词法**。
- 浏览器 `JSON.parse` 遇到 NaN 抛 SyntaxError，整包遥测被丢弃（报文计数在 parse 之前已 +1，所以计数仍在涨）；`device/info` 里没有 NaN，系统信息不受影响。
- 板侧验证用 Python `json.loads` 恰好**容忍 NaN**（Python 扩展），掩盖了问题；换 Node/浏览器立即复现。

## 正确做法 / 规避

- **ROS 侧（源头兜底）**：发布前递归清洗非有限浮点（NaN/±Inf → `None`），并 `json.dumps(..., allow_nan=False)` 双保险——漏网时直接抛异常落日志，而不是污染线上所有订阅者。
- **前端（防御纵深）**：`JSON.parse` 失败时先把 `/\b(?:-?Infinity|NaN)\b/g` 替换为 `null` 重试；仍失败才丢弃并计数/留原文用于诊断。
- **验证时**：板侧断言必须用严格解析器（`json.dumps(obj, allow_nan=False)` 或对原文做 `NaN|Infinity` 词法正则），**不要只依赖 Python `json.loads`**——它接受 NaN。

## 相关路径

- `src/a3_mqtt_bridge/a3_mqtt_bridge/ros2mqtt_bridge.py`（`_sanitize_nonfinite`、`_publish`）
- deep-trace 外部仓库：`frontend/src/composables/useRk3588Mqtt.js`（容错解析 + `parseErrorCount`）
- 需求：[docs/edge/REQUIREMENTS.md](../edge/REQUIREMENTS.md) F31
