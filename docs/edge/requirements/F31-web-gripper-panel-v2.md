# F31 — Web 端夹爪面板 v2：位置模式直驱 / 双曲线 / NaN 遥测毒化修复（跨仓）


- **说明：** 真机 2026-09-06 联调发现夹爪面板状态/曲线全为空：`/joint_states` 对未连接关节 L1–L6 发布 NaN 速度，桥接层 `json.dumps` 默认 `allow_nan=True` 把 `"vel_L1": NaN` 写入 telemetry，构成非法 JSON；浏览器 `JSON.parse` 抛错后整包丢弃，所有 telemetry 衍生 UI 显示「—」（Python `json.loads` 容忍 NaN，故板侧测试未暴露）。本轮：① 桥接层递归清洗非有限浮点（NaN/±Inf → null）并对全部载荷 `allow_nan=False` 硬兜底；② 前端容错解析（NaN 词法替换为 null）+ 解析错误计数/原文面板；③ 夹爪面板 v2：状态行中文模式映射（position 位置模式/force 力矩模式/release 释放/stop 停止）、模式选择器（位置/力矩）、位置模式 0–1 开合滑块节流直驱（新 op `gripper_set_position` → `/a3/gripper/command mode=position`）、力矩模式滑块节流抓取、目标 vs 实测双曲线（位置/力矩两页签，各页签同轴对比）；④ `GripperStatus.msg` 追加 `target_position`（position/release 命令写入，未命令前跟随实测），bridge 展平为 `grip_target_position`。
- **验收标准：**
  1. telemetry JSON 任意时刻均为合法 JSON 且无 NaN/Infinity 词法（板侧严格解析器 10 s 采样 0 异常）
  2. MQTT 下发 `gripper_set_position {position: 0..1}` 回 `cmd_result.ok=true`，`grip_target_position` 随动；越界/非法值 `ok=false` 且服务端不执行
  3. 浏览器：夹爪状态/模式（中文）/接触/错误正常显示；模式选择器切换控件；位置滑块拖动节流下发并回显实测；两页签曲线目标 vs 实测同轴实时刷新
  4. 调试面板显示原始报文、grip 各 code 在线/新鲜度、解析错误计数（修复后保持 0）
  5. 回归：`a3_test.sh gripper`、`mqtt_cmd`、F23 臂面板 10 op 不受影响
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[QUICKSTART.md](QUICKSTART.md)（§14）；[LL-011](../../lessons_learned/LL-011-nan-poisons-json-telemetry.md)；F18/F26/F27/F28；外部前端仓库 `deep-trace`（分支 `rk3588`，`GripperPanel.vue` / `Rk3588HubPanel.vue` / `useRk3588Mqtt.js` / `RealtimePerSignalChart.vue` / `HOME-DEMO.RK3588.yaml`）
- **状态：** `completed`（2026-09-06；浏览器硬刷新目检由用户确认；真机位置直驱待 can_bridge 轨迹订阅 QoS 对齐 BEST_EFFORT，见 F32）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
