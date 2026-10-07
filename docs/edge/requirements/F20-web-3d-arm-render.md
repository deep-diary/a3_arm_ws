# F20 — Web 端 3D 机械臂实时渲染（跨仓）


- **说明：** 在 deep-trace 前端（外部仓库 `/home/cat/deep-trace`，分支 `rk3588`）的 RK3588 设备详情页新增 3D 机械臂视图：浏览器加载 `src/a3_description/urdf/el_a3.urdf`（含 8 个 `.stl` mesh），复用 F18 已打通的 MQTT `telemetry` 中 `points.pos_L1..L7`（弧度）实时驱动 `L1_joint..L7_joint`。渲染用 `three` + `urdf-loader`（不兼容时回退 `@gkjohnson/urdf-loader`）；MQTT 复用 `useRk3588Mqtt`，不新建连接。
- **验收标准：**
  1. rk3588 详情页加载 `el_a3.urdf` 后 3D 模型正确渲染（含 mesh 与关节层级）
  2. 起 `a3_mqtt_bridge` 后订阅 `telemetry`，7 关节随 `pos_L1..L7` 实时转动（单位一致，无需换算）
  3. 断连重连后 3D 视图恢复跟随（复用 `useRk3588Mqtt` 的重连逻辑）
  4. 页面卸载时正确释放 `three` renderer 与 `requestAnimationFrame`，无内存泄漏告警
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)；外部前端仓库 `deep-trace`（分支 `rk3588`）
- **状态：** `in progress`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
