# F23 — Web 端机械臂控制面板（MQTT cmd 下行 UI，跨仓）


- **说明：** 在 deep-trace 前端（外部仓库 `/home/cat/deep-trace`，分支 `rk3588`）RK3588 详情页新增 `a3_arm_controller` 节点卡片，点击展开「机械臂控制面板」。浏览器经 MQTT（mqtt.js 直连 EMQX WS，与现有遥测同一连接）下发 `cmd`（op 白名单 10 个：`init`/`enable`/`disable`/`goto`/`teach_start`/`teach_stop`/`save`/`playback`/`enter_ai`/`exit_ai`），订阅 `cmd_result` 把执行结果以 `ElMessage` toast + 面板内「操作消息列表」反馈到 UI；同时展示编排层 `/a3/arm_status`（`arm_state`/`arm_mode`/`arm_message`）实时状态。所有动作类指令须经 `ElMessageBox` 二次确认防误触，MQTT 未连接时按钮禁用。设备侧 `a3_mqtt_bridge` 已实现 10 op 下行与回执（F21/F22），本需求仅在 `bridge.yaml` 增加 `/a3/arm_status` 的 `scalar` 展平配置（`state`/`mode`/`message` → `arm_state`/`arm_mode`/`arm_message`），不改桥接 Python。Django 后端不经手指令（与全仓 IoT 面板一致），仅 `load_device_config` 合入 YAML 契约。
- **验收标准：**
  1. 设备 YAML `topics.cmd_result` 存在；`nodes` 含 `a3_arm_controller`（话题 `/a3/arm_status`，信号 `arm_state`/`arm_mode`/`arm_message`）；`points` 含上述 3 个 `discrete` 信号；`load_device_config` 后 `GET /auth/my-lines` 的 `edge.nodes`/`edge.topics` 体现
  2. RK3588 页出现「机械臂编排节点」卡片，点击展开控制面板；选中其它节点保持现有遥测曲线联动
  3. 面板状态区显示 `arm_state`（READY=绿/FAULT=红/其余蓝或黄）、`arm_mode`、`arm_message`，随 `/a3/arm_status` 实时刷新
  4. 10 个动作可下发：初始化/使能/失能、示教开始/结束、保存/回放（带轨迹名输入）、goto（zero/idle/ready 下拉）、进入/退出 AI；点击先弹二次确认，确认后才 publish
  5. 收到 `cmd_result` 后：`ok=true` 弹成功 toast、`ok=false` 弹失败 toast 并在消息列表标红；消息列表保留最近约 20 条（时间、op、成败 tag、message）
  6. MQTT 未连接/断网时所有动作按钮禁用；轨迹名为空或 goto 未选姿态时本地拦截提示，不下发
  7. `bridge.yaml` 新增 `/a3/arm_status` 展平后 `colcon build --packages-select a3_mqtt_bridge`，telemetry 的 `points.arm_state/arm_mode/arm_message` 随编排节点发布
  8. 回归 `scripts/a3_test/a3_test.sh mqtt_cmd` 仍 PASS（10 op 回执契约一致）
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（cmd/cmd_result JSON 契约）；F18（MQTT 遥测桥）、F20（Web 3D）、F21（编排节点）、F22（分层测试）；外部前端仓库 `deep-trace`（分支 `rk3588`，`useRk3588Mqtt.js` / `ArmControlPanel.vue` / `Rk3588HubPanel.vue` / `HOME-DEMO.RK3588.yaml`）
- **状态：** `implemented`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
