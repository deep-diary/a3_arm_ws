# F27 — Web 端夹爪控制面板（MQTT 下行 UI，跨仓）


- **说明：** 在 deep-trace 前端（外部仓库 `/home/cat/deep-trace`，分支 `rk3588`）RK3588 详情页新增夹爪卡片 `GripperPanel.vue`。设备 YAML `HOME-DEMO.RK3588.yaml` 增加 gripper 节点（话题 `/a3/gripper_status`）、topics（含 `cmd_result`）、points（`grip_state`/`grip_target_torque`/`grip_actual_torque`/`grip_position`/`grip_contact`）与 4 个 cmd op 契约，Django 侧仅 `load_device_config` 合入 YAML，不经手指令。UI：握力档位（弱/中/强）单选 + 目标力滑块（标注硬上限，超限本地拦截不下发）、最大握力设置（输入框 + 二次确认）、抓取/释放/停止按钮（动作类 `ElMessageBox` 二次确认，MQTT 未连接时禁用）、实时握力曲线（复用 telemetry `grip_actual_torque`/`eff_L7`）与状态指示（`GRASPED` 绿/`FAULT` 红/力控中蓝）、`cmd_result` 回执 toast + 消息列表。复用 `useRk3588Mqtt` 同一连接，不新建 MQTT。
- **验收标准：**
  1. 设备 YAML 含 gripper 节点/topics/points/cmd；`GET /auth/my-lines` 的 edge 配置体现；前端信号 code 与 `bridge.yaml` 完全一致
  2. RK3588 页出现夹爪卡片：档位切换、滑块、最大握力设置、抓取/释放/停止按钮齐备
  3. 滑块/输入超过硬上限时本地提示且不下发；抓取/释放/停止点击后弹二次确认
  4. 下发后订阅 `cmd_result`：成功 toast、失败标红；消息列表保留最近约 20 条
  5. 握力曲线随 `grip_actual_torque` 实时刷新；`GRASPED`/`FAULT` 状态颜色正确；MQTT 断连时所有控件禁用
  6. 回归 F23 机械臂控制面板 10 op 不受影响
- **关联：** F18/F23（MQTT 通道与面板模式）、F26（op/points 契约）；外部前端仓库 `deep-trace`（分支 `rk3588`，`GripperPanel.vue` / `HOME-DEMO.RK3588.yaml` / `useRk3588Mqtt.js`）
- **状态：** `implemented`（前端 `GripperPanel.vue` + YAML 契约已合入 `rk3588` 分支且 `vite build` 通过；需 `load_device_config` 合入并浏览器联调）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
