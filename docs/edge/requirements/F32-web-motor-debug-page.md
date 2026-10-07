# F32 — Web 端电机调试页（CAN 扫描 / MIT 保持 / 实时曲线，跨仓）


- **说明：** 仿 sparkbot 电机模块页，为 RK3588 增加 `motor_protocol_node` 专用单电机调试详情页：`Rk3588HubPanel` 的电机节点卡片跳转 `device-module` 路由（moduleId `rk3588_motor`），新面板 `Rk3588MotorPanel.vue` 提供 CAN 扫描（`/a3/motor/scan_and_collect` 聚合返回 ids/uids）→ 电机选择 → 状态卡片（在线/温度/模式/故障位/原始力矩/原始角）→ 使能/复位/设零（二次确认）→ 模式单选（MIT/位置/速度，复用 `/a3/motor/set_param` 0x7005）→ MIT 面板（`MotorParamField` 滑条：位置 ±12.57、速度 ±50、kp 0–500、kd 0–5、力矩 ±6；**ROS 侧定时保持**——前端只发目标+时长，`motor_protocol_node` 内部定时器按 hz 发帧、超时自动停，另提供单发与 `motor_stop` 卸力）→ 该节点话题/信号下拉 + `RealtimePerSignalChart` 实时曲线（默认 `mp/mtq/temp_L{n}`）。ROS 侧补齐：类型化逐电机遥测 `/a3/motor/states`（`MotorStates` 包装消息 50 Hz，7 电机 temp/err/mode/online/原始 MIT 角/力矩）；**互锁**——`gate_open=true`（电源序列 Running）时拒绝使能/复位/设零/MIT/参数/模式写入（扫描、读类、`motor_stop` 不受限），gate 打开瞬间 MIT 保持自动取消；仿真 `sim_motor_node` 同步补齐新话题/服务（sim 不实现互锁，有意分歧）。MQTT bridge 新增 9 个电机级 op + `motor_state` flatten（42 个新 telemetry 点），设备 YAML `HOME-DEMO.RK3588.yaml` 同步 points 与 nodes 目录。
- **验收标准：**
  1. `/a3/motor/states` 以 50 Hz 稳定发布 7 条 `MotorState`；`ros2 topic hz` 达标；无 CAN 时 `fresh=false`、单电机台架时仅对应 ID `fresh=true`
  2. `/a3/motor/mit_command`：`hold_duration_s<=0` 单发一帧；`>0` 按 `hold_hz` 定时发帧、到期自动停、新保持替换旧保持；`/a3/motor/stop` 立即取消保持并发卸力帧（kp=kd=t=0）；gate 打开瞬间保持自动取消并 WARN
  3. 互锁：`gate_open=true` 时 enable/reset/set_zero/MIT/set_param/set_mode 服务返回拒绝文案；scan/scan_and_collect/stop/get_device_id/request_version 照常可用
  4. `/a3/motor/scan_and_collect` 在 timeout_s（默认 1.5 s）内返回 `ids/uids`；busy 时拒绝重复扫描；`/a3/motor/scan`（F19/F22 兼容）行为不变
  5. MQTT：9 个电机 op 按契约返回 `cmd_result`（成功/参数非法/gate 拒绝三种文案）；telemetry 出现 `temp/err/mode/online/mtq/mp_L1..L7` 共 42 点且均为合法 JSON（无 NaN，F31 兜底回归）
  6. `./scripts/a3_test/a3_test.sh motor_debug` 仿真闭环全 PASS；前端在 `edge_web_sim` 下扫描→选择→保持→曲线全链路可用，`vite build` 通过；F22/F23/F26/F31 回归不受影响
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（电机调试节）；[shared/SAFETY.md](../shared/SAFETY.md)（MOTOR_DEBUG 互锁）；[QUICKSTART.md](QUICKSTART.md)（电机调试页节）；F17（EL05 协议命令集）、F19（总线扫描）、F22（单电机回归基座）、F23/F27（MQTT 面板模式）、F31（telemetry NaN 兜底）；外部前端仓库 `deep-trace`（分支 `rk3588`，`Rk3588MotorPanel.vue` / `DeviceModuleView.vue` / `registry.js` / `Rk3588HubPanel.vue` / `HOME-DEMO.RK3588.yaml`；需求文档 REQ-IOT-311）
- **状态：** `in_progress`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
