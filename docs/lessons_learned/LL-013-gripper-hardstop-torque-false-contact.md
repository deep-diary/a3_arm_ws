# LL-013 — 0 位硬止位静置力矩致接触误判 + web 力控不传 timeout 落 5s 默认：真机力控卡 42% 开合

> **日期：** 2026-09-06
> **产品线：** Edge
> **环境：** RK3588 真机（lubancat）+ ROS 2 Humble，单电机台架 can1 / ID7，泡棉软物体

## 现象

web 力控按键目标 0.3 Nm：电机以 0.3 rad/s 闭合到 q≈1.08 rad（42% 开合）后停住，实际力矩 0.13 Nm，状态 FAULT：

```
state: FAULT, error_code: 2（ERR_GRASP_TIMEOUT）
message: grasp timeout > 5.0s (meas=0.15)
```

预期行为：接触前恒速软闭合到泡棉（q≈1.0 rad）→ 接触后 PI 持续推进位置直到力矩达 0.3 Nm。

## 根因

两个叠加原因：

1. **接触误判（跳过快速接近段）**：L7 标定把 0 位设在丝杠硬止位，静置时电机顶住硬止位有 ≈0.16 Nm 力矩。接触阈值 = max(contact_detect_min_nm=0.1, 0.35×0.3)=0.105 < 0.16，力控起步第一 tick 就置 `contact_detected=True`，恒速软闭合段被跳过，PI 从 e≈0.14 起步以 force_kp=0.5 缓慢爬升（约 0.07 rad/s）。
2. **超时太短**：web 力控按键只发 `{torque}` 不传 `timeout`（GripperPanel.vue `sendTorque`），bridge 不填 `timeout_s`，落到配置默认 `grasp_timeout_s=5.0`。软泡棉接近 + PI 压缩 >5s，FAULT 冻结力环，位置停在 42% 开合不再前进。

## 正确做法 / 规避

- 接触检测的「力矩阈值」必须加「位移门」：从全开硬止位（`q_open`）算起的位移 ≥ `contact_min_travel_rad`（0.1 rad）后阈值判定才生效。硬止位静置力矩只存在于起点，离开后自然消退；真接触点（泡棉 q≈1.0 rad）远超位移门，不受影响。
- **位移门基准必须是 `q_open`，不能是力控起点**（F34 实测教训）：已夹持中重发力控命令时力控起点就在物体上，按起点算位移会盲跑完整快速接近段（0.3 rad/s 恒速），把已压实的泡棉再压 0.1 rad → 力矩尖峰 1.5455 Nm 超 1.5 硬限 FAULT。基准取 q_open 后：0 位起步时位移≈0 门仍生效；物体上重抓时位移≈全行程，第一个 tick 即判接触切入 PI。
- 真机上所有可能被省略的可选参数，默认值要按真机时序标定：默认 `grasp_timeout_s` 5.0→15.0（web 不传 timeout 的兜底）。
- 诊断抓手：`gripper_status.target_position` 在力控期间更新为 PI 实时输出（F33）——q_cmd 不涨 = 力环问题；q_cmd 涨而电机不动 = 电机侧 TX，两者从此可区分。
- 修复点：`gripper_controller_node.py`（`_tick_force` 接触判定位移门基准 `q_open`、每 tick 更新 `target_position`）；`gripper_config.yaml`（`contact_min_travel_rad: 0.1`、`grasp_timeout_s: 15.0`）。

## 相关路径

- `src/a3_gripper_controller/a3_gripper_controller/gripper_controller_node.py`
- `src/a3_gripper_controller/config/gripper_config.yaml`
- 关联：LL-010（同场景 timeout 教训，本次是默认值而非命令参数）；F33
