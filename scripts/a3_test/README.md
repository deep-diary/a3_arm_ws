# A3 机械臂单电机分层测试套件（F22）

在最小硬件（can1 接 1 个空载电机，CAN_ID=7 = L7 夹爪，24V 锂电池供电）下，
分层验证机械臂全栈功能。对应需求 `docs/edge/REQUIREMENTS.md` F22。

## 前置条件

- can1 UP（1Mbps，`can-up.service`）；电机 CAN_ID=7 接 can1，空载。
- ROS 2 Humble 已 source；`a3_can_bridge / a3_mqtt_bridge / a3_bringup / a3_msgs` 已构建。
- EMQX `192.168.3.73` 的 1883（MQTT/TCP）可达；`paho-mqtt` 已装（`~/.local`，脚本自动处理 `PYTHONNOUSERSITE`）。
- MoveIt Servo 仿真阶段需要 `ros-humble-moveit-servo`。

## 用法

```bash
./scripts/a3_test/a3_test.sh env        # 阶段0 环境自检
./scripts/a3_test/a3_test.sh hw         # 阶段一 真机底层（scan/设零/使能/小角度/零力矩/失能/软限位）
./scripts/a3_test/a3_test.sh telemetry  # 阶段三 MQTT 上行：转电机，断言 pos_L7 变化
./scripts/a3_test/a3_test.sh mqtt_cmd   # 阶段二 MQTT 下行：mock 编排层，10 op 全链路
./scripts/a3_test/a3_test.sh servo      # 阶段四 仿真：MoveIt Servo 六方向直线 jog
./scripts/a3_test/a3_test.sh web        # 网页人工确认（保持遥测栈运行，打印操作清单）
./scripts/a3_test/a3_test.sh all        # env→hw→telemetry→mqtt_cmd→servo
```

每阶段打印 `[PASS]/[FAIL]` 与汇总；非零退出码表示有失败项。

## 分层说明（为什么这样测）

| 阶段 | 链路 | 为什么这样设计 |
|---|---|---|
| hw | 真机 → `/a3/motor/*` 底层服务 | 编排层 `/a3/arm/init` 要求 7 电机全部回零，单电机会 FAULT，故真机直连底层并指定 `motor_id:=7`，以 `use_power_sequence:=false` 关轨迹门禁 |
| telemetry | 真机 `/joint_states` → mqtt_bridge → EMQX | 验证真实电机角度经 MQTT 上报为 `pos_L7` |
| mqtt_cmd | EMQX `cmd` → mqtt_bridge → **mock** `/a3/arm/*` | 网页下行的 10 个 op 映射到编排层服务；用 mock 做确定性断言（路由+参数+`cmd_result`），不依赖 7 电机 |
| servo | 仿真 `servo.launch.py`（sim_executor 闭环，不碰 CAN） | Servo 笛卡尔 jog 作用于 L1–L6（arm 组），ID=7 是夹爪不在 arm 组；且真机有 SERVO 模式互锁（CONTROL_ROADMAP 待办） |
| web | 真机遥测 + deep-trace 网页 | 前端 RK3588 页只读、无控制按钮；控制链路用 mqtt_cmd 阶段的 MQTT CLI 等价验证，网页确认曲线/3D 渲染 |

## 安全

- 所有真机运动经 `safety_limits.py` 限幅：单次目标 ≤ 0.30 rad、每段 ≥ 2.5 s（限速 ~0.12 rad/s）、
  空载降低 MIT 刚度（kp=40/kd=2）、运动前使能、结束必失能（含异常兜底）。
- 零力矩阶段脚本会提示人工轻转电机轴采样，请在旁监护，可随时对电机失能：
  `ros2 service call /a3/motor/reset a3_can_bridge/srv/MotorCommand "{motor_id: 7, command: 2}"`
- 锂电池供电，注意电量；长测前确认 24V 电量充足。

## 文件

- `a3_test.sh` — 统一入口，管理 launch 生命周期
- `safety_limits.py` — 集中安全限幅与安全轨迹构造
- `common.py` — 报告/ROS 服务调用/MQTT 连接等公共工具
- `hw_motor_test.py` — 阶段一真机
- `mqtt_telemetry_test.py` — 阶段三 MQTT 上行
- `mock_arm_services.py` / `mqtt_cmd_test.py` — 阶段二 MQTT 下行
- `servo_sim_test.py` — 阶段四 Servo 仿真
