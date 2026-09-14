# A3 机械臂单电机分层测试套件（F22）

在最小硬件（can1 接 1 个空载电机，CAN_ID=7 = L7 夹爪，24V 锂电池供电）下，
分层验证机械臂全栈功能。对应需求 `docs/edge/REQUIREMENTS.md` F22。

## 前置条件

- can1 UP（1Mbps，`can-up.service`）；电机 CAN_ID=7 接 can1，空载。
- ROS 2 Humble 已 source；`a3_can_bridge / a3_mqtt_bridge / a3_bringup / a3_msgs` 已构建。
- EMQX（默认 `bluemac.local`，可用 `A3_MQTT_HOST` 覆盖）的 1883（MQTT/TCP）可达；`paho-mqtt` 已装（`~/.local`，脚本自动处理 `PYTHONNOUSERSITE`）。
- MoveIt Servo 仿真阶段需要 `ros-humble-moveit-servo`。

## 用法

```bash
./scripts/a3_test/a3_test.sh env        # 阶段0 环境自检
./scripts/a3_test/a3_test.sh hw         # 阶段一 真机底层（scan/设零/使能/小角度/零力矩/失能/软限位）
./scripts/a3_test/a3_test.sh telemetry  # 阶段三 MQTT 上行：转电机，断言 pos_L7 变化
./scripts/a3_test/a3_test.sh mqtt_cmd   # 阶段二 MQTT 下行：mock 编排层，10 op 全链路
./scripts/a3_test/a3_test.sh servo      # 阶段四 仿真：MoveIt Servo 六方向直线 jog
./scripts/a3_test/a3_test.sh gripper    # 阶段五 夹爪力控（默认 sim 闭环，无硬件可跑）
./scripts/a3_test/a3_test.sh motor_debug # 阶段六 单电机调试下行（sim 闭环，9 个电机 op，无硬件可跑）
./scripts/a3_test/a3_test.sh incident   # 阶段八 LL-039 事故回归（F51，纯仿真/mock 电机，无硬件/root）
./scripts/a3_test/a3_test.sh web        # 网页人工确认（保持遥测栈运行，打印操作清单）
./scripts/a3_test/a3_test.sh force_web  # 阶段七 web 路径力控阶梯验收 0.3→0.5→0（真机，见下）
./scripts/a3_test/a3_test.sh all        # env→gripper→hw→telemetry→mqtt_cmd→servo→motor_debug→incident
```

LL-039 事故回归（F51，`incident`）：
- 八a **执行层**：Python mock 电机（对接 `/can_tx_frames` ↔ `/can_rx_frames`，**不走 SocketCAN**，无需 root）
  对接真 `motor_protocol_node`；脚本按事故时间线注入：使能 → 零力矩 → 拖到 1.98 rad → 退出零力矩（执行层 F38 重锚）
  → `/a3/motor/stop`（断言此后**无** kp>0 帧）→ reset → 搬回 0.03 rad → 使能（断言命令位≈反馈位、kp 0.8 s 斜坡、关节未被甩动）。
- 八b **看门狗/编排层**：`sim_motor_node` 冒充执行层 + 真 `arm_controller`/`arm_monitor`；
  用 `/a3/motor/mit_command` 改仿真臂位姿等价「人手拖动」。断言 M1–M4：初始化无触发、jog 无触发、
  **示教退出后 4 s 零触发**（假阳性修复）、带外 reset **必须**被判 `UNEXPECTED_DISABLE` 且编排层转 DISABLED（真阳性）。
- **两个脚本各自拉起被测栈并跑在独立 `ROS_DOMAIN_ID=57`**，与真机栈（domain 0）DDS 隔离——mock 伪造全部反馈，
  同域运行等于让测试的 enable 打到真电机上。被测进程用 `start_new_session=True` 起、按**进程组** `killpg` 收
  （`ros2 run` 只是包装器，`terminate()` 杀不掉它孵化的节点）。
- 有牙验证法：`git show HEAD:<file> > <file>` 还原修复前版本重启被测栈，脚本**必须失败**。

夹爪力控（F28）两种模式：
- 默认（`sim`）：脚本在独立 `ROS_DOMAIN_ID=77` 自起 `gripper_controller` + 假「电机+物体」植物
  （订阅 L7 轨迹→一阶位置跟随→接触后按物体刚度反算 `eff_L7`），闭环断言 12 项：配置越界/合法、
  POSITION 开合、软/硬物体力控收敛 ±10% 且 GRASPED、超硬限 FAULT(4)、反馈看门狗 FAULT(3)。
- 真机（`A3_GRIPPER_TEST_MODE=hw`）：起 can_bridge + gripper_controller，做服务/配置/开合安全检查；
  力控阶跃需人工放海绵/硬阻挡观察（不自动断言真机力矩）。

每阶段打印 `[PASS]/[FAIL]` 与汇总；非零退出码表示有失败项。

web 路径力控阶梯验收（F34，`force_web`）：
- 纯 MQTT 客户端（模拟 web 按键），不启动任何节点，走**已在运行的生产栈**（can_bridge +
  gripper_controller + a3_mqtt_bridge），broker `bluemac.local:1883`（可用 `A3_MQTT_HOST` 覆盖）。
- 前置：泡棉（软物体）已放在夹爪中、电机使能、gate 关（force 互锁不拦截）。
- 流程：`gripper_release` → `gripper_grasp {torque:0.3, timeout:20}` → `{torque:0.5, timeout:20}`
  → `{torque:0}`（F33 硬逻辑直接全开）；断言两步 GRASPED 且实际力矩在目标 ±15% 内、
  采样记录 grip_target_position 逐步变大趋势、force 0 后 3s 内 grip_position→1.0（0 位）且无 FAULT。
- 不进 `all`（需人工放泡棉 + 在线监督）；失败时兜底发 `gripper_release` 松开。

## 分层说明（为什么这样测）

| 阶段 | 链路 | 为什么这样设计 |
|---|---|---|
| hw | 真机 → `/a3/motor/*` 底层服务 | 编排层 `/a3/arm/init` 要求 7 电机全部回零，单电机会 FAULT，故真机直连底层并指定 `motor_id:=7`，以 `use_power_sequence:=false` 关轨迹门禁 |
| telemetry | 真机 `/joint_states` → mqtt_bridge → EMQX | 验证真实电机角度经 MQTT 上报为 `pos_L7` |
| mqtt_cmd | EMQX `cmd` → mqtt_bridge → **mock** `/a3/arm/*` | 网页下行的 10 个 op 映射到编排层服务；用 mock 做确定性断言（路由+参数+`cmd_result`），不依赖 7 电机 |
| servo | 仿真 `servo.launch.py`（sim_executor 闭环，不碰 CAN） | Servo 笛卡尔 jog 作用于 L1–L6（arm 组），ID=7 是夹爪不在 arm 组；且真机有 SERVO 模式互锁（CONTROL_ROADMAP 待办） |
| motor_debug | EMQX `cmd` → mqtt_bridge → sim `/a3/motor/*`（独立 `ROS_DOMAIN_ID=44`） | 9 个电机调试 op 全链路（扫描 JSON 列表/MIT 单发与保持/停止/模式/参数 + 非法拒绝），hold 期间断言 `mp_L1` 收敛与 `temp/mode/online` 遥测；sim 有意不实现 gate 互锁（真机才验证） |
| incident | 八a：mock 电机 ↔ 真 `motor_protocol_node`；八b：sim 栈 + 真 `arm_controller`/`arm_monitor`（独立 `ROS_DOMAIN_ID=57`） | LL-039 事故回归（F51）。仿真栈的 `sim_motor_node` **替换了整个 C++ 执行层**，覆盖不到本次改动的 C++ 路径 → 八a 用真执行层对接 Python mock 电机（不走 SocketCAN，无需 root）；八b 用 `/a3/motor/mit_command` 改仿真臂位姿来等价「人手拖动」。**必须与真机栈 DDS 隔离**：mock 伪造全部反馈、同名 `/a3/motor/*` 服务在真机栈上存在 |
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
- `motor_debug_test.py` — 阶段六 单电机调试下行（F32）
- `incident_regression_test.py` — 阶段八a 执行层 F51 事故回归（mock 电机 ↔ 真 motor_protocol_node）
- `incident_monitor_regression_test.py` — 阶段八b 看门狗/编排层 F51 事故回归（sim 栈 + 真 arm_controller/arm_monitor）
- `mqtt_force_ladder_test.py` — 阶段七 web 路径力控阶梯验收（F34，纯 paho，走生产 MQTT 桥）
