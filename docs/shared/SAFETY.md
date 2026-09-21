# 安全原则

> **Status:** active

A3 Edge 与 A3 CloudEdge 共同遵守的安全设计原则。具体参数以配置文件为准，实现位置因产品线而异。

## 核心原则

1. **轨迹门控：** 电源序列未完成前，禁止向电机发送运动指令。
2. **软限位：** 关节指令必须在 URDF / 配置定义的范围内。
3. **急停：** 必须存在不依赖规划栈的停机路径（软件 shutdown + 硬件急停）。
4. **断连保护：** 控制链路中断时，设备端必须在超时后 disable 或保持安全状态。
5. **总线互斥：** 同一 CAN 总线上不得同时运行多套电机控制工具（如 MotorBridge 与 `a3_can_bridge`）。

## 控制模式互锁

话题：`/a3/control_mode`（`std_msgs/String`）

| 模式 | 含义 | 允许 |
|------|------|------|
| `IDLE` | 空闲 | 可进轨迹 / 重力 / 零力矩 / Servo / 夹爪力控 |
| `TRAJ_RUNNING` | 轨迹执行中 | 拒绝重力启动、零力矩、Servo、夹爪力控 |
| `GRAVITY_COMP` | 重力补偿开启 | 新轨迹应退出重力；拒绝零力矩/Servo/夹爪力控 |
| `ZERO_TORQUE` | 软阻抗拖动（低 kp + 重力 FF） | 拒绝轨迹 Action / Servo / 夹爪力控 |
| `SERVO` | MoveIt Servo | 拒绝轨迹 Action / 零力矩 / 夹爪力控 |
| `GRIPPER_FORCE` | 夹爪力控（PI 力外环，L7） | 臂侧轨迹/Servo/零力矩启动须先终止力环 |

夹爪力控为 L7 单关节关节层力控（非 L8 末端六维力），力环在 `gripper_controller_node` 本地 50 Hz 运行；gate 关闭时拒绝 `force` 命令。

- gate 关闭时：强制退出运动相关模式，禁止新轨迹
- Servo：`incoming_command_timeout` 超时后应回 `IDLE` 并停止下发
- 零力矩 ≠ 纯 `tau=0`：默认同重力前馈叠加，退出时恢复原 `kp`/`kd`
- **手柄双模式死人开关（F60；F64 修订）：** 生产映射 `mapping:=default` 时，摇杆按动作分两个死人开关——**L1 按住** 才允许摇杆**平移** Twist（左摇杆 Y/Z、右摇杆 X），**R1 按住** 才允许摇杆**旋转** Twist（右摇杆左右=偏航）；松开立即发零速度（servo_mode_bridge 0.5 s 超时兜底停）。平移/旋转速度档相互独立，由 **D-pad 上下调平移档、左右调旋转档**（步进 0.15，clamp 0.1..1.0，F64；R1 的「全速档」绑定已删除）。**R2 夹爪力控不经 L1/R1 门控**（夹持操作需要独立于 jog 死人开关，F60 起）。`mapping:=simple` 调试档关闭门控（见 `config/mappings/simple.yaml`）。命名位姿/示教/电源键不要求 L1/R1。
- **键位总览（F60，F64 修订，取代 F55）：** L3 短按=一键上电+使能（power start → gate → enable）、R3 短按=失能（F40 safe park）、**Cross(X) 长按 1 s=硬急停**（power shutdown，门禁关）、Triangle/Circle 短按=ready/home 命名位姿、Share/Options 短按/ Square = 示教开始/结束（自动保存）/回放、PS 短按=init、Options 长按 3 s=调零、**L1/R1 按住=平移/旋转死人开关**、D-pad 上下/左右=平移/旋转速度档。完整映射表见 `src/a3_teleop_ps4/README.md`，操作员流程见 [docs/edge/PS4_OPERATOR_GUIDE.md](../edge/PS4_OPERATOR_GUIDE.md)。
- **R2 力控扳机（F36，F60 起不门控）：** 松开 → 夹爪全开（`release`）；按过 0.22 → 目标力矩 0.1..1.0 Nm（上限 = 硬限 1.0 Nm）；迟滞 0.15 防抖动。F60 起 R2 与 L1/R1 无关——无需按住 L1 即可开合夹爪。臂运动中（TRAJ_RUNNING/SERVO/ZERO_TORQUE/GRAVITY_COMP）力控被互锁拒绝 → 每 0.5 s 重试、松手即停。力控本身仍受抓取超时/看门狗/超硬限三重保护（见下节）。

## 轨迹门控（gate）

- 话题：`/power_sequence/gate_open`（`std_msgs/Bool`）
- `motor_protocol_node` 在 `enable_power_sequence_gate: true` 时，仅当 gate 为 `true` 才转发轨迹到 CAN
- 配置：[control_gains.yaml](../../src/a3_can_bridge/config/control_gains.yaml)、[power_sequence.yaml](../../src/a3_can_bridge/config/power_sequence.yaml)

启动流程（Edge，F60）：

1. PS4 **L3 短按**（一键）或 `start` 命令 → 电源序列
2. 序列完成 → `gate_open = true` → teleop 自动调 `/a3/arm/enable`（F48 读数门禁）→ READY
3. 此后方可接受 `JointTrajectory`；关机/硬急停 = **Cross(X) 长按 1 s**，恢复需重新 L3

## 软限位与速率

配置文件：[control_gains.yaml](../../src/a3_can_bridge/config/control_gains.yaml)

| 参数 | 典型值 | 说明 |
|------|--------|------|
| `joint_cmd_min_rad` / `joint_cmd_max_rad` | 见配置 | 关节指令硬限位 |
| `command_max_velocity_rad_s` | 1.5 | 指令速度上限 |
| `max_tx_rate_per_motor_hz` | 200 | CAN 发送速率上限 |
| `feedback_joint_states_timer_hz` | 50 | 关节状态发布频率 |
| `feedback_fresh_timeout_s` | 0.30 | 反馈超时 |

## 停机与调零

| 操作 | Edge 触发方式 | 命令 |
|------|---------------|------|
| 启动+使能 | PS4 **L3 短按**（一键：start → gate → enable，F60）/ `start` | `/power_sequence/command` + `/a3/arm/enable` |
| 硬急停（关机） | PS4 **Cross(X) 长按 1 s**（F60；F55 的 Triangle 长按与原 L1+R1+Share 三键组合均已废弃） | `shutdown` |
| 失能（保电） | PS4 **R3 短按**（F40：离 home 先 safe park） | `/a3/arm/disable` |
| 调零 | PS4 **Options 长按 3 s**（短按是结束示教） | `set_zero` |
| Servo jog | PS4 **L1 按住**（平移）/ **R1 按住**（旋转）+ 摇杆（F64；D-pad 上下/左右分调两通道速度档） | Twist |
| 夹爪力控 | PS4 **R2**（F60 起不需 L1） | 夹爪 force-release（F36） |

CloudEdge 须在 ESP32 固件中实现等效逻辑；网络侧 `shutdown` 命令可作为补充，**不能**作为唯一安全手段。

## 开机零位校验（F48）

MIT 电机上电用单圈编码器 + 多圈推算恢复绝对角：zero_sta=1（-π~π）下正常断电不丢，但超 ±180° 的断电转动仍会 +2π 环绕（LL-019：-22° 读 +338°）。环绕读数超关节限位时 **kp×误差会瞬间猛拉——严禁使能**。

1. **使能门禁（自动）**：`/a3/arm/enable` 发 enable 前检查 7 关节读数全部落在 URDF 限位内（`enable_position_check: true` 默认；未收到 /joint_states 也拒绝），越限拒绝并点名关节与限位。
2. **恢复路径**：`/a3/arm/init` 越限只 WARN 不阻断——set_zero 把**当前位姿**定义为零位，仅当臂确实摆在已知位姿（工装摆 URDF 零位）时执行，否则位姿帧无意义。
3. **开机检查脚本（人工）**：`scripts/a3_check_zero_frame.py`——限位硬检查（exit 1 严禁使能，判断是否环绕的唯一标准）+ 期望位姿软检查（L2/L3/L5/L6/L7 ≈ 0、L4 ≈ 0 或 0.34 下垂、L1 自由；默认容差 ±20°，仅提示性）。**硬检查同时验 /joint_states 新鲜度**（stamp ≤1 s）：桥异常时 js 冻结旧值，读旧值校验形同虚设（LL-020）。
4. **使能≠保位（LL-022）**：服务直驱使能（通信类型 3）只让电机进入 MIT 模式，若其后无轨迹/jog 帧流，桥播种按模式三态处理——模式未知/失能播零增益保活（无力矩）；**已知使能且反馈新鲜则锚定反馈位以 runtime 增益真实保位**；反馈陈旧不盲发。判定「已使能并保持」必须用帧级证据（02 反馈帧扩展 ID bit22-23 模式位 + `/a3/motor/states` fresh + `/a3/motor/tx_stats` tx_hz），不能只看 `/a3/arm_status` 的位姿读数（冻结读数会假性「无漂移」）。

## 夹爪力控安全（L7，需求 F24–F26）

夹爪（L7）力控是关节层「PI 力外环 + 电机位置内环」，目标是把接触/握力维持在设定值、自适应抓取软硬物体，不是 L8 末端六维力控。

1. **握力硬限双保险：**
   - 固件层：电机使能时经 `/a3/motor/set_param`（`motor_id=7`，`param_id=0x700B`）写入力矩限制；写入失败则夹爪不报就绪、禁止力控。
   - 软件层：节点对目标力与输出位置增量做 clamp，任何设定/反馈不得超过 `max_grasp_torque_nm`（且在 MIT ±6 Nm 量程内）。2026-09-07 起出厂硬上限为 **1.0 Nm**（原 2.0；1.5 持续出力几分钟即过热，见 LL-014）。
2. **参数下发校验：** web/服务下发的目标力或最大握力必须 `≤ max_grasp_torque_nm`；越界一律拒绝（`error_code=5`），不执行、不落盘。合法值落盘 `data/gripper_overrides.yaml`，重启加载。
3. **超力保护：** 力控中实测力矩瞬时超过硬限 × `overtorque_ratio`（1.5，瞬态带；固件 0x700B 仍硬钳 1.0 Nm），立即停止积分、停止下发并回退/停机，置 `FAULT`（`error_code=4`）。
4. **看门狗：** `feedback_fresh_timeout_s`（默认 0.30 s）内无新鲜 `eff_L7` 反馈，停止力环并置 `FAULT`（`error_code=3`）；节点退出/断连不得让电机维持夹紧力。
5. **抓取超时：** `force` 命令带 `timeout_s`，在时限内未进入 `GRASPED`（力矩入目标带 ±10% 并维持 settle 时间）则安全停止（`error_code=2`）。**超时仅约束「进入 GRASPED 前」的时限**：曾抓稳后滑脱（软物体带内带外振荡、状态回退 FORCE_CLOSING 再闭合）是正常调节，不得触发超时（LL-021）；滑脱失控仍由超硬限与看门狗兜底。命令传 `timeout_s` 时建议不短于配置默认 15 s（软物体收敛实测，LL-013）。
6. **力环仅边缘：** PI 力环必须在 Edge（RK3588）/ CloudEdge ESP32 固件本地闭环；**禁止**云端以 50–200 Hz 闭环力控，网络只下发目标力/档位等稀疏参数。
7. **模式互锁：** gate 关闭或臂处于 `TRAJ_RUNNING`/`SERVO`/`ZERO_TORQUE`/`GRAVITY_COMP` 时拒绝 `force`（`error_code=1`）；力控运行中臂侧轨迹/Servo 启动须先终止力环。
8. **位置安全：** 力环输出的 L7 位置目标始终钳位在 `joint_cmd_min/max_rad` 内，位置增量变化率受限，防止积分饱和导致猛夹。
9. **失能即松脱（F37）：** web「停止」= `gripper_stop` + `motor_reset {motor:7}`（失能 L7）——夹持物会**立即掉落**；停止前先移开/托住工件。`gripper_stop` 单独下发只停力环、电机仍使能保持（LL-017）。
10. **设置零位限全开硬止位（F37）：** `motor_set_zero {motor:7}` 平移整个开=0/闭=1.79 区间、破坏力控基线（接触门限 LL-013）；仅在夹爪处于全开硬止位时执行（流程：使能 → 释放到头 → 设零位）。另：失能期间手工拨动夹爪，重新使能会被 refresh keeper 以当前增益拽回旧目标角——**夹手风险**，失能后勿把手伸入夹爪。

配置见 `a3_gripper_controller/config/gripper_config.yaml`；接口契约见 [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md)。

## 单电机调试（MOTOR_DEBUG，需求 F32）

Web 端单电机调试（CAN 扫描 / MIT 直驱 / 保持）的安全边界：

1. **互锁（gate 关闭才可调试写）：** `gate_open=true`（电源序列运行中）时，`motor_protocol_node` 拒绝使能/复位/设零/MIT/模式切换/参数写入（带 gate 文案的 `success=false`）。扫描、读类（device_id/version）与 `motor_stop` **永不拦截**——停止能力在任何时刻都必须可用。
2. **保持自动取消：** `gate_open` 由关→开的瞬间，正在进行的 MIT 保持被 C++ 侧自动取消并记 WARN；前端不承担安全职责。
3. **停止即重力支撑保持：** `/a3/motor/stop` 取消保持并发送 **重力保持帧**——反馈新鲜（`feedback_fresh_timeout_s`）时逐电机 `kp=stop_hold_kp / kd=stop_hold_kd / t=重力前馈`、p=最近反馈角（F58），停在原位不垂落；反馈陈旧/缺失时才退回 `kp=kd=t=0` 裸卸力帧（没有反馈就不能信任保持）。保持期间若 CAN 中断，电机固件按帧停超时自然卸力（与轨迹路径同一机制）。**危险位形（重力敏感）下 stop 不再掉臂。**
4. **参数安全：** `mit_command` 的 `motor_id` 禁止 0 广播（只允许 1..127 单电机）；位置/速度/增益/力矩在服务端 clamp 到 `ProtocolCodec` 常量；保持时长上限 `max_hold_duration_s`（默认 30 s），发送频率上限 `min(200, max_tx_rate_per_motor_hz)`。
5. **保持与轨迹互斥：** 调试保持仅限 gate 关闭期间（此时轨迹插值与 refresh 均被 gate 阻断），保持是唯一 CAN 发送者，无总线争用；gate 打开瞬间保持即取消。
6. **仿真有意分歧：** sim 闭环不实现互锁（`sim_power_sequence_node` gate 恒 true），互锁只真机验证；前端在 `gate_open=true` 时展示提示横幅但不自行拦截（拒绝文案经 `cmd_result` 回传）。

## 失能保护（F40）

`/a3/arm/disable` 不在 home 容差（`disable_home_tol_rad: 0.15`）内时，先自动平滑回 home（`SAFE_PARK`，3 s/150 点轨迹）并连续确认收敛（`disable_home_confirm_s: 0.5`）再失能，防止 ready 位直接掉臂：

1. **park 超时 → FAULT 且不 reset**：保持使能、停在半途，需人工介入——宁停在半途也不盲目失能掉臂。
2. **reset 受 gate 互锁**：电源序列 Running 时 `/a3/motor/reset` 被 C++ 权威拒绝（MOTOR_DEBUG 互锁同一张表）——park 前拒绝 → `disable` 返回失败 + 原文（先 stop power sequence）；park 完成后被拒 → 回 READY（已在 home 位，安全）。
3. **紧急失能保留**：`/a3/motor/reset` 直达（通信类型 4）仍是急停链路，不经 park。
4. **DISABLED 下运动命令被拒**：`_can_move()`/`_set_joint_positions_cb` 拒绝表含 DISABLED/COOLING/SAFE_PARK——disable 后 move_to/goto/playback/set_joint_positions 均被拒，须显式 enable（原 IDLE 允许运动的语义混乱消除）。
5. **无反馈兜底**：无 `/joint_states` 时 disable 直达 reset + WARN（旧行为保留）。

## 力矩方向钳位（F42）

执行层（`motor_protocol_node`，权威）在每条指令帧对每个关节做方向性碰撞保护（阈值 `torque_protection_limit_nm: [5,5,5,3,3,3,3]`，RS00 5 / EL05 3，与 LL-024 codec 量程同源）：

1. **trip + latch**：反馈力矩 |τ| ≥ 阈值 → 冻结该关节目标在反馈位（τ>0 → target=min(target, fb)；τ<0 → target=max(target, fb)）并 latch τ 符号——无 latch 会形成 0→3 Nm 周期极限环。
2. **反向放行**：目标越过反馈位反向侧且超 `release_margin`（0.02 rad）才释放——「增矩方向冻结、反向自由」。
3. **新轨迹清 latch**：收到新轨迹即清空全部 latch（新轨迹 = 新意图）。**残留 latch 会把新轨迹钉在旧 trip 位**——慢速跟踪滞后 ~0.006 rad 永远到不了 release margin，回程/park 会被钉死超时 → FAULT（LL-026 真机实测）；保护不减弱——阻力仍在时钳位在一个 tick 内按反馈力矩重新 trip。
4. **例外不钳**：kp≤0.01（zero_torque/播种）或反馈不新鲜时清 latch 不钳位。
5. **兜底**：固件 0x700B 力矩硬限独立于本软件层。钳位只拦「增矩方向」；个别姿态保持力矩超阈时该姿态增矩方向运动受限（语义符合预期）。不做「持续超限 → FAULT」升级（执行层无状态机；编排层可观测 `mtq_L{n}` 扩展）。

## 温度策略（F44）

`temp_warn_c: 90.0` / `temp_protect_c: 95.0`（2026-09-13 由 65 上调——官方电机自带 130°C 兜底；65 使 ready 位保位发热几分钟即误触，LL-023）、迟滞 `temp_hysteresis_c: 5.0`：

1. **warn**（≥90）：仅置标志 + `arm_temp_warn` 遥测 + WARN 日志，不打断运动。
2. **protect**（≥95）：READY/TRAJ → 复用 F40 流程 safe park → **COOLING**（message 带关节与温度）；IDLE/DISABLED/COOLING → 直接 COOLING；SAFE_PARK 进行中不打断；reset 被 gate 拒 → FAULT（温度保护不可放弃）。
3. **降温恢复**：COOLING 下 enable/init 先查**全 fresh 关节** < protect−hysteresis 才放行；无 fresh 关节不阻碍 + WARN。
4. **fresh 门控**：温度判读一律以 `MotorState.fresh` 为准——无反馈时温度=0.0（不是 NaN），断连不得被误判「已冷却」放行使能。
5. **固件故障监视**：fault_mask≠0（含固件过温锁存 bit3）→ reset 广播 + FAULT。

## TX 帧率监视（F46）

`/a3/motor/tx_stats`（5 s 窗口，先发布再清零）：每电机 `tx_hz`、`tx_traj_total`/`tx_refresh_total`、跳过计数（`skip_max_rate`/`skip_bus_disabled`/`skip_power_gate`，定位帧丢失）、`tx_rate_ok`。

- 口径：轨迹期间理想帧率 = min(200, `max_tx_rate_per_motor_hz`)；`tx_rate_ok` 仅当本窗口有轨迹帧时校验 ≥90% 理想帧率——静止期只有 refresh 帧（~50 Hz）属正常，不误报。
- **读帧率须对照同时段桥日志**：5 s 窗口旋转会把轨迹尾巴切到下一窗口（瞬时读 tx_traj_total 可能是 7 而非 4098，LL-026）。
- 示教卡顿/帧丢失定位：先看三个 skip 计数器（限速丢弃 / 总线禁用 / gate 关闭）再查 CAN 层。

## 故障监视看门狗（F50）

`a3_arm_monitor`（20 Hz，独立节点，随编排层默认启动）：**分层保护的最后一层兜底**——F42（200 Hz 力矩钳位）、F44/F40（编排层温度/失能保护）保留原位，看门狗只做它们覆盖不到的**跨数据源比对**（js vs 轨迹 vs 电机状态 vs 编排状态），动作只走公开服务（stop → 升级 reset），不直接改任何节点内部状态。

- 故障类：FOLLOW_STUCK / HOLD_DRIFT（stop → 3 s 未消升级 reset）、STALE_JS（reset）、UNEXPECTED_DISABLE（仅报告，避免 F40 park 在失能电机上失败）、TEMP_UNRESPONSIVE（reset，F44 失灵兜底）。阈值与阶梯见 [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md)「故障监视看门狗」。
- **自动 reset 过重力门禁（F58/LL-053）**：stop→reset 阶梯在升级前查新鲜 `max|τ_grav|`（`/a3/gravity_torque`，1 s 内）——危险位形（超 `reset_max_gravity_torque_nm: 5.0`）**拒绝自动 reset**，记 `RESET_DENIED_gravity_unsafe` 并保持重力支撑不动，等人工介入（先回安全位再 reset）。样本缺失/陈旧按旧行为放行（不破坏无 gravity 节点场景与 F51-F52 回归）。**失败路径仍可人工 disable/急停。**
- 抑制规则：零力矩/重力补偿模式、失能态、启动宽限、触发 cooldown、清除保持——设计目标是**零误报**（误报会让操作者关掉看门狗）。
- 期望位置用自建轨迹插值器（time_from_start 线性插值），不依赖 ArmStatus.positions（目标快照语义）。
- 实现要点：服务客户端必须挂独立 ReentrantCallbackGroup + MultiThreadedExecutor + 纯轮询等 future，回调内 `spin_until_future_complete` 会死锁自身（LL-034）；过期判据用接收时刻 monotonic 时间戳，不可与消息墙钟 stamp 混减（LL-034）。

## 使能安全（F51，LL-039 事故条款）

**使能 = 保当前位置，绝不执行历史目标。** 2026-09-14 真机事故：示教退出后目标被重锚到拖动位姿（1.98 rad），看门狗假触发 stop→reset（臂卸力、人工搬回 home 0.03 rad），使能时**无人校验「目标 vs 实际」**，kp=80 对 1.956 rad 误差满增益输出 → L2 3.1 s 冲 1.97 rad → 甩断 L6 打印关节（F42 是防撞设计，拦不住满速甩动）。

1. **使能前必须校验目标-实际距离**：执行层使能时逐电机把 MIT 目标重锚到**新鲜反馈位**；反馈缺失或陈旧（> `feedback_fresh_timeout_s` 0.30 s）→ **整体拒绝使能**（`success=false`，一帧不发）。不知实际位置就使能 = 盲拉（LL-018/LL-022）。
2. **停止绝不锚回旧目标，但可重力支撑**：`/a3/motor/stop` 与 reset/set_zero 置**保持抑制 latch**，之后 keepalive 的 **p 始终锚回反馈位**（不得把目标锚回旧位姿，事故二级根因，日志中 `cmd_angle=nan` 0 次）；反馈新鲜时 keepalive 增益为 `stop_hold_kp/kd` + 重力前馈（F58 重力支撑保持），反馈陈旧/失能模式才退回零增益（p=反馈位、kp=kd=τ=0）——消除「stop → refresh 零增益覆盖」导致的整窗裸卸力掉臂窗口。
3. **使能软起步**：kp/kd 在 0.8 s 内从 0 线性升到额定，残余误差不被满增益放大成甩动（τ_ff 重力前馈不受影响）。
4. **失能期陈旧目标要出声**：失能态下 `|目标−反馈| > 0.15 rad` 限频 WARN——事故时该状态静默保留了 2 分钟。
5. **意图边界必须重基准**：看门狗在示教/零力矩退出、整臂失能→使能沿，把保持参照 `_last_goal ← 当前实际位姿`、清运动窗口、给 2 s 宽限。否则「合法的新位姿」被判成保持漂移 → 假触发 stop→reset（事故触发源）。修假阳性**必须同时验证真阳性**：带外失能仍要报 UNEXPECTED_DISABLE 且编排层转 DISABLED。
6. **带外失能双通道**：看门狗 UNEXPECTED_DISABLE（持续窗 0.5 s）**必须短于**编排层本地兜底（1.0 s），否则编排层先转 DISABLED 会让看门狗持续窗清零、真故障永远报不出来。

## 产品线实现差异

| 安全能力 | A3 Edge | A3 CloudEdge |
|----------|---------|--------------|
| 轨迹门控 | `power_sequence_node` on RK3588 | ESP32 固件本地 |
| 软限位 | `motor_protocol_node` | ESP32 固件本地（移植限位表） |
| 断连看门狗 | 可选 ROS 层 | **必须** ESP32 本地（建议 &lt; 100 ms 级检测） |
| 硬件急停 | 板载 GPIO / 急停回路 | ESP32 GPIO，独立于 WiFi |
| 重力补偿闭环 | 可板载 Pinocchio | 轨迹级开环在服务器；实时闭环在边缘 |
| 夹爪握力硬限 | 固件 `0x700B` + `gripper_controller` 软件 clamp | 固件 `0x700B` + ESP32 本地 PI 力环（E12） |

## CloudEdge 断连策略（要求）

ESP32 固件必须实现：

- 若在 **T_watchdog**（建议 100–500 ms，可配置）内未收到有效轨迹或心跳，停止发送 MIT 指令并 disable 电机
- WiFi 断连与服务器宕机均须触发同一本地安全路径
- 门控状态在断连时默认视为 `false`

## 禁止事项

- 在 gate 未打开时发送运动轨迹
- 云端以 200 Hz 闭环力控替代边缘实时环
- 将急停、看门狗仅放在服务器侧
- MotorBridge 与 `a3_can_bridge` 同时占用 `can1`
- 在电机反馈缺失/陈旧时使能（必须先重锚到新鲜反馈位，F51）
- 在失能期保留可执行的历史目标（stop/reset 后必须置保持抑制，F51/LL-039）

## 关联文档

- [CONTROL_ROADMAP.md](CONTROL_ROADMAP.md) — 控制栈分层（含 L8 力控前置与 Edge C8）
- [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md)
- [edge/ARCHITECTURE.md](../edge/ARCHITECTURE.md)
