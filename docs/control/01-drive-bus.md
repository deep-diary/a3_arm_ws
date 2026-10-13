# 01. 驱动与总线 — 进阶清单

> **定位：** 机械臂最底层——如何让电机「动、读、停、稳」，是其余所有模块的物理基础。
> **覆盖 [CONTROL_ROADMAP](../shared/CONTROL_ROADMAP.md) 层级：** L0（硬件与总线）、L1（关节级运动）。
> **涉及本仓需求：** F2/F17/F19/F42/F44/F51/F56/F72/F73/F81/F84/F85/F86/F87/F100/F103/F122/F146/F160。
> **图例：** ✅ 已实现 · ⚠️ 部分实现 · ❌ 未实现 · 🔜 已立项规划 · ※ 社区非官方

## 1. 阶梯总览

```mermaid
flowchart LR
    D0["D0 单电机使能/反馈"] --> D1["D1 多关节同步/状态"]
    D1 --> D2["D2 电源门控/软限位"]
    D2 --> D3["D3 MIT 阻抗式关节控制"]
    D3 --> D4["D4 力矩闭环/限幅保护"]
    D4 --> D5["D5 零力矩/自由拖动"]
    D5 --> D6["D6 通信健康/实时调度"]
    D6 --> D7["D7 多厂商/多传输 HAL"]
```

| 级别 | 名称 | 一句话 | 相对上一级新增的能力 | 本仓状态 |
|---|---|---|---|---|
| D0 | 单电机使能 + 反馈解析 | 让一个 ID 的电机能 enable 并读回角度/力矩 | 从「总线能通」到「一个关节可控可测」 | ✅ F17/F19 |
| D1 | 多关节同步 / 关节状态发布 | N 个关节同时下发指令并发布 `/joint_states` | 从单关节到整臂协同，对外提供统一状态 | ✅ a3_can_bridge |
| D2 | 电源序列与门控 + 执行层软限位 | 上下电时序、gate 关闭拒绝运动、限位落地 | 从「能动」到「安全地动」 | ✅ F2/F51/F111 |
| D3 | MIT 阻抗式关节控制 | 位置/速度/kp/kd/力矩前馈一体下发 | 从「生硬位置指令」到「关节层柔性执行」 | ✅ F56/F160 |
| D4 | 力矩/电流闭环 + 逐电机限幅保护 | 电机侧力矩限制、方向钳位、温度/故障保护 | 从「能柔顺」到「柔顺但不烧、不撞坏」 | ✅ F42/F44/F84/F87 |
| D5 | 零力矩 / 自由拖动 | 去 kp，只留重力感知，可手推拖动 | 从「被动执行」到「可示教、可协作」 | ✅ F13/F73/F85/F89b |
| D6 | 通信健康与看门狗 + 实时调度 | 断线检测、总线健康、控制环 SCHED_FIFO | 从「能跑」到「异常能发现、实时能保证」 | ✅ F81/F86/F100/F103/F122 |
| D7 | 多厂商 / 多传输标准抽象 | 统一 HAL，换电机只换驱动插件 | 从「绑定某型号」到「可移植标准栈」 | ⚠️ F72/F70–F78；多厂商 🔜 F146 |

## 2. 逐级详解

### D0 — 单电机使能 + 反馈解析

- **是什么：** 用底层总线协议（本项目为 SocketCAN + 厂商 MIT 帧）让指定 ID 的电机使能，并解析其回传的位置、速度、力矩、温度。
- **新增功能点（相对「总线能通」）：** 从「总线上有帧」到「能控制一个关节 + 能读它的状态」——这是所有闭环控制的第一块砖。
- **关键方法 / 组件：** CAN 帧编解码（`protocol_codec` / `frame_codec`）、电机型号参数（减速比、力矩标定、读数符号）、总线扫描服务。
- **工业对标：** CiA402/DS402 状态机（Switch on / Enable operation）、MIT Cheetah 式紧凑力矩帧。
- **本仓状态：** ✅ 实现于 [protocol_codec.hpp](../../src/a3_can_bridge/include/a3_can_bridge/protocol_codec.hpp)、[frame_codec.hpp](../../src/a3_can_bridge/include/a3_can_bridge/frame_codec.hpp)；需求 [F17](../edge/requirements/F17-el05-motor-protocol-commands.md)（命令集）、[F19](../edge/requirements/F19-can-bus-scan-service.md)（总线扫描）。
- **验收 / 自测点：** 指定 ID 电机可 enable；反馈角度/力矩/温度可读；与另一套工具不同时抢占同一 `can1`。

### D1 — 多关节同步 / 关节状态发布

- **是什么：** 对全部关节（本项目 7 个，`L1_joint`..`L7_joint`，L7 为夹爪）在一个控制周期内同步下发目标，并统一发布 `/joint_states`。
- **新增功能点（相对 D0）：** 从「单关节」到「整臂协调」——多关节要在同一时刻收到指令才不会互相拉扯；对外提供标准关节状态话题，供 RViz/规划/遥测消费。
- **关键方法 / 组件：** 关节映射表（`motor_map.yaml`）、发送周期与限流（CAN 200 Hz/电机上限）、`joint_states` 发布器。
- **工业对标：** 多轴插补（多轴同一插补周期）、总线同步（EtherCAT DC 分布式时钟）。
- **本仓状态：** ✅ [motor_protocol_node.cpp](../../src/a3_can_bridge/src/motor_protocol_node.cpp)、关节状态发布（JTC 的 JointStateBroadcaster）；关节模型见 [ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)。
- **验收 / 自测点：** 整臂可同时点到目标；`/joint_states` 频率稳定；关节顺序与 `motor_map.yaml` 一致。

### D2 — 电源序列与门控 + 执行层软限位

- **是什么：** 引入「电源状态机 + 安全门（gate）」：只有在门打开、电源序列完成后才允许运动；软限位不只在规划器（MoveIt/URDF）里，还要在**执行层**落地。
- **新增功能点（相对 D1）：** 从「能动」到「有条件地动」——gate 关闭拒动、上下电有序、限位在执行层拦截，避免上层 bug 直接打到底层。
- **关键方法 / 组件：** 独立电源序列节点、`/power_sequence/gate_open`、执行层限位钳位。
- **工业对标：** 使能/急停时序（Enable/STO 时序）、软限位（soft limit）与安全限位分层。
- **本仓状态：** ✅ 需求 [F2](../edge/requirements/F2-power-sequence-gating.md)、[F51](../edge/requirements/F51-enable-safety-gate.md)、真机电源节点 [F111](../edge/requirements/F111-hardware-power-sequence-node.md)、mock [F110](../edge/requirements/F110-mock-power-sequence.md)；实现 [power_sequence_node.cpp](../../src/a3_can_bridge/src/power_sequence_node.cpp)。
- **验收 / 自测点：** gate 关闭时任何轨迹被拒；shutdown 后电机 disable；越界指令被执行层钳位或拒绝。

### D3 — MIT 阻抗式关节控制

- **是什么：** 每次下发的不再只是位置，而是 `(q*, q̇*, kp, kd, τ_ff)` 五元组——位置环 + 阻尼 + 力矩前馈，即关节层的**阻抗式**控制。
- **新增功能点（相对 D2）：** 从「生硬位置伺服」到「可调刚度的柔性执行」——kp/kd 可调意味着关节可以软也可以硬；τ_ff 为空，是后续动力学补偿（模块 04）的入口。
- **关键方法 / 组件：** MIT 帧 V 域目标速度前馈、逐关节 kp/kd 表、运行时 kp/kd 注入。
- **工业对标：** 关节空间阻抗控制（joint impedance）、串联弹性/力矩电机的位置-力矩混合模式。
- **本仓状态：** ✅ [F56](../edge/requirements/F56-mit-frame-velocity-feedforward.md)（速度前馈）、[F160](../edge/requirements/F160-mit-position-velocity-accel-feedforward.md)（速度+加速度+科氏前馈）；kp/kd 表 [kp_kd_gains.yaml](../../src/a3_description/config/kp_kd_gains.yaml)；自动整定 [F138](../edge/requirements/F138-mit-kp-kd-autotune.md)。
- **验收 / 自测点：** 可在不换电机的前提下把某关节调软/调硬；位置环跟踪在加入前馈后误差下降。

### D4 — 力矩/电流闭环 + 逐电机限幅保护

- **是什么：** 在电机侧布防力矩限制（`0x700B` Type-18 写入）、方向钳位 latch、温度/故障码解码，构成「力矩可限、故障可停」的保护层。
- **新增功能点（相对 D3）：** 从「能柔顺」到「柔顺但安全」——限制最大出力、防止反向冲击、过热/堵转时按状态机降级。
- **关键方法 / 组件：** 力矩方向钳位（碰撞保护 latch）、温度管理（warn/protect/COOLING）、故障码解码、额定负载与占空比静态门禁。
- **工业对标：** 驱动器力矩限幅（torque limit）、I²t 过载保护、电机温度模型。
- **本仓状态：** ✅ [F42](../edge/requirements/F42-torque-direction-clamp.md)（方向钳位）、[F44](../edge/requirements/F44-temperature-management.md)（温度管理）、[F84](../edge/requirements/F84-temperature-fault-decode.md)（故障码）、[F87](../edge/requirements/F87-motor-torque-limit-arm.md)（力矩限幅）、[F107](../edge/requirements/F107-rated-payload-duty-gate.md)（额定负载/占空比）。
- **验收 / 自测点：** 力矩上限可设且被遵守；触发热保护后按状态机进入 COOLING；故障码可读并可复位。

### D5 — 零力矩 / 自由拖动

- **是什么：** 去 kp（只保留阻尼/重力感知），使机械臂可被手动推动（拖动示教），并可对标官方 `ZeroTorqueController` 做标准控制器切换。
- **新增功能点（相对 D4）：** 从「被动执行轨迹」到「人可介入」——为示教录制、主从、协作任务提供基础。
- **关键方法 / 组件：** 零力矩/重力补偿控制器、自由拖动速度自适应 Kd（Lorentzian 速度曲线 + EMA）、控制器 `switch_controller` 切换。
- **工业对标：** 零重力/浮动模式（gravity compensation / floating）、手动引导（hand-guiding）。
- **本仓状态：** ✅ [F13](../edge/requirements/F13-zero-torque-mode.md)（零力矩模式）、[F73](../edge/requirements/F73-torque-command-mode-zero-torque.md)（标准零力矩控制器）、[F85](../edge/requirements/F85-free-drive-adaptive-kd.md)（自适应 Kd）、[F89b](../edge/requirements/F89b-fsm-free-drive-controller-switch.md)（FSM 控制器切换）；实现 [gravity_compensation_controller.cpp](../../src/a3_hardware_interface/src/gravity_compensation_controller.cpp)。
- **验收 / 自测点：** 悬停无明显下坠；可自由拖动；退出无猛烈冲击（历史上靠 ramp-out 消除 clack/jerk，见 [F139](../edge/requirements/F139-gravity-comp-smooth-exit.md) 待办）。

### D6 — 通信健康与看门狗 + 实时调度

- **是什么：** 对每条链路（CAN 总线、单电机反馈、MQTT、话题频率）做健康监测与看门狗，并把控制环真正跑在实时调度（SCHED_FIFO）上。
- **新增功能点（相对 D5）：** 从「能跑」到「跑不稳/断了会被发现并能确定性恢复」——单电机断线锁存、总线 bus-off 可观测、控制环不被普通负载抢占。
- **关键方法 / 组件：** 单电机 last-rx 超时看门狗、电机侧通信超时配置（`0x7028`）、CAN 物理层健康、主机/服务双层 watchdog、`controller_manager` 实时调度。
- **工业对标：** 通信看门狗（communication watchdog）、心跳超时停机、实时以太网/DC 同步抖动预算。
- **本仓状态：** ✅ [F81](../edge/requirements/F81-motor-feedback-loss-watchdog.md)（反馈断线）、[F86](../edge/requirements/F86-motor-comm-timeout-config.md)（电机侧超时）、[F100](../edge/requirements/F100-realtime-sched-hardening.md)（实时调度）、[F103](../edge/requirements/F103-can-bus-health-diagnostics.md)（总线健康）、[F122](../edge/requirements/F122-disabled-zero-gain-keepalive-decouple.md)（保活解耦）、[F101](../edge/requirements/F101-watchdog-hardening.md)（watchdog 硬化）。
- **验收 / 自测点：** 拔线/静默后能被检测并冻结保持；`controller_manager` 线程确为 SCHED_FIFO；掉电/挂死能自动恢复。

### D7 — 多厂商 / 多传输标准抽象（HAL）

- **是什么：** 把「订阅话题 → 发 CAN」的能力封装为标准硬件接口插件（`SystemInterface`），上层用统一接口；换电机只换驱动，上层配置大体保留。
- **新增功能点（相对 D6）：** 从「绑定本项目某型号电机/总线」到「可移植」——同一硬件接口上可切换轨迹/Servo/导纳/零力矩控制器，为后续标准化（模块 03/05 挂标准控制器）铺路。
- **关键方法 / 组件：** `SystemInterface` 的 `read()`/`write()`、position/velocity/effort 接口、控制器 `switch_controller`；多传输（EtherCAT/CANopen/CAN-FD/串口）抽象。
- **工业对标：** `ros2_control` 硬件插件标准、EtherCAT CoE / CANopen DS402、SERCOS。
- **本仓状态：** ⚠️ 标准真机 HAL 已落地（[F72](../edge/requirements/F72-hardware-systeminterface-plugin.md)、[F70](../edge/requirements/F70-ros2-control-sim-activation.md)–[F78](../edge/requirements/F78-unified-bringup-single-entry.md)），实现 [a3_mit_hardware_interface.cpp](../../src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp)；**多厂商/多传输抽象仍缺**，见 [F146](../edge/requirements/F146-multi-vendor-motor-abstraction.md)（🔜 proposed）。
- **验收 / 自测点：** 同一上层在 mock/真机间切换只改硬件配置；换电机型号只需改插件与标定。

## 3. 本仓可继续进阶（缺口清单）

- 🔜 **多厂商/多传输抽象**（[F146](../edge/requirements/F146-multi-vendor-motor-abstraction.md)）：兼容 MotorBridge 等第三方驱动，接入非本项目电机。
- ❌ **总线升级**：CAN-FD（更高带宽/更低延迟）或 EtherCAT（分布式时钟同步）迁移，提升多轴同步精度。
- ❌ **功能安全硬使能（STO/SBC）**：硬件安全回路，配合模块 06 的功能安全（S5）。
- ❌ **完整 DS402/CiA402 状态机**：本仓为 MIT 帧直接控制，未实现标准伺服状态机的完整迁移路径。
- ⚠️ **实时抖动量化**：SCHED_FIFO 已有（F100），但控制环周期抖动预算尚未量化与门禁化。

## 4. 参考

- 本仓：[CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)（L0–L1）、[ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)、[PLATFORM_CAN.md](../edge/PLATFORM_CAN.md)、[TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- 需求：[F2](../edge/requirements/F2-power-sequence-gating.md)、[F13](../edge/requirements/F13-zero-torque-mode.md)、[F44](../edge/requirements/F44-temperature-management.md)、[F72](../edge/requirements/F72-hardware-systeminterface-plugin.md)、[F87](../edge/requirements/F87-motor-torque-limit-arm.md)、[F100](../edge/requirements/F100-realtime-sched-hardening.md)
- 外部：CiA402/DS402 伺服状态机、MIT Cheetah 电机控制、`ros2_control` 硬件接口文档
