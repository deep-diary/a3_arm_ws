# A3 Arm 平台架构文档图集（docs/architecture）实施计划

## 一、Summary

在 `docs/architecture/` 下，**完全复刻 `docs/architecture-ref/` 的 HTML + Mermaid 网页图集风格**，为当前项目（A3 Arm Platform，ROS 2 Humble 七自由度机械臂工作区）实现**完整 8 页**架构文档：

`index` / `overview`（宏观）/ `detail`（微观）/ `components`（组件）/ `control_flow`（控制流）/ `signal_flow`（信号流）/ `data`（数据）/ `mechanisms`（机制）+ 共用 `style.css` + `mermaid-init.js` + `README.md`（Markdown 速览/索引）。

内容以 **A3 Edge 主线为主、A3 CloudEdge 支线为辅**，全部信息基于已调研的源码与既有文档（TOPIC_CONTRACT / REQUIREMENTS / edge&cloud_edge ARCHITECTURE / lessons_learned），**不修改任何源代码**。同时把「架构不合理之处」评估结论写进文档（index 现状块 + mechanisms 或 overview 的建议节）。

## 二、Current State Analysis

### 2.1 项目本质

- **A3 Arm Platform**：ROS 2 Humble 工作区，EDULITE A3（EL-A3）7 关节机械臂（`L1_joint..L7_joint`，L7 为夹爪；电机 ID 1..7，主机 0xFD，控臂总线默认 `can1`）。
- **两条产品线**：
  - A3 Edge（`edge`，**主线，active**）：RK3588 板载完整 ROS 2 全栈 + SocketCAN → MIT 电机，追求板内低延迟（控制环 < 10 ms）。
  - A3 CloudEdge（`cloud_edge`，支线，draft）：内网服务器集中规划（MoveIt/Pinocchio）+ ESP32-S3 micro-ROS 客户端本地 200 Hz 插值 + CAN MIT；本仓只含 Linux 端 Agent/mock 验证栈。

### 2.2 包与节点清单（调研结论，已核实）

| 包 | 职责 | 关键节点/入口 |
|---|---|---|
| `a3_msgs` | 自定义 msg/srv/action | ArmStatus / MonitorStatus / GripperStatus；MoveToPoseIK / GotoNamedPose / SetJointPositions / MoveToJointPositions / SaveNamedPose / SaveTrajectory / PlaybackTrajectory / GripperCommand / GripperSetConfig；MoveToPose action |
| `a3_can_bridge` | CAN 执行层（C++） | `motor_protocol_node`（200 Hz 插值→MIT 帧、反馈解码、F42/F46/F51）、`power_sequence_node`（电源序列+gate）、`can_transport_node`（SocketCAN 双总线）；配置 control_gains/motor_map/power_sequence/bridge + 5j/generic 档位 |
| `a3_description` | URDF / ros2_control | el_a3.urdf.xacro、named_poses.yaml、el_a3_controllers.yaml、multi_arm_config.yaml、inertia_params.yaml（F49 标定产物） |
| `a3_moveit_config` | MoveIt2 规划 | el_a3.srdf、kinematics(pick_ik)、ompl、servo_config、moveit_controllers、demo/robot launch |
| `a3_bringup` | 总启动 + 工具节点 | 9 个 launch（a3_bringup/a3_hardware/servo/edge_web_sim/edge_sim_wave_a/edge_moveit_execute/edge_teleop_sim/urdf_dir_check）；helper：trajectory_bridge / sim_executor / sim_motor_node / sim_power_sequence_node / gravity_torque_node / follow_joint_trajectory_action / move_to_pose_ik_node / draw_rectangle_demo / servo_mode_bridge / zero_to_ready_publisher 等 |
| `a3_arm_controller` | 编排门面 | `arm_controller`（11 态状态机 + 统一服务门面 + ArmStatus 10 Hz）、`arm_monitor_node`（F50 看门狗 20 Hz MonitorStatus） |
| `a3_gripper_controller` | 夹爪力控 | `gripper_controller_node`（50 Hz PI 力外环 + 位置内环，F24–F37） |
| `a3_mqtt_bridge` | ROS2→MQTT 遥测/下行 | `ros2mqtt_bridge`（bridge.yaml 白名单展平→telemetry；`<prefix>/cmd` op 白名单→服务回执 cmd_result；5 Hz 节流 F35） |
| `a3_teleop_ps4` | PS4 遥操作 | ds4_hid_node / ps4_arm_teleop / ps4_mapper（YAML 映射）/ joy_dump；action_registry + mappings |
| `a3_lerobot_config` | LeRobot 脚手架 | 纯 config/docs |
| `a3_cloud_edge` | CloudEdge Linux 栈 | micro_ros_agent / microros_mock_client / gravity_compensation_node / trajectory_test_publisher / moveit_plan_node |
| `third_party/` | junction | micro_ros_host（存在）；moveit_debs、../a3_arm_vendor（按需探测，本机不存在） |

### 2.3 核心跨包契约（文档要突出的「四条总线」）

1. **轨迹总线**：`/joint_group_effort_controller/joint_trajectory` —— 所有执行层指令汇聚点（arm_controller / gripper / FJT / servo / trajectory_bridge / gravity_compensation）。
2. **状态总线**：`/joint_states`（50 Hz，BEST_EFFORT，双 QoS 消费 LL-030）、`/a3/arm_status`（10 Hz）、`/a3/motor/states`（50 Hz）、`/a3/gripper_status`、`/a3/monitor/status`（20 Hz）。
3. **模式仲裁总线**：`/a3/control_mode`（IDLE/TRAJ_RUNNING/SERVO/ZERO_TORQUE/GRAVITY_COMP，**多节点共写**，BLOCKED_MODES 集合一致）。
4. **电源门控**：`/power_sequence/gate_open`（transient_local）+ `command/state`。
5. **MQTT 北向/南向**：前缀 `deep-trace/HOME-DEMO/RK3588/`（info/status/telemetry/cmd/cmd_result），与外部 deep-trace 设备 YAML 契约耦合（本仓有 mqtt-contract-coupling 规则）。

### 2.4 参考风格要点（已读 index/overview/components + style.css + mermaid-init.js）

- 每页：`<nav>`（固定导航条）+ `header.main`（渐变色标题头）+ `<div class="wrap">` 内多个 `<section>`。
- 内容元素：`<pre class="mermaid">`（flowchart/sequence）、`<table>`、`.note`/`.note info`、`.hub-bar`（原则总结条）、`.legend-inline`（图例）、`.grid2`/`.navcard`（主页导航卡）、`classDef` 语义配色（ui/eng/io/data/cfg/stat）。
- 全局：`style.css`（CSS 变量 --ui/--cfg/--eng/--io/--data/--stat + 卡片/表格/note/hub-bar 样式）、`mermaid-init.js`（base 主题，CDN jsdelivr mermaid@11）。
- 编号体系：下行 D1–D6 蓝、上行 U1–U7 绿，线标 `编号｜from·说明` + 速查表；「改哪里」决策流；核心组件档案速查表。
- 布局：`.layout` + `.toc` 侧边目录（detail 类长页用）。

## 三、Proposed Changes（全部新建于 `docs/architecture/`）

> 所有 HTML 复用同一 `style.css`/`mermaid-init.js`，nav 互相链接；配色语义按本项目重映射：UI→工具/演示节点、ENG→编排与规划、IO→CAN/执行层、DATA→配置/持久化、CFG→配置/launch、STAT→遥测/监视。

### 3.1 基础设施（2 文件）
- `style.css`：从 `docs/architecture-ref/style.css` 复刻（保持全部 CSS 类），按需微调主色变量（保持同一视觉语言即可）。
- `mermaid-init.js`：从参考复刻（base 主题，startOnLoad）。

### 3.2 index.html — 架构主页
- `header.main`：A3 Arm Platform 一句话（ROS 2 Humble · EDULITE A3 7-DOF · RK3588/SocketCAN/MIT/PS4 + 双产品线）+ 2026-09 架构决策与落地现状块（Edge 主线 / CloudEdge 支线 / 关键机制落地 F40-F52 / 权威文档指向）。
- `navcard` 8 张导航卡（LEVEL 1-4 分级，参照参考项目文案风格）。
- 「一句话主路径」mermaid：Edge 主路径（launch → trajectory 总线 → motor_protocol 200Hz → MIT CAN → 反馈 / joint_states；编排层 arm_controller 门面；MQTT 桥 → deep-trace Web；看门狗/夹爪力控旁路）+ CloudEdge 支线（服务器 MoveIt → gravity_compensation → micro-ROS Agent → ESP32 → CAN）。
- 图例（legend-inline）+「架构评估与改进建议」note 块（见第五节）。

### 3.3 overview.html — LEVEL 1 宏观架构
- 系统分层总览（含数据流编号）：Planning/HMI → Orchestration（arm_controller）→ Execution（a3_can_bridge 三节点）→ Platform（SocketCAN）→ Hardware；CloudEdge 服务器/ESP32 分层；MQTT 北向/南向 + 外部 deep-trace。
- 数据流编号速查表（D/U，参照参考项目 D1-D6/U1-U7 表格式，映射本项目：下行=轨迹/命令/配置，上行=反馈/状态/遥测）。
- 端到端价值流：launch → 建栈 → init/enable → 指令（web/PS4/话题）→ 执行 → 遥测上云 → web 展示/控制。
- 四条硬原则（hub-bar/mermaid）：①轨迹总线收敛执行入口；②状态总线 + 双 QoS 消费；③编排层门面统一对外、执行层权威拦截；④分层保护（F42 执行层钳位 / F44·F40 编排层 / F50 独立看门狗）。
- 技术栈关系（ROS 2 Humble / rclcpp·rclpy / Pinocchio / MoveIt / paho-mqtt / SocketCAN）。
- 双产品线对照表（Edge vs CloudEdge，沿用 docs/README 差异表）。
- 启动对象图（a3_bringup.launch 组合关系）。

### 3.4 detail.html — LEVEL 2 微观架构
- `a3_can_bridge` 内部：motor_protocol_node（订阅→插值 auto 线性/三次/五次→MIT 帧；反馈→/joint_states；F42 力矩方向钳位、F46 TX 帧率、F51 使能安全门禁/重锚/软起步）、power_sequence_node（Idle→…→Running 状态机 + gate）、can_transport_node（双总线队列）；F52 档位机制（7J/6J/5J 配对）。
- `a3_arm_controller`：11 态状态机图（IDLE→INIT→READY↔TRAJ/SERVO/TEACH/AI；SAFE_PARK→DISABLED/COOLING/FAULT）+ 服务/话题表 + arm_monitor（故障类→处置阶梯表）。
- `a3_gripper_controller`：PI 力外环 + 位置内环、状态机（IDLE/POSITION/FORCE_CLOSING/GRASPED/RELEASING/FAULT）、固件 0x700B 硬限 + 软件 clamp 双保险。
- `a3_mqtt_bridge`：bridge.yaml 结构（topic 白名单、flatten 规则、op 白名单、节流/队列）、双 QoS 订阅。
- 仿真栈：sim_motor_node / sim_power_sequence_node / sim_executor 与真机节点的对应关系。
- `a3_teleop_ps4`：YAML 映射架构（mapping.yaml → ps4_mapper → actions）。
- 长页布局用 `.layout` + `.toc` 侧边目录。

### 3.5 components.html — LEVEL 3 组件关系
- 包依赖总图（mermaid：a3_msgs ← 各包；a3_can_bridge ← a3_bringup/arm_controller/gripper；arm_controller ← mqtt_bridge；cloud_edge 独立栈）。
- 话题中枢 hub-and-spoke（轨迹总线 / 状态总线 / 模式总线 / 电源门控 四中枢）。
- 设计模式落点（门面 arm_controller / 状态机 / 桥接 trajectory_bridge / 观察者 QoS / 策略 F52 档位）。
- 「我想改什么」决策流（参照参考项目，映射到本项目文件：改轨迹执行→motor_protocol_node.cpp；改安全→control_gains.yaml/F42；改对外接口→arm_controller.py；改遥测→bridge.yaml；改力控→gripper_controller_node.py…）。
- 核心组件档案速查表（组件/层/职责/源文件）。

### 3.6 control_flow.html — LEVEL 4 控制流
- 启动流（can-up.service → launch 组合 → 节点起栈）。
- 初始化闭环 init → enable → READY（时序图）。
- 命令流：web MQTT cmd（goto/set_joints/teach/playback/enter_ai…）→ bridge → arm_controller 服务 → 轨迹总线 → 执行层。
- 电源序列时序（start/shutdown/set_zero + gate）。
- 失能保护 disable（F40 safe park）流程图。
- 状态机转移图（11 态完整转移链）。

### 3.7 signal_flow.html — LEVEL 4 信号流
- 轨迹总线拓扑（多源→单入口）。
- 状态总线拓扑 + QoS 矩阵表（话题/发布者/类型/QoS/消费方）。
- 模式仲裁流（control_mode 各发布者、BLOCKED_MODES、F29 教训）。
- MQTT 上/下行时序图（telemetry 展平/节流/NaN 清洗；cmd→cmd_result）。
- 跨线程/跨进程时序（编排层 vs 看门狗 vs 执行层的分层保护信号）。

### 3.8 data.html — LEVEL 3 数据架构
- 配置层级链：包内 YAML（真源）→ 用户层 `~/.a3/` 覆盖（poses.yaml / gripper_overrides.yaml / trajectories / stats/torque_stats.yaml / calibration）。
- 持久化数据：示教轨迹、命名点位、力矩统计（F43）、力控覆盖（F24）、重力标定数据（F49 JSONL → inertia_params.yaml）。
- 日志与遥测数据：MQTT points 目录（arm_*/grip_*/motor 42 点/txhz*/mtqmax*）。
- 数据流编号（复用 overview 的 D/U 表，data 视角）。

### 3.9 mechanisms.html — LEVEL 4 工作机制
- 分层安全保护矩阵（F42 执行层 200 Hz 力矩钳位 / F40 编排层 park / F44 温度 warn·protect·COOLING / F48 使能位置门禁 / F50 看门狗处置阶梯 / F51 使能重锚+软起步+保持抑制 / F52 档位降级）。
- 插值语义（auto 线性/三次/五次 + effort 线性）。
- MQTT 机制（展平、5 Hz 节流最新值胜出、NaN 清洗、cmd QoS、发布线程解耦）。
- QoS 双订阅机制（LL-030 / LL-020 冻结判据）。
- 降级档配对机制（7J/6J/5J 三件套）。

### 3.10 README.md — Markdown 速览/索引（满足 docs-update 每目录 README 规则）
- 一句话职责 + 8 页 HTML 索引表（每页一句说明 + 相对链接）+ 与现有 docs/edge、docs/shared、docs/lessons_learned 的关系 + 打开方式（浏览器 / `python -m http.server` 可选说明）。

## 四、Assumptions & Decisions

- **只新建 `docs/architecture/` 下文件**，不改任何源代码、不迁移/删除现有 docs（`docs/ARCHITECTURE.md` 迁移存根保持，可在 README 提一句指向新图集）。
- 内容口径以调研到的源码 + TOPIC_CONTRACT + REQUIREMENTS + 既有 ARCHITECTURE 为准；不确定项在文档中标「待确认」而非编造。
- 覆盖双产品线：Edge 详尽、CloudEdge 支线一页内概述（链接 cloud_edge/ARCHITECTURE.md）。
- 图集内 mermaid 用 CDN（jsdelivr，与参考一致），README 注明离线时显示源码文本。
- 架构评估结论写入文档（index 现状块 + mechanisms 建议），**不实施代码改动**，是否修复由用户后续决策。

## 五、架构评估与改进建议（将写入文档）

1. **规则文件漂移（高优先）**：`.trae/rules/`、`.cursor/rules/` 内容实为另一个项目（deep-trace 云端 API/前端/边缘 modbus）的规则（如 architecture-sync 指向 `docs/design/`、unified-device-architecture、edge-v2、Django、`D:\working\test-platform\docs\architecture` 权威图），与 a3_arm_ws（ROS2 机械臂）**完全无关且相互矛盾**，会误导 Agent 索引。建议：清理/替换为本项目规则，或至少将跨仓权威指向改为本项目真实文档。
2. **死代码（低）**：`a3_bringup` 的 `zero_to_work_publisher.py` 仅剩 `__pycache__` 残留、无源码、无 setup 入口 → 建议删除残留。
3. **模式总线多写者（中，F29 教训）**：`/a3/control_mode` 由 5 个节点共写（motor_protocol / gravity_torque / servo_mode_bridge / FJT / arm_controller），无单一 owner，曾导致 TRAJ_RUNNING 永久锁存（F29）。建议收敛为「arm_controller 为对外权威发布者 + 各底层节点向它上报状态」或引入仲裁层。
4. **配置档位配对脆弱（中）**：F52 的 5J/6J/7J 三件套（control_gains_*j + motor_map_*j + arm_controller_*j）靠人工配对，配错才退回 7J 并 ERROR。建议启动时校验三文件一致或由单一 motor_map 派生。
5. **MQTT 跨仓契约耦合（已知、已被规则约束）**：bridge.yaml 信号 code 与 deep-trace 设备 YAML 两端耦合，改动须两端同步（本仓规则已列检查清单，文档中标注）。
6. **QoS 陷阱（已知）**：`/joint_states` 为 BEST_EFFORT，消费方必须双 QoS 订阅（LL-030）；`/power_sequence/gate_open` transient_local。文档数据/信号页标注。
7. **文档体系**：现有 `docs/edge/ARCHITECTURE.md` 与新 `docs/architecture/` 图集内容重叠 → README 里划清定位（图集=可视化总览，md=细节真源）。

## 六、Verification

1. 每个 HTML 检查：nav 链接相对路径正确、`<pre class="mermaid">` 语法合法（本地可用 mermaid CLI `mmdc` 校验，或浏览器打开 `index.html` 目检渲染；无浏览器时用 mermaid 在线校验）。
2. 内容抽查：图集里的话题/服务名/配置路径与 TOPIC_CONTRACT / REQUIRements / 源码一致（随机抽查 3-5 处）。
3. `docs/architecture/README.md` 存在且索引完整（满足每目录 README 规则）。
4. 8 页 HTML + 2 资产 + 1 README 共 11 个新文件全部就位。
