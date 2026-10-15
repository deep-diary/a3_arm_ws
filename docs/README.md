# A3 Arm Platform — 文档索引

> **Status:** active

本工作区（**A3 Arm Platform**）在同一机器人模型（EDULITE A3）下支持两种**部署形态（Deployment Profile）**，文档按产品线归类。

## 两条产品线

| 产品线 | 代号 | 中文名 | 硬件形态 | 状态 |
|--------|------|--------|----------|------|
| **A3 Edge** | `edge` | 边缘全栈线（**主线**） | RK3588 + SocketCAN，ROS 2 全栈在板上 | 当前生产/开发主线 |
| **A3 CloudEdge** | `cloud_edge` | 云端薄边缘线（**支线**） | 内网服务器（ROS 2 + MCP）+ ESP32-S3（CAN 桥） | 规划/演进目标 |

### 对比

| 维度 | A3 Edge | A3 CloudEdge |
|------|---------|--------------|
| 单臂主控 | RK3588（Linux + ROS 2） | ESP32-S3（固件） |
| 轨迹规划 | 板载 MoveIt / reBot | 内网服务器集中规划 |
| CAN 执行 | RK3588 SocketCAN → `a3_can_bridge` | ESP32 本地 MIT 协议 |
| 控制延迟 | 板内 &lt; 10 ms | WiFi 轨迹块下发 + 边缘 200 Hz 插值 |
| 多臂算法共用 | 每臂独立 ROS 栈 | 服务器一套算法多臂共用 |
| 单臂 BOM | 较高（需 Linux 板） | 较低（ESP32 + WiFi） |
| 适用场景 | 单机低延迟力控、现网部署 | 多臂实验室、降本量产、MCP 编排 |

**当前默认开发路径：** A3 Edge。CloudEdge 为演进支线，见 [cloud_edge/ROADMAP.md](cloud_edge/ROADMAP.md)。

## 智能体入口

跨仓职责、WSL/Windows 绝对路径、外置固件提示词模板见仓库根目录 **[AGENT.md](../AGENT.md)**（固件在 Windows、本仓库在 WSL 时必读）。

## 文档导航

```
AGENT.md                      ← 智能体协作（仓库根）
docs/
├── README.md                 ← 本页
├── shared/                   两线共用契约
│   ├── ROBOT_MODEL.md
│   ├── TOPIC_CONTRACT.md
│   ├── SAFETY.md
│   ├── CONTROL_ROADMAP.md    控制功能开发路线
│   └── AI_ROADMAP.md         AI 功能开发路线
├── control/                  控制「从简到难」进阶清单（模块化学习手册）
│   ├── README.md             模块索引 + 学习路径 + L0–L8 映射
│   ├── 01-drive-bus.md       驱动与总线
│   ├── 02-kinematics.md      运动学
│   ├── 03-trajectory.md      轨迹规划与时间律
│   ├── 04-dynamics.md        动力学与补偿
│   ├── 05-force-compliance.md 力控与柔顺
│   └── 06-safety-realtime.md 安全与实时
├── edge/                     A3 Edge 主线
│   ├── REQUIREMENTS.md         需求总览 + 索引（正文在 requirements/）
│   ├── requirements/           需求正文 F###-slug.md（一条一文件）
│   ├── ARCHITECTURE.md
│   ├── STATE_MACHINE.md       编排层状态机（指令×状态矩阵）
│   ├── QUICKSTART.md
│   ├── PS4_OPERATOR_GUIDE.md  PS4 手柄操作员手册（F60/F64 键位/五色灯/震动）
│   ├── TRAJECTORY_MODE_SELECTION.md  轨迹规划模式选型指南（F165：G64 圆角/模式对比/推荐器）
│   ├── ROS_COMMANDS.md        ROS 命令速查（source/构建/launch/服务/排障）
│   └── PLATFORM_CAN.md
├── cloud_edge/               A3 CloudEdge 支线
│   ├── REQUIREMENTS.md
│   ├── ARCHITECTURE.md
│   ├── QUICKSTART.md
│   └── ROADMAP.md
├── lessons_learned/          踩坑备忘（换机参考）
│   ├── README.md
│   └── LL-*.md
└── dev/
    └── WSL2_SETUP.md         开发环境（两线共用）
```

### 共享层

| 文档 | 说明 |
|------|------|
| [shared/ROBOT_MODEL.md](shared/ROBOT_MODEL.md) | 7 关节模型、电机类型、CAN ID 映射 |
| [shared/TOPIC_CONTRACT.md](shared/TOPIC_CONTRACT.md) | 轨迹输入、关节状态、电源门控 ROS 契约 |
| [shared/SAFETY.md](shared/SAFETY.md) | 软限位、gate、急停、断连策略原则 |
| [shared/CONTROL_ROADMAP.md](shared/CONTROL_ROADMAP.md) | 控制功能分层路线（L0–L8）、reBot/A3 对比、Edge C1–C8 |
| [shared/AI_ROADMAP.md](shared/AI_ROADMAP.md) | AI 赋能分层路线（A0–A5）、reBot AI 功能清单、复刻/自研两步路线 |

> `CONTROL_ROADMAP.md` 是**规划/对标**用层级底稿（L0–L8、Wave A/B）；面向新人的**模块化学习清单**见下方 `control/`（按模块逐级说明每个功能点「从简到难」进阶与增量能力）。

### 控制学习手册（control/）

按「从简到难」逐级说明六个核心控制模块的功能点进阶、`新增能力`、工业对标、本仓状态与可继续进阶缺口。定位是新人学习/查阅（学习手册），与 `CONTROL_ROADMAP.md`（规划底稿）分工，通过 L0–L8↔模块映射表对齐。

| 文档 | 覆盖层级 | 说明 |
|------|----------|------|
| [control/README.md](control/README.md) | — | 模块索引、六模块总览、新人学习路径、L0–L8↔模块映射 |
| [control/01-drive-bus.md](control/01-drive-bus.md) | L0–L1 | 驱动与总线（单电机 → 多厂商 HAL） |
| [control/02-kinematics.md](control/02-kinematics.md) | L3 | 运动学（FK/IK → 无奇异路径与任务约束） |
| [control/03-trajectory.md](control/03-trajectory.md) | L2/L4/L5 | 轨迹规划与时间律（PTP → 工业级统一调度 + 在线 OTG） |
| [control/04-dynamics.md](control/04-dynamics.md) | L3/L7 | 动力学与补偿（重力 → 全模型前馈 + 辨识 + 观测器） |
| [control/05-force-compliance.md](control/05-force-compliance.md) | L8 | 力控与柔顺（关节阻抗 → 末端力控与接触任务） |
| [control/06-safety-realtime.md](control/06-safety-realtime.md) | 横切 | 安全与实时（软限位 → 功能安全与可观测性） |

### A3 Edge（主线）

| 文档 | 说明 |
|------|------|
| [edge/REQUIREMENTS.md](edge/REQUIREMENTS.md) | 边缘全栈需求（总览 + 索引，正文在 requirements/） |
| [edge/requirements/](edge/requirements/) | 需求正文 F###-slug.md（一条一文件，F1–F174） |
| [edge/ARCHITECTURE.md](edge/ARCHITECTURE.md) | 运行时分层与数据流 |
| [edge/STATE_MACHINE.md](edge/STATE_MACHINE.md) | 编排层状态机（11 态 + 指令×状态×模式矩阵，F53） |
| [edge/QUICKSTART.md](edge/QUICKSTART.md) | 真机快速上手清单 |
| [edge/PS4_OPERATOR_GUIDE.md](edge/PS4_OPERATOR_GUIDE.md) | PS4 操作员手册：流程、五色灯、震动（F60/F64） |
| [edge/TRAJECTORY_MODE_SELECTION.md](edge/TRAJECTORY_MODE_SELECTION.md) | 轨迹规划模式选型指南（F165/F166：9 模式注册表、G64 圆角、逐点 exact、离线 auto 规划） |
| [edge/ROS_COMMANDS.md](edge/ROS_COMMANDS.md) | ROS 命令速查：source/构建/launch/服务/排障 |
| [edge/PLATFORM_CAN.md](edge/PLATFORM_CAN.md) | RK3588 SocketCAN 上板配置 |

### A3 CloudEdge（支线）

| 文档 | 说明 |
|------|------|
| [../AGENT.md](../AGENT.md) | 本仓 vs 外置固件职责、路径、阶段 A 提示词 |
| [cloud_edge/REQUIREMENTS.md](cloud_edge/REQUIREMENTS.md) | 云端薄边缘需求（含 ESP32 阶段 A/B） |
| [cloud_edge/ARCHITECTURE.md](cloud_edge/ARCHITECTURE.md) | 服务器 + ESP32 架构 |
| [cloud_edge/ROADMAP.md](cloud_edge/ROADMAP.md) | 分阶段落地路线 |
| [cloud_edge/QUICKSTART.md](cloud_edge/QUICKSTART.md) | Linux Agent + mock client 链路测试 |

### 开发环境

| 文档 | 说明 |
|------|------|
| [dev/WSL2_SETUP.md](dev/WSL2_SETUP.md) | WSL2 + Humble mock/MoveIt 搭建 |
| [dev/WAVE_A_SIM_TEST_REPORT.md](dev/WAVE_A_SIM_TEST_REPORT.md) | Wave A 双链路仿真验收（zero→ready） |
| [lessons_learned/README.md](lessons_learned/README.md) | 踩坑与换机备忘索引 |

### 包内文档

| 文档 | 说明 |
|------|------|
| [../src/a3_lerobot_config/docs/INTEGRATION.md](../src/a3_lerobot_config/docs/INTEGRATION.md) | LeRobot 集成脚手架 |
| [../src/a3_can_bridge/README.md](../src/a3_can_bridge/README.md) | Edge CAN 栈节点说明 |
| [../src/a3_robot_model/README.md](../src/a3_robot_model/README.md) | **系统级模型配置**：默认描述包与档位文件（F174，可 `~/.a3` 覆盖） |
| [../src/a3_description_6j/README.md](../src/a3_description_6j/README.md) | EL-A3 **6 轴 + 连杆** 描述包（F174，与 7 轴 `a3_description` 整包隔离；当前默认档） |
| [../src/app_melodion/README.md](../src/app_melodion/README.md) | 机械臂弹奏口风琴 app（键位/几何/曲谱/IK/播放） |

## 构建与运行

工作区编译、launch 命令见根目录 [README.md](../README.md)。
