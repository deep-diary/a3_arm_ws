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
├── edge/                     A3 Edge 主线
│   ├── REQUIREMENTS.md
│   ├── ARCHITECTURE.md
│   ├── QUICKSTART.md
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

### A3 Edge（主线）

| 文档 | 说明 |
|------|------|
| [edge/REQUIREMENTS.md](edge/REQUIREMENTS.md) | 边缘全栈需求 |
| [edge/ARCHITECTURE.md](edge/ARCHITECTURE.md) | 运行时分层与数据流 |
| [edge/QUICKSTART.md](edge/QUICKSTART.md) | 真机快速上手清单 |
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
| [dev/WAVE_A_SIM_TEST_REPORT.md](dev/WAVE_A_SIM_TEST_REPORT.md) | Wave A 双链路仿真验收（zero→work） |
| [lessons_learned/README.md](lessons_learned/README.md) | 踩坑与换机备忘索引 |

### 包内文档

| 文档 | 说明 |
|------|------|
| [../src/a3_lerobot_config/docs/INTEGRATION.md](../src/a3_lerobot_config/docs/INTEGRATION.md) | LeRobot 集成脚手架 |
| [../src/a3_can_bridge/README.md](../src/a3_can_bridge/README.md) | Edge CAN 栈节点说明 |

## 构建与运行

工作区编译、launch 命令见根目录 [README.md](../README.md)。
