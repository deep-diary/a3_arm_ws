# A3 CloudEdge — 需求文档

> **Status:** draft  
> **产品线：** A3 CloudEdge（`cloud_edge`）— 云端薄边缘线（支线）

## 背景与目标

将轨迹规划、重力补偿（开环）、多臂调度等机械臂算法集中部署在公司内网服务器，每臂仅保留 ESP32-S3 作为 CAN 桥与安全网关。多臂共用一套 ROS 2 + MCP 算法栈，降低单臂 BOM（去掉每臂 RK3588），便于统一运维与算法迭代。

**目标形态：** 内网服务器 + ESP32-S3（每臂无 RK3588）。

## 目标用户与场景

- 实验室内多臂协作（2+ 臂）
- 通过公司内网 MCP 统一调用机械臂能力
- 降本量产（单臂仅 ESP32 + CAN + WiFi）
- 算法团队集中维护 MoveIt / Pinocchio / reBot 栈

## 功能需求

### 服务器侧

| ID | 需求 | 说明 |
|----|------|------|
| S1 | MoveIt 轨迹规划 | 碰撞检测、路径规划，输出 `JointTrajectory` |
| S2 | 重力补偿（开环） | 规划阶段预计算 `effort`，写入轨迹点 |
| S3 | micro-ROS Agent | XRCE-DDS Agent，管理多 ESP32 Client |
| S4 | 多臂 namespace | 每臂独立话题前缀，如 `/arm1/...` |
| S5 | MCP 高层 API | 任务级接口（移动、抓取、归位等） |
| S6 | 关节状态汇聚 | 订阅各臂 `joint_states`，供规划与监控 |

### ESP32-S3 侧

| ID | 需求 | 说明 |
|----|------|------|
| E1 | micro-ROS Client | WiFi 连接 Agent，订阅轨迹、发布关节状态 |
| E2 | 轨迹本地插值 | 按 `time_from_start` 插值，200 Hz 发 CAN |
| E3 | MIT CAN 协议 | 移植 `protocol_codec` / `frame_codec` 逻辑 |
| E4 | 关节状态上行 | 50 Hz 发布 `JointState` |
| E5 | 电源序列 / 门控 | 本地实现 gate，与 Edge 契约一致 |
| E6 | 断连看门狗 | 超时无指令 → disable 电机 |
| E7 | 本地软限位 | 移植 `control_gains.yaml` 限位表 |
| E8 | 硬件急停 | GPIO，独立于 WiFi |

## 非功能需求

| 指标 | 要求 |
|------|------|
| 轨迹下发 | 块式 `JointTrajectory` + lookahead（2–5 s），非每帧闭环 |
| ESP32 CAN 频率 | 200 Hz / 电机（本地定时器） |
| 关节状态上行 | 50 Hz |
| 断连看门狗 | 100–500 ms 可配置 |
| WiFi | 实验室 2.4/5 GHz，内网隔离 |
| 多臂并发 | ≥ 2 臂稳定运行（P3 验收） |

## 硬件与网络依赖

- **服务器：** Ubuntu 22.04 + ROS 2 Humble，内网可达
- **每臂：** ESP32-S3 + CAN 收发器 + WiFi
- **电机：** 与 Edge 相同（MIT，ID 1..7）
- **开发期：** 可借用 Edge RK3588 验证网络契约（非最终形态）

## 边界与不做事项

- **不做：** 云端 200 Hz 闭环力控（往返延迟不可接受）
- **不做：** 安全逻辑仅依赖云端（看门狗、急停必须本地）
- **不做：** 将 MCP 直接接入 CAN 实时环
- **不做：** 公网暴露未鉴权的机械臂控制接口

## 验收标准

1. 单臂：服务器下发 2 s 轨迹，ESP32 跟踪误差在配置容差内
2. 断网：拔掉 WiFi 或停 Agent 后 &lt; 500 ms 电机 disable
3. gate：未 start 时轨迹不执行
4. 多臂：2 臂并发轨迹，无明显丢包导致的运动异常
5. MCP：通过 MCP 调用完成一次「规划 + 执行」闭环

## 关联文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [ROADMAP.md](ROADMAP.md)
- [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- [shared/SAFETY.md](../shared/SAFETY.md)
- [shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)
