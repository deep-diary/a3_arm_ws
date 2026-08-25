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
| S7 | Agent launch | `a3_cloud_edge` 提供 micro-ROS Agent launch（UDP 8888 可配置） | **implemented** |
| S8 | 测试轨迹发布 | 按 [TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md) 发布 `JointTrajectory` | **implemented** |
| S9 | Linux mock client | `microros_mock_client` 经 Agent 订阅轨迹、50 Hz 发布 `joint_states` | **implemented** |
| S10 | 链路冒烟 launch | `cloud_edge_link_test.launch.py` 一键 Agent + Mock + 测试发布 | **implemented** |

### ESP32-S3 侧（外置固件，见 [QUICKSTART.md](QUICKSTART.md)）

> **实现仓库：** [deep-diary/xiaozhi-esp32](https://github.com/deep-diary/xiaozhi-esp32) — `main/boards/deep-dog/`  
> **本仓库角色：** 只维护需求、契约、Agent/联调；**不**存放 ESP32 固件源码。跨仓协作见仓库根目录 [AGENT.md](../../AGENT.md)。

| ID | 需求 | 说明 | 状态 |
|----|------|------|------|
| E1 | micro-ROS Client | WiFi 连接 Agent，订阅轨迹、发布关节状态 | 见阶段 A |
| E2 | 轨迹本地插值 | 按 `time_from_start` 插值，200 Hz 发 CAN | pending |
| E3 | MIT CAN 协议 | 移植 `protocol_codec` / `frame_codec` 逻辑 | pending |
| E4 | 关节状态上行 | 50 Hz 发布 `JointState` | 见阶段 A |
| E5 | 电源序列 / 门控 | 本地实现 gate，与 Edge 契约一致 | pending |
| E6 | 断连看门狗 | 超时无指令 → disable 电机 | pending（阶段 B） |
| E7 | 本地软限位 | 移植 `control_gains.yaml` 限位表 | pending |
| E8 | 硬件急停 | GPIO，独立于 WiFi | pending |
| E9 | micro-ROS Client（板级） | xiaozhi-esp32 `deep-dog` + `micro_ros_espidf_component`（Humble） | 见阶段 A |
| E10 | 200 Hz 轨迹插值 + TWAI CAN MIT | 复用 `motor/`、`trajectory/` | pending（阶段 B） |
| E11 | 断连看门狗 → disable 电机 | 固件本地，&lt; 500 ms | pending（阶段 B） |

### 阶段 A — micro-ROS 链路冒烟（当前优先）

覆盖 **E1 / E9**，以及 **E4 的发布侧最小实现**（允许 mock 关节数据，**不要求**发 CAN）。

| 项 | 要求 |
|----|------|
| 板级 | ESP32-S3，`deep-dog` |
| ROS / 组件 | ROS 2 **Humble**；[micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component) **humble**（Component Registry `>=22.0.0`），勿混用 rolling |
| 传输 | UDPv4 over WiFi |
| Agent | 与板同网段可达的主机 IP + 端口 **8888**（可配置；**禁止**在真机上写 `127.0.0.1`） |
| 订阅 | `/joint_group_effort_controller/joint_trajectory`（`trajectory_msgs/JointTrajectory`） |
| 发布 | `/joint_states`（`sensor_msgs/JointState`），目标 **50 Hz** |
| 关节名 | `L1_joint` … `L7_joint`（见 [TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)） |
| 配置 | WiFi SSID/密码、Agent IP、端口须可配置（menuconfig / Kconfig），勿硬编码密钥入库 |
| 重连 | Agent 或 Client 重启后可重新建立 XRCE session |

**阶段 A 验收标准：**

1. 服务器侧 `ros2 launch a3_cloud_edge micro_ros_agent.launch.py port:=8888` 已运行
2. 板上电后能关联实验室 WiFi，并连上 Agent（Agent 日志可见 session）
3. `ros2 topic hz /joint_states` ≈ 50 Hz（允许 mock position）
4. 服务器发布测试 `JointTrajectory` 后，固件侧回调/日志确认收到 `points`
5. 重启 Client 或 Agent 后可再次建立 session，无需重新烧录配置以外的操作

**阶段 A 不做：** CAN 发送、200 Hz 插值、看门狗 disable、gate/急停（归阶段 B / E2–E8、E10–E11）。

### 阶段 B — 执行与安全（链路通过后）

实现 E2、E3、E6、E10、E11 等，须遵守 [SAFETY.md](../shared/SAFETY.md)；真机验收见下文「验收标准」第 1–5 条。

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

## 本阶段验收（Linux 链路）

1. `ros2 launch a3_cloud_edge cloud_edge_link_test.launch.py` 无报错
2. `ros2 topic hz /joint_states` ≈ 50 Hz
3. 测试轨迹发布后 `joint_states.position` 随时间变化
4. Mock client 重启后可重新建立 XRCE session

完整真机验收（E 系列）见上文「验收标准」第 1–5 条。

## 关联文档

- [AGENT.md](../../AGENT.md)（跨仓智能体协作 / 路径）
- [QUICKSTART.md](QUICKSTART.md)
- [ARCHITECTURE.md](ARCHITECTURE.md)
- [ROADMAP.md](ROADMAP.md)
- [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- [shared/SAFETY.md](../shared/SAFETY.md)
- [shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)
