# A3 CloudEdge — 落地路线图

> **Status:** draft  
> **产品线：** A3 CloudEdge（`cloud_edge`）

## 目标形态

**内网服务器（ROS 2 + MCP）+ ESP32-S3（每臂）**，每臂无 RK3588。

开发期可借用 A3 Edge 硬件验证网络契约，**不作为最终量产形态**。

## 阶段概览

| 阶段 | 内容 | 验证手段 | 产出 |
|------|------|----------|------|
| **P0** | 服务器下发 `JointTrajectory`，Edge RK3588 仅跑 `a3_can_bridge` | 借用 Edge 硬件 | 网络契约验证 |
| **P1** | ESP32 CAN MIT 固件（移植 `protocol_codec`） | 台架、单电机 | `firmware/cloud_edge/` 初版 |
| **P2** | micro-ROS WiFi + 轨迹插值 200 Hz | 单臂 WiFi | Client 联通 Agent |
| **P3** | 多臂 namespace + 内网 MCP | 实验室内网 2+ 臂 | MCP 任务 API |
| **P4** | 去掉每臂 RK3588，ESP32-only 量产 | 验收测试 | 量产 BOM |

## P0 — 网络契约验证

**目标：** 在不改 ESP32 的前提下，验证「远程规划 → 本地执行」话题契约。

**做法：**

1. 内网服务器运行 MoveIt / 轨迹发布节点
2. RK3588 仅启动 `a3_can_bridge`（不跑本地规划）
3. 通过网络 ROS 2（或桥接）将 `JointTrajectory` 送到板端
4. 验证 `joint_states` 回传

**验收：** 与 [edge/QUICKSTART.md](../edge/QUICKSTART.md) 等效的轨迹测试，由服务器触发。

## P1 — ESP32 CAN 固件

**目标：** 脱离 RK3588，ESP32 直接驱动 CAN。

**任务：**

- 移植 `protocol_codec.hpp`、`frame_codec.hpp` MIT 编解码
- TWAI / 外部 CAN 收发器驱动
- 单电机 enable / MIT / 反馈解析
- 参考脚本：`a3_can_bridge/scripts/a3_motor_cansend.sh`

**验收：** 台架上 ID 1 电机 MIT 控制与 Edge 行为一致。

## P2 — micro-ROS 单臂联通

**目标：** WiFi 上完成轨迹下发与状态回传。

**任务：**

- ESP32 micro-ROS Client（WiFi transport）
- 服务器 micro-ROS Agent
- 轨迹缓冲 + 200 Hz 插值定时器
- 50 Hz `JointState` 发布
- 断连看门狗初版

**验收：** 单臂完成 2 s 轨迹，断 WiFi 后电机 disable。

## P3 — 多臂 + MCP

**目标：** 实验室内网多臂调度。

**任务：**

- 每 ESP32 独立 namespace（`arm1_`、`arm2_`）
- 服务器多 Agent 会话或单 Agent 多 Client
- MCP 封装：move_to_pose、execute_trajectory、get_joint_states
- 与 [multi_arm_config.yaml](../../src/a3_description/config/multi_arm_config.yaml) 对齐

**验收：** 2 臂并发无严重丢包；MCP 完成一次规划执行闭环。

## P4 — 量产形态验收

**目标：** 确认可去掉每臂 RK3588。

**任务：**

- BOM 清单：ESP32-S3 + CAN + 电源 + WiFi 模块
- 完整安全测试（急停、看门狗、软限位、gate）
- 运维文档：服务器部署、固件烧录、臂注册流程

**验收：** 满足 [REQUIREMENTS.md](REQUIREMENTS.md) 全部验收标准。

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| WiFi 抖动 | 轨迹块 + lookahead；边缘插值 |
| ESP32 算力 | 仅插值 + CAN，不做完整 Pinocchio |
| micro-ROS 内存 | 限制轨迹缓冲深度；静态分配 |
| 多臂 WiFi 拥塞 | 5 GHz AP；每臂限速；有线 Agent 备选 |

## 关联文档

- [REQUIREMENTS.md](REQUIREMENTS.md)
- [ARCHITECTURE.md](ARCHITECTURE.md)
- [edge/ARCHITECTURE.md](../edge/ARCHITECTURE.md)
- [文档索引](../README.md)
