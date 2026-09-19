<!-- cursor-meta: description=架构同步：功能与架构/契约有出入时及时更新 docs/edge、docs/cloud_edge、docs/shared 对应文档 | alwaysApply=true -->

# 架构同步

## 触发条件

新开发的功能与现有架构有出入时——新增/迁移模块、话题/服务/消息变化、控制链路变化、部署形态调整——必须**及时**更新对应文档。

## 本仓架构/契约文档（docs/，图表优先 mermaid）

| 主题 | 文档 | 更新时机 |
|---|---|---|
| 产品线总览 | `docs/README.md` | 产品线/部署形态变化 |
| Edge 架构（主线） | `docs/edge/ARCHITECTURE.md` | 运行时分层、启动/部署方式变化 |
| CloudEdge 架构（支线） | `docs/cloud_edge/ARCHITECTURE.md` | Agent/mock/固件链路变化 |
| 话题/服务/关节契约 | `docs/shared/TOPIC_CONTRACT.md` | 话题、服务、消息、op 白名单变化 |
| 安全契约 | `docs/shared/SAFETY.md` | 限位、门禁、急停、安全原则变化 |
| 控制能力对标 | `docs/shared/CONTROL_ROADMAP.md` | 控制能力（Wave A/B、插值/重力/MoveIt）或对标缺口变化 |
| 需求文档 | `docs/edge/REQUIREMENTS.md`、`docs/cloud_edge/REQUIREMENTS.md` | 需求条目新增/状态变化 |

- 新增设计文档时同步更新 `docs/README.md` 索引。

## 控制对标权威

本仓控制能力对标 reBot（L0–L8 / Wave A·B）以 `docs/shared/CONTROL_ROADMAP.md` 为权威；
参考项目实现与缺口维护在 `CONTROL_ROADMAP.md` 与 `reference-first.md`。

## 运行时架构（速览，供对照）

Edge 运行时分层：Platform（`can-up.service`）→ Execution（`a3_can_bridge`：`can_transport_node` + `motor_protocol_node` + `power_sequence_node`）→ Description（`a3_description` URDF / `a3_moveit_config`）→ Shell（`trajectory_bridge` 桥接 reBot 话题）→ HMI（`a3_teleop_ps4`）→ Orchestration（`a3_arm_controller` 状态机）。详见 `docs/edge/ARCHITECTURE.md`。
