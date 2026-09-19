# A3 Arm Platform — 架构图集（docs/architecture）

> 本目录为 **HTML + Mermaid 网页架构图集**（风格复刻 `docs/architecture-ref/`），面向「快速理解全项目」：
> 浏览器打开 `index.html` 即可导航。细节真源仍以 Markdown 文档为准（见下「真源文档」）。

## 使用方式

- 直接双击 `index.html`（需联网加载 jsdelivr CDN 的 mermaid；离线时流程图显示为源码文本）
- 或本地起静态服务：`python3 -m http.server 8899` 后访问 `http://localhost:8899/index.html`

## 页面索引

| 页面 | 粒度 | 一句话内容 |
|------|------|-----------|
| [index.html](index.html) | — | 主页：双产品线、一句话主路径、2026-09 现状与**架构评估与改进建议** |
| [overview.html](overview.html) | LEVEL 1 · 系统级 | 宏观架构：分层+数据流 D/U、价值流、四条硬原则、技术栈、双产品线对照、启动组合 |
| [detail.html](detail.html) | LEVEL 2 · 模块级 | 微观架构：a3_can_bridge 三节点、arm_controller 11 态状态机、夹爪力控、MQTT 桥、仿真栈、PS4 映射、CloudEdge 验证栈 |
| [components.html](components.html) | LEVEL 3 · 组件级 | 包依赖总图、话题中枢 Hub-Spoke（轨迹/状态/模式/门控四总线）、设计模式、「改哪里」决策流、组件档案表 |
| [control_flow.html](control_flow.html) | LEVEL 4 · 控制 | 启动、init→enable、Web cmd 下行、电源序列、失能保护 park、11 态转移 |
| [signal_flow.html](signal_flow.html) | LEVEL 4 · 信号 | 轨迹/状态/模式/门控四总线拓扑、QoS 矩阵、MQTT 上下行时序、分层保护信号 |
| [data.html](data.html) | LEVEL 3 · 数据 | 配置层级链（包内 YAML → `~/.a3/` 用户层覆盖）、持久化、MQTT telemetry points 目录 |
| [mechanisms.html](mechanisms.html) | LEVEL 4 · 机制 | 分层安全保护矩阵（F40–F52）、插值语义、MQTT 展平/节流/NaN 清洗、QoS 双订阅、降级档配对 |

## 真源文档（细节权威）

| 主题 | 文档 |
|------|------|
| 产品线总览 / 文档索引 | [docs/README.md](../README.md) |
| Edge 运行时架构与分层 | [docs/edge/ARCHITECTURE.md](../edge/ARCHITECTURE.md) |
| Edge 功能需求（F 系列） | [docs/edge/REQUIREMENTS.md](../edge/REQUIREMENTS.md) |
| CloudEdge 架构 | [docs/cloud_edge/ARCHITECTURE.md](../cloud_edge/ARCHITECTURE.md) |
| 机器人模型（关节/电机/CAN） | [docs/shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md) |
| ROS 话题/服务契约 | [docs/shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md) |
| 安全原则 | [docs/shared/SAFETY.md](../shared/SAFETY.md) |
| 踩坑备忘 | [docs/lessons_learned/README.md](../lessons_learned/README.md) |

## 定位约定

- **图集 = 可视化总览**（本项目）；**Markdown = 细节真源**。改行为/契约时先改 Markdown（见 `.trae/rules/docs-update.md`），再按需回刷图集。
- 图集内容基于 2026-09 源码调研；与真源冲突时以真源为准。

## 维护提示

- 所有页面共用 `style.css` 与 `mermaid-init.js`；导航条（`<nav>`）与页面间链接需保持一致。
- 新增页面后同步更新本 README 索引表与 `index.html` 的导航卡。
