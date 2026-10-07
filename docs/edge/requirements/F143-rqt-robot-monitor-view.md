# F143 — rqt_robot_monitor 诊断值班视图与排障 SOP【P2】


- **说明：** F71/F82/F96/F102–F104 已把硬件、主机、CAN、MQTT、话题速率全部汇入 `/diagnostics` 与 `/diagnostics_agg`，但缺面向现场值班的统一视图与排障手册。对标社区 fork 的 `/diagnostics` overlay for `rqt_robot_monitor`：提供预配置 rqt 布局 + 一页「告警级别→含义→处置」SOP（QUICKSTART/ROS_COMMANDS），不新增自研监控节点。
- **验收标准：**
  1. 一条命令/一个 rqt preset 打开全系统诊断树，各分组（Hardware/Host/Comms/Topic Rates）正确归类
  2. 文档列出每个 ERROR/STALE 项的含义与首步处置；拔 CAN/断 MQTT/停话题可在视图复现对应告警
- **关联：** F82（diagnostic_aggregator）、F96（主机诊断）、F102/F103/F104（链路诊断）、F95（自检）
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
