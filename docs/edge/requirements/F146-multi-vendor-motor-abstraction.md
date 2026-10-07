# F146 — 多厂商电机/多传输抽象层评估（motorbridge 兼容）【P3】


- **说明：** reBot 经 motorbridge 一套 API 支持 Robstride/Damiao/Mota/Gaoqing/Hexfellow 及串口桥/SocketCAN。本仓仅 MIT over SocketCAN。仅在未来引入非 MIT 电机（如 RS 私有总线）或串口 CAN 桥时立项：评估直接采用 motorbridge 或在 `a3_hardware_interface` 内做传输/协议抽象，避免届时重写编解码。**当前无硬件需求，不预先开发。**
- **验收标准：**
  1. 输出选型评估（motorbridge 直接集成 vs 自研抽象）与对 ros2_control read/write 语义、延迟（<10 ms）的影响
  2. 若实施：新增电机型号仅改配置/YAML 即可在同一 bringup 下切换
- **关联：** F72（SystemInterface 插件）、[CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) L0；[motorbridge](https://motorbridge.seeedstudio.com)
- **状态：** `proposed`（条件触发，无硬件计划前冻结）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
