# F103 — CAN 总线物理层健康入标准诊断（bus-off/错误帧可观测；链路 down/丢失 → ERROR 进现场状态）


- **说明：** 现有 CAN 安全机制全部在「应用/设备」层：F80 socket 硬化、F81 反馈 staleness、F86 电机侧超时。但 **CAN 物理层本身**（线缆脱落、终端电阻缺失、干扰导致错误计数增长 / ERROR-WARN / ERROR-PASSIVE / BUS-OFF）在系统中完全不可见：`can-up.service` 只配了 `restart-ms 100`（内核自动恢复 bus-off），外部无法知道发生过什么、恢复了几次；若接口被误置 down 或网卡硬件消失，也只能等 F81 在 10 s 后间接报「反馈陈旧」。工业现场要求总线作为独立设备状态上报：
  1. 新增 `can_bus_monitor` 节点，每 2 s 解析 `ip -s -d link show <iface>`，经 diagnostic_updater 发布任务 `a3_can_bus: CAN link <iface>`：管理态 UP 且 `state ERROR-ACTIVE`（vcan 为 operstate UNKNOWN）= OK；`ERROR-WARN` / `ERROR-PASSIVE` = WARN；接口不存在 / 未 UP / `BUS-OFF` = ERROR。消息携带 CAN state、restart-ms、以及计数器行（RX/TX packets、`re-started`、`bus-errors`、`arbit-lost`、`error-warn`、`error-pass`、`bus-off`）。
  2. 接入 aggregator Hardware 分组（startswith 增加 `a3_can_bus:`），总线 ERROR 进入 `/diagnostics_toplevel_state`。
  3. 仅在 `hardware:=can` 且 `use_diagnostics:=true` 时随产品栈启动（mock 栈无 CAN 接口，不该产生误报）；接口名取 `can_interface` 参数。
  - 与既有机制的关系：F81 管「反馈帧时间维健康」，F103 管「总线物理维健康」；restart-ms 100 保证 bus-off 后内核自动恢复，节点报出 state 翻转与累计计数，恢复后回 OK（历史 bus-off 次数保留在消息中可追溯）。
- **改动：**
  1. 新增 `src/a3_bringup/a3_bringup/can_bus_monitor_node.py`（参数 `interface`，默认 can1），setup.py entry
  2. `src/a3_bringup/launch/a3_bringup.launch.py`：hardware=can + diagnostics 条件启动
  3. `src/a3_bringup/config/diagnostics.yaml`：Hardware startswith 增加 `a3_can_bus:`
  4. 新增 `scripts/a3_test/f103_can_bus_health_acceptance.py`（vcan103 接口，隔离域 103）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f103_can_bus_health_acceptance.py`）：**
  1. vcan103 存在且 UP：15 s 内诊断 OK，aggregator 输出 `/A3/Hardware/a3_can_bus: CAN link vcan103`
  2. `ip link set vcan103 down`：10 s 内诊断与聚合项均变 ERROR
  3. `ip link set vcan103 up`：10 s 内诊断与聚合项恢复 OK
  4. 指向不存在的接口（`interface:=can99`）：10 s 内诊断 ERROR（不崩、不静默）
- **关联：** F80（socket 硬化）、F81（反馈看门狗）、F82（aggregator）、F86（电机 CAN 超时）、can-up.service
- **状态：** `completed`（2026-09-24，vcan103 验收 4/4：UP→诊断+聚合 OK 1.3 s；down→双 ERROR 1.0 s；up→双恢复 OK 1.0 s；can99 不存在→ERROR 1.6 s 且节点存活；见 LL-115）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
