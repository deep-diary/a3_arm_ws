# LL-087 — 切到 ros2_control 栈后旁路话题静默断链：FSM 温度/故障门禁无人喂数据，F44 形同虚设

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，ros2_control 2.24
> **关联：** [[LL-074-ros2-control-system-interface-vcan-xacro-humble-pitfalls.md]]、[[LL-086-humble-hw-inactive-boot-switch-controller-no-auto-activate.md]]、F44/F43/F84

## 现象

F84 排查时发现：hardware:=can 栈（`a3_bringup.launch.py`）里 FSM 的超温保护（F44）与电机故障码紧急失能（F43）从未触发过——不是阈值问题，而是 `/a3/motor/states`（`a3_can_bridge/MotorStates`）话题上根本没有发布者，FSM 订阅回调一次都不执行。

## 根因

栈迁移留下的「话题债」：

1. F44/F43 当年基于 Execution 层 `a3_can_bridge` 实现，`motor_protocol_node` 负责发布 `/a3/motor/states`。
2. F72 起真机执行改为 ros2_control SystemInterface 插件（SocketCAN + MIT 解码全部在 controller_manager 进程内），hardware:=can 入口**不再启动任何 bridge 节点**。
3. FSM 代码一行没改，订阅还挂在原话题上；没有发布者时 rclpy 不报错、FSM 也没有「订阅无数据」自检——保护逻辑静默失效。温度/故障位在插件里已经解出来了，却只存在于插件内部。

同类隐患普遍存在：任何「订阅某话题做安全联锁」的节点，在话题生产方被架构替换后都会变成静默摆设。

## 正确做法 / 规避

- **复用既有契约，不改 FSM**：插件把解码结果（temperature/mode/fault）按 F44 的既有消息类型在原话题 `/a3/motor/states` 上 50 Hz 重发布；F44/F43 门禁、fresh 判断、ArmStatus.temp_warn 全部原样生效。
- **QoS 必须配对**：FSM 订阅是 `BEST_EFFORT`（传感器数据约定），插件发布端必须也是 `best_effort`；发布端默认 RELIABLE 与订阅端 BEST_EFFORT 在 DDS 中不兼容，会表现为「话题在、数据不来」，比没有发布者更难查。
- **安全联锁要可观测**：插件同时发 `a3_hardware:motor_health` 诊断（fault≠0 → ERROR，≥保护温度 → ERROR，≥告警温度 → WARN），F82 aggregator 自动归集；保护链路是否活着看诊断即可，不用猜。
- **验收必须注入真实故障**：vcan sim 增加 health 注入（JSON 改温度/故障位），92 °C 必须看到 WARN+temp_warn、96 °C 必须走到 SAFE_PARK→COOLING、fault≠0 必须到 FAULT——只验「常温全绿」永远发现不了断链。

## 追加坑：故障后反馈停止，诊断必须锁存故障字

首轮验收 FSM 正确进入 FAULT，`motor_health` 却回到 OK——因为 MIT 固件只在收到指令帧后才发反馈，紧急失能后指令流量停止，故障字随年龄变 stale，而诊断最初对 fault 也做 fresh 门控，等于「故障导致停机，停机把故障从诊断上抹掉」。固件故障字是**锁存状态**，正确做法是：fresh 帧一旦报非零字就锁存 ERROR，直到下次 `on_activate` 的清故障编排才解除；温度是实时量，仍必须 fresh 门控（[[LL-011]]）。另：本机 rclpy 中 `diagnostic_msgs/DiagnosticStatus` 的 OK/WARN/ERROR/STALE 常量本身就是 `bytes`（`b'\x00'`–`b'\x03'`），消息字段也可能以 int 或 bytes 两种形态到达（LL-084 亦记录）——验收脚本统一把字段归一化成 int 再与 int 字面量比较；常量定义在 `DiagnosticStatus` 而非 `DiagnosticArray`。

## 相关路径

- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（`PublishMotorStates` + motor_health 诊断）
- `src/a3_hardware_interface/include/a3_hardware_interface/protocol_codec.hpp`（fault_code 解码，与 a3_can_bridge 镜像同步）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（F44/F43 消费侧，未改动）
- `scripts/a3_test/vcan_motor_sim.py`（`--health-file` 注入）、`scripts/a3_test/f84_thermal_fault_acceptance.py`
