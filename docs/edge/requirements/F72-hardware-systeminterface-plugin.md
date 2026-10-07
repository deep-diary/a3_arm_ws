# F72 — 真机 SystemInterface 插件（MIT/SocketCAN 直驱，ros2_control 标准栈真机化）


- **说明：** F70 的真机迁移：新建 ament_cmake 包 `a3_hardware_interface`，提供 `hardware_interface::SystemInterface` 插件 `a3_hardware_interface/A3MITHardwareInterface`——controller_manager 直接打开 SocketCAN（默认 `can1`）、收线程解 MIT 反馈帧（类型 2 / 0x18），`write()` 直接打 MIT 控制帧；F70 的 JTC/JSB/move_group 配置**原样复用**，真机路径上 `motor_protocol_node`（3361 行）+ 200 Hz 插值器 + `/can_tx_frames` 话题中转整体被旁路。MIT 编解码**复用本仓真机验证过的 ProtocolCodec**（motor_model.hpp + protocol_codec.hpp 两份头文件 vendored 进新包，仅换命名空间；不重写协议常量），SocketCAN 传输按官方 el_a3_hardware 结构写单总线实现（SOCK_RAW、SO_RCVTIMEO、send mutex + ENOBUFS 重试、CAN_RAW_FILTER 只收反馈帧）。关节→电机映射取 URDF 既有参数：每关节 `motor_id`、`direction`（L1..L7 = −1,+1,−1,+1,−1,+1,+1，与 control_gains.yaml joint_signs 一致）、`position_offset=0`；硬件参数 `can_interface`、`kp`（默认 80）/`kd`（默认 2）、`command_rate_hz`（默认 200）、力矩/速度量程按 motor_id（1–3 RS00 ±14 Nm/±33 rad/s，4–7 EL05 ±6 Nm/±50 rad/s，LL-024）。生命周期：`on_configure` 打开 CAN 并起收线程；`on_activate` 先发 reset（清故障）再发 enable、随后以反馈位重锚指令（F51 语义）；`on_deactivate` 发零增益续流帧后 reset 失能。xacro 新增 `use_real_hardware` 分支（mock 分支不动）。**增量并存：不删旧栈、不改 F70 仿真栈。**
- **验收标准：**
  1. `vcan0` + 电机反馈仿真器（收到 MIT 指令帧→一阶跟随→回类型 2 反馈）下，真机 launch 启动无致命错误：插件 loaded/active，JSB 发布 7 关节 /joint_states（position/velocity 非全零）
  2. 直连 `/arm_controller/follow_joint_trajectory`（31 点五次 S 曲线 home→ready→home）与夹爪 JTC 开合：action 成功、到位（误差 ≤0.02 rad）
  3. move_group plan+execute 端到端经插件完成（无 motor_protocol_node 进程）
  4. 数值验收脚本：起止速度≈0、v/a 不超限（与 F70 同判据），结论 ALL PASS；CAN 线侧抓包确认指令帧位置=direction×joint+offset
- **关联：** F70（仿真标准栈，本需求真机化）、F51（使能重锚语义）、LL-024（量程按型号）；[shared/SAFETY.md](../shared/SAFETY.md)；官方 el_a3_hardware（结构蓝本，协议常量以本仓为准）
- **状态：** `仿真验收通过`（2026-09-22，vcan0 + `f72_ros2_control_vcan_acceptance.py` ALL PASS：JTC/夹爪/move_group 到位 ≤0.01、CAN 指令与反馈映射偏差 0、kp=80/kd=2；真机上电验收另约，见 LL-074）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
