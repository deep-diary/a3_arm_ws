# LL-089 — 电机寄存器的类型/量纲以协议手册为准：0x7028 是 uint32 计数，不是 float 秒

> **日期：** 2026-09-22  
> **产品线：** Edge  
> **环境：** RK3588 lubancat + ROS 2 Humble + RobStride MIT 电机（0x70xx 参数区）

## 现象

F86 要给电机侧 CAN 通信超时（寄存器 `0x7028`）布防。动手前先查本仓 codec，两处 `protocol_codec.hpp` 的注释都写着「CAN 超时（float, s）：0=电机端不超时」。若按此实现，会把 0.2 用 `BuildSetParamFrame` 编成 IEEE754 四字节（`0x3e4ccccd`）写进电机——该寄存器按 uint32 解析时是 1045220557 计数 ≈ 14.5 小时，等于永久关闭，且超时触发语义完全不会发生。更危险的是：Type-18 写入没有应答，仿真器（F86 改造前）直接忽略 0x70xx 帧，这类错误在仿真里**不会报任何错**。

## 根因

寄存器参数没有跨文件的 schema（pluginlib/URDF 不管电机固件），codec 注释是早期按推测写的，没有协议手册背书，之后一路被当作事实引用。关键事实（经协议手册与参考实现交叉确认）：

1. **0x7028 类型是 uint32，不是 float**；量纲约 50 µs/count，**20000 ≈ 1 s**；0=关闭；写入**易失**（掉电恢复出厂值 0）。
2. 超时触发后电机**自行进入 RESET 模式**（去使能/阻尼），不是上报故障码等主机处理。
3. 官方 EDULITE_A3 参考代码在**每次使能时显式写 0**，完全依赖主机侧看门狗——F86 有意偏离：主机崩溃/掉电/线缆脱落时主机看门狗（F81）随主机一起失效，电机侧超时是唯一仍生效的防线（纵深防御）。

## 正确做法 / 规避

1. 任何电机寄存器写入前，**类型、量纲、默认值、易失/Flash、触发后行为**五件事以厂商协议手册为准；codec 注释只作指针，不作事实来源。本仓两份 codec 注释已更正。
2. 无原生应答的写参数命令，验收必须加 Type-17（读回）对照——F86 criterion 2 即此。
3. 数值换算写常量并在代码里注明依据：`20000 counts ≈ 1 s`；默认 0.2 s = 4000 counts，约为正常 5 ms 帧间隔的 40 倍，避开环路抖动。
4. 仿真器必须实现「总线静默 → 电机自行 RESET」语义，且用 SIGKILL 模拟主机死亡（SIGTERM 会走 `on_deactivate` 的撤防流程，测不到这条路径）。
5. 状态确定性：`on_deactivate` 先撤防（写 0）再 reset；`on_activate` 每次重新布防——因为写入易失，不能假设上次状态还在。

## 相关路径

- `src/a3_hardware_interface/include/a3_hardware_interface/protocol_codec.hpp`（注释更正；`BuildSetParamRawFrame` / `BuildGetParamFrame`）
- `src/a3_can_bridge/include/a3_can_bridge/protocol_codec.hpp`（注释更正）
- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（`on_activate` 布防 / `on_deactivate` 撤防）
- `scripts/a3_test/vcan_motor_sim.py`（0x7028 存储、Type-17 应答、总线静默跳闸）
- `scripts/a3_test/f86_can_timeout_acceptance.py`（vcan 29/29）
- `docs/edge/REQUIREMENTS.md`（F86）
