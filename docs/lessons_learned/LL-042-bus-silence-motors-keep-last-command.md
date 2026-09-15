# LL-042 执行层进程死亡 = 总线静默：电机保持最后命令不失能，看门狗的动作也发不出去

> **日期：** 2026-09-15  
> **产品线：** Edge  
> **环境：** RK3588 (lubancat) + ROS 2 Humble，can1 真机（事故后 L1–L5，5J 档）

## 现象（真机实测，2026-09-15 07:49–07:51）

臂自然趴下、无关节承重（全部 mode 0、29–30 °C），先 `/a3/motor/enable` 广播使能
（5/5 `mode=2`，F51 重锚 `最大丢弃目标距离 0.000000 rad`，位置零变化），然后
`kill -9` 掉 **`motor_protocol_node`**——栈里唯一发 MIT 命令帧的节点（`can_transport_node`
只做转发，不产生命令）：

| 时刻 | 观测 | 结果 |
|---|---|---|
| 07:49:35 | 杀进程，此后总线无任何命令帧 | — |
| 07:49:36–42 | `candump` 被动监听 6 s | **0 帧**：电机不主动广播（所以「总线静默」只能靠主动查询确认） |
| 07:51:00（静默 85 s） | `probe` 读 mode / angle | **5/5 仍 `mode=2`**，位置与使能时刻**逐关节完全一致**（L1 −0.0524 / L2 −0.0002 / L3 +0.0152 / L4 +1.1698 / L5 +0.2307），29–31 °C |

**结论：RS00/EL05 在通信丢失后不会自动失能，会一直保持最后一条 MIT 命令（kp/kd/τ）。**
（本次为静止姿态，所以看不出力矩；承重姿态下就是"一直出力保位"。）

## 根因 / 两个缺口

1. **无通信超时保护**：电机固件以"最后一条命令"为准，没有心跳/超时→失能。执行层一死，
   臂会永远保持那个 kp/kd/τ——不会掉，但同时**永远失去 F42 力矩钳位、F44 温度保护、
   F50 看门狗**。承重姿态（如 ready 位 L3 ≈3 Nm）下就是持续发热，直到电机自带的
   130 °C 兜底（F44 的 warn 90 / protect 95 都不在了）。
2. **看门狗的动作路径与它要保护的对象同源**：`arm_monitor_node` 的 stop/reset/disable
   是 `/a3/motor/{stop,reset,disable}` 的**服务客户端**，而这三个服务都由
   `motor_protocol_node` 提供（`motor_protocol_node.cpp:475/511`）。**死的正好是它**——
   动作发到已消失的服务上，静默失败。另外 `STALE_JS` 只在
   `STALE_JS_STATES={READY,TRAJ,SAFE_PARK,SERVO,TEACH,AI}` 里评估（`arm_monitor_node.py:50/360`），
   本次控制器停在 `IDLE`（走执行层 `/a3/motor/enable` 使能，没进 READY）→ **连检测都没有**。

## 实测方法（可复用，注意污染）

- 判断"总线是否真静默"用**被动** `candump`：`timeout 6 candump -T 6000 can1`，
  0 帧即无节点在发（电机自己不广播）。
- 读 mode/angle 只能用 `scripts/mit_noenable_stream.py probe`，但它发的是**零增益 MIT 帧**
  （kp=kd=τ=0）→ 一 probe 下去电机就从"保位"变"零增益"，**测量本身破坏被测状态**：
  只能测一次，且测完臂已不再保位（本例就此收尾时直接补了 `reset` 失能）。
  要测"保持中的力矩"得用非零增益帧或读 `/a3/motor/states`——**但那依赖执行层活着**，
  正是本场景没有的。

## 正确做法 / 规避

- **无人监护运行的前提是执行层进程本身活着**：真机栈用 systemd（`Restart=always`）或
  `setsid` 起，不要挂在会随会话/内存压力被回收的地方（姊妹条目：
  QUICKSTART「缺电机降级档」里 2026-09-15 00:21 内存压力回收整栈的事故）。
- **后续要评估**（尚未实现，别当成已有保护）：让看门狗的动作不依赖被测对象——
  例如独立 CAN 通道发送、或整臂断电的硬件路径；以及执行层死亡本身要不要算一类故障
  （活着的 `can_transport_node`/编排层可观测量：`/a3/motor/states` 停更 = 执行层死了）。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`（`/a3/motor/{stop,reset}` 服务在此）
- `src/a3_arm_controller/a3_arm_controller/arm_monitor_node.py`（`_stop_cli`/`_reset_cli`、`STALE_JS_STATES`）
- `scripts/mit_noenable_stream.py`（`probe` 的零增益污染、`reset`）
- [LL-020](LL-020-bridge-seed-zero-gain-refresh.md)（桥无目标历史时总线静默 → js 冻结；同一族"静默"问题）
