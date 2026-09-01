# LL-004 — LubanCat-4-V1 CAN 物理口对应：软件 can0 无收发器、can1 才是能通电机的那路

> **日期：** 2026-09-01  
> **产品线：** Edge  
> **环境：** LubanCat-4-V1（RK3588S 一体板）+ ROS 2 Humble + SocketCAN（1 Mbps）

## 现象

两路 CAN（`can0`、`can1`）都显示 UP，但只有一路能收到电机应答：

- 对 `can1` 发 `get_device_id` 探测帧（`cansend can1 0000FD07#0000000000000000`）→ 有 `000007FE` 应答。
- 对 `can0` 发同样探测帧 → 无任何帧。
- `ip -details link show can0` 显示 `can state ERROR-PASSIVE`（线上无第二节点 ACK），`can1` 是 `ERROR-ACTIVE`。

曾把 `can0`/`can1` 两路物理短接，导致两路都能看到彼此帧，被误判成「两路都通」。拔掉短接线后只剩一路有反馈。

## 根因

- 软件 `can0` = RK3588 **CAN0** 控制器（`parentdev fea50000.can`），overlay `can0-m0`，引脚 GPIO0_C0(RX)/GPIO0_B7(TX)。这是一体板 40pin 上的**芯片原生 TX/RX 逻辑电平，无板载收发器**。
- 软件 `can1` = RK3588 **CAN2** 控制器（`parentdev fea70000.can`），overlay `can2-m0`，引脚 GPIO3_C4(RX)/GPIO3_C5(TX)，**底板已接板载 CAN 收发器**。
- 内核按使能顺序注册：只开 `can0-m0`、`can2-m0` 两个 overlay 时，CAN0→`can0`、CAN2→`can1`。所以软件名 `can1` 实际是芯片 CAN2，命名易混淆。
- EL05 电机驱动器走差分 CANH/CANL，必须经过收发器，故只能接在有收发器的 `can1` 上直接通信；接 `can0`（原生逻辑电平）不会收到应答。

## 正确做法 / 规避

1. 确认物理口归属：

```bash
ip -details link show can0 can1   # 看 parentdev fea50000.can(CAN0) vs fea70000.can(CAN2)、can state
```

2. 两路分别探测，哪路有 `000007FE` 应答，电机就接在哪路：

```bash
cansend can1 0000FD07#0000000000000000 && candump -tz can1
```

3. 控臂默认总线必须与物理口一致。走 `can1` 时需同时改：
   - `arm_mapper.hpp` 的 `kTemporaryIndexMap` 总线字段 → `CanBus::CAN1`
   - `config/control_gains.yaml` 反转 `tx_enable_can0: false` / `tx_enable_can1: true`（否则 `can1` 帧被丢弃，电机不动）

4. 若坚持用 `can0`，需在 40pin 的 CAN0 TX/RX 外接 TTL↔CAN 收发器模块（如 TJA1050），否则无法与差分电机通信。

## 相关路径

- `docs/edge/PLATFORM_CAN.md`
- `src/a3_can_bridge/include/a3_can_bridge/arm_mapper.hpp`
- `src/a3_can_bridge/config/control_gains.yaml`
