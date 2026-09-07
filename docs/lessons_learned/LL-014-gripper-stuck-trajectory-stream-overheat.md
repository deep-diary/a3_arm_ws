# LL-014 — 轨迹插值卡死持续流送 + 单帧卸力被覆盖：电机顶泡棉 1.55 Nm 过热 125°C

> **日期：** 2026-09-06
> **产品线：** Edge
> **环境：** RK3588 真机（lubancat）+ ROS 2 Humble，单电机台架 can1 / ID7，泡棉

## 现象

力控阶梯验收 0.5 Nm 步超硬限 FAULT（actual=1.5455 > 1.5）后，兜底 release 轨迹未生效：电机卡在 1.225 rad 持续以 1.55 Nm 顶压泡棉，温度 104→125°C 持续上升。`/a3/motor/stop`（MotorStop 服务）返回 `stopped (1 limp frame(s))`，力矩纹丝不动。

## 根因（三层叠加）

1. **FAULT 冻结力环后 trajectory 流送器卡死**：力控期间 gripper 每 tick 发一条轨迹，motor_protocol_node 200 Hz 插值流送；FAULT 冻结后轨迹停发，但插值器**永远重发最后一个目标点**（cmd_angle=1.24885 恒值，can1 TX ~166 帧/s 实测）。此时新到的 release 轨迹被拒（F32 回归的「无效轨迹帧」症状），电机继续执行流送的旧目标。
2. **MIT 固件持有最后命令**：MIT 电机收到一帧后无限期执行，直到下一帧到来。杀掉流送器（motor_protocol_node）并不解除——最后一帧（kp=80、目标 1.24885）仍在电机里执行。
3. **单帧卸力被覆盖**：MotorStop 只发一帧 kp=kd=t=0，插值流 200 Hz 下 ~5 ms 后即被旧目标帧覆盖，等于没发。

## 正确做法 / 规避

- **卸力必须让「零力矩帧成为电机收到的最后一帧」**：先停流送器（杀 motor_protocol_node），再用 `cansend can1 01800007#<pos>800000000000` 直发一帧 kp=kd=t=0（can_id = (0x01<<24)|(torque_u16<<8)|7，torque=0→0x8000 中段；data = 位置/速度/kp/kd 各 2 字节大端，kp=kd=0 时位置值无关紧要）。验证：发查询帧 `cansend can1 1800FD07#0102030405060100`（主动上报 cmd 0x18），应答 data[4..5]=0x7FFF 即零力矩。
- **MotorStop 语义要升级**（F32 域）：卸力不能只发一帧——要么「连续发 N 帧/持续 100ms」，要么同时终止插值流。单帧卸力在有插值流的任何场景下都是空操作。
- **力环 FAULT 后必须主动卸力**：`_fault()` 只冻结力环，电机位置环仍按最后 q_cmd 顶物体。安全设计应让 FAULT 时把 gains 降为零或发回退轨迹（本轮暂未改代码，仅文档记录）。
- **过热**：MIT 电机 1.5 Nm 持续出力几分钟即到 125°C，固件锁存 temp_error 位（应答 ID bit18）；降到 72°C 仍锁存，需持续降温观察。
- pkill -f 会匹配自己 shell 的命令行（LL-006 已记，本次再犯）：用字符类技巧 `pkill -f "motor_protocol_nod[e]"`。

## 相关路径

- `src/a3_can_bridge/include/a3_can_bridge/protocol_codec.hpp`（`BuildMitControlCanId`/`BuildCommandFrame`：29 位 ID 布局与 16 位大端编码）
- 关联：LL-006（pkill 自匹配）、LL-013（同一测试场次）；F32（插值器缺陷，转 motor-debug 会话）
