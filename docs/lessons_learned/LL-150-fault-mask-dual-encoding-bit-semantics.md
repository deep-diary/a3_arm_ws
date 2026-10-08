# LL-150 — fault_mask 双编码源 + EL05 故障位语义错配致误判

> **日期：** 2026-10-08
> **产品线：** Edge
> **环境：** RK3588 真机 + ROS 2 Humble（ros2_control 栈）

## 现象

巡游（`/a3/arm/random_pose_tour`）连续两次运行中途失能，日志报：

```
[ERROR] motor fault: joint=L5_joint mask=0x1 -> emergency reset
state -> FAULT (motor fault: L5_joint mask=0x1, disabled)
```

分析 `mask=0x1` 时，先误判为「未标定」（bit21），后经用户对照 EL05 手册纠错，实为「欠压」（bit16）。

## 根因

1. **`fault_mask` 有两条编码路径，bit 布局完全相反**：
   - 旧执行层 `motor_protocol_node.cpp`：把 `fault_code` **重新编码**成聚合 mask（bit0=综合、bit1~5=具体故障，顺序是 hall/magnet/temp/current/voltage）。
   - ros2_control 栈 `a3_mit_hardware_interface.cpp`：`s.fault_mask = j.hw_fault = fb->fault_code`，直接透传**原始 fault_code**（bit0=can_id bit16=欠压 … bit5=bit21=未标定）。

   `arm_controller` 订阅的 `/a3/motor/states` 实际由 hardware interface 发布（原始 fault_code），但分析时误套用了 `motor_protocol_node` 的重新编码逻辑 → `mask=0x1` 被误判为 bit21（未标定），实际是 bit0（欠压）。

2. **故障位语义命名错配**（codec 里）：`current_error` 对应 bit17 实为「驱动故障」，`hall_error` 对应 bit20 实为「堵转过载」；bit21「未标定」完全未解码。

3. **fault 一刀切 reset**：任何 `fault_mask != 0` 都 `emergency reset + FAULT`，欠压这类瞬时/可恢复故障也被硬复位掉臂。

## 正确做法 / 规避

1. **分析 fault 位前先确认数据源与编码**——同一话题 `/a3/motor/states` 有多个发布者时，先定位当前栈到底谁在发、用什么编码，别套错逻辑。
2. **`fault_mask` 统一存原始 `fault_code`**，bit 布局直接对照 EL05 手册：

   | bit | 值 | 含义 |
   |---|---|---|
   | 0 | 0x01 | 欠压 |
   | 1 | 0x02 | 驱动故障 |
   | 2 | 0x04 | 过温 |
   | 3 | 0x08 | 磁编码故障 |
   | 4 | 0x10 | 堵转过载故障 |
   | 5 | 0x20 | 未标定 |

3. **字段按手册命名**：`undervoltage_error / driver_error / temp_error / magnet_error / stall_error / uncalibrated`，废弃 `hall_error / current_error / voltage_error` 这类语义误导名。
4. **故障分级处置**（非一刀切 reset）：
   - 硬故障（驱动/磁编码/堵转过载）→ 立即 `emergency reset + FAULT`。
   - 软故障（欠压/过温/未标定）→ 计时（`soft_fault_grace_s`，默认 1.5s）→ `safe park → disable`，可自恢复不 reset。
5. 欠压是锂电瞬间大电流压降的典型表现；固件 fault 是否自清取决于固件（`hw_fault=fault_code` 是实时量，`fault_latched` 才是软件锁存），「计时」在自清/锁存两种情况下都稳健（分别对应过滤瞬时/延迟处置）。

## 相关路径

- `src/a3_can_bridge/include/a3_can_bridge/protocol_codec.hpp`
- `src/a3_can_bridge/include/a3_can_bridge/motor_model.hpp`
- `src/a3_can_bridge/src/motor_protocol_node.cpp`
- `src/a3_can_bridge/msg/MotorState.msg`
- `src/a3_hardware_interface/include/a3_hardware_interface/protocol_codec.hpp`
- `src/a3_hardware_interface/include/a3_hardware_interface/motor_model.hpp`
- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`
