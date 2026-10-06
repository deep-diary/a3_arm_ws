# LL-147 — torque_max 双重用途：钳位值改小 → 零重力补偿力矩被编码放大 1.75× 回拉

> **日期：** 2026-10-05
> **产品线：** Edge
> **环境：** RK3588 + ROS 2 Humble + ros2_control（A3MITHardwareInterface）

## 现象

零重力（示教 free-drive）模式下，**L2 关节被「拉回 home」**：伸直时会被往上弹、往下压会反抗，像有个位置弹簧；松手后快速弹回半抬位。L3 也有「有力」感（实为正常的重力托举，补偿 2.669 Nm），L4/L5/L6 free-drive 正常。位置模式轨迹运行**正常**，掩盖了问题。改 tau_scale、换 inertia_params（nominal/03-18/F49）、回退版本都无效。

## 根因

`torque_max` 一个参数被**两个用途复用**：

1. 力矩钳位：`clamp(cmd_eff, -torque_max, torque_max)`
2. MIT 帧 `torque_ff` 的 **CAN 编码量程**：`FloatToUint16(torque_ff, -torque_max, torque_max, 16)`

L2 的 `torque_max` 被从 RS00 默认 **14** 改成 **8** 后，编码量程从 ±14 缩到 ±8，但 **RS00 电机固件仍按 ±14 解码** → 下发力矩被放大 `14/8 = 1.75×`。

零重力下 kp=0，重力补偿力矩（τ_ff）被放大 1.75× → 补偿过度 → L2 被往上推，体感「回拉 home」。位置模式有 kp 兜底，F108 前馈 t_ff 同样被放大 1.75× 却被位置环拉回、看不出；反馈力矩解码量程同用 `torque_max`，还会导致反馈力矩按 8 解码少报（14/8）。

> 同源坑见 [LL-024](LL-024-codec-per-motor-torque-scale.md)：力矩/速度编解码量程必须按电机型号（RS00 ±14/±33，EL05 ±6/±50），不能用统一值。

## 正确做法 / 规避

1. **`torque_max` 只做力矩钳位**，保持电机默认值（RS00=14 / EL05=6，即 LL-024 固有量程）；不要把钳位值当编码量程用。
2. **CAN 编解码量程独立成一个不可被 xacro 覆盖的字段**（本仓 `torque_encode_max`），编码/解码都用它。
3. 想调「静态保位力矩门禁」用 arm_controller 的 `joint_rated_torque`（F107），**不要动 `torque_max`**。
4. 排查「改某参数后行为怪」先问这个参数有没有被多处复用；改小一个「上限」却观察到某个量被放大，往往是编码/钳位量程混用。

## 相关路径

- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（新增 `torque_encode_max`，effort/position 编码与反馈解码改用它）
- `src/a3_description/urdf/el_a3_ros2_control.xacro`（关节 `torque_max` 仅作钳位）
- `src/a3_arm_controller/config/arm_controller.yaml`（`joint_rated_torque` F107 门禁）
- `src/a3_can_bridge/include/a3_can_bridge/protocol_codec.hpp`（`kTMax/kTMin`、`BuildMitControlFrame` 量程）
