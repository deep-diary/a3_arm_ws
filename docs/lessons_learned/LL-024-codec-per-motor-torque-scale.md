# LL-024 力矩/速度编解码统一量程：RS00 反馈少报 2.333 倍、τ_ff 反向放大 2.333 倍

- **日期**：2026-09-13
- **产品线**：Edge（can1 真机 7 自由度臂）
- **严重度**：中（测量与指令双向失真，无直接伤害；发现前 F42 钳位 3 Nm 对 RS00 关节形同虚设）

## 现象

ready 位保位稳态对照：kp×误差 = 3.43 Nm（L3）、1.74 Nm（L2），反馈力矩却只报 1.45 / 0.76；EL05 关节（L4 kp×err≈93×0.008）正常。官方协议文档「通信数据映射范围对照表」证实：**各型号量程不同**——RS00 力矩 ±14 Nm、速度 ±33 rad/s；EL05 ±6 Nm、±50 rad/s；Kp/Kd、角度量程相同。

## 根因

`protocol_codec.hpp` 对所有电机统一用 ±6 Nm / ±50 rad/s 编解码：
- **解码**（反馈）：RS00（L1–L3）真实力矩 14/6=2.333 倍被低报（L3 实际 3.4 Nm 显示 1.45）；速度同理（±33 vs ±50）。
- **编码**（指令 τ_ff）：同一数字反向放大 2.333 倍——写 3 Nm 前馈实际发 7 Nm。
- 连带：F42 力矩钳位阈值 3.0 对 RS00 过低（L3 真实保位 3.4 > 3.0 会 latch 下爬），且阈值本身在低报尺度下不可比。

## 修复

按电机 ID 传入 ±量程（config 按 motor_id-1 索引）：
- `protocol_codec.hpp`：`BuildMitControlFrame`/`DecodeFeedback` 增加 `torque_max`/`speed_max` 可选参数（默认 ±6/±50 兼容），内部用 ±max 编解码。
- `motor_protocol_node.cpp`：新增参数 `motor_torque_range_nm`（默认 7×6.0）、`motor_speed_range_rad_s`（默认 7×50.0）与 `TorqueRangeNmFor`/`SpeedRangeRadSFor` 辅助函数；6 处调用点（SendMitFrame/OnTxRefreshTimer/mit_hold 两帧/F32 直驱 clamp+帧/motor_stop 帧/DecodeFeedback/ComputeMitTorqueFf clamp）全部按 motor_id 传量程。
- `control_gains.yaml`：`motor_torque_range_nm: [14,14,14,6,6,6,6]`、`motor_speed_range_rad_s: [33,33,33,50,50,50,50]`；`torque_protection_limit_nm` 同步调 [5,5,5,3,3,3,3]（RS00 连续额定 5，EL05 3）。generic yaml 保持默认。
- 真机验证：L2 −0.76→−1.83、L3 1.45→3.37（×14/6=2.333 精确吻合）、EL05 不变；7/7 保位正常。

## 注意

- 接入新电机型号必查协议量程表，按 motor_id 维护量程数组；默认值只对 EL05 正确。
- 稳态对照判据复用：`kp×err ≈ 反馈力矩`（同域）可离线验证任何编码尺度。
- codec 修复后 L3 真实保位 3.4 Nm 被暴露 → 几分钟即升温至 95°C 触发 F44（LL-023 阈值已放宽，ready 位持续保位发热是位姿固有，与重力补偿无关）。

相关：[[LL-023]]、[[LL-025]]
