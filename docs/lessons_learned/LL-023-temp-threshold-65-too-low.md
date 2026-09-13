# LL-023 F44 温度阈值 65°C 过低：真机 ready 位 L3 保位几分钟即触发 overtemp park

- **日期**：2026-09-13
- **产品线**：Edge（can1 真机 7 自由度臂）
- **严重度**：中（保护误触发，路径安全——自动回 home 失能，无伤害）

## 现象

全臂使能悬空保位（ready 工作位）几分钟后，L3（RS00）温度 63°C→65°C（约 25 s 升 2°C），触发 F44 overtemp 保护：自动 safe-park 回 home（3 s）→ 失能 7/7 → COOLING。当时正要下发 `goto_named_pose home`，被 `state=COOLING` 拒绝——但保护路径本身已完整执行「当前位置→home→失能」序列（落点误差 <0.003 rad，7/7 mode=0）。失能卸力后 L3 65→55°C 快速回落。

## 根因

1. ready 悬空位 L3 保位力矩约 1.26 Nm，纯 P 保位（kp=80）微振荡下 RS00 持续发热，几分钟即 63°C 基线、25 s 内破 65。
2. 初版 F44 阈值 warn=60/protect=65 按 6J 通用臂经验设定，对 7J 真机悬空保位工况过低——**官方电机自带 130°C 保护兜底**，65°C 余量过大且与正常带载工况冲突。

## 修复

阈值调高 **warn=90 / protect=95**（迟滞 5，恢复门槛 protect−hysteresis=90）：`arm_controller.yaml` + `arm_controller_6j.yaml` + `arm_controller.py` 默认值三处同步，`a3_arm_controller` 已重编（yaml 经 build 目录副本生效，改 yaml 必须重编）并重启验证（`ros2 param get` 回读 95.0/90.0）。

**纠误（同轮）**：重力补偿不会消除发热——抗重力 1.26 Nm 仍需电机持续输出，τ_ff 只是把力矩从误差环记账转到前馈记账，稳态铜耗不变；它消除的是 P 稳态下垂（τ_g/kp≈1°）并实现自由拖动。

## 注意

- F44 保护路径（overtemp → safe-park home → disable → COOLING）真机首次触发即按设计工作，可放心依赖。
- ready 位持续保位就会发热（L3 1.26 Nm 持续出力，P 保位与重力补偿稳态等效），这是位姿固有带载；降温靠失能/换位姿，不要靠换控制方式。
- F40–F46 文档任务（REQUIREMENTS F44 节等）落笔时以 90/95 为准。
- 顺带教训：初版 ready 记录值（L2 1.0241）与拿掉泡沫后的实测保位值（L2 1.0839）差 0.06 rad——记录位姿要以**新鲜反馈**为准，且用户挪臂后再核对一次。

相关：[[LL-019]]、[[LL-020]]、[[LL-022]]
