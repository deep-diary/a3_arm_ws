# F25 — 夹爪自适应力控（PI 力外环 + 位置内环）


- **说明：** 新增独立 Python 包 `a3_gripper_controller`（节点 `gripper_controller_node`，50 Hz 力外环，不改动 C++ CAN 核心的实时路径）。四种模式：`POSITION`（现有开合语义，订阅 `/a3/gripper_cmd` 的 0–1 归一化并映射 0–1.5708 rad，补齐「有发布无执行」缺口）、`FORCE`（力控抓取）、`RELEASE`（张开到安全位）、`STOP`（停止力环并保持）。FORCE 模式：以设定握力 `tau_target` 为目标，读 `/joint_states` 的 `eff_L7`（经 `gripper_torque_sign` 校正方向）作为反馈，PI 调节器输出**位置增量**——力矩不足（夹得不够紧）则向闭合方向累加位置目标，力矩超了则回退；积分限幅、位置增量变化率限幅、位置目标钳位在 L7 软限位内。位置目标经现有 L7 单关节 `JointTrajectory` 通道下发（与 PS4 `set_joint_L7` 同路径），电机内置位置环顶住物体，接触力即被维持在设定值，从而自适应抓取软硬不同物体。接触/抓稳判定：位置停滞且力矩进入目标带（±10%）并维持 `settle_s` → 状态 `GRASPED`。安全：抓取超时、反馈看门狗超时（`feedback_fresh_timeout_s` 内无新 `eff_L7`）、瞬时力矩超硬限 → 立即停止积分、回退/停机并置 `FAULT`。力环全程在 Edge 本地，不依赖云端。
- **验收标准：**
  1. POSITION 模式：`/a3/gripper_cmd` 发 0/1，L7 运动到 0/1.5708 rad（容差 0.05 rad）；PS4 开合行为不回归
  2. FORCE 模式空载：夹爪闭合到机械限位后力矩收敛不超目标，进入 `GRASPED` 或超时安全停止，无冲击声
  3. FORCE 模式软物体（海绵）与硬阻挡（手指/木块）两种负载下，弱/中/强档位的稳态 `eff_L7` 均收敛到目标 ±10%
  4. 抓取过程中突然抽出/塞入物体：力矩随动调整，不超硬限；目标带内维持 `settle_s` 后上报 `GRASPED`
  5. 反馈超时（停发 `/joint_states`）≤ 看门狗时限内进入 `FAULT` 且停止下发；人为造成瞬时超硬限立即停机回退
  6. 力控期间 L7 位置目标不越过 `joint_cmd` 软限位；退出 FORCE 时恢复夹爪专用 kp/kd 之外不影响 L1–L6
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)（关节层 MIT 力控，非 L8 末端六维力）；[shared/SAFETY.md](../shared/SAFETY.md)；F24（参数）、F26（接口）；`a3_gripper_controller`
- **状态：** `implemented`（仿真软/硬物体闭环 ±10% 且 GRASPED、超力/看门狗 FAULT 均通过；真机 2026-09-13 软泡棉 0.3 N 力控抓取通过：~10 s 进入 GRASPED（contact=true、meas=0.30）、随后 20 s+ 稳态保持 0.30±0.01 Nm、release 干净回全开。同日修复抓取超时误判：曾 GRASPED 后滑脱振荡（软物体带内带外往返）不得再触发超时——超时只约束进入 GRASPED 前的时限，见 LL-021）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
