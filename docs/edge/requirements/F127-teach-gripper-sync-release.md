# F127 — 示教（free-drive）期间夹爪 L7 同步释放 + 回放仅到终点位


- **说明：** 用户实测：示教（start_teach，ZERO_TORQUE 重力补偿自由拖动）期间夹爪（L7）仍上电有力矩。根因：`zero_torque_controller`（GravityCompensationController，rnea 全模型）joints 列表只有 L1–L6，L7 不在重力补偿范围 → TEACH 时 L7 保持位置/力矩。本需求两件：
  1. **示教释放**：`el_a3_controllers.yaml` 的 `zero_torque_controller.joints` 追加 `L7_joint`（rnea 按名映射 order-independent，gripper_link 惯性参与重力项）。**接口冲突**：gripper_controller 与 zero_torque 都 claim `[L7_joint/effort]` → Humble STRICT switch 拒绝「zero_torque active 时 gripper 仍 active」→ `_cm_freedrive_switch` 必须扩展为同一原子请求 deactivate `[arm_controller, gripper_controller]` + activate `[zero_torque]`（enter），exit 反向；用 ListControllers 探测 gripper 实际 active 才纳入（新参 `freedrive_gripper_controller` 默认 `"gripper_controller"`）。
  2. **回放语义（用户已选「记录 L7 + 回放终点位」）**：录制期间 L7 随体位记录；回放时经现有 GripperCommand 路径只重放记录轨迹的**最终 L7 值**（`_dispatch_trajectory` L7 末点分支），不回放 L7 全轨迹。
- **验收标准：**
  1. start_teach 后 list_controllers：`gripper_controller==inactive`、`zero_torque_controller==active`（STRICT 原子 swap 通过；zero_torque 覆盖 L1–L7）；TEACH 稳态 1 s 漂移 ≤0.02 rad（含 L7）
  2. stop_teach 后反向恢复：gripper active、zero_torque inactive
  3. 回放示教文件：L7 到位最终录制值，L1–L6 走关节轨迹
  4. legacy 栈（edge_web_sim，无 controller_manager）不进 `_cm_freedrive_switch`，零影响
- **关联：** F54（auto-save）、F73（standard ZeroTorqueController）、F87/F98（L7→GripperActionController 与限位 [0,1.78]）、F89b（freedrive FSM switch）、F110（mock 栈），[SAFETY.md](../shared/SAFETY.md) 示教小节
- **状态：** `accepted`（2026-09-27 定稿；mock 验收进行中）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
