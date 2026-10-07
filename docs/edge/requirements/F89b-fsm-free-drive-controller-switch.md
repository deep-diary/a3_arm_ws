# F89b — FSM 自由拖动走标准控制器切换（switch_controller：arm_controller ↔ zero_torque_controller；PS4 示教经同一 FSM 服务）


- **说明：** F89 证明标准栈的 STRICT `switch_controller`（停 arm_controller、激活 zero_torque_controller）是可靠的自由拖动路径，但 FSM 的 `start_teach/stop_teach` 仍在调 legacy 执行层服务 `/a3/zero_torque/start|stop`（motor_protocol_node / sim_motor_node 提供，标准栈不存在该服务）。结果：产品标准栈上按 Share 进入示教必然失败（service unavailable），操作员只能像 F89 验收 B 段那样手工拼 switch_controller——「自己折腾总会出错」。本项按工业路径把切换收进 FSM：标准后端（`motor_service_backend=controller_switch`）下，start/stop teach 各发**一次原子 STRICT 切换**（同一请求内 activate+deactivate，失败保持原控制器不变），模式语义与 legacy 对齐（`/a3/control_mode` 发 ZERO_TORQUE / 退出后发 READY，BLOCKED_MODES 门禁照旧生效）；stop 先恢复位置闭环再做自动保存，切换失败则留在 TEACH 态由操作员重试（不允许「报成功但臂还在自由态」）。PS4 侧 Share/Options 早已指向 `/a3/arm/start_teach|stop_teach`，无需改映射。
- **改动：**
  1. `arm_controller.py` 新增参数 `freedrive_arm_controller`（"arm_controller"）、`freedrive_controller`（"zero_torque_controller"）、`freedrive_switch_timeout_s`（5.0）；新增 `_cm_freedrive_switch(enter: bool)`：经现有 `/controller_manager/switch_controller` 客户端发原子 STRICT 请求（enter=deactivate arm + activate zero_torque；exit 反之），超时/拒绝返回 (False, msg)。
  2. `_start_teach_cb`：controller_switch 后端走 `_cm_freedrive_switch(True)`，成功后 `_publish_mode("ZERO_TORQUE")`（回环 echo 置 `_mode`，BLOCKED_MODES 语义不变）；legacy 后端保持调 `/a3/zero_torque/start`，零行为变化。
  3. `_stop_teach_cb`：controller_switch 后端**先** `_cm_freedrive_switch(False)`——失败则不切状态、不保存、返回 success=false（臂仍有重力补偿，重试 stop_teach 即可）；成功后 `_publish_mode("READY")` 再走既有自动保存（latest.yaml + 时间戳备份）。
- **验收标准（仿真；断电；脚本 `scripts/a3_test/f89b_freedrive_switch_acceptance.py`，vcan89b 标准栈，ROS_DOMAIN_ID=92）：**
  1. enable→READY 后调 start_teach：状态 TEACH；`list_controllers` 显示 zero_torque_controller active、arm_controller inactive；`~/gravity_torque` 持续发布；settle 1 s 后稳态 1 s 漂移 ≤ 0.02 rad
  2. 调 stop_teach：状态 READY；arm_controller active、zero_torque inactive；响应消息含 auto-saved；`/a3/control_mode` 恢复 READY
  3. 拒绝路径：DISABLED 下 start_teach 被拒；TEACH 中重复 start_teach 被拒；非 TEACH 下 stop_teach 被拒
  4. 退出切换失败时状态保持 TEACH、success=false（运行时经 set_parameters_atomically 把 freedrive_arm_controller 改为不存在的名字，STRICT 拒绝；LL-100）
- **关联：** F89（同一 STRICT 切换与漂移判据）、F73/F85（zero_torque 控制器与阻尼）、F75（后端分流范式）、F54（停止自动保存）
- **状态：** 已完成（2026-09-23，仿真 20/20：`scripts/a3_test/f89b_freedrive_switch_acceptance.py` 全绿，vcan89b 闭环，脚本退出码 0；稳态漂移实测最差 0.0023 rad；LL-100）。真机验收待通电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
