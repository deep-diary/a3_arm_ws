# F99 — JTC 指令超时 cmd_timeout（话题接口陈旧指令在轨迹结束后确定性切 hold；堵住末速指令长期驻留）


- **说明：** F97 给 JTC 配了 `constraints.goal_time: 1.0`，但**只对 action goal 生效**——goal_time 触发时 goal 被 abort、控制器切 hold。JTC 同时在话题 `~/joint_trajectory` 上接受轨迹（工业旁路/工具链常用接口，无 action 生命周期管理）：一条轨迹插值到末点后，如果没有新指令到来，控制器会一直停留在该轨迹的末段采样上。配合本栈 `allow_nonzero_velocity_at_trajectory_end: true`（open_loop + position 接口，末点速度非零也不报错），陈旧的末段指令没有任何确定性的「到期」边界。工业惯例（ros2_controllers JTC 官方参数）是配置 `cmd_timeout`：**从轨迹最后一点计时**，超过该秒数仍未收到新轨迹，控制器主动告警并切到当前位置的 hold 点（`Aborted due to command timeout`）。硬约束：`cmd_timeout` 必须 **严格大于 `constraints.goal_time`，否则该参数在 on_configure 里被静默置 0（仅一条 WARN）**；取 `2.0`（goal_time 1.0 + 1.0 s 裕量）。action goal 永远先在 goal_time=1.0 处拿到 SUCCESSFUL/GOAL_TOLERANCE_VIOLATED，不会走到 cmd_timeout，故对产品 FJT 路径零行为变化。
- **改动（仅配置 + 验收脚本）：**
  1. `src/a3_description/config/el_a3_controllers.yaml` arm_controller：`cmd_timeout: 2.0`
  2. 新增 `scripts/a3_test/f99_jtc_cmd_timeout_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f99_jtc_cmd_timeout_acceptance.py`）：**
  1. **参数生效**：栈启动后 `ros2 param get /arm_controller cmd_timeout` = 2.0（控制器参数挂在各控制器自己的节点 `/arm_controller` 上；`/controller_manager` 只有 `arm_controller.type`），且栈日志无「Command timeout must be higher than goal_time tolerance」警告（出现即参数被忽略）
  2. **action 无回归**：标准两点 action 轨迹仍 SUCCESSFUL（f97 正常路径）
  3. **话题接口兜底**：向 `/arm_controller/joint_trajectory` 发一条短轨迹（L1 小幅移动），末点之后约 2 s，栈日志出现 `Aborted due to command timeout`，之后关节位置保持稳定（无漂移/继续运动）
- **关联：** F97（goal_time / 跟踪容差；本项是其话题接口侧的对称兜底）、F70（JTC 标准栈）、F94（速度限幅）
- **状态：** `completed`（2026-09-24，vcan 验收 5/5）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
