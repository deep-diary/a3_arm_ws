# F11 — 统一 MoveIt Execute launch + 画矩形 demo


- **说明：** 一键启动 move_group + 执行层（仿真或 CAN）；提供最小笛卡尔/关节矩形 demo
- **验收标准：**
  1. `edge_moveit_execute.launch.py use_sim:=true` 可起
  2. 画矩形 demo 在仿真下完成闭环
  3. `use_rviz:=true` 启动 `el_a3_view.rviz`（默认 `false`）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C1
- **状态：** `implemented`（仿真；launch 含 sim_executor + FJT + IK，完整 move_group 可后续叠加）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
