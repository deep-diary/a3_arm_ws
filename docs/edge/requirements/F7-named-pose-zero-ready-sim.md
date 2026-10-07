# F7 — 命名姿态 zero→ready 仿真闭环（C1）


- **说明：** 支持命名姿态 `zero`（上电全零）与 `ready`（悬空工作位）；无真机 CAN 时可通过仿真执行器完成 Plan/下发与 `/joint_states` 跟踪。原 `work` 与 `ready` 语义重叠，2026-09-13 统一为 `ready`（`work` 全仓删除；真机 `ready` 实测值在用户层 `~/.a3/poses.yaml` 覆盖包级值）
- **验收标准：**
  1. SRDF/`named_poses.yaml` 含 `ready`（悬空工作位）
  2. 仿真 launch 下从 `zero` 运动到 `ready`，最终关节误差在容差内
  3. 文档化 launch/脚本命令（见 QUICKSTART / WAVE_A 测试报告）
  4. `use_rviz:=true` 启动 `el_a3_view.rviz` 可视化模型（默认 `false`，无屏验收不启 GUI）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C1；[shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)
- **状态：** `implemented`（仿真验收，见 [WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
