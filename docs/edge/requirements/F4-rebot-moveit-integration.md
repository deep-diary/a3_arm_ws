# F4 — reBot / MoveIt 集成


- **说明：** `trajectory_bridge` 桥接 reBot 话题到 `a3_can_bridge` 默认输入；MoveIt demo（ros2_control mock）提供 RViz MotionPlanning 拖动球 + Plan / Execute（不发 CAN，也不走 `sim_executor`）
- **验收标准：**
  1. `trajectory_bridge` 将 reBot 轨迹话题转到默认执行输入
  2. `ros2 launch a3_moveit_config demo.launch.py use_rviz:=true` 可起；Interact 工具拖末端交互球后可 **Plan**、可 **Execute**
  3. RViz 同时显示目标模（Query Goal State，橙色半透明）与实际模（Scene Robot，跟 `/joint_states`）；mock 下反馈等于指令，Execute 到位后两模重合
  4. TF 坐标轴 / 关节名缩小（`Marker Scale`≈0.08）；启动后窗口最大化（`wmctrl`）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C1；`a3_moveit_config/config/moveit.rviz`、`demo.launch.py`
- **状态：** `implemented`（mock demo 可视化；真机 Plan&Execute 仍见 C1 待办）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
