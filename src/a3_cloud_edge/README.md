# a3_cloud_edge

CloudEdge Linux 端：micro-ROS Agent、MoveIt 规划（只 plan）、开环重力 `effort`、Linux mock client。

## Launch

| 文件 | 说明 |
|------|------|
| `micro_ros_agent.launch.py` | 仅 Agent（apt 或 third_party 构建） |
| `moveit_planning.launch.py` | `move_group` + RSP，无 ros2_control execute |
| `cloud_edge_link_test.launch.py` | Agent + mock + MoveIt plan + 重力补偿（默认） |
| `cloud_edge_demo.launch.py` | 含 robot_state_publisher |

## 依赖

- ROS 2 Humble
- micro-ROS Agent：apt 或 `./scripts/setup_microros_host.sh`
- MoveIt：apt `ros-humble-moveit` 或 `./scripts/setup_moveit_debs.sh`（无 sudo）
- `moveit_plan_node.py` 只调用 `plan_kinematic_path` / `compute_cartesian_path`，**不** `execute`
- mock client 运行时需 `rmw_microxrcedds`：`source src/third_party/micro_ros_host/setup_microros.bash`

详见 [docs/cloud_edge/QUICKSTART.md](../../docs/cloud_edge/QUICKSTART.md)。
