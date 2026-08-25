# a3_cloud_edge

CloudEdge Linux 端：micro-ROS Agent launch、Linux mock client、测试轨迹发布。

## Launch

| 文件 | 说明 |
|------|------|
| `micro_ros_agent.launch.py` | 仅 Agent（apt 或 third_party 构建） |
| `cloud_edge_link_test.launch.py` | Agent + mock client + 测试轨迹（链路冒烟） |
| `cloud_edge_demo.launch.py` | 同上 + robot_state_publisher |

## 依赖

- ROS 2 Humble
- micro-ROS Agent：apt 或 `./scripts/setup_microros_host.sh`
- mock client 运行时需 `rmw_microxrcedds`：`source src/third_party/micro_ros_host/setup_microros.bash`

详见 [docs/cloud_edge/QUICKSTART.md](../../docs/cloud_edge/QUICKSTART.md)。
