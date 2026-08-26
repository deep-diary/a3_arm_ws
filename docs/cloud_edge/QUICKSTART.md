# A3 CloudEdge — Linux 端快速上手

> **Status:** active  
> **本页范围：** 内网服务器 micro-ROS Agent + Linux mock client 链路验证。ESP32 固件在外部仓库实现。

## 前置条件

- Ubuntu 22.04 + ROS 2 Humble
- 已编译本工作区（含 `a3_cloud_edge`）

```bash
sudo apt update
sudo apt install -y ros-humble-micro-ros-agent   # 若 apt 无此包，见下方 setup 脚本
```

若系统 apt 源无 `ros-humble-micro-ros-agent`，使用本仓库脚本从源码构建 Agent 与 host RMW：

```bash
cd ~/a3_arm_ws
./src/a3_cloud_edge/scripts/setup_microros_host.sh
```

**要不要重跑这个脚本？**

| 情况 | 要不要跑 |
|------|----------|
| 本机还没有 `src/third_party/micro_ros_host/`（首次、换机、清过目录） | **必须跑**（首次可达十几分钟到数十分钟） |
| Agent / host RMW / mock 二进制已在，只是新开终端做链路测试 | **不必重跑**，只 `source` overlay |
| 改了 `host_microros_mock/` 源码 | 再跑一次即可（脚本会复制源码并重编 mock；不必重编 Agent） |
| 只改了 `a3_cloud_edge` 的 launch / Python 测试节点 | `colcon build --packages-select a3_cloud_edge`，不必跑 setup |

成功后会在 `src/third_party/micro_ros_host/` 生成 overlay。每次新终端需：

```bash
source /opt/ros/humble/setup.bash
source ~/a3_arm_ws/src/third_party/micro_ros_host/setup_microros.bash
source ~/a3_arm_ws/install/setup.bash
```

若 apt 未装 MoveIt（无 sudo 时），把 Humble 相关 deb 解压到工作区（不进 git）：

```bash
./src/a3_cloud_edge/scripts/setup_moveit_debs.sh
source ~/a3_arm_ws/src/third_party/moveit_debs/setup.bash
```

## 编译

```bash
source /opt/ros/humble/setup.bash
# 若已构建 micro_ros host：
source ~/a3_arm_ws/src/third_party/micro_ros_host/setup_microros.bash
# 若 MoveIt 来自 vendored debs：
source ~/a3_arm_ws/src/third_party/moveit_debs/setup.bash

cd ~/a3_arm_ws
colcon build --symlink-install --packages-select a3_cloud_edge a3_moveit_config a3_description
source install/setup.bash
```

仅 Agent + Python 测试节点（不编 mock client）：

```bash
colcon build --packages-select a3_cloud_edge --cmake-args -DBUILD_MICROROS_MOCK=OFF
```

## 链路冒烟（推荐）

默认 **MoveIt 只规划**（TCP 沿 `base_link` +X 0.10 m）→ Pinocchio 写入 `effort` → Agent → mock。不经 ros2_control `execute`，也不走本机 SocketCAN。

```bash
ros2 launch a3_cloud_edge cloud_edge_link_test.launch.py
# 显式：use_moveit:=true delta_x_m:=0.10
```

另开终端验证：

```bash
ros2 topic echo /joint_group_effort_controller/joint_trajectory --once
# 应有 velocities 与非空 effort
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once   # name 须为 L1_joint…
```

**通过标准：**

1. 可见 `/joint_states`、`/a3/planned_joint_trajectory`、`/joint_group_effort_controller/joint_trajectory`
2. 契约话题的点含 `velocities` 与 `effort`（肩/肘力矩非全 0）
3. mock 日志含 `[mock] trajectory received`（点数 ≤ 32）
4. `joint_states` 约 50 Hz，且 `position` 随轨迹变化

不用 MoveIt、仅 PyKDL 位置轨迹：`use_moveit:=false traj_mode:=forward_dx`。正弦：`use_moveit:=false traj_mode:=sine`。关闭重力：`gravity_ff_scale:=0.0`（launch 里重力节点参数）。

### Mock → 真 ESP32 切换（同一套 ROS 2 节点）

Linux mock 与 ESP32 Client **订阅/发布同一对话题**。联测时停掉 mock、换上板端节点即可：

```bash
# 1) 只起 Agent + 轨迹发布（不启动 microros_mock_client）
ros2 launch a3_cloud_edge cloud_edge_link_test.launch.py use_mock:=false

# 2) ESP32 固件连接「可达的」Agent IP:8888（不要用 127.0.0.1）
# 3) 确认 ros2 topic hz /joint_states ≈ 50 Hz 后来自板端
```

详见下文「外置 ESP32 固件对接」。

## 仅启动 Agent

```bash
ros2 launch a3_cloud_edge micro_ros_agent.launch.py port:=8888
```

## Demo（含 robot_state_publisher）

```bash
ros2 launch a3_cloud_edge cloud_edge_demo.launch.py
```

## 外置 ESP32 固件对接

跨仓说明与 Windows 访问本仓库路径见 [AGENT.md](../../AGENT.md)。阶段 A 需求/验收见 [REQUIREMENTS.md](REQUIREMENTS.md)。

| 项 | 值 |
|----|-----|
| 仓库 | [deep-diary/xiaozhi-esp32](https://github.com/deep-diary/xiaozhi-esp32)（**Windows 侧开发**） |
| 板级 | `main/boards/deep-dog/`（ESP32-S3，已有 `can/`、`motor/`、`trajectory/`） |
| Agent 地址 | **Windows 主机局域网 IP**（或同网段服务器），端口默认 `8888`；真机禁用 `127.0.0.1`，一般也连不到 WSL 的 `172.28.x.x` |
| 轨迹订阅 | `/joint_group_effort_controller/joint_trajectory` |
| 状态发布 | `/joint_states`（50 Hz；阶段 A 允许 mock） |
| 关节名 | `L1_joint` … `L7_joint` |
| micro-ROS 组件 | `micro_ros_espidf_component` **Humble**（`>=22.x`） |

契约详见 [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)。MIT 编解码参考本仓库 `src/a3_can_bridge/include/a3_can_bridge/`。

### Mock → 真 ESP32 切换清单

1. 服务器（WSL）运行 `micro_ros_agent.launch.py`（监听 `0.0.0.0:8888`）；若板在实验室 WiFi，确保 Windows LAN IP:8888 能转到 WSL（portproxy / 镜像网络）
2. 停止 `microros_mock_client`
3. ESP32 固件配置 WiFi + **可达的** Agent IP/端口（阶段 A 见 REQUIREMENTS）
4. 确认 `ros2 topic echo /joint_states` 有真机反馈（≈ 50 Hz）
5. 下发测试轨迹；阶段 A 以固件收到 `points` 为准，阶段 B 再验收运动

## 故障排查

| 现象 | 处理 |
|------|------|
| mock client 启动失败 | 确认已 `source` micro_ros host overlay |
| 无 `joint_states` | 检查 Agent 是否运行；mock client 日志是否连上 127.0.0.1:8888 |
| `joint_states` 关节名不是 `L1_joint` | 本机其它机器人占用同一 `ROS_DOMAIN_ID`（默认 0）。看 `name` 字段，或统一换 domain（Agent 与 ROS 2 须相同） |
| `rclc_take` 刷屏 | XRCE 空 take 噪声，只要随后有 `[mock] trajectory received` 且 `joint_states` 在跟踪即可 |
| `rclc_take` / 轨迹收不到 | 缩小 `JointTrajectory`（少点数、勿填 vel/effort）；mock 需预分配消息缓冲 |
| `micro_ros_agent` 找不到 | `sudo apt install ros-humble-micro-ros-agent`，或再跑 setup 脚本 |
| rosdep / sudo 要密码 | 脚本会把 `libasio-dev`、`libncurses-dev` 解压到 `src/third_party/micro_ros_host/deps/`，无需 sudo |
| 端口占用 | `port:=8889` 并同步 mock client 的 `agent_port` 参数 |

## 关联文档

- [AGENT.md](../../AGENT.md)
- [REQUIREMENTS.md](REQUIREMENTS.md)
- [ARCHITECTURE.md](ARCHITECTURE.md)
- [ROADMAP.md](ROADMAP.md)
- [../dev/WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)（与 Edge 双 domain 仿真对照）
- [../lessons_learned/LL-001-microros-host-setup.md](../lessons_learned/LL-001-microros-host-setup.md)（构建慢 / apt 无 Agent / XRCE 缓冲）
