# A3 Arm Workspace

ROS 2 workspace implementing: **EDULITE_A3 structure/URDF** + **reBot toolchain shell** + **trotbot RK3588/SocketCAN/MIT/PS4 platform**.

## 两条产品线

本工作区（**A3 Arm Platform**）支持两种部署形态，详见 [docs/README.md](docs/README.md)：

| 产品线 | 代号 | 说明 | 状态 |
|--------|------|------|------|
| **A3 Edge** | `edge` | RK3588 + SocketCAN，ROS 2 全栈在板上（**主线**） | 当前生产/开发 |
| **A3 CloudEdge** | `cloud_edge` | 内网服务器 + ESP32-S3 CAN 桥（薄边缘） | 规划/演进 |

- Edge 架构：[docs/edge/ARCHITECTURE.md](docs/edge/ARCHITECTURE.md)
- CloudEdge 架构：[docs/cloud_edge/ARCHITECTURE.md](docs/cloud_edge/ARCHITECTURE.md)

## Layout

```
a3_arm_ws/
  src/
    a3_description/       # from EDULITE_A3 el_a3_description
    a3_moveit_config/     # from EDULITE_A3 el_a3_moveit_config
    a3_can_bridge/        # from trotbot_can_bridge (7-joint ArmMapper)
    a3_bringup/           # launch + trajectory bridge to reBot topics
    a3_teleop_ps4/        # PS4 start/shutdown/set_zero + joint jog
    a3_lerobot_config/    # LeRobot robot-type scaffold
    a3_cloud_edge/        # CloudEdge: micro-ROS Agent launch + mock client
    third_party/          # junctions to ../a3_arm_vendor
  systemd/                # can-up + bringup examples
  docs/                   # 双产品线文档（见 docs/README.md）
a3_arm_vendor/            # cloned reBot software (sibling directory)
  reBotArm_control_py/
  reBotArmController_ROS2/
```

## Dev on PC (recommended)

Use **WSL2 Ubuntu 22.04 + ROS 2 Humble** for mock/MoveIt before flashing the board:

- Step-by-step: [`docs/dev/WSL2_SETUP.md`](docs/dev/WSL2_SETUP.md)

## Build (on RK3588 / Ubuntu 22.04 + Humble)

```bash
source /opt/ros/humble/setup.bash
cd ~/a3_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  a3_description a3_msgs a3_can_bridge a3_bringup a3_teleop_ps4 a3_moveit_config a3_lerobot_config a3_cloud_edge
source install/setup.bash
```

## 无硬件仿真运行

不接 SocketCAN / 电机。

若 `~/.bashrc` 已 source [`scripts/a3_shell_env.sh`](scripts/a3_shell_env.sh)（新开终端自动生效），可省略下面四行。看板载 HDMI 时必须 **`DISPLAY=:0`**（不要用 `${DISPLAY:-:0}`，也不要 SSH `-X`/`-Y`）。把 GUI 转到笔记本时：`export A3_KEEP_DISPLAY=1`。

```bash
source /opt/ros/humble/setup.bash
source ~/a3_arm_ws/install/setup.bash
export PYTHONNOUSERSITE=1
export DISPLAY=:0              # 仅当板子有图形会话 /tmp/.X11-unix/X0
cd ~/a3_arm_ws
```

**不要**再 `source src/third_party/micro_ros_host/setup_microros.bash`（那是 CloudEdge XRCE，会换 RMW）。Edge 请用 `scripts/a3_shell_env.sh` 或 `install/local_setup.bash`，不要用 `install/setup.bash`（后者会把当时 colcon 记下的 micro-ROS / trotbot underlay 全部链进来，表现为找不到 `controller_manager`）。

一次依赖：`sudo apt install -y ros-humble-pinocchio ros-humble-moveit-servo ros-humble-ros2-control ros-humble-controller-manager ros-humble-pick-ik ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager ros-humble-moveit-ros-visualization wmctrl`

### 1) Wave A：zero→work + 重力（自动下发一条轨迹）

```bash
# 接显示器时看机械臂从 zero 运动到 work（el_a3_view.rviz）
ros2 launch a3_bringup edge_sim_wave_a.launch.py duration_s:=3.0 use_rviz:=true
# 无屏 / 脚本验收：省略 use_rviz（默认 false）
```

`use_rviz` 默认 `false`。SSH 进板子看 HDMI：`export DISPLAY=:0`。约 2 s 后自动下发轨迹，RViz 里应看到从竖直 `zero` 运动到 `work`（此配置 **没有** 拖动球 / Plan / Execute，见第 4 节）。

另开终端观察：

```bash
ros2 topic echo /joint_states --once          # 应变到 work
ros2 topic echo /a3/gravity_torque --once     # L3 约 −2.48 Nm，L4 非零
ros2 topic echo /a3/control_mode --once       # IDLE / TRAJ_RUNNING / GRAVITY_COMP
ros2 service call /a3/gravity_compensation/start std_srvs/srv/Trigger {}
ros2 service call /a3/gravity_compensation/stop  std_srvs/srv/Trigger {}
```

到位期望：`work = [0, 0.8901179, -0.9948377, 0, 0, 0, 0]`（容差 0.02 rad）。`duration_s` 为轨迹时长（秒）。

双 ROS domain（互不抢 `/joint_states`）：

```bash
./scripts/dual_domain_zero_to_work.sh          # 默认 EDGE_DOMAIN=10 CE_DOMAIN=20 DURATION_S=3.0
```

### 2) Wave B：执行栈（FJT / IK / 重力 / 可选画矩形）

终端 1 保持栈运行：

```bash
ros2 launch a3_bringup edge_moveit_execute.launch.py \
  use_sim:=true use_gravity:=true use_ik:=true run_demo:=false use_rviz:=true
# 启动时自动画矩形：run_demo:=true
```

| 参数 | 默认 | 含义 |
|------|------|------|
| `use_sim` | `true` | `sim_executor` 跟踪轨迹并发 `/joint_states`（无 CAN） |
| `use_gravity` | `true` | `gravity_torque_node`（Pinocchio） |
| `use_ik` | `true` | `/a3/move_to_pose_ik` |
| `run_demo` | `false` | 延时启动 `draw_rectangle_demo` |
| `use_rviz` | `false` | 启动 `el_a3_view.rviz`（接显示器时设 `true`） |

`sim_executor` 插值固定为 `trajectory_interpolation_method:=auto`：仅位置→线性；带 `velocities`→三次；带 `accelerations`→五次。

终端 2 发指令（栈已起）：

```bash
# 多点轨迹（带速度 → 三次样条）
ros2 topic pub --once /joint_group_effort_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint],
    points: [
      {positions: [0,0,0,0,0,0,0], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 0}},
      {positions: [0,0.5,-0.4,0,0,0,0], velocities: [0,0.2,-0.1,0,0,0,0], time_from_start: {sec: 2}},
      {positions: [0,0.8,-0.7,0,0,0,0], velocities: [0,0,0,0,0,0,0], time_from_start: {sec: 4}}
    ]}"

# FollowJointTrajectory Action
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint],
    points: [{positions: [0,0.5,-0.5,0,0,0,0], time_from_start: {sec: 2}}]}}"

# 笛卡尔 IK（任意位姿可能无解，有应答即服务正常）
ros2 service call /a3/move_to_pose_ik a3_msgs/srv/MoveToPoseIK \
  "{pose: {header: {frame_id: base_link}, pose: {position: {x: 0.2, y: 0.0, z: 0.3},
    orientation: {w: 1.0}}}, seed_positions: []}"

ros2 run a3_bringup draw_rectangle_demo
```

零力矩服务在 `motor_protocol_node` 上，上面这个 launch **不含**该节点。仿真只测服务、不发 CAN：

```bash
ros2 run a3_can_bridge motor_protocol_node --ros-args \
  -p enable_power_sequence_gate:=false -p tx_enable_can0:=false -p tx_enable_can1:=false
# 另开终端：
ros2 service call /a3/zero_torque/start std_srvs/srv/Trigger {}
ros2 service call /a3/zero_torque/stop  std_srvs/srv/Trigger {}
```

### 3) MoveIt Servo（笛卡尔速度）

与上一节 **不要同时** 开（都会占 `sim_executor` / 轨迹话题）。

```bash
ros2 launch a3_bringup servo.launch.py
```

另开终端：

```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
# Twist 必须带有效 stamp（全 0 会被丢掉），推荐 Python 填 now()
```

### 4) MoveIt 规划 demo（mock 硬件，拖动球 Plan & Execute）

Wave A / Wave B 的 `el_a3_view.rviz` **没有** MotionPlanning 插件。要拖末端球再点 **Plan** / **Execute**：

```bash
export DISPLAY=:0    # SSH 看 HDMI；bashrc 已配则可省略
# 若报 controller_manager not found：重开终端；并 sudo apt install -y ros-humble-controller-manager
ros2 launch a3_moveit_config demo.launch.py use_rviz:=true
```

1. 工具栏选 **Interact**
2. 拖末端橙色交互球（目标模，半透明）
3. 左侧 MotionPlanning 面板：**Plan** 出轨迹，**Execute** 让 mock 臂跟上
4. **实际模**（Scene Robot）跟 `/joint_states`；mock 下反馈 = 指令，到位后与目标模重合

这是 ros2_control **mock**，不发 CAN，也不走 `sim_executor`。真机路径见下一节 `a3_bringup.launch.py`。窗口启动后会尝试 `wmctrl` 最大化。

### 一键验收脚本

```bash
./scripts/verify_wave_a_sim.sh    # 脚本内 ROS_DOMAIN_ID=55
./scripts/verify_wave_b_sim.sh    # 默认 77，含 Wave A 回归
```

报告：[docs/dev/WAVE_A_SIM_TEST_REPORT.md](docs/dev/WAVE_A_SIM_TEST_REPORT.md) · [docs/dev/WAVE_B_SIM_TEST_REPORT.md](docs/dev/WAVE_B_SIM_TEST_REPORT.md) · 细节：[docs/dev/WAVE_B_SIM_NOTES.md](docs/dev/WAVE_B_SIM_NOTES.md) · 路线图：[docs/shared/CONTROL_ROADMAP.md](docs/shared/CONTROL_ROADMAP.md)

## Run (A3 Edge 主线)

```bash
# 1) CAN up (see docs/edge/PLATFORM_CAN.md)
sudo systemctl start can-up.service

# 2) Hardware stack
ros2 launch a3_bringup a3_bringup.launch.py can0_name:=can0 use_teleop:=true use_rviz:=true

# 3) Optional: print reBot topic contract
ros2 run a3_bringup rebot_remap_info
```

Quick start checklist: [docs/edge/QUICKSTART.md](docs/edge/QUICKSTART.md)

## Run (A3 CloudEdge Linux 链路)

```bash
# 安装 Agent（一次性）
sudo apt install ros-humble-micro-ros-agent

# 构建 mock client 依赖（一次性，见 docs/cloud_edge/QUICKSTART.md）
./src/a3_cloud_edge/scripts/setup_microros_host.sh
source src/third_party/micro_ros_host/setup_microros.bash

# 链路冒烟
ros2 launch a3_cloud_edge cloud_edge_link_test.launch.py
```

详见 [docs/cloud_edge/QUICKSTART.md](docs/cloud_edge/QUICKSTART.md)。ESP32 固件在外部 [xiaozhi-esp32/deep-dog](https://github.com/deep-diary/xiaozhi-esp32) 仓库实现。

## 参考开源项目（检索入口）

| 用途 | 仓库 / 链接 | 本仓库关系 |
|------|-------------|------------|
| **结构 / URDF / MoveIt 起点** | [RobStride/EDULITE_A3](https://github.com/RobStride/EDULITE_A3) | `a3_description`、`a3_moveit_config` 来源；重力方向约定同源 |
| **硬件参考（无控制栈）** | [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) | 对标硬件；软件不在此仓 |
| **ROS2 + MoveIt 壳层** | [Seeed-Projects/reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2) | 话题/Action/demo；经 `trajectory_bridge` 对接 |
| **Python / Pinocchio 控制** | [vectorBH6/reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) | 重力补偿、IK 参考；本地可放 `a3_arm_vendor/` |
| **RK3588 / SocketCAN / MIT / PS4** | 内部 trotbot 派生 → `a3_can_bridge`、`a3_teleop_ps4` | Edge 生产执行层 |
| **PC 台架 MotorBridge** | [motorbridge.seeedstudio.com](https://motorbridge.seeedstudio.com) | 仅台架；**勿与** `a3_can_bridge` 同占 `can0` |
| **CloudEdge ESP32 固件** | [deep-diary/xiaozhi-esp32](https://github.com/deep-diary/xiaozhi-esp32)（`deep-dog`） | 外置仓；见 [AGENT.md](AGENT.md) |
| **micro-ROS ESP-IDF** | [micro-ROS/micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component) | CloudEdge 固件依赖（Humble） |
| Wiki | [Seeed robotics hub](https://wiki.seeedstudio.com/robotics_page/) · [B601-RS](https://wiki.seeedstudio.com/rebot_arm_b601_rs_ros2_integration/) | 官方联调说明 |

**控制能力对标与缺口：** [docs/shared/CONTROL_ROADMAP.md](docs/shared/CONTROL_ROADMAP.md)（Wave A/B、插值/重力/MoveIt）。

本地 `reBot-DevArm` 仅为硬件参考。软件克隆建议放兄弟目录 `a3_arm_vendor/`。reBot 规划/遥操作经 `a3_bringup/trajectory_bridge` → `a3_can_bridge`。

## Safety

- Soft limits in `a3_can_bridge/config/control_gains.yaml`
- Trajectory blocked until `/power_sequence/gate_open` is true
- PS4 Triangle or L1+R1+Share → shutdown
- 完整安全原则：[docs/shared/SAFETY.md](docs/shared/SAFETY.md)
