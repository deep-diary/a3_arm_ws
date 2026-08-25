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
  a3_description a3_can_bridge a3_bringup a3_teleop_ps4 a3_moveit_config a3_lerobot_config a3_cloud_edge
source install/setup.bash
```

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

## reBot software location

Local `reBot-DevArm` is **hardware only**. Software clones live in `d:\working\dev\a3_arm_vendor`:

| Component | Repo |
|-----------|------|
| Python / Pinocchio | https://github.com/vectorBH6/reBotArm_control_py |
| ROS2 + MoveIt shell | https://github.com/Seeed-Projects/reBotArmController_ROS2 |
| MotorBridge (PC bench) | https://motorbridge.seeedstudio.com |
| Wiki hub | https://wiki.seeedstudio.com/robotics_page/ |

Wire reBot planning/teleop outputs through `a3_bringup/trajectory_bridge` into `a3_can_bridge`. Do not use MotorBridge realtime on the same `can0` as the bridge.

## Safety

- Soft limits in `a3_can_bridge/config/control_gains.yaml`
- Trajectory blocked until `/power_sequence/gate_open` is true
- PS4 Triangle or L1+R1+Share → shutdown
- 完整安全原则：[docs/shared/SAFETY.md](docs/shared/SAFETY.md)
