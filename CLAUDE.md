# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 仓库概述

A3 机械臂（EDULITE_A3 结构 + reBot 工具链 + RK3588/SocketCAN/MIT 平台）的 ROS 2 Humble 工作区，同时承载两条产品线：

- **A3 Edge（主线）**：RK3588 板载全栈 ROS 2，SocketCAN 直连 7 个 MIT 电机
- **A3 CloudEdge（支线）**：内网服务器 + ESP32-S3 CAN 桥（微边缘）

**详细文档入口**：仓库根 [AGENT.md](AGENT.md)（智能体协作、仓库边界、外部仓库 deep-trace / xiaozhi-esp32、set_joints 落地细节）→ [README.md](README.md) → [docs/README.md](docs/README.md)。架构见 [docs/edge/ARCHITECTURE.md](docs/edge/ARCHITECTURE.md)，话题/关节契约见 [docs/shared/TOPIC_CONTRACT.md](docs/shared/TOPIC_CONTRACT.md)，安全见 [docs/shared/SAFETY.md](docs/shared/SAFETY.md)，控制对标见 [docs/shared/CONTROL_ROADMAP.md](docs/shared/CONTROL_ROADMAP.md)。

## 环境与 source 规则（易踩坑）

- 本机是 RK3588 板（hostname `lubancat`），工作区 `/home/cat/a3_arm_ws`。AGENT.md §3 的 `/home/Blue/dev/a3_arm_ws` 是 WSL 开发机路径，勿混。
- 始终 `source scripts/a3_shell_env.sh`（或 `install/local_setup.bash`）。**不要** source `install/setup.bash`——会把 micro-ROS/trotbot underlay 链进来，导致找不到 `controller_manager`。
- 需要 `export PYTHONNOUSERSITE=1`（`a3_shell_env.sh` 已设置）：`~/.local` 的 NumPy 2.x 会使 ros-humble-pinocchio 崩溃。
- 板载 HDMI 看 RViz 用 `DISPLAY=:0`；SSH 不要加 `-X/-Y`。把 GUI 转笔记本时 `export A3_KEEP_DISPLAY=1`。
- CloudEdge 才需要 `source src/third_party/micro_ros_host/setup_microros.bash`（换 RMW）；Edge 别 source。
- `bridge.launch.py` 内部会清空 `PYTHONNOUSERSITE` 以加载 `~/.local` 的 paho-mqtt。

## 构建

```bash
# 全量（各包 build_type：a3_msgs/a3_can_bridge/a3_description/a3_cloud_edge 是 ament_cmake，
# 其余 Python 包是 ament_python）
colcon build --symlink-install

# 按需构建
colcon build --symlink-install --packages-select a3_msgs a3_arm_controller a3_mqtt_bridge
```

- 改了 `a3_msgs`（srv/msg）后要重编依赖它的 Python 包（如 `a3_arm_controller`、`a3_mqtt_bridge`）；`a3_can_bridge` 是 C++ 执行层，不依赖 `a3_msgs`，改 srv 无需重编。
- 改了 `a3_bringup` 的 launch 文件或 `setup.py` 的 `entry_points` 后必须重编 `a3_bringup`，否则 `install/share` 的链接 / `install/a3_bringup/lib` 的入口不更新。

## 运行与仿真

无电机/CAN 时用模拟闭环（三个模拟节点替代硬件栈：`sim_motor_node`、`sim_power_sequence_node`、`gravity_torque_node`，构成「反馈≈指令 + 一阶跟随 + L7 接触弹簧」全闭环）：

```bash
# 先停硬件栈，避免 /joint_states 冲突
pkill -f "a3_can_bridge can_bridge.launch"; pkill -f motor_protocol_node; pkill -f can_transport_node
source /opt/ros/humble/setup.bash && source ~/a3_arm_ws/install/local_setup.bash
export PYTHONNOUSERSITE=1
ros2 launch a3_bringup edge_web_sim.launch.py use_gripper:=true
```

其他常用入口：`edge_sim_wave_a.launch.py`（zero→work 轨迹）、`edge_moveit_execute.launch.py`（FJT/IK/重力/demo）、`servo.launch.py`（笛卡尔速度）、`a3_moveit_config demo.launch.py`（MoveIt 拖动球）。真机：`sudo systemctl start can-up.service` 后 `ros2 launch a3_bringup a3_bringup.launch.py`。完整验证步骤见 [docs/edge/QUICKSTART.md](docs/edge/QUICKSTART.md)。

## 测试

```bash
./scripts/a3_test/a3_test.sh env        # 环境自检
./scripts/a3_test/a3_test.sh hw         # 真机底层（单电机 CAN_ID=7，需硬件）
./scripts/a3_test/a3_test.sh telemetry  # MQTT 上行
./scripts/a3_test/a3_test.sh mqtt_cmd   # MQTT 下行（mock 编排层，10 op）
./scripts/a3_test/a3_test.sh servo      # MoveIt Servo 仿真
./scripts/a3_test/a3_test.sh gripper    # 夹爪力控（默认 sim 闭环，无硬件可跑；真机 A3_GRIPPER_TEST_MODE=hw）
./scripts/a3_test/a3_test.sh all        # env→gripper→hw→telemetry→mqtt_cmd→servo
./scripts/verify_wave_a_sim.sh          # Wave A 仿真验收（ROS_DOMAIN_ID=55）
./scripts/verify_wave_b_sim.sh          # Wave B（默认 77，含 Wave A 回归）
```

分层回归测试套件（F22）说明见 [scripts/a3_test/README.md](scripts/a3_test/README.md)。所有真机运动经 `safety_limits.py` 限幅（单次 ≤ 0.30 rad、每段 ≥ 2.5 s、结束自动失能）。

## 架构大图

运行时分层（Edge）：Platform（`can-up.service`，can1 @ 1Mbps）→ Execution（`a3_can_bridge`：`can_transport_node` + `motor_protocol_node` + `power_sequence_node`）→ Description（`a3_description` URDF / `a3_moveit_config`）→ Shell（`trajectory_bridge` 桥接 reBot 话题）→ HMI（`a3_teleop_ps4`）→ Orchestration（`a3_arm_controller` 状态机）。

核心数据流：轨迹走 `/joint_group_effort_controller/joint_trajectory` → `motor_protocol_node`（200 Hz 插值 + MIT 协议）→ CAN → 电机；反馈 `/joint_states` @ 50 Hz（7 关节 `L1_joint`–`L7_joint`，L7 是夹爪）。轨迹下发被 `/power_sequence/gate_open` 门禁。

近期功能层（web 闭环）：`a3_mqtt_bridge`（ROS2↔MQTT 遥测/指令，EMQX `192.168.3.73`，前缀 `deep-trace/HOME-DEMO/RK3588/`，信号 code 与 deep-trace 设备 YAML `points[].code` 对齐）、`a3_gripper_controller`（夹爪自适应力控）、`/a3/arm/set_joint_positions` 服务（关节直驱，web 滑动条 → 限位 clamp + 短插值 → jog 覆盖语义）。细节见 AGENT.md §8。

## 工作流硬规则

- **需求先行**：实现新功能前先在 `docs/edge/REQUIREMENTS.md`（Edge）或 `docs/cloud_edge/REQUIREMENTS.md`（CloudEdge）加需求 ID + 验收标准；话题/安全变更同步 `docs/shared/`；完成后更新对应 `QUICKSTART.md`。
- **踩坑落文档**：问题确认修复后，同一轮回复里在 `docs/lessons_learned/` 按 `LL-NNN` 模板写条目并更新索引（环境/source 顺序/DISPLAY/依赖缺失这类换机必踩坑）。
- **外部仓库**：Web 前端/后端在 `/home/cat/deep-trace`（分支 `rk3588`），ESP32 固件在外置 xiaozhi-esp32 仓库——本仓只做 ROS 侧与契约文档，不要在本仓写固件或前端代码。
- 机器人安全：真机改动要过 [docs/shared/SAFETY.md](docs/shared/SAFETY.md)；轨迹在 gate_open 前被阻断；PS4 Triangle 或 L1+R1+Share 急停。
