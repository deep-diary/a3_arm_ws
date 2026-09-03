# A3 Arm Platform — Agent 协作指南

> **读者：** 在本仓库或外置固件仓库工作的编码智能体 / 开发者  
> **目的：** 分清「本仓库 vs 固件仓库」职责，避免在错误工作区改代码；快速定位对标开源项目。

本仓库同时承载两条产品线：**A3 Edge（主线，RK3588）** 与 **A3 CloudEdge（支线，ESP32）**。下文 §1–4 偏 CloudEdge 跨仓固件；Edge 控制对标见 [docs/shared/CONTROL_ROADMAP.md](docs/shared/CONTROL_ROADMAP.md)。

## 0. 参考开源项目（检索入口）

| 用途 | 仓库 | 说明 |
|------|------|------|
| URDF / MoveIt 起点 | [RobStride/EDULITE_A3](https://github.com/RobStride/EDULITE_A3) | `a3_description` / `a3_moveit_config` 来源；重力方向约定 |
| 硬件参考 | [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) | 硬件仓；控制软件不在此 |
| ROS2 控制壳 | [Seeed-Projects/reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2) | MoveIt、FJT Action、demo；仿真走 JTC 样条 |
| Python/Pinocchio | [vectorBH6/reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) | 重力补偿 / IK 参考实现 |
| CloudEdge 固件 | [deep-diary/xiaozhi-esp32](https://github.com/deep-diary/xiaozhi-esp32) | 板级 `main/boards/deep-dog/` |
| micro-ROS 组件 | [micro-ROS/micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component) | ESP-IDF **Humble**（勿用 rolling） |
| Wiki | [Seeed reBot RS](https://wiki.seeedstudio.com/rebot_arm_b601_rs_ros2_integration/) · [hub](https://wiki.seeedstudio.com/robotics_page/) | 官方联调 |

更完整表格见根目录 [README.md](README.md)。

## 1. 仓库边界（必读）

| 仓库 | 路径 / 位置 | 负责内容 | 不负责 |
|------|-------------|----------|--------|
| **本仓库** `a3_arm_ws` | WSL Linux（见 §3） | 双产品线文档与契约；Edge 全栈 ROS；CloudEdge 服务器侧（Agent / mock） | ESP32 固件源码 |
| **外置固件** `xiaozhi-esp32` | 通常在 **Windows** 本机另一工作区 | ESP32-S3 micro-ROS Client、`deep-dog` 板级、CAN/电机固件 | 在本仓库内新建完整 ROS 工作区 |

**固件远程：** https://github.com/deep-diary/xiaozhi-esp32 — 板级 `main/boards/deep-dog/`

**工作流（需求先行）：**

1. Edge 新功能 → `docs/edge/REQUIREMENTS.md`；CloudEdge → `docs/cloud_edge/REQUIREMENTS.md`  
2. 契约变更同步 `docs/shared/`（`TOPIC_CONTRACT.md`、`SAFETY.md`、必要时 `CONTROL_ROADMAP.md`）  
3. **固件实现只在外置仓库进行**  
4. 联调步骤回填对应 `QUICKSTART.md`

若智能体当前打开的是固件仓库：默认**只读参考**本仓库文档；不要把 `a3_arm_ws` 源码树复制进固件工程。若需求条目尚未写入本仓库，先输出需求草案，等确认后再写固件代码。

## 2. CloudEdge 当前优先：阶段 A

详见 [docs/cloud_edge/REQUIREMENTS.md](docs/cloud_edge/REQUIREMENTS.md)「阶段 A」。

摘要：

- WiFi → micro-ROS Agent（UDP **8888**）
- 订 `/joint_group_effort_controller/joint_trajectory`
- 发 `/joint_states` @ 50 Hz（可 mock，可不发 CAN）
- 组件：[micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component) **Humble**（`>=22.x`）
- **ESP-IDF 不内置 micro-ROS**，必须依赖上述组件

服务器侧已实现参考：`S7`–`S10`（Agent + Linux mock）。真机替换 mock 时停掉 `microros_mock_client`。

## 3. 本机绝对路径（WSL ↔ Windows）

本仓库在 **WSL2 Ubuntu** 中；固件工程常在 **Windows** Cursor 窗口。可以告知固件侧 Agent 下列路径，便于只读打开契约文档。

| 场景 | 路径 |
|------|------|
| WSL 内（Linux） | `/home/Blue/dev/a3_arm_ws` |
| Windows 资源管理器 / Cursor「打开文件夹」 | `\\wsl.localhost\Ubuntu-22.04\home\Blue\dev\a3_arm_ws` |
| 等价写法 | `\\wsl$\Ubuntu-22.04\home\Blue\dev\a3_arm_ws` |

**建议优先只读打开的文件（给固件 Agent）：**

- `docs/cloud_edge/REQUIREMENTS.md`（阶段 A 验收）
- `docs/shared/TOPIC_CONTRACT.md`
- `docs/shared/SAFETY.md`
- `docs/cloud_edge/QUICKSTART.md`
- 本文件 `AGENT.md`

**注意：**

1. **可以**把上述绝对/UNC 路径告诉固件开发 Agent，作为只读参考。  
2. **不要**在 Windows 侧对 UNC 路径做 `colcon build`（应始终在 WSL Linux 文件系统内编译本仓库）。  
3. 固件仓库自身的 Windows 绝对路径（如 `C:\...\xiaozhi-esp32`）由固件维护者填入其本地提示词；本文件不臆造。  
4. 若 UNC 访问不稳定，把上述 Markdown **复制/粘贴**进固件侧对话比挂载整个工作区更稳妥。

### 3.1 Agent IP（真机联调，易踩坑）

| 地址 | 能否给 ESP32 用 |
|------|-----------------|
| `127.0.0.1` | 否（指板子自己） |
| WSL `eth0`（常见 `172.28.x.x`） | 通常 **否**（NAT，实验室 WiFi 上的 ESP32 到不了） |
| Windows 主机局域网 IP（`192.168.x.x` / `10.x` 等） | **是**（板与 PC 同 WiFi/路由） |

开发期常见做法：在 WSL 跑 `micro_ros_agent`，并用 Windows `netsh interface portproxy`（或镜像网络）把主机 `:8888` 转到 WSL；固件 menuconfig 填 **Windows LAN IP:8888**。细节见 [docs/dev/WSL2_SETUP.md](docs/dev/WSL2_SETUP.md) 与 QUICKSTART。

本机启动 Agent：

```bash
source /opt/ros/humble/setup.bash
# 若用源码 Agent overlay：
# source /home/Blue/dev/a3_arm_ws/src/third_party/micro_ros_host/setup_microros.bash
source /home/Blue/dev/a3_arm_ws/install/setup.bash
ros2 launch a3_cloud_edge micro_ros_agent.launch.py port:=8888
```

## 4. 给外置固件 Agent 的最终提示词

在 **固件仓库** Cursor 对话中整段粘贴；先替换三处占位符：

- `FIRMWARE_ROOT` — Windows 上 xiaozhi-esp32 绝对路径  
- `AGENT_LAN_IP` — Windows 主机局域网 IP（与 ESP32 同 WiFi；勿用 127.0.0.1 / WSL 172.28.x）  
- `WIFI_SSID` / 密码 — 实验室 AP（勿把真实密码提交进 git）

```text
# 角色与工作区（硬约束）
你是 A3 CloudEdge 的 ESP32-S3 micro-ROS 固件工程师。
当前工作区是外置固件仓库 FIRMWARE_ROOT（xiaozhi-esp32），不是 a3_arm_ws。
- 实现只改本固件仓，板级：main/boards/deep-dog/（ESP32-S3）
- a3_arm_ws 仅只读参考契约；禁止在固件仓内新建/编译 ROS 工作区，禁止把 a3_arm_ws 源码树拷进来
- 禁止擅自改话题名、关节名、消息类型（须先改契约文档并由人确认）

# 需求先行
阶段 A 需求已在契约仓写好，请先阅读再编码：
  \\wsl.localhost\Ubuntu-22.04\home\Blue\dev\a3_arm_ws\AGENT.md
  \\wsl.localhost\Ubuntu-22.04\home\Blue\dev\a3_arm_ws\docs\cloud_edge\REQUIREMENTS.md
  \\wsl.localhost\Ubuntu-22.04\home\Blue\dev\a3_arm_ws\docs\shared\TOPIC_CONTRACT.md
  \\wsl.localhost\Ubuntu-22.04\home\Blue\dev\a3_arm_ws\docs\cloud_edge\QUICKSTART.md
若 UNC 打不开，让用户粘贴上述 Markdown 后再动手。
覆盖需求 ID：E1、E9，以及 E4 发布侧最小实现。阶段 B（CAN/看门狗/gate）本次不做。

# Agent / 网络（真机）
- 传输：UDPv4 over WiFi
- Agent：AGENT_LAN_IP:8888（可配置；禁止 127.0.0.1；不要用 WSL eth0 的 172.28.x.x）
- WiFi：SSID=WIFI_SSID（密码 menuconfig/本地配置，勿提交仓库）
- 服务器侧（WSL）已有：ros2 launch a3_cloud_edge micro_ros_agent.launch.py port:=8888
- ROS 发行版：Humble（与服务器一致）

# 技术选型
- ESP-IDF 不含 micro-ROS；必须用 https://github.com/micro-ROS/micro_ros_espidf_component
- 版本：humble 分支 / Component Registry micro-ros/micro_ros_espidf_component>=22.0.0（禁止 rolling）
- 参考 examples/int32_publisher（WiFi + Agent IP）；按需扩展到 JointTrajectory / JointState
- 编译 micro-ROS 组件时 shell 不要 source ROS 2 setup；IDF venv 需 catkin_pkg、colcon-common-extensions、lark
- 复用 deep-dog 已有目录结构；阶段 A 可不接 can/motor，关节数据可 mock

# 阶段 A 要实现
1. WiFi STA 连接实验室 AP
2. micro-ROS Client 连接 Agent AGENT_LAN_IP:8888，断线可重连 XRCE session
3. 订阅 /joint_group_effort_controller/joint_trajectory（trajectory_msgs/JointTrajectory）
4. 发布 /joint_states（sensor_msgs/JointState）目标 50 Hz
5. joint_names：L1_joint, L2_joint, L3_joint, L4_joint, L5_joint, L6_joint, L7_joint
6. WiFi / Agent IP / 端口：menuconfig 或 Kconfig 可配，密钥不入库

# 阶段 A 验收（你完成后对照自检）
1. Agent 日志可见 session
2. 服务器 ros2 topic hz /joint_states ≈ 50 Hz
3. 服务器发测试 JointTrajectory 后，固件日志/回调确认收到 points
4. 重启 Client 或 Agent 后可再连，无需改代码

# 阶段 A 明确不做
CAN 发送、200 Hz 插值、断连 disable 电机、gate、软限位、急停 GPIO（阶段 B）

# 交付物（仅固件仓）
- 可编译烧录的 deep-dog 改动 + idf_component.yml（或等价依赖声明）
- 配置说明：如何设 WiFi、Agent IP、端口；如何 idf.py build/flash/monitor
- 一份「请回填 a3_arm_ws docs/cloud_edge/QUICKSTART.md」的联调步骤（你不要直接改 a3_arm_ws）
```

## 5. 本仓库 Agent 检查清单

- [ ] 新功能是否先更新了对应 `docs/*/REQUIREMENTS.md`？（Edge 主线勿只改 cloud_edge）
- [ ] 话题/安全是否需改 `docs/shared/`？
- [ ] 控制能力对标是否查阅 [docs/shared/CONTROL_ROADMAP.md](docs/shared/CONTROL_ROADMAP.md)？
- [ ] 若任务是 ESP32 固件：是否应转到外置仓库，而不是在本仓库写 `.c`/`.cpp` 固件？
- [ ] 联调文档是否需更新对应 `QUICKSTART.md`？

## 6. 文档索引

| 文档 | 用途 |
|------|------|
| [README.md](README.md) | 工作区入口 + 开源参考表 |
| [docs/README.md](docs/README.md) | 产品线与文档树 |
| [docs/shared/CONTROL_ROADMAP.md](docs/shared/CONTROL_ROADMAP.md) | Edge 控制对标 reBot（L0–L8 / Wave A·B） |
| [docs/edge/REQUIREMENTS.md](docs/edge/REQUIREMENTS.md) | Edge 需求（主线） |
| [docs/edge/QUICKSTART.md](docs/edge/QUICKSTART.md) | Edge 真机 / Wave A 仿真 |
| [docs/cloud_edge/REQUIREMENTS.md](docs/cloud_edge/REQUIREMENTS.md) | CloudEdge 需求（含阶段 A） |
| [docs/cloud_edge/QUICKSTART.md](docs/cloud_edge/QUICKSTART.md) | Agent / mock / 真机切换 |
| [docs/shared/TOPIC_CONTRACT.md](docs/shared/TOPIC_CONTRACT.md) | 话题与关节名 |
| [docs/shared/SAFETY.md](docs/shared/SAFETY.md) | 安全契约 |
| [docs/dev/WSL2_SETUP.md](docs/dev/WSL2_SETUP.md) | WSL 环境 |
| [.cursor/rules/requirements-first.mdc](.cursor/rules/requirements-first.mdc) | 需求先行规则 |

## 7. 外部前端仓库 deep-trace（Web 展示）

A3 Edge 遥测 Web 展示的前端/后端代码在**外部仓库 `deep-trace`**，与本仓并列，**非本仓子目录**。

| 项 | 值 |
|----|----|
| 路径 | `/home/cat/deep-trace` |
| 分支 | `rk3588`（跟踪 `origin/rk3588`；与 master 同源于 `0e9b339`） |
| 切换 | `cd /home/cat/deep-trace && git checkout rk3588` |
| 技术栈 | 前端 Vue3 + Element Plus + ECharts；后端 Django + DRF；边缘 Python |

**职责分工：**

- 本仓 `a3_arm_ws`：ROS 侧 `src/a3_mqtt_bridge`（ROS2→MQTT 遥测上报，需求 [F18](docs/edge/REQUIREMENTS.md)）
- deep-trace：后端设备 seed / ingest + 前端设备详情页实时曲线

**MQTT 契约（EMQX `192.168.3.73`，TCP 1883 / WS 8083）：**

| 话题 | 方向 | 说明 |
|------|------|------|
| `deep-trace/HOME-DEMO/RK3588/device/info` | 发布（retained） | 板级信息 + 节点/话题/信号目录 |
| `deep-trace/HOME-DEMO/RK3588/device/status` | 发布（~1 Hz） | 心跳 + CPU/内存/温度 + 运行中节点 |
| `deep-trace/HOME-DEMO/RK3588/telemetry` | 发布 | 展平后的 `points`（key 与设备 YAML `points[].code` 一致） |
| `deep-trace/HOME-DEMO/RK3588/cmd` | 订阅（预留） | 双向交互骨架 |

**deep-trace 关键文件：**

| 文件 | 说明 |
|------|------|
| `edge/device_firmware/rk3588/config/HOME-DEMO.RK3588.yaml` | 设备契约（program/nodes/points/topics/mqtt） |
| `backend/config/seed/homes/HOME-DEMO.json` | 工位 `RK3588`（seed） |
| `backend/metadata/device_loader.py` | `rk3588/config` 目录 + `nodes` 透传 |
| `frontend/src/config/deviceRegistry.js` | 注册 `rk3588` program |
| `frontend/src/views/iot/components/Rk3588HubPanel.vue` | 系统信息 + 节点卡片 + 两级下拉 + 缩放曲线 |
| `frontend/src/composables/useRk3588Mqtt.js` | 订阅 info/status/telemetry |
| `docs/requirements/features/iot/modules/ros-device-telemetry.md` | 需求 REQ-IOT-310 |

**联调顺序：** 先 `cd /home/cat/deep-trace/backend && python manage.py load_device_config`（合入 YAML），再起 `a3_mqtt_bridge`，浏览器打开 `/homes/HOME-DEMO/devices` → RK3588。信号 code 两侧必须一致：本仓 `bridge.yaml` ↔ deep-trace `HOME-DEMO.RK3588.yaml` 的 `points[].code`。

**环境准备（RK3588 板实测，Django 6 需 Python 3.12+，系统默认 3.10/3.11）：**

```bash
# 后端（端口 8001）
cd /home/cat/deep-trace/backend
uv python install 3.13                      # uv 装独立 Python，免 sudo
uv venv --python 3.13 .venv && source .venv/bin/activate
uv pip install -r requirements.txt
cp .env.example .env                        # 按需改 MQTT_HOST=192.168.3.73 MQTT_PORT=1883
python manage.py migrate
python manage.py load_line_config
python manage.py load_device_config         # 工位不存在的 YAML 已改为跳过(warning)，不再中断
python manage.py runserver 0.0.0.0:8001

# 前端（端口 5173）
cd /home/cat/deep-trace/frontend
npm install
npm run dev -- --host                        # Network http://192.168.3.78:5173
```

演示账号 `wangwu / demo123`；详情页 `/device/HOME-DEMO/RK3588`。

## 8. 关节直驱控制 + 仿真闭环（set_joints，2026-09 落地）

**当前部署机是 RK3588 板（hostname `lubancat`），路径 `/home/cat/a3_arm_ws`**（§3 的 `/home/Blue/dev/a3_arm_ws` 是 WSL 开发机，勿混）。

### 8.1 控制链路（web 滑动条 → 关节直驱）

```
web 滑动条拖动 --MQTT cmd {op:set_joints, args:{positions[7], duration}}--> ros2mqtt_bridge
  --> 服务 /a3/arm/set_joint_positions (a3_msgs/srv/SetJointPositions) --> a3_arm_controller
  --> 限位 clamp + 短插值 JointTrajectory --> /joint_group_effort_controller/joint_trajectory
  --> 执行层(motor_protocol_node 真机 / sim_motor_node 仿真) --> /joint_states(50Hz)
  --> ros2mqtt_bridge 展平 pos_L1..L7 --> MQTT telemetry --> web(3D + 滑动条实时值)
```

- 新 srv：`src/a3_msgs/srv/SetJointPositions.srv`（`positions[7]` + `duration`）。
- 新服务：`a3_arm_controller` 的 `/a3/arm/set_joint_positions`（`_set_joint_positions_cb`）：限位从 `a3_description/urdf/el_a3.urdf` 读，越界 clamp；短插值 11 点下发；jog 覆盖语义（`TRAJ` 中仅允许 jog 自覆盖，空闲 ~0.5s 自动回 `READY`）。
- 新 op：`a3_mqtt_bridge` 白名单 `set_joints`，映射到上述服务；参数校验 `positions` 须 7 个。
- 契约：`docs/shared/TOPIC_CONTRACT.md` 已补 `set_joints` op 与 `set_joint_positions` 服务行。
- 前端（deep-trace）：`Rk3588HubPanel` 右侧 `ArmJointSliders.vue`（7 滑动条，上下限来自 URDF `joint-limits` emit）；拖动节流 ~120ms 下发 + 松手补发；首次拖动二次确认。

### 8.2 无电机 / 无 CAN 仿真闭环（三模拟节点，2026-09 更新）

不接电机、电池充电时，用三个**模拟节点**替代硬件栈，构成「反馈≈指令 + 一阶跟随 + L7 接触弹簧」的完整闭环，可全链路验证前端（编排、夹爪力控、关节直驱、重力补偿、电源序列全部在线）：

| 模拟节点（name=对齐设备 YAML） | 替代 | 职责 |
|------|------|------|
| `sim_motor_node`（name=`motor_protocol_node`） | 真机 C++ `motor_protocol_node` | 订阅轨迹插值回发 `/joint_states`（7 关节 position/velocity/effort；一阶跟随 `follow_alpha=0.35`；L7 用接触弹簧 `tau=contact_k·max(0, contact_q−q)`，默认 `contact_q=0.6 contact_k=20`）；提供 `/a3/motor/{set_zero,enable,reset,get_device_id,request_version}`、`/a3/motor/set_param`（夹爪力矩硬限写入）、`/a3/zero_torque/{start,stop}`（示教） |
| `sim_power_sequence_node`（name=`power_sequence_node`） | 真机 C++ `power_sequence_node` | 上电即发 `/power_sequence/gate_open=true`、`/power_sequence/state=Running`（transient_local 锁存）；响应 `/power_sequence/command` start/prone/shutdown/set_zero |
| `gravity_torque_node`（name=`gravity_torque_node`） | 真机同名节点（复用现有 executable） | Pinocchio 重力矩发 `/a3/gravity_torque`；name 覆盖对齐 YAML（原节点名 `a3_gravity_torque`） |

L7 接触弹簧让夹爪力控（`gripper_controller_node`）能真实收敛到 `GRASPED`（闭合到 `contact_q` 后力矩上升、力环 PI 调节），无需真实电机/物体。

**一键整体启动（已落地）：**

```bash
# 1) 停掉硬件栈（否则 /joint_states 与 sim_motor_node 冲突）
pkill -f "a3_can_bridge can_bridge.launch"; pkill -f motor_protocol_node; pkill -f can_transport_node

# 2) 整体启动：rsp + sim_motor + sim_power + gravity + arm_controller + mqtt_bridge + gripper
source /opt/ros/humble/setup.bash && source ~/a3_arm_ws/install/local_setup.bash
export PYTHONNOUSERSITE=1
ros2 launch a3_bringup edge_web_sim.launch.py use_gripper:=true
```

`edge_web_sim.launch.py`（`src/a3_bringup/launch/`）组合了：`robot_state_publisher` + `sim_motor_node` + `sim_power_sequence_node` + `gravity_torque_node` + `arm_controller.launch.py` + `bridge.launch.py` + `gripper_controller.launch.py`（`use_gripper` 默认 `true`）。真机模式则用 `a3_bringup.launch.py`（硬件栈）+ 单独起 `arm_controller.launch.py` + `bridge.launch.py` + `gripper_controller.launch.py`。

**端到端验证命令（paho，无需 ros2 CLI）：**

```bash
unset PYTHONNOUSERSITE
python3 - <<'PY'
import json, time, paho.mqtt.client as mqtt
P = "deep-trace/HOME-DEMO/RK3588"; res=[]
c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2); 
def on_msg(c,u,m):
    p=json.loads(m.payload.decode())
    if m.topic.endswith("/cmd_result"): res.append(p)
c.on_message=on_msg; c.connect("192.168.3.73",1883); c.loop_start()
c.subscribe(f"{P}/cmd_result"); time.sleep(1)
c.publish(f"{P}/cmd", json.dumps({"op":"set_joints","args":{"positions":[0,0.8,-0.5,0.3,0,0,0],"duration":0.5}}))
time.sleep(2); print(json.dumps(res, ensure_ascii=False)); c.disconnect()
PY
# 期望回执 {"op":"set_joints","ok":true,"message":"jog 7 joints (0.50s)",...}
# telemetry pos_L2/pos_L3/pos_L4 收敛到 0.8 / -0.5 / 0.3
```

### 8.3 构建与重启要点

- 改 `a3_msgs` 后：`colcon build --symlink-install --packages-select a3_msgs a3_arm_controller a3_mqtt_bridge`。
- 新增/改 launch 文件或 `a3_bringup/setup.py` 的 `entry_points`（如 `sim_motor_node`、`sim_power_sequence_node`）后：需 `colcon build --packages-select a3_bringup`（`install/share` 的 launch 是 symlink，重新 build 才会建链接；新 entry point 也要重建才生成 `install/a3_bringup/lib/a3_bringup/<name>`）。
- `bridge.launch.py` 内部 `SetEnvironmentVariable("PYTHONNOUSERSITE","")` 解除 `.bashrc` 屏蔽，加载 `~/.local` 的 paho-mqtt。
- `can_bridge`（`motor_protocol_node`/`can_transport_node`）是 C++ 硬件执行层，不依赖 `a3_msgs`，改 srv 无需重编。
