# A3 Edge — 需求文档

> **Status:** active  
> **产品线：** A3 Edge（`edge`）— 边缘全栈线（主线）

## 背景与目标

EDULITE A3 机械臂在 RK3588（LubanCat 等）上运行完整 ROS 2 Humble 栈，通过板载 SocketCAN 控制 7 个 MIT 电机。目标是在单板上实现低延迟轨迹跟踪、电源安全序列、PS4 遥操作，并兼容 reBot / MoveIt 工具链。

## 目标用户与场景

- 单机实验室调试与演示
- 多臂同台（每臂独立 CAN，RK3588 多接口或扩展）
- 真机 MoveIt 规划 + 轨迹执行
- LeRobot / 数据采集集成（通过 `a3_lerobot_config`）

## 功能需求

### F1 — 轨迹执行

- 订阅 `trajectory_msgs/JointTrajectory`，转换为 MIT CAN 指令
- 支持 7 关节：`L1_joint`..`L7_joint`
- 单电机 CAN 发送上限 200 Hz（可配置）
- 关节状态反馈 50 Hz 发布到 `/joint_states`

### F2 — 电源序列与门控

- 启动序列：precheck → enable → MIT 模式 → 开门控
- 支持 `start` / `shutdown` / `set_zero` 命令
- gate 未打开时拒绝轨迹执行

### F3 — PS4 遥操作（电源）

- Square 长按：启动电源序列
- Triangle / L1+R1+Share：shutdown
- Options 长按：set_zero
- 笛卡尔 Servo / 夹爪 / 命名姿态映射见 **F16**；旧关节 jog 节点 `ps4_arm_teleop` 保留但不默认启动

### F4 — reBot / MoveIt 集成

- **说明：** `trajectory_bridge` 桥接 reBot 话题到 `a3_can_bridge` 默认输入；MoveIt demo（ros2_control mock）提供 RViz MotionPlanning 拖动球 + Plan / Execute（不发 CAN，也不走 `sim_executor`）
- **验收标准：**
  1. `trajectory_bridge` 将 reBot 轨迹话题转到默认执行输入
  2. `ros2 launch a3_moveit_config demo.launch.py use_rviz:=true` 可起；Interact 工具拖末端交互球后可 **Plan**、可 **Execute**
  3. RViz 同时显示目标模（Query Goal State，橙色半透明）与实际模（Scene Robot，跟 `/joint_states`）；mock 下反馈等于指令，Execute 到位后两模重合
  4. TF 坐标轴 / 关节名缩小（`Marker Scale`≈0.08）；启动后窗口最大化（`wmctrl`）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C1；`a3_moveit_config/config/moveit.rviz`、`demo.launch.py`
- **状态：** `implemented`（mock demo 可视化；真机 Plan&Execute 仍见 C1 待办）

### F5 — 多臂（可选）

- 支持最多 4 臂配置（`arm1`..`arm4`）
- 每臂独立 namespace 与 CAN 接口

### F6 — 轨迹时间跟踪（C2）

- **说明：** 执行层接收多点 `JointTrajectory` 后，按各点 `time_from_start` 插值，再向电机/仿真下发；禁止仅取 `points.front()` 作为整轨目标
- **验收标准：**
  1. 含 ≥2 个航点、时长约 2 s 的轨迹，中间采样时刻关节角位于首末点之间（非阶跃到首点）
  2. 轨迹结束后稳定在末点姿态
  3. `enable_power_sequence_gate` 为真且 gate 关闭时拒绝执行
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C2；CloudEdge mock/ESP32 插值语义对齐
- **状态：** `implemented`（仿真验收，见 [WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)）

### F7 — 命名姿态 zero→ready 仿真闭环（C1）

- **说明：** 支持命名姿态 `zero`（上电全零）与 `ready`（悬空工作位）；无真机 CAN 时可通过仿真执行器完成 Plan/下发与 `/joint_states` 跟踪。原 `work` 与 `ready` 语义重叠，2026-09-13 统一为 `ready`（`work` 全仓删除；真机 `ready` 实测值在用户层 `~/.a3/poses.yaml` 覆盖包级值）
- **验收标准：**
  1. SRDF/`named_poses.yaml` 含 `ready`（悬空工作位）
  2. 仿真 launch 下从 `zero` 运动到 `ready`，最终关节误差在容差内
  3. 文档化 launch/脚本命令（见 QUICKSTART / WAVE_A 测试报告）
  4. `use_rviz:=true` 启动 `el_a3_view.rviz` 可视化模型（默认 `false`，无屏验收不启 GUI）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C1；[shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)
- **状态：** `implemented`（仿真验收，见 [WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)）

### F8 — 重力力矩计算（C3 仿真可验部分）

- **说明：** 基于模型（Pinocchio 优先，否则标定惯量解析近似）按当前 `joint_states` 计算重力补偿力矩并发布；提供启停服务骨架，与轨迹模式互斥标志
- **验收标准：**
  1. 在 `zero` 与 `ready` 稳态可采样到有限力矩
  2. `ready` 下 L2/L3 重力力矩量级大于腕部小关节（抬臂）
  3. 报告注明：仿真验算 ≠ 真机拖动示教
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C3；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（仿真 + Pinocchio 全关节；真机拖动待板测）

### F9 — 重力补偿 MIT 前馈入环（C3 真机路径）

- **说明：** `motor_protocol_node` 订阅 `/a3/gravity_torque`（URDF 系 `τ_g`），按官方方向约定换算为 MIT `tau` 前馈；可用 YAML/运行时参数开关。方向与 EDULITE `DEFAULT_JOINT_DIRECTIONS` / `joint_signs` 一致：`[-1, +1, -1, +1, -1, +1, +1]`（L1…L7），且只乘一次（勿在 `gravity_torque_node` 再乘同号方向）。
- **验收标准：**
  1. `enable_gravity_compensation:=false` 时 MIT `tau` 不含重力项（仅 bus/default）
  2. 开启且 `/a3/gravity_torque` 新鲜时：`τ_mit[i] ≈ gravity_ff_scale * joint_signs[i] * τ_g[i]`
  3. 可运行时 `ros2 param set /motor_protocol_node enable_gravity_compensation true|false`
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C3；`control_gains.yaml`
- **状态：** `implemented`（接线 + 开关；真机标定/拖动板测中）

### F15 — JTC 兼容样条插值（C2 增强）

- **说明：** 执行层按航点字段自动选用线性 / 三次 Hermite / 五次样条（对齐 ros2_control JTC）；`effort` 始终线性；参数 `trajectory_interpolation_method: auto|linear|cubic|quintic`
- **验收标准：**
  1. 仅 `positions` 轨迹行为与线性回归一致；Wave A 脚本仍 PASS
  2. 带非零 `velocities` 时选用三次，航点处速度连续
  3. 带 `accelerations` 时选用五次
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C2
- **状态：** `implemented`（仿真；真机板测中）

### F10 — FollowJointTrajectory Action（C1）

- **说明：** 提供 `/arm_controller/follow_joint_trajectory` Action Server，将 goal 转到执行层轨迹话题并回报 feedback/result；尊重 gate 与 `control_mode` 互斥
- **验收标准：**
  1. `ros2 action send_goal` 多点轨迹可成功结束
  2. gate 关闭或 `ZERO_TORQUE`/`SERVO`/`GRAVITY_COMP` 时拒绝 goal
  3. cancel 中止执行
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- **状态：** `implemented`（仿真）

### F11 — 统一 MoveIt Execute launch + 画矩形 demo

- **说明：** 一键启动 move_group + 执行层（仿真或 CAN）；提供最小笛卡尔/关节矩形 demo
- **验收标准：**
  1. `edge_moveit_execute.launch.py use_sim:=true` 可起
  2. 画矩形 demo 在仿真下完成闭环
  3. `use_rviz:=true` 启动 `el_a3_view.rviz`（默认 `false`）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C1
- **状态：** `implemented`（仿真；launch 含 sim_executor + FJT + IK，完整 move_group 可后续叠加）

### F12 — 笛卡尔 MoveToPose / IK

- **说明：** Pinocchio IK 服务 `/a3/move_to_pose_ik`；可选 Action `/a3/move_to_pose` 解算后经 FJT/轨迹执行
- **验收标准：**
  1. 给定可达位姿返回关节解
  2. 无解/奇异返回失败
  3. Action 路径可驱动仿真到位
- **关联：** CONTROL_ROADMAP L3/C1
- **状态：** `implemented`（仿真）

### F13 — 零力矩模式（C5）

- **说明：** `/a3/zero_torque/start|stop`；MIT `kp` 极小 + 可配 `kd` + 重力 FF；`control_mode=ZERO_TORQUE`；与轨迹/Servo 互斥
- **验收标准：**
  1. 模式可脚本切换并恢复增益
  2. 进入时拒绝新轨迹 Action
- **关联：** [shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（motor_protocol 服务；真机手感板测中）

### F14 — MoveIt Servo（C4）

- **说明：** 接入 `servo_config.yaml`；笛卡尔速度 → 关节轨迹；`control_mode=SERVO`；命令超时停机
- **验收标准：**
  1. Twist 命令引起关节连续变化（仿真）
  2. 超时后停止
- **关联：** CONTROL_ROADMAP C4
- **状态：** `implemented`（`servo.launch.py` + mode bridge；依赖 `moveit_servo`）

### F16 — PS4 映射遥操作（笛卡尔 + 夹爪）

- **说明：** YAML 将手柄轴映射到带归一化参数的函数（`analog_01` / `analog_n11`），按键映射到无参动作。默认 **`mapping:=simple`**：D-pad 命名姿态、双摇杆 MoveIt Servo（`base_link` 系）、L2/R2 直控 L6/L7、Square/Circle 夹爪开/合；**无 L1 死人开关**（开发调试用）。生产/安全映射用 **`mapping:=default`**（L1 按住才发 Servo Twist 与 R2 夹爪）。
- **验收标准：**
  1. `joy_dump` 能打印当前手柄全部 `axes[]` / `buttons[]`；轴序写入 `ds4_linux.yaml` 后映射生效
  2. 改 `config/mappings/*.yaml` 即可换绑，不必改 Python
  3. 仿真 `edge_teleop_sim.launch.py`（`simple`）：右摇杆基座系左右/上下；左摇杆 Y 前后；松杆停止
  4. `simple`：L2→L6、R2→L7 模拟量；Square/Circle 夹爪开/合；`default`：L1+R2 夹爪
  5. D-pad 上/下/左/右分别到 `ready` / `zero` / `home` / `ready`（上键暂与右键同）
  6. Cross 立即停；Square/Triangle/Options 长按电源语义与 F3 一致（`default` 映射）
  7. 无 `/joy` 或 1 s 无更新时 mapper 不发任何轨迹/Servo 令
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[shared/SAFETY.md](../shared/SAFETY.md)；`a3_teleop_ps4`
- **状态：** `implemented`（仿真路径；无手柄门控与 Servo/命名姿态仲裁已自动化验；**手柄轴向与真机 CAN 板测待办**）

### F17 — EL05 电机协议命令集补齐（L0 增强）

- **说明：** 对照 EL05 电机说明书（外置固件 `main/boards/deep-dog/motor/protocol_motor.*`）补齐 `motor_protocol_node` 除 MIT 运控（通信类型 1）与反馈解析（类型 2/24）之外的命令编解码，并在节点暴露 ROS 服务。覆盖：获取设备 ID（类型 0，含 MCU UID 应答）、使能（3）、失能（4）、设零（6）、**设置电机 CAN_ID（类型 7，立即生效）**、软件版本（0x17）、写参数（18，通用 float 与 raw）、主动上报开关（24）与上报周期换算（EPScan_time）。
- **验收标准：**
  1. 各命令帧 CAN ID 与数据域与固件 `buildCanId` / `buildMitControlCanId` / `setCanId` / `setMotorParameter` 一致（可用 `cansend` 回读对照）
  2. `SET_CAN_ID`（类型 7）：`ID = (7<<24) | (new_id<<16) | (0xFD<<8) | current_id`，data 全 0
  3. 服务 `/a3/motor/{enable,reset,set_zero,get_device_id,request_version,set_can_id,set_param}` 可被 `ros2 service call` 正常调用
  4. `OnRxFrame` 能解析设备 ID 应答与软件版本应答，发布到 `/a3/motor/device_id` / `/a3/motor/version`
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) L0；`a3_can_bridge`
- **状态：** `implemented`（编解码 + 服务；真机板测待办）

### F18 — ROS2→MQTT 遥测上报与 Web 实时展示（跨仓）

- **说明：** 在 RK3588 上新增 `a3_mqtt_bridge` 包，订阅 A3 应用话题（精选白名单，YAML 可配），把每条消息展平为 `points`，经 MQTT（EMQX `192.168.3.73`）上报 `deep-trace/HOME-DEMO/RK3588/{device/info,device/status,telemetry}`。deep-trace 前端（外部仓库 `deep-trace`，分支 `rk3588`）以静态 seed 预置 `rk3588` 设备，详情页展示系统信息 + ROS 节点卡片 + 话题/信号两级下拉 + 带 dataZoom 缩放的实时曲线；预留 MQTT `cmd` 下行骨架用于后续双向交互。
- **验收标准：**
  1. 起 bridge 后 `mosquitto_sub -h 192.168.3.73 -t 'deep-trace/HOME-DEMO/RK3588/#' -v` 能观察到 `device/info`、`device/status`、`telemetry`
  2. `telemetry` 载荷 `points` 键与设备 YAML `points` 对齐，`ts` 为 ISO8601
  3. 浏览器 `/homes/HOME-DEMO/devices` 可见 `rk3588` 设备；详情页系统信息卡有值、节点卡片在线
  4. 选 `motor_protocol_node → /joint_states → pos_L1..L7` 曲线实时刷新且可 dataZoom 缩放
  5. bridge 断连可重连；`device/info` 以 retained 发布，重启后前端仍可取到板级信息
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；外部前端仓库 `deep-trace`（本机 `/home/cat/deep-trace`，分支 `rk3588`）
- **状态：** `in progress`

### F19 — 总线扫描服务（通信类型 0 范围探测）

- **说明：** 在 `motor_protocol_node` 暴露 `/a3/motor/scan` 服务，对 `[id_min, id_max]` 内每个 CAN_ID 发送通信类型 0 获取设备 ID 探测帧（只发不等，对齐固件 `sendGetDeviceIdProbes`）。应答帧（cmd=0 且 bit0-7=0xFE，motor_id=bit8-15，data 为 8 字节大端 MCU UID）经 `OnRxFrame` 解析后发布到 `/a3/motor/device_id`，用于验证机械臂 7 关节电机是否全部在线、以及 `SET_CAN_ID` 改号后复核。
- **验收标准：**
  1. `ros2 service call /a3/motor/scan "{id_min: 1, id_max: 127, bus: 0}"` 返回 `sent=127`
  2. 扫描期间 `ros2 topic echo /a3/motor/device_id` 能列出总线在线电机的 motor_id 与 UID
  3. 配合 `SET_CAN_ID` 改号后重新扫描，在线 ID 列表随之变化
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) L0；`a3_can_bridge`
- **状态：** `implemented`（服务 + 编解码；真机板测见 F17）

### F20 — Web 端 3D 机械臂实时渲染（跨仓）

- **说明：** 在 deep-trace 前端（外部仓库 `/home/cat/deep-trace`，分支 `rk3588`）的 RK3588 设备详情页新增 3D 机械臂视图：浏览器加载 `src/a3_description/urdf/el_a3.urdf`（含 8 个 `.stl` mesh），复用 F18 已打通的 MQTT `telemetry` 中 `points.pos_L1..L7`（弧度）实时驱动 `L1_joint..L7_joint`。渲染用 `three` + `urdf-loader`（不兼容时回退 `@gkjohnson/urdf-loader`）；MQTT 复用 `useRk3588Mqtt`，不新建连接。
- **验收标准：**
  1. rk3588 详情页加载 `el_a3.urdf` 后 3D 模型正确渲染（含 mesh 与关节层级）
  2. 起 `a3_mqtt_bridge` 后订阅 `telemetry`，7 关节随 `pos_L1..L7` 实时转动（单位一致，无需换算）
  3. 断连重连后 3D 视图恢复跟随（复用 `useRk3588Mqtt` 的重连逻辑）
  4. 页面卸载时正确释放 `three` renderer 与 `requestAnimationFrame`，无内存泄漏告警
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)；外部前端仓库 `deep-trace`（分支 `rk3588`）
- **状态：** `in progress`

### F21 — 机械臂编排节点（a3_arm_controller）

- **说明：** 新增 `a3_arm_controller` 包与同名节点，作为机械臂对外交互的统一编排层（门面）。职责：统一状态机（`IDLE→INIT→READY`，`READY↔TRAJ/SERVO/TEACH/AI`，`READY→FAULT`）、电机初始化闭环（设零 + 异步确认 7 电机到位）、使能/失能、运行到 MoveIt 预设点、状态聚合与记录、示教（开始/结束/回放/保存）与 AI 模式。底层全部复用现有服务/话题（`/a3/motor/{enable,reset,set_zero}`、`/power_sequence/*`、`/arm_controller/follow_joint_trajectory`、`/servo_node/*`、`/a3/zero_torque/*`、`/a3/gravity_compensation/*`），不重写 CAN 编解码/插值/规划。状态聚合发布到新话题 `/a3/arm_status`（`a3_msgs/msg/ArmStatus`），作为前端与外部唯一状态入口。
- **验收标准：**
  1. `init` 服务：调用 `/a3/motor/set_zero` 后轮询 `/joint_states`，7 关节位置均在 `init_zero_tol_rad` 容差内才返回 `success`，并在 `message` 携带确认数量（如 `7/7`）
  2. `enable`/`disable` 服务分别调用 `/a3/motor/enable`/`/a3/motor/reset`，状态机随之在 `READY`/`IDLE` 间切换
  3. `goto_named_pose` 服务按 `named_poses.yaml` 从当前位姿插值到目标预设点并发布多点轨迹；gate 关闭或处于 `ZERO_TORQUE`/`SERVO`/`GRAVITY_COMP` 时拒绝
  4. `/a3/arm_status` 持续发布（含 `state`、`mode`、7 关节位置、时间戳），频率可配（默认 10 Hz）
  5. `start_teach` 切入零力矩拖动并开始记录 `/joint_states`；`stop_teach` 停止并切回 `READY`；`save_trajectory` 持久化到本地文件；`playback` 回放
  6. `a3_mqtt_bridge` 收到 MQTT `cmd` 后按白名单 op 调用对应服务，并把结果回发 `cmd_result`
  7. `enter_ai`/`exit_ai` 切换 AI 状态，供 LeRobot 数据采集/策略回放接入
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；`a3_msgs`；`a3_mqtt_bridge`；[`a3_lerobot_config`](../../src/a3_lerobot_config)
- **状态：** `implemented`（编排层；真机闭环与拖动示教板测待办）

### F22 — 单电机分层回归测试套件（can1 / ID7 空载）

- **说明：** 新增可重复运行的测试套件 `scripts/a3_test/`，在仅接 1 个空载电机（can1，CAN_ID=7 = L7 夹爪，24V 锂电池供电）的最小硬件下，尽可能覆盖全栈功能。分层设计：真机直连底层 `/a3/motor/*` 服务（编排层 `/a3/arm/init` 要求 7 电机齐全，单电机下预期 FAULT，不作硬性通过项）；MQTT 遥测上行走真机；MQTT 指令下行用 mock 编排层服务做确定性断言；MoveIt Servo 六方向直线走仿真栈（真机 Servo 入环为 CONTROL_ROADMAP 待办）。所有真机运动经集中安全限幅（单次目标 ≤0.30 rad、时长 ≥2.5 s、运动前使能、结束必失能）。
- **验收标准：**
  1. `./scripts/a3_test/a3_test.sh env` 校验 can1 UP、EMQX `192.168.3.73:1883` 可达、ROS/paho 依赖就绪
  2. `hw`：扫描到 `motor=7`；`/joint_states` 的 `L7_joint` 反馈有效且 ~50 Hz；`set_zero`→角度归零；`enable`→mode_status=2；小角度多点轨迹（±0.25 rad）跟随到位且无过冲；`/a3/zero_torque/start|stop` 可切换并恢复增益；`reset`→退出使能；超限位命令被软限幅
  3. `telemetry`：真机驱动 L7 转动时，MQTT `deep-trace/HOME-DEMO/RK3588/telemetry` 的 `points.pos_L7` 同步变化
  4. `mqtt_cmd`：mock 编排层 10 个服务在线时，经 MQTT `cmd` 下发 init/enable/disable/goto/teach_start/teach_stop/save/playback/enter_ai/exit_ai 均收到 `cmd_result.ok=true` 且服务端收到正确参数；未知 op 回 `ok=false`
  5. `servo`：仿真 `servo.launch.py` 下，`base_link` 系 ±x/±y/±z 方向 Twist 命令引起末端位移方向正确，命令停止后自动 halt
  6. 每阶段输出 PASS/FAIL 汇总，非零退出码表示失败；可随时重复运行；真机阶段无报警、无冲击
- **关联：** [QUICKSTART.md](QUICKSTART.md) 第 12 节；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[shared/SAFETY.md](../shared/SAFETY.md)；`a3_can_bridge`；`a3_mqtt_bridge`；`a3_bringup/servo.launch.py`
- **状态：** `implemented`

### F23 — Web 端机械臂控制面板（MQTT cmd 下行 UI，跨仓）

- **说明：** 在 deep-trace 前端（外部仓库 `/home/cat/deep-trace`，分支 `rk3588`）RK3588 详情页新增 `a3_arm_controller` 节点卡片，点击展开「机械臂控制面板」。浏览器经 MQTT（mqtt.js 直连 EMQX WS，与现有遥测同一连接）下发 `cmd`（op 白名单 10 个：`init`/`enable`/`disable`/`goto`/`teach_start`/`teach_stop`/`save`/`playback`/`enter_ai`/`exit_ai`），订阅 `cmd_result` 把执行结果以 `ElMessage` toast + 面板内「操作消息列表」反馈到 UI；同时展示编排层 `/a3/arm_status`（`arm_state`/`arm_mode`/`arm_message`）实时状态。所有动作类指令须经 `ElMessageBox` 二次确认防误触，MQTT 未连接时按钮禁用。设备侧 `a3_mqtt_bridge` 已实现 10 op 下行与回执（F21/F22），本需求仅在 `bridge.yaml` 增加 `/a3/arm_status` 的 `scalar` 展平配置（`state`/`mode`/`message` → `arm_state`/`arm_mode`/`arm_message`），不改桥接 Python。Django 后端不经手指令（与全仓 IoT 面板一致），仅 `load_device_config` 合入 YAML 契约。
- **验收标准：**
  1. 设备 YAML `topics.cmd_result` 存在；`nodes` 含 `a3_arm_controller`（话题 `/a3/arm_status`，信号 `arm_state`/`arm_mode`/`arm_message`）；`points` 含上述 3 个 `discrete` 信号；`load_device_config` 后 `GET /auth/my-lines` 的 `edge.nodes`/`edge.topics` 体现
  2. RK3588 页出现「机械臂编排节点」卡片，点击展开控制面板；选中其它节点保持现有遥测曲线联动
  3. 面板状态区显示 `arm_state`（READY=绿/FAULT=红/其余蓝或黄）、`arm_mode`、`arm_message`，随 `/a3/arm_status` 实时刷新
  4. 10 个动作可下发：初始化/使能/失能、示教开始/结束、保存/回放（带轨迹名输入）、goto（zero/home/ready 下拉）、进入/退出 AI；点击先弹二次确认，确认后才 publish
  5. 收到 `cmd_result` 后：`ok=true` 弹成功 toast、`ok=false` 弹失败 toast 并在消息列表标红；消息列表保留最近约 20 条（时间、op、成败 tag、message）
  6. MQTT 未连接/断网时所有动作按钮禁用；轨迹名为空或 goto 未选姿态时本地拦截提示，不下发
  7. `bridge.yaml` 新增 `/a3/arm_status` 展平后 `colcon build --packages-select a3_mqtt_bridge`，telemetry 的 `points.arm_state/arm_mode/arm_message` 随编排节点发布
  8. 回归 `scripts/a3_test/a3_test.sh mqtt_cmd` 仍 PASS（10 op 回执契约一致）
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（cmd/cmd_result JSON 契约）；F18（MQTT 遥测桥）、F20（Web 3D）、F21（编排节点）、F22（分层测试）；外部前端仓库 `deep-trace`（分支 `rk3588`，`useRk3588Mqtt.js` / `ArmControlPanel.vue` / `Rk3588HubPanel.vue` / `HOME-DEMO.RK3588.yaml`）
- **状态：** `implemented`

### F24 — 夹爪握力配置与安全限幅

- **说明：** 第 7 电机（L7 夹爪）新增独立握力参数配置，控制「手抓不能抓太紧」。新增 `a3_gripper_controller/config/gripper_config.yaml`：最大握力硬上限 `max_grasp_torque_nm`（出厂不可越界）、默认目标力与弱/中/强档位 `torque_presets_nm`、夹爪力矩方向符号 `gripper_torque_sign`（真机标定，把「夹紧阻力」校正为正）、PI 参数（`force_kp`/`force_ki`/积分限幅/位置增量速率限幅）、位置/速率限幅、接触判定阈值、抓取/看门狗超时。节点启动（电机使能）时经现有 `/a3/motor/set_param` 服务（`motor_id=7`，`param_id=0x700B` 力矩限制）把固件级力矩上限写入电机，形成「固件硬限 + 节点软件 clamp」双保险。参数支持 YAML 默认值与运行时 web/服务下发：下发值一律校验 `≤ max_grasp_torque_nm` 且在 MIT 力矩量程（±6 Nm）内，越界拒绝；通过后落盘 `data/gripper_overrides.yaml`，重启自动加载。
- **验收标准：**
  1. `gripper_config.yaml` 含上述全部参数且注释标明单位与典型取值；缺 key 时节点用内置安全默认值启动并告警
  2. 调用配置服务把最大握力设为合法值：返回成功，落盘文件更新，重启节点后值保留
  3. 下发超过 `max_grasp_torque_nm` 或超过 ±6 Nm 的值：服务返回失败且不改变当前配置
  4. 电机使能流程中 `/a3/motor/set_param`（ID7, 0x700B）被调用且值与配置一致；服务失败时节点不上报就绪并报错
  5. 弱/中/强三档目标力均 ≤ 硬上限；不修改 L1–L6 的任何增益/限位配置
- **关联：** [shared/SAFETY.md](../shared/SAFETY.md)（夹爪力控安全段）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；`a3_gripper_controller/config/gripper_config.yaml`；F17（`/a3/motor/set_param`）；F26（配置服务/MQTT）
- **状态：** `implemented`（仿真闭环验证；真机 0x700B 写入与力矩方向标定板测中。2026-09-07：出厂硬上限 2.0→1.0 Nm（1.5 持续出力几分钟过热，见 LL-014），PI 减半为 kp=0.25/ki=0.3，运行时持久化上限同步 1.0 Nm；sim 植物接触方向随 2026-09-06 标定反转。同日起超硬限 FAULT 瞬态带放宽为 `overtorque_ratio` 1.0→1.5：硬物体 PI 过冲会顶穿 1.0 误报 `error_code=4`，软件 FAULT 只做失控兜底，固件 0x700B 仍硬钳 1.0 Nm）

### F25 — 夹爪自适应力控（PI 力外环 + 位置内环）

- **说明：** 新增独立 Python 包 `a3_gripper_controller`（节点 `gripper_controller_node`，50 Hz 力外环，不改动 C++ CAN 核心的实时路径）。四种模式：`POSITION`（现有开合语义，订阅 `/a3/gripper_cmd` 的 0–1 归一化并映射 0–1.5708 rad，补齐「有发布无执行」缺口）、`FORCE`（力控抓取）、`RELEASE`（张开到安全位）、`STOP`（停止力环并保持）。FORCE 模式：以设定握力 `tau_target` 为目标，读 `/joint_states` 的 `eff_L7`（经 `gripper_torque_sign` 校正方向）作为反馈，PI 调节器输出**位置增量**——力矩不足（夹得不够紧）则向闭合方向累加位置目标，力矩超了则回退；积分限幅、位置增量变化率限幅、位置目标钳位在 L7 软限位内。位置目标经现有 L7 单关节 `JointTrajectory` 通道下发（与 PS4 `set_joint_L7` 同路径），电机内置位置环顶住物体，接触力即被维持在设定值，从而自适应抓取软硬不同物体。接触/抓稳判定：位置停滞且力矩进入目标带（±10%）并维持 `settle_s` → 状态 `GRASPED`。安全：抓取超时、反馈看门狗超时（`feedback_fresh_timeout_s` 内无新 `eff_L7`）、瞬时力矩超硬限 → 立即停止积分、回退/停机并置 `FAULT`。力环全程在 Edge 本地，不依赖云端。
- **验收标准：**
  1. POSITION 模式：`/a3/gripper_cmd` 发 0/1，L7 运动到 0/1.5708 rad（容差 0.05 rad）；PS4 开合行为不回归
  2. FORCE 模式空载：夹爪闭合到机械限位后力矩收敛不超目标，进入 `GRASPED` 或超时安全停止，无冲击声
  3. FORCE 模式软物体（海绵）与硬阻挡（手指/木块）两种负载下，弱/中/强档位的稳态 `eff_L7` 均收敛到目标 ±10%
  4. 抓取过程中突然抽出/塞入物体：力矩随动调整，不超硬限；目标带内维持 `settle_s` 后上报 `GRASPED`
  5. 反馈超时（停发 `/joint_states`）≤ 看门狗时限内进入 `FAULT` 且停止下发；人为造成瞬时超硬限立即停机回退
  6. 力控期间 L7 位置目标不越过 `joint_cmd` 软限位；退出 FORCE 时恢复夹爪专用 kp/kd 之外不影响 L1–L6
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)（关节层 MIT 力控，非 L8 末端六维力）；[shared/SAFETY.md](../shared/SAFETY.md)；F24（参数）、F26（接口）；`a3_gripper_controller`
- **状态：** `implemented`（仿真软/硬物体闭环 ±10% 且 GRASPED、超力/看门狗 FAULT 均通过；真机 2026-09-13 软泡棉 0.3 N 力控抓取通过：~10 s 进入 GRASPED（contact=true、meas=0.30）、随后 20 s+ 稳态保持 0.30±0.01 Nm、release 干净回全开。同日修复抓取超时误判：曾 GRASPED 后滑脱振荡（软物体带内带外往返）不得再触发超时——超时只约束进入 GRASPED 前的时限，见 LL-021）

### F26 — 夹爪服务/话题契约与 MQTT 桥接

- **说明：** 为夹爪力控定义对外接口并打通 web 下行。`a3_msgs` 新增：`srv/GripperCommand.srv`（`mode`：position/force/release/stop，`position` 0–1，`torque_nm` 目标握力，`timeout_s`）、`srv/GripperSetConfig.srv`（`max_torque_nm` 等键值，含校验结果）、`msg/GripperStatus.msg`（模式、目标/实际力矩、位置、接触/抓稳标志、错误码、时间戳）。状态话题 `/a3/gripper_status`（`a3_msgs/msg/GripperStatus`，默认 10 Hz，力控期间 50 Hz）。`a3_mqtt_bridge`：`bridge.yaml` 新增 `/a3/gripper_status` 的 scalar 展平（`grip_state`/`grip_mode`/`grip_target_torque`/`grip_actual_torque`/`grip_position`/`grip_contact`/`grip_error`）；cmd 白名单新增 4 个 op：`gripper_grasp`（带 torque/档位）、`gripper_release`、`gripper_stop`、`gripper_set_max_torque`（带 value），均映射到上述服务并回 `cmd_result`。模式互锁：`gripper_controller_node` 订阅 `/a3/control_mode` 与 `/power_sequence/gate_open`，gate 关闭或臂处于 `TRAJ_RUNNING`/`SERVO`/`ZERO_TORQUE`/`GRAVITY_COMP` 时拒绝 FORCE 启动（POSITION 开合随臂轨迹互锁规则一致）；力控运行时臂侧轨迹/Servo 启动须先终止夹爪力环。
- **验收标准：**
  1. `colcon build --packages-select a3_msgs a3_gripper_controller a3_mqtt_bridge` 通过；服务/消息可 `ros2 interface show`
  2. `ros2 service call /a3/gripper/command` 各模式返回 `success` 且 `/a3/gripper_status` 随之变化；非法力矩/互锁状态返回 `success=false` 并带 `message`
  3. MQTT `cmd` 下发 4 个 gripper op 均收到 `cmd_result.ok=true`；未知参数/越界值 `ok=false` 且服务端未执行
  4. telemetry 中 `points.grip_state/grip_target_torque/grip_actual_torque/grip_contact` 随力控过程实时变化
  5. gate 关闭或 `control_mode=SERVO` 时 `gripper_grasp` 被拒；力控中启动臂轨迹则力环先安全停止
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（gripper 话题/服务/MQTT op/points）；[shared/SAFETY.md](../shared/SAFETY.md)（互锁表 `GRIPPER_FORCE`）；F21（编排/互锁风格）、F23（cmd 白名单模式）；`a3_msgs`、`a3_mqtt_bridge`
- **状态：** `implemented`（服务/消息/桥接编译通过；MQTT 4 op 回执与 `grip_*` telemetry 端到端验证通过）

### F27 — Web 端夹爪控制面板（MQTT 下行 UI，跨仓）

- **说明：** 在 deep-trace 前端（外部仓库 `/home/cat/deep-trace`，分支 `rk3588`）RK3588 详情页新增夹爪卡片 `GripperPanel.vue`。设备 YAML `HOME-DEMO.RK3588.yaml` 增加 gripper 节点（话题 `/a3/gripper_status`）、topics（含 `cmd_result`）、points（`grip_state`/`grip_target_torque`/`grip_actual_torque`/`grip_position`/`grip_contact`）与 4 个 cmd op 契约，Django 侧仅 `load_device_config` 合入 YAML，不经手指令。UI：握力档位（弱/中/强）单选 + 目标力滑块（标注硬上限，超限本地拦截不下发）、最大握力设置（输入框 + 二次确认）、抓取/释放/停止按钮（动作类 `ElMessageBox` 二次确认，MQTT 未连接时禁用）、实时握力曲线（复用 telemetry `grip_actual_torque`/`eff_L7`）与状态指示（`GRASPED` 绿/`FAULT` 红/力控中蓝）、`cmd_result` 回执 toast + 消息列表。复用 `useRk3588Mqtt` 同一连接，不新建 MQTT。
- **验收标准：**
  1. 设备 YAML 含 gripper 节点/topics/points/cmd；`GET /auth/my-lines` 的 edge 配置体现；前端信号 code 与 `bridge.yaml` 完全一致
  2. RK3588 页出现夹爪卡片：档位切换、滑块、最大握力设置、抓取/释放/停止按钮齐备
  3. 滑块/输入超过硬上限时本地提示且不下发；抓取/释放/停止点击后弹二次确认
  4. 下发后订阅 `cmd_result`：成功 toast、失败标红；消息列表保留最近约 20 条
  5. 握力曲线随 `grip_actual_torque` 实时刷新；`GRASPED`/`FAULT` 状态颜色正确；MQTT 断连时所有控件禁用
  6. 回归 F23 机械臂控制面板 10 op 不受影响
- **关联：** F18/F23（MQTT 通道与面板模式）、F26（op/points 契约）；外部前端仓库 `deep-trace`（分支 `rk3588`，`GripperPanel.vue` / `HOME-DEMO.RK3588.yaml` / `useRk3588Mqtt.js`）
- **状态：** `implemented`（前端 `GripperPanel.vue` + YAML 契约已合入 `rk3588` 分支且 `vite build` 通过；需 `load_device_config` 合入并浏览器联调）

### F28 — 夹爪力控真机回归测试

- **说明：** 在 F22 的 can1 / ID7 单电机最小硬件基座上，新增 `scripts/a3_test/` 的 `gripper` 子命令（`hw_gripper_test.py`），覆盖力控全链路安全验收。沿用 F22 安全限幅（运动前使能、结束必失能、小步运动）。测试项：(a) 配置下发与落盘（合法/越界）；(b) 固件硬限写入（ID7, 0x700B）确认；(c) POSITION 开合；(d) 力控阶跃——空载/软阻挡（海绵）/硬阻挡（手指或木块）下弱中强档位力矩收敛 ±10% 与 `GRASPED`；(e) 超力保护——设小目标 + 硬阻挡，固件/软件双限均不超硬限；(f) 看门狗——停止 `/joint_states` 后 ≤ 时限进 `FAULT`；(g) MQTT gripper op 回执与 `grip_*` 遥测同步；(h) 互锁——gate 关闭/SERVO 模式下力控被拒。
- **验收标准：**
  1. `./scripts/a3_test/a3_test.sh gripper` 可独立重复运行，输出各子项 PASS/FAIL 汇总，非零退出码表示失败
  2. 力控阶跃项：三种负载 × 三档位共 9 组，稳态力矩在目标 ±10% 内，无报警无冲击
  3. 超力保护项：全过程 `eff_L7` 不超过硬上限（留 10% 测量余量断言）
  4. 看门狗项：反馈中断后 ≤ `feedback_fresh_timeout_s + 1` 个周期进 `FAULT` 且无新 CAN 指令
  5. 配置项：越界值被拒、合法值落盘且重启保留；MQTT 项 4 op 回执契约一致
  6. 测试结束电机失能、gate 状态复原
- **关联：** [QUICKSTART.md](QUICKSTART.md)（夹爪力控验证节）；F22（测试基座/安全限幅）；F24/F25/F26；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（`./scripts/a3_test/a3_test.sh gripper` 仿真闭环 12 项 PASS；真机 `A3_GRIPPER_TEST_MODE=hw` 服务/安全检查就绪，力控阶跃需人工放海绵/硬阻挡板测）

### F29 — 互锁模式回收（/a3/control_mode 发布缺陷修复，真机实测）

- **说明：** 真机 2026-09-06 实测发现：`a3_arm_controller` 只在轨迹开始时发布 `TRAJ_RUNNING`，轨迹结束（`_back_to_ready`）与节点启动均不发布非阻塞模式；`/a3/control_mode` 为 VOLATILE 事件型话题，无周期性兜底 → 夹爪力控互锁（F25/F26 的 `GRIPPER_FORCE` 互锁表）在真机跑过任意轨迹后**永久锁存 `TRAJ_RUNNING`**，力控指令永远被拒（仿真闭环不经过该节点，故 F28 未暴露）。修复：① 轨迹结束发布 `READY`；② 节点启动即发布 `IDLE`，并 2 s 后重发一次（启动首条发布可能早于订阅发现、被静默丢弃）。
- **验收标准：**
  1. 真机跑完任意轨迹（`goto` / `set_joint_positions` / `playback`）后，立即发 `/a3/gripper/command {mode: force}` 成功（无需重启任何节点）
  2. `a3_arm_controller` 重启后，无需其他操作，夹爪力控指令成功（2 s 重发覆盖发现窗口）
  3. 夹爪互锁恢复全程无人工干预，仿真闭环回归不退化
- **关联：** F25（力控）、F26（互锁表）、[shared/SAFETY.md](../shared/SAFETY.md)（`GRIPPER_FORCE` 互锁）；[LL-009](../../lessons_learned/LL-009-arm-control-mode-no-release.md)
- **状态：** `implemented`（2026-09-06 真机泡棉力控测试验证：跑完 set_joints 轨迹后力控 0.3 Nm 直接成功）

### F30 — 夹爪力控超时以命令参数为准（真机实测）

- **说明：** 真机 2026-09-06 实测发现：`GripperCommand.timeout_s` 传入 `_start_force` 后被丢弃，`_tick_force` 恒用配置 `grasp_timeout_s=5.0` → 软物体（泡棉）0.3 Nm 力环尚未收敛即误报 `ERR_GRASP_TIMEOUT` 进 FAULT。修复：每次夹取把命令 `timeout_s` 存入 `_force_timeout_s`（缺省/非正回落配置值），超时判断改用它。
- **验收标准：**
  1. 真机泡棉 0.3/0.5/0.7/1.0 Nm 四档均在 `timeout_s: 12.0` 内 `GRASPED`，稳态力矩在目标 ±10% 内
  2. 不传 `timeout_s` 时回落配置 `grasp_timeout_s`（5.0 s）行为不变
  3. 超时进 FAULT 后，下一次指令（release/force）清除错误码（F25 原语义不变）
- **关联：** F25（力环）、F28（真机回归）；[LL-010](../../lessons_learned/LL-010-gripper-force-timeout-ignored.md)
- **状态：** `implemented`（2026-09-06 真机泡棉四档力控实测通过）

### F31 — Web 端夹爪面板 v2：位置模式直驱 / 双曲线 / NaN 遥测毒化修复（跨仓）

- **说明：** 真机 2026-09-06 联调发现夹爪面板状态/曲线全为空：`/joint_states` 对未连接关节 L1–L6 发布 NaN 速度，桥接层 `json.dumps` 默认 `allow_nan=True` 把 `"vel_L1": NaN` 写入 telemetry，构成非法 JSON；浏览器 `JSON.parse` 抛错后整包丢弃，所有 telemetry 衍生 UI 显示「—」（Python `json.loads` 容忍 NaN，故板侧测试未暴露）。本轮：① 桥接层递归清洗非有限浮点（NaN/±Inf → null）并对全部载荷 `allow_nan=False` 硬兜底；② 前端容错解析（NaN 词法替换为 null）+ 解析错误计数/原文面板；③ 夹爪面板 v2：状态行中文模式映射（position 位置模式/force 力矩模式/release 释放/stop 停止）、模式选择器（位置/力矩）、位置模式 0–1 开合滑块节流直驱（新 op `gripper_set_position` → `/a3/gripper/command mode=position`）、力矩模式滑块节流抓取、目标 vs 实测双曲线（位置/力矩两页签，各页签同轴对比）；④ `GripperStatus.msg` 追加 `target_position`（position/release 命令写入，未命令前跟随实测），bridge 展平为 `grip_target_position`。
- **验收标准：**
  1. telemetry JSON 任意时刻均为合法 JSON 且无 NaN/Infinity 词法（板侧严格解析器 10 s 采样 0 异常）
  2. MQTT 下发 `gripper_set_position {position: 0..1}` 回 `cmd_result.ok=true`，`grip_target_position` 随动；越界/非法值 `ok=false` 且服务端不执行
  3. 浏览器：夹爪状态/模式（中文）/接触/错误正常显示；模式选择器切换控件；位置滑块拖动节流下发并回显实测；两页签曲线目标 vs 实测同轴实时刷新
  4. 调试面板显示原始报文、grip 各 code 在线/新鲜度、解析错误计数（修复后保持 0）
  5. 回归：`a3_test.sh gripper`、`mqtt_cmd`、F23 臂面板 10 op 不受影响
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[QUICKSTART.md](QUICKSTART.md)（§14）；[LL-011](../../lessons_learned/LL-011-nan-poisons-json-telemetry.md)；F18/F26/F27/F28；外部前端仓库 `deep-trace`（分支 `rk3588`，`GripperPanel.vue` / `Rk3588HubPanel.vue` / `useRk3588Mqtt.js` / `RealtimePerSignalChart.vue` / `HOME-DEMO.RK3588.yaml`）
- **状态：** `completed`（2026-09-06；浏览器硬刷新目检由用户确认；真机位置直驱待 can_bridge 轨迹订阅 QoS 对齐 BEST_EFFORT，见 F32）

### F32 — Web 端电机调试页（CAN 扫描 / MIT 保持 / 实时曲线，跨仓）

- **说明：** 仿 sparkbot 电机模块页，为 RK3588 增加 `motor_protocol_node` 专用单电机调试详情页：`Rk3588HubPanel` 的电机节点卡片跳转 `device-module` 路由（moduleId `rk3588_motor`），新面板 `Rk3588MotorPanel.vue` 提供 CAN 扫描（`/a3/motor/scan_and_collect` 聚合返回 ids/uids）→ 电机选择 → 状态卡片（在线/温度/模式/故障位/原始力矩/原始角）→ 使能/复位/设零（二次确认）→ 模式单选（MIT/位置/速度，复用 `/a3/motor/set_param` 0x7005）→ MIT 面板（`MotorParamField` 滑条：位置 ±12.57、速度 ±50、kp 0–500、kd 0–5、力矩 ±6；**ROS 侧定时保持**——前端只发目标+时长，`motor_protocol_node` 内部定时器按 hz 发帧、超时自动停，另提供单发与 `motor_stop` 卸力）→ 该节点话题/信号下拉 + `RealtimePerSignalChart` 实时曲线（默认 `mp/mtq/temp_L{n}`）。ROS 侧补齐：类型化逐电机遥测 `/a3/motor/states`（`MotorStates` 包装消息 50 Hz，7 电机 temp/err/mode/online/原始 MIT 角/力矩）；**互锁**——`gate_open=true`（电源序列 Running）时拒绝使能/复位/设零/MIT/参数/模式写入（扫描、读类、`motor_stop` 不受限），gate 打开瞬间 MIT 保持自动取消；仿真 `sim_motor_node` 同步补齐新话题/服务（sim 不实现互锁，有意分歧）。MQTT bridge 新增 9 个电机级 op + `motor_state` flatten（42 个新 telemetry 点），设备 YAML `HOME-DEMO.RK3588.yaml` 同步 points 与 nodes 目录。
- **验收标准：**
  1. `/a3/motor/states` 以 50 Hz 稳定发布 7 条 `MotorState`；`ros2 topic hz` 达标；无 CAN 时 `fresh=false`、单电机台架时仅对应 ID `fresh=true`
  2. `/a3/motor/mit_command`：`hold_duration_s<=0` 单发一帧；`>0` 按 `hold_hz` 定时发帧、到期自动停、新保持替换旧保持；`/a3/motor/stop` 立即取消保持并发卸力帧（kp=kd=t=0）；gate 打开瞬间保持自动取消并 WARN
  3. 互锁：`gate_open=true` 时 enable/reset/set_zero/MIT/set_param/set_mode 服务返回拒绝文案；scan/scan_and_collect/stop/get_device_id/request_version 照常可用
  4. `/a3/motor/scan_and_collect` 在 timeout_s（默认 1.5 s）内返回 `ids/uids`；busy 时拒绝重复扫描；`/a3/motor/scan`（F19/F22 兼容）行为不变
  5. MQTT：9 个电机 op 按契约返回 `cmd_result`（成功/参数非法/gate 拒绝三种文案）；telemetry 出现 `temp/err/mode/online/mtq/mp_L1..L7` 共 42 点且均为合法 JSON（无 NaN，F31 兜底回归）
  6. `./scripts/a3_test/a3_test.sh motor_debug` 仿真闭环全 PASS；前端在 `edge_web_sim` 下扫描→选择→保持→曲线全链路可用，`vite build` 通过；F22/F23/F26/F31 回归不受影响
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（电机调试节）；[shared/SAFETY.md](../shared/SAFETY.md)（MOTOR_DEBUG 互锁）；[QUICKSTART.md](QUICKSTART.md)（电机调试页节）；F17（EL05 协议命令集）、F19（总线扫描）、F22（单电机回归基座）、F23/F27（MQTT 面板模式）、F31（telemetry NaN 兜底）；外部前端仓库 `deep-trace`（分支 `rk3588`，`Rk3588MotorPanel.vue` / `DeviceModuleView.vue` / `registry.js` / `Rk3588HubPanel.vue` / `HOME-DEMO.RK3588.yaml`；需求文档 REQ-IOT-311）
- **状态：** `in_progress`

### F33 — 夹爪力控真机行为修正（接触位移门 / 默认超时 / 释放柔顺 / 零力矩直开 / 力环目标位置遥测）

- **说明：** 用户真机实测（2026-09-06，泡棉）发现 4 个问题并逐一修正：
  1. **目标 0.3 Nm 实际仅 0.13 Nm、卡在 42% 开合**：两个叠加根因——(a) 0 位硬止位静置力矩 ≈0.16 Nm ≥ 接触阈值 max(0.1, 0.35×0.3)=0.105，力控起步即误判「已接触」，跳过恒速软闭合段，PI 从 e≈0.14 缓慢爬升；(b) web 力控按键不传 timeout，落到默认 `grasp_timeout_s: 5.0`，「grasp timeout > 5.0s」FAULT 冻结力环（LL-013）。修正：新增 `contact_min_travel_rad: 0.1` 接触判定位移门（离开力控起点 ≥0.1 rad 后力矩阈值判定才生效）；默认 `grasp_timeout_s` 5.0→15.0。
  2. **释放过快**（旧 1.08→0 rad 用位置增益 80/2 + 0.8s 轨迹）：新增释放柔顺参数 `release_duration_s: 1.5`、`release_kp: 30.0`、`release_kd: 5.0`，释放轨迹用专用低增益 + 较高阻尼，轨迹结束 +0.3s 后一次性 timer 自动恢复位置增益 80/2（重复释放先取消旧 timer）。
  3. **缺零力矩硬逻辑**：force 模式 `torque_nm<=0 且 preset 为空` → 直接 `_release()` 全开，跳过 PI 与力矩校验（返回 `"force 0 -> full open"`）；`_tick_force` 加安全网：`_target_torque<=0` 立即转释放。注意：此改动使「force 无参数用默认 0.6 Nm」路径失效（web 永远显式传 torque>0），按用户规格接受。
  4. **力控期间目标位置遥测**：`_tick_force` 每 tick 把 PI 输出更新到 `target_position`（`_target_pos_commanded=True`），前端力控期间可观察目标位置逐步变大的趋势。
- **验收标准：**
  1. sim 闭环（`./scripts/a3_test/a3_test.sh gripper`）回归全 PASS（该套件自身用 `-p grasp_timeout_s:=6.0` 覆盖，不受 15s 默认影响）
  2. 真机（泡棉）：目标 0.3 Nm 时力环持续推进，实际力矩稳定在 0.3±10% 且 GRASPED；0.5 Nm 同样
  3. 真机：force 0 → 3s 内夹爪回 0 位（归一化 position→1.0），状态非 FAULT，释放无冲击（峰值速度明显低于旧版）
  4. 释放后 gains 在 `release_duration_s + 0.3s` 内恢复 position_mode_kp/kd
  5. 力控期间 `gripper_status.target_position` 随 PI 输出实时变化（不再停留命令陈旧值）
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（GripperStatus 字段语义）；[lessons_learned/LL-013](../lessons_learned/LL-013-gripper-hardstop-torque-false-contact.md)；F24/F25/F26（力控基线）、F31（target_position 字段）、F34（web 路径阶梯验收）
- **状态：** `completed`（2026-09-06 真机 F34 阶梯全 PASS：0.3Nm actual=0.281、0.5Nm actual=0.508 均 GRASPED；力控期间 target_position 随 PI 输出逐步变大；force 0 硬逻辑 3s 内回 0 位无 FAULT；释放柔顺生效。重抓超硬限问题经位移门基准改 q_open 修复；期间暴露的 F32 插值流缺陷由电机调试会话修复（LL-014/LL-015））

### F34 — web 路径力控阶梯验收（MQTT 模拟按键 0.3 → 0.5 → 0）

- **说明：** 新增 `scripts/a3_test/mqtt_force_ladder_test.py`：走生产 MQTT 路径（broker `bluemac.local:1883`、前缀 `deep-trace/HOME-DEMO/RK3588`，下行 `.../cmd`、回执 `.../cmd_result`、订阅 `.../telemetry` 拿 grip_* 信号）模拟 web 按键序列：`gripper_release` → `gripper_grasp {torque:0.3, timeout:20}` → `gripper_grasp {torque:0.5, timeout:20}` → `gripper_grasp {torque:0}`；每步断言 GRASPED、actual ∈ 目标 ±15%、force 0 后 3s 内 position→1.0。`scripts/a3_test/a3_test.sh` 新增 `force_web` 阶段（真机 + 生产 bridge，不进 `all`）；`scripts/a3_test/README.md` 补前置说明（泡棉在夹爪中、生产 bridge 在线、电机使能、gate 关）。
- **验收标准：**
  1. 真机运行 `./scripts/a3_test/a3_test.sh force_web`：0.3/0.5 两步均 GRASPED 且实际力矩在目标 ±15% 内，采样记录 grip_target_position 逐步变大趋势
  2. force 0 步 3s 内 grip_position→1.0（0 位），无 FAULT
  3. 测试结束电机温度正常（<50°C），夹爪停在 0 位
- **关联：** F33（本测试的验证目标）；F26/F27（MQTT 桥契约）；[QUICKSTART.md](QUICKSTART.md)（测试节）
- **状态：** `completed`（2026-09-06 真机验收 6 PASS / 0 FAIL：0.3Nm actual=0.281、0.5Nm actual=0.508 均 GRASPED 且在 ±15% 内；force 0 硬逻辑 3s 内回 0 位（grip_position 0.9988）；测试结束电机 temp=44°C、mode=2、err=0）

### F35 — MQTT 遥测降采样 5 Hz（全局）

- **说明：** 桥接节点按源话题速率发布全量遥测（叠加最坏 ~140–180 Hz，每条为 ~60 点 JSON），前端浏览器处理不过来，且 cmd（QoS 1）PUBACK 5 s 超时。改造 `a3_mqtt_bridge`：新增全局配置 `telemetry_min_interval_sec`（默认 0.2 = 5 Hz 上限），各话题回调只更新 `_points_cache` 置脏，由 0.05 s flusher 定时统一发布完整 points（最新值胜出）；发布路径解耦为有界队列（256）+ 独立发布线程，JSON 串行化与 paho publish 全部移出单线程 executor——慢 socket 不再阻塞遥测回调、cmd 分发与服务回执；telemetry 队列满可丢（最新值胜出），`cmd_result`/`device/status`/`device/info` 永不丢、不受节流；paho 线程内发布点（on_connect info、未知 op 回执）经队列后天然线程安全。
- **验收标准：**
  1. 生产栈遥测实测 ≤5 Hz（10 s 计数 ≈50 条），points 仍为全量聚合
  2. `a3_test.sh mqtt_cmd`（含 6 个 gripper op）与 `telemetry` 套件仍 PASS；cmd 下发后 `cmd_result` ~1 s 内到达
  3. web 端 cmd 发布（QoS 1）不再触发 5 s PUBACK 超时（前端零改动）
  4. broker 断连/慢 socket 期间 executor 不卡死：状态/回执重连后正常恢复
- **关联：** F18（MQTT 遥测桥）、F23（cmd 契约）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（遥测速率契约）；`a3_mqtt_bridge/config/bridge.yaml`
- **状态：** `implemented`（2026-09-07 真机实测：连续运动期 12 s 计 55 条 ≈4.6 Hz、间隔中位 0.232 s ≈5 Hz 上限；`a3_test.sh mqtt_cmd` 18/18、`telemetry` 6/6 PASS，cmd_result 均 <1 s 回达；顺带修复 SIGINT 退出竞态 exit code 1，见 LL-016。web 端 PUBACK 不再超时由前端观察复验）

### F36 — PS4 R2 扳机力控夹爪

- **说明：** 改造 PS4 映射（`simple` 与 `default` 两份）：R2 从「夹爪位置模拟量」升级为「扳机力控」——新 action `gripper_force`（`a3_teleop_ps4/actions.py`，经 `/a3/gripper/command` 服务，`a3_msgs` 依赖新增）。语义：松开（v<0.15，迟滞下沿）→ 下降沿发一次 `release`（全开）；按过 0.22（迟滞上沿）→ `force`，扳机 0.2..1 线性映射目标力矩 **0.1..1.0 Nm**（下限 0.1 因目标 ≤0 触发全开硬逻辑且接触判定下限 0.1 Nm），按得越深抓得越紧；持按期间目标变化 ≥0.1 Nm 才重发（避免 50 Hz 重发把积分清零）；服务未就绪/互锁拒绝（臂运动中等）→ 0.5 s 间隔重试，松手即停；`timeout_s=15` 与节点默认一致，GRASPED 后持续持握。L2→L6 不动，急停键（Triangle/L1/R1/Share 等）不动；顺带把 `set_gripper`/`set_joint_L7`/`gripper_toggle` 的旧标定常量对齐 2026-09-06 标定（open=0/close=1.79，修复 Square/Circle 开合反向）。
- **验收标准：**
  1. `simple.yaml`/`default.yaml` 经 `validate_mapping` 校验通过，mapper 正常启动
  2. sim 注入假 `/joy`（axes[5] 扫描 0→1→0）：收到 force 且目标力矩随扳机单调递增，下沿只发一次 release
  3. 真机（泡棉）：按住 R2 → GRASPED 且实际力矩 ≈ 映射值；松手 → 3 s 内回 0 位全开；扳机深浅变化抓力跟随
  4. 手臂运动中按 R2：命令被拒（互锁）且 0.5 s 重试、松手停止；急停按键行为不变
- **关联：** F24（力矩上限 1.0 Nm，映射上限对齐）、F26（GripperCommand 服务）、F33（force 0 全开/互锁）；[shared/SAFETY.md](../shared/SAFETY.md)（R2 力控安全）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（扳机契约）
- **状态：** `implemented`（2026-09-07 真机验收：`simple`/`default` 双映射 validate_mapping 通过；泡棉手柄实测 R2 深浅 → 目标力矩 0.3→1.0 Nm 跟随（含持按期间深浅双向调制）、GRASPED 实际力矩 ±2% 内（1.00→1.01、0.96→0.97、0.78→0.79）、每次松手下降沿单次 release 全开、全程 error_code=0；验收项 4（臂运动互锁重试）在单电机台架（仅 ID7）无法触发臂运动，代码路径由 sim 互锁检查覆盖）

### F37 — Web 夹爪面板：停止=失能、使能/设置零位按钮、4 曲线合并单图双轴（跨仓）

- **说明：** web 测试反馈三项改造（前端在外部仓 `/home/cat/deep-trace` 分支 `rk3588`，ROS 侧**零代码改动**）：
  1. **停止按钮修复**：原「停止」只发 `gripper_stop`——仅停 Python 力环，C++ refresh keeper 仍以当前增益续推最后目标角（[LL-017](../lessons_learned/LL-017-mit-hold-end-refresh-kp80.md) 机理），夹爪继续挤压、表现为「停止无效」。现「停止」= 顺序下发 `gripper_stop`（停力控）+ `motor_reset {motor:7}`（失能 L7，固件 `0x04` 停止帧）；确认框提示夹持物会掉落。
  2. **新增按钮**：`使能` → `motor_enable {motor:7}`；`设置零位` → `motor_set_zero {motor:7}`（确认框要求全开硬止位：先使能 → 释放到头 → 设零位）。复用 F32 已登记的 `motor_*` MQTT op，gate 互锁仍由 C++ 权威拦截、拒绝文案经 `cmd_result` 回传。
  3. **曲线合并**：位置曲线 tab + 力矩曲线 tab（内含 2 图）合并为单图 4 曲线（`grip_target_torque`/`grip_actual_torque`/`grip_target_position`/`grip_position`），双 y 轴（左 Nm、右 0–1），图例点击隐藏；图表组件新增 `group_by_unit` 轴模式（同 unit 共轴、空 unit 共一根轴，严格增量不影响其他消费方）。顺带把面板 `HARD_MAX_TORQUE` 2.0→1.0 与 F24 硬上限对齐（否则滑块可设 1.x 被 ROS 拒）。
  4. **回执修复**：`ArmControlPanel` 回执 watcher 加 `gripper_*`/`motor_*` 守卫，消除 hub 页两面板同挂载时的夹爪回执双 toast（既有 bug，本批一并修）。
- **验收标准：**
  1. 真机停止序列：两条 ok 回执；`grip_actual_torque`→~0、L7 `mode_status`→0；≥30 s 无继续挤压（MP TX window `tx_refresh` 仍增长 = keeper 发帧但固件忽略）
  2. 使能 → 释放 → 设置零位流程：`mode_status`→2 夹爪原位无跳变；设零位后 `mp_L7`≈0、`grip_position`≈1.0
  3. 合并图：单图 4 曲线、左轴 Nm 右轴 0–1、图例点击显隐正常
  4. 回归：`a3_test.sh mqtt_cmd` 全 PASS（桥未动）；前端 `npm run build` 干净
- **关联：** F26/F27（夹爪指令与面板）、F31/F33（曲线与遥测）、F32（`motor_*` op 与 gate 互锁）、F24（1.0 Nm 硬上限对齐）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（停止语义小节）、[shared/SAFETY.md](../shared/SAFETY.md)（失能松脱/设零位规则）、[LL-017](../lessons_learned/LL-017-mit-hold-end-refresh-kp80.md)
- **状态：** `implemented`（2026-09-07 真机验收：① 停止序列 MQTT 实测两条 ok 回执（`gripper_stop`→"stop"、`motor_reset`→"ok (1 frame(s))"），L7 `mode_status` 2→0、力矩 0.19→0 Nm，30 s 观察力矩恒 ~0 且 MP TX window `tx_refresh`≈234/5s 持续发帧（keeper 发帧、固件忽略——LL-017 现象消除）；② `motor_enable` ok→`mode_status` 2，位置 0.387→0.394（亚 0.01 rad 无跳变），释放到硬止位后 `motor_set_zero` ok→`position_rad` 0.0006、`grip_position` 0.9997、error 0；③ 前端 `npm run build` 干净（exit 0），双轴分组与图例交互留待浏览器视觉确认；④ `a3_test.sh mqtt_cmd` 18/18 PASS（首跑 15/18 为真机桥与测试桥共享 MQTT cmd 话题串扰所致，停真机桥后全过——已补 QUICKSTART 提示））

### F38 — 通用 N 关节臂 init/示教/回放（URDF 无关）+ 回放首点插值 + 示教停止防弹回

- **说明：** can1 换接非 A3 机械臂（6 关节、无夹爪、URDF 不同）提前验证初始化/示教/回放流程，要求整套流程与 URDF 无关、任何 N 关节 MIT 臂可用。三处改动：
  1. **参数化 N 关节**：`arm_controller` 仅靠 `joint_names` 参数（新增 `arm_controller_6j.yaml`，去掉 L7_joint）；`a3_can_bridge` 新增 `control_gains_generic.yaml`——kp=40/kd=2（safety_limits 空载值）、`joint_cmd_min/max_rad` 放宽到 MIT 全量程 ±12.57（A3 限位会裁剪非 A3 臂目标）、`enable_startup_smoothing: false`（boot_feedback 是 set_zero 前旧值，播种会触发伪斜坡→真实运动）。init = set_zero 广播 + `/joint_states` 计数确认（`init_zero_tol_rad=0.05`）+ enable 广播，广播服务对缺席电机无感，天然适配 N 关节。
  2. **回放首点插值（arm_controller F38b）**：`_playback_cb` 发布前从当前位姿线性插值到首记录点，时长 `playback_ramp_duration_s`（默认 2.5 s，≤0.05 关闭），插值段前插、记录点时间戳整体平移，执行层 startup smoothing 锚定首次指令对回放不生效，故必须在编排层做。
  3. **示教停止防弹回（a3_can_bridge F38a）**：`zero_torque/stop` 恢复增益前把 `last_commanded_mit_rad_` 重锚定到 `last_feedback_mit_rad_`（同域 MIT 原始角）——零力矩期间 refresh 流按旧目标持续发帧，不重锚定会在恢复 kp 后把臂拉回示教前位姿。
- **验收标准：**
  1. 6 关节臂（can1，ID 1–6）init 成功：确认 `6/6`、6 关节位置读数均 |p|<0.05 rad；全程先 set_zero 后 enable
  2. zero_torque/stop 后臂保持释放位（1.5 s 采样 Δ<0.15 rad，无弹回）
  3. 示教 5 s 记录 ≥200 样本；保存后回放：首样本≈回放前位姿（无初始跳变）、前 2.5 s 插值段逐样本步长 <0.06 rad/20ms、插值结束≈首记录点、回放结束≈末记录点并保持末位
  4. MQTT/web 路径零改动：telemetry `pos_L1..L6` 实时可见
- **关联：** F17（MIT 协议命令集）、F22（分层回归测试）；[shared/SAFETY.md](../shared/SAFETY.md)（增益/限幅）；[QUICKSTART.md](QUICKSTART.md)（通用臂验证流程节）
- **状态：** `completed`（2026-09-11 真机验收：① init `6/6` 确认、6 关节 |p|<0.001 rad、先设零后使能；② zero_torque/stop 后 1.6s 采样无弹回（各关节漂移 <0.01 rad，无拉回旧目标）；③ 示教记录 2470 样本（50Hz）；④ 回放验证 `scripts/a3_test/arm6_playback_verify.py` 9/9 PASS：无初始跳变 0.0004 rad、ramp 期最大步长 0.052 rad/20ms、ramp 结束距首点 0.089 rad、结束距末点 0.053 rad、末位保持；⑤ MQTT telemetry pos_L1..L6 实时可见；三坑入 [LL-018](../lessons_learned/LL-018-generic-arm-bringup-three-pits.md)）

### F39 — 命名点位保存（用户层覆盖）+ 通用平滑移动指令 move_to

- **说明：** 通用臂流程需要「把当前位姿记为命名点」和「按指定时长平滑移动到任意目标位姿」两类指令（对应 web 已有 init/teach/playback 之上的点位语义）。三处改动：
  1. **`/a3/arm/save_named_pose`**（新 srv `a3_msgs/srv/SaveNamedPose`）：`name` + 可选 `positions`（留空 = 当前 `/joint_states` 位姿），写入用户层 `~/.a3/poses.yaml` 并**运行时即时生效**；`_load_poses` 改为「包内 named_poses.yaml → 用户层 poses.yaml 同名覆盖」两级合并，重启后保留。
  2. **`/a3/arm/move_to`**（新 srv `a3_msgs/srv/MoveToJointPositions`）：`positions` + `duration_s`（0.05–60s 钳制，默认 1s），当前位姿 → 目标位姿按 `goto_waypoints` 个点线性插值（复用 goto 插值模式），不做 URDF 限位 clamp（执行层 `joint_cmd_limits` 仍权威拦截）；互锁同 `_can_move`（INIT/TEACH/AI/TRAJ/SERVO 与 ZERO_TORQUE 等拒绝）。
  3. `_goto_cb` 顺带修复：命名点位长于 `joint_names` 数时裁剪尾部（7 关节包内点位用于 6 关节臂时不再隐式依赖 zip 截断）。
- **验收标准：**
  1. `save_named_pose` 保存当前位姿为 `ready`、全零为 `home` 后，`goto_named_pose` 立即能列出并执行（6 关节臂实测）
  2. `move_to` 以 5s 时长插值平滑到位：全程逐样本步长无阶跃（复用 arm6_playback_verify 步长断言）、到位误差 <0.15 rad
  3. `/a3/arm/disable` 后 6 电机 mode_status=0（失能）
  4. 全流程：任意位姿 → move_to home(5s) → move_to ready(5s) → move_to home(5s) → disable
- **关联：** F23（set_joint_positions 滑动条 jog，保持 URDF clamp 语义不变）、F38（通用臂流程）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（arm 服务表）；[QUICKSTART.md](QUICKSTART.md)（通用臂验证流程节）
- **状态：** `implemented`（2026-09-11 通用 6 关节臂真机验证中）

### F47 — 0x7029 zero_sta 参数读写协议（断电多圈窗口选择）

- **说明：** 真机 7 关节臂实测：断电后手动转动关节再上电，多圈计数在默认 zero_sta=0（0~2π 重建）下会环绕——负向转 ~22° 的 L6 上电读 +5.8994 rad（=+338°，+2π 环绕），用户担心的「-10° 变 350°」真实发生；环绕读数超关节限位时严禁使能（kp×误差会瞬间猛拉）。固件参数 0x7029 `zero_sta`（uint8，1 字节，默认 0）选择上电位置重建窗口：0=0~2π，1=-π~π。置 1 后 ±180° 内的断电转动可被正确记录（L2/L3 行程超 π，超 ±180° 的大转动仍会环绕）。本需求为 a3_can_bridge 新增三个协议能力（均为 EL05 通信类型，单电机或广播）：
  1. **`/a3/motor/get_param`**（新 srv `GetMotorParam`）：通信类型 17（0x11）读参数——发请求后收集应答直至超时（默认 0.4 s），`motor_id=0` 时广播到全臂映射电机并聚合；应答 data[4] 为 u8 值、data[4..7] 为 float32 小端，两数组同时返回（uint8 参数看 `values_u8`，float 参数看 `values_f32`）。
  2. **`/a3/motor/set_param_u8`**（新 srv `SetMotorParamU8`）：通信类型 18（0x12）的 uint8 形式（值写 data[4]），`motor_id=0` 广播；float 参数继续用既有 `/a3/motor/set_param`。
  3. **`/a3/motor/save_param`**（MotorCommand `command=5`）：通信类型 22（0x16）保存参数到 flash，0=广播。
  读操作不受 power-sequence gate 互锁（与 get_device_id/request_version 同语义）；set/save 为写操作，gate 打开时拒绝。
- **验收标准：**
  1. 上电后 `get_param`（motor_id=0, param_id=0x7029）读到 7 关节 `values_u8` 全为 0（出厂默认）
  2. `set_param_u8` 广播置 1 → `save_param` 广播 → 断电重启后 `get_param` 读回全为 1（flash 持久）
  3. zero_sta=1 后断电手动转动各关节 ±（<180°）再上电：probe 读数与转动方向/幅度一致，负向转动不再 +2π 环绕（L6 类场景回归）
  4. 读参数无应答时返回 `success=false` + `"no response (timeout)"`；忙时拒绝（"get_param busy"）
- **关联：** F32（扫描收集 DeferResponse 模式，复用同一实现路径）；[QUICKSTART.md](QUICKSTART.md)（真机断电记忆验证流程）；[lessons_learned/](../lessons_learned/)（断电多圈环绕 LL 条目）
- **状态：** `implemented`（2026-09-13 真机 7 关节臂验证通过：2/4/5/6/7 读回=1 且跨断电保留；断电负转后 L5/L6/L7/L1/L3 读数全部连续——L5=-0.8361、L6=-1.6068、L1=-0.6055、L3=-0.5200 不再环绕（L3 旧固件行为验证通过，无需升级）；L1/L3 旧固件 0.0.3.4 读回恒 0 但写+保存生效。save 帧数据域须 `01 02 03 04 05 06 07 08`，全零不触发保存，见 LL-019）

## F48 开机零位校验（读数限位硬检查 + 期望位姿软检查 + 使能门禁）

- **说明：** F47 之后正常断电（断电间不超 ±180° 转动）读数跨上电保持连续，但环绕（+2π 多圈推算）仍可能发生（L2/L3 行程超 π；zero_sta 丢失、超范围转动等异常）。上电后校验通过前**不使能**。本需求两层：
  1. **编排层使能门禁**（arm_controller）：`/a3/arm/enable` 发 enable 前检查 7 关节读数全部落在 URDF 限位内**且 /joint_states 新鲜**（stamp 距今 ≤ `js_max_stale_s: 1.0`，参数 `enable_position_check: true` 默认开启；未收到 /joint_states 也拒绝）——环绕读数超限时 kp×误差会瞬间猛拉（LL-019），越限拒绝并点名关节与限位，恢复零位走 `/a3/arm/init`。`init` 是恢复路径（set_zero 重建零位帧后再使能），越限只 WARN 不阻断——WARN 指明「当前位姿将被定义为零位，仅在已知位姿（工装摆 URDF 零位）执行」。
  2. **独立开机校验脚本** `scripts/a3_check_zero_frame.py`：读 /joint_states 对照 URDF 限位（硬检查，任一越限、消息陈旧（>1 s）或超时无消息 exit 1——判断是否环绕的唯一标准）；再对照期望位姿模板软检查（L2/L3/L5/L6/L7 ≈ 0、L4 ≈ 0（URDF 零位）或 ≈ 0.34（折叠自然下垂）、L1 自由——水平转动 ±178° 由机械限位约束；默认容差 0.35 rad ≈ ±20°，仅提示性、不改变退出码）。
  3. **桥保活播种（执行层配套，三态语义，LL-020/LL-022）**：refresh 流在无指令历史（last_commanded 为 NaN）时按电机模式播种：①模式未知/失能 → 零增益保活帧（p=反馈位或 0，kp=kd=tau=0）；②已知使能且反馈新鲜 → 目标一次性锚定到最新反馈位，以 runtime kp/kd/tau 真实保位（服务直驱使能后无轨迹流的掉臂修复，LL-022 真机实测）；③已知使能但反馈陈旧 → 不盲发。保证开机/使能后总线有帧流、反馈持续上送——否则 js 冻结旧值（LL-020），或已使能电机静默无力矩（LL-022）。
- **验收标准：**
  1. 正常上电（读数在限位内）→ `/a3/arm/enable` 放行进入电机使能流程
  2. 人为构造越限读数（仿真注入 L6=5.9）→ enable 拒绝，message 点名关节与限位；init 放行但 WARN
  3. 未收到 /joint_states（桥未起）→ enable 拒绝并提示
  4. 脚本：真机正常位 → 硬检查 PASS exit 0（软检查打印位姿匹配结论）；越限注入 → FAIL exit 1
  5. 桥重启后无任何指令下发：5 s 内 /joint_states 持续新鲜发布且读数跟踪手动拖动（播种生效）；停桥后脚本报 stale FAIL
  6. 服务直驱使能（/a3/motor/enable）后无轨迹下发：已使能关节被锚定保位（runtime 增益），/a3/motor/states 全关节 fresh=true、tx_stats.tx_hz≈refresh 频率×7；02 反馈帧扩展 ID bit22-23 解析 mode=2（LL-022 真机验证）
- **关联：** F47（zero_sta 窗口，本需求是其开机侧兜底）；[lessons_learned/](../lessons_learned/)（环绕机理与「使能前必须 probe 校验」红线）；F45（状态机使能路径）
- **状态：** `implemented`（2026-09-13 真机验证：零位帧恢复后 7/7≈0，脚本硬检查 PASS exit 0 且软检查匹配「URDF 零位」；隔离域仿真验证 enable 门禁三场景——无 /joint_states 拒绝、L6=5.9 越限拒绝并点名、限内放行；init 越限 WARN 放行。限位比较需裕量（编码器量化噪声 ±0.0002，L2/L3/L7 下界为 0），参数 `position_check_margin_rad: 0.001`。补充：真机发现桥无指令历史时 js 冻结旧值（LL-020）——新增 refresh 零增益播种 + 脚本/门禁 stamp 新鲜度检查，软检查容差按用户反馈放宽至 ±20°）

## F49 重力标定（7J 真机 inertia_params 重标定，复刻官方 dynamics_calibration）

- **说明：** 现有 `inertia_params.yaml` 是装夹爪前 6J 臂的官方标定数据（home 位 L3 模型预测 ≈2.9 Nm vs 实测 ≈0.55 Nm），零力矩/重力补偿不可用（LL-025 推飞事故的残留前置条件）。复刻官方 `dynamics_calibration.py` 做 7J 臂重标定：`scripts/gravity_calibration.py` 在本栈接口上实现——FJT action（/arm_controller/follow_joint_trajectory）下发 2 点轨迹（当前→目标，LL-015 单点轨迹坑），/joint_states（双订阅 BEST_EFFORT+RELIABLE，兼容真机/仿真两端 QoS，LL-030）读位姿与反馈力矩（Nm，LL-024 codec 已按型号校正），/a3/arm_status（state 作 READY 门禁 + temperatures 作温度守护）。采样位姿网格复用官方：基位 [0, 0.785, -0.785, 0.5, 0.5, 0]，L2×10、L3×10、L2×L3 网格 5×5、L4×L5 网格 6×5（去重+碰撞过滤后 ~67 点）+ rest 折叠位与 --anchors 锚点位追加（网格外姿态的 τ 外推锚定，服务 home/ready 验收）；每点 1.5 s 稳定 + 40 次×20 ms 采样均值。**安全适配（本仓红线）**：所有移动 ≤0.25 rad/步链式分段（3 s/段，F41 兜底之上）；碰撞过滤复用官方判据；每点移动前用当前模型预测 τ_g，任一关节 >3.5 Nm 跳过该点（网格最坏预测 L3 2.86 Nm、EL05 关节 ≤0.36 Nm——3.5 阈值不误跳且有裕量，F42 限 RS00 5/EL05 3）；L3 温度超 `--temp-pause`（默认 **85°C**，2026-09-14 由 75 上调——关节外壳是 3D 打印件（ABS/ASA/PC）且直接固定电机、散热差，85 仍留 5°C 余量给 F44 warn 90/protect 95，远低于电机自保护 130°C）自动回折叠 home 等降至 `--temp-resume`（默认 **75°C**）再续采（被动降温实测 ~2°C/min，单次 ~5 min）；JSONL 增量落盘 `~/.a3/calibration/calibration_data.jsonl`（每点 fsync，断点续采/--start/--restart/--optimize-only/--data-file）。拟合：12 参数（L2–L6 质量+质心主分量，夹爪质量吸收进 L5/L6——重力补偿只需要模型正确，不需逐杆物理真实）L-BFGS-B 有界优化（**实测 effort 是电机域，拟合前 ×`joint_signs` 换回 URDF 域（LL-037，不换算 RMSE 1.66 vs 0.16）；初值取本臂 URDF 现值、边界放宽到质量 [0.01,2.0]/质心 ±0.25——官方边界把 URDF 本值排除在外，最优解顶边界（LL-037）**；--fix-masses 为官方 8 参数 COM-only 分支；`--effort-domain motor|urdf` 区分真机/仿真，并随数据文件 meta 留档），**按各关节下方杆改写 inertia（官方 inertias[2..6] 同下标，LL-033 映射错位已修）**，输出 `a3_description/config/inertia_params.yaml`（官方格式，备份旧文件）。L7 全程 0（全开）；L1 恒 0（竖直轴自身重力矩为 0）。**零力矩实测协议（LL-025）**：先离线验证 τ_g 随姿态变化 + home 位模型≈实测（RMSE 目标 ≤0.15 Nm），再由用户托臂从 home 启动零力矩——臂应悬浮无推力、轻拖失重、stop 干净。
- **验收标准：**
  1. 仿真冒烟：quick 模式全流程（移动/采样/JSONL/拟合/输出 yaml）机制跑通
  2. 真机采集 ≥50 有效点（跳过点记录原因），L3 温度全程 <80°C，无 F42 trip（WARN 0 条）
  3. 拟合输出：RMSE ≤0.15 Nm（官方 6J 数据 0.10 Nm 同量级）；free 位姿（ready）模型 vs 实测 |Δτ| ≤0.2 Nm
     - **2026-09-14 实测偏差**：RMSE 0.1592（超 6%）；ready 位 max|Δτ| 0.302 Nm（L3，超 50%），L2 0.054 / L4 0.093 / L5 0.047 / L6 0.019。**已证明是本臂硬件地板**（见 LL-038）：20 参数全一阶矩拟合 0.158 ≈ 12 参数 0.159，再放开腕部关节原点几何 0.159 仍不降，而同姿态跨会话/跨方向重复测量只差 0.03 Nm（残差可复现、非噪声）→ 残差不属刚体模型可表示的部分，属传动/打印件柔性/线束等非刚性效应。**home 位（折叠支撑位）不可用于该项验收**——臂搭在支撑上 τ 恒≈0（LL-035），验收位姿取 ready。
     - **相较替换**：同批数据旧官方 6J 参数 RMSE 0.2036、ready |Δτ| 0.403 → 新参数 0.1592 / 0.302（URDF 默认更差：ready RMSE 0.705）。**新参数在所有已测指标上优于旧参数与 URDF 默认**，故按「更优即采用」落地；验收线差额按硬件地板记录，是否放宽验收线待用户签署
  4. 重力节点重启后日志 `Applied calibrated inertia to 5 links`，τ_g 随姿态变化（LL-025 验证协议）；零力矩实测：托臂启动无推力、轻拖失重感、无越限
- **关联：** LL-025（零力矩推飞事故与验证协议）；LL-024（力矩 codec 量程）；LL-030（BEST_EFFORT js）；LL-015（单点轨迹）；LL-033（杆映射错位修复）；F42/F44（力矩/温度保护，采集期间依赖）；F50（采集期间的跟随误差看门狗）
- **状态：** `implemented`（2026-09-13 脚本完成 + 杆映射修复（LL-033）+ 离线合成数据拟合冒烟 PASS：rmse 0.0193、home/ready/伸展外推 ≤0.014 Nm；域 55 仿真采集冒烟 20/20 + 续采路径 PASS。**2026-09-14 真机标定完成**：full 模式 68 点（一次链式长轨迹失败跳过 1 点），两次降温守护（85/75 阈值前的 75/60），L3 全程 ≤77°C、F42 trip 0 条、F50 无处置动作（1 次状态抖动，见 LL-034/F50 记录）；拟合 RMSE 0.1592 / R² 0.9835，yaml 已更新（`inertia_params.official-6J-20260318.bak.yaml` 留旧版），重力节点重启日志 `Applied calibrated inertia to 5 links` 且发布 τ 与离线模型逐位一致（差 0.0 Nm）；ready 位真机复测 max|Δτ| 0.302 Nm（L3）。**未做**：零力矩实测（协议第 4 条，需用户托臂在场）、F50 真机长跑观察。验收线差额按 LL-038 硬件地板记录待签署）

## F50 故障监视看门狗（arm_monitor：期望 vs 实际偏差 → 升级处置）

- **说明：** 补「运行中跨数据源比对」缺口。分层保护原则下**现有保护保留原位不动**——F42（执行层 200 Hz 力矩方向钳位，撞堵立即冻结）在 C++ 执行层（实时性要求，Python 20 Hz 达不到）、F44/F40（温度/失能保护）在编排层状态机（状态转移权责，两节点抢状态会出锁存/竞态，LL-009 教训）；monitor 定位为**独立看门狗**，只做需要跨源比对的检测并通过公开服务动作（/a3/motor/stop、/a3/motor/reset、/a3/arm/disable），不直接改任何节点内部状态。订阅 /joint_states（双 QoS BE+RELIABLE，LL-030）、/a3/arm_status、/a3/motor/states、轨迹话题 /joint_group_effort_controller/joint_trajectory，20 Hz 节拍。**期望位置不依赖 ArmStatus.positions**（目标快照语义、运动期含 transient）——自建轨迹插值器：收到 JointTrajectory 记 (t0, points)，按 time_from_start 线性插值（与执行层一致）出 q_d(t)；运动窗口 = [t0, t_end+2 s]；窗口关闭后以轨迹末点作保持位（LL-009：control_mode 只发不回收，不能用 mode 判运动期，用自建窗口）。
- **故障类（v1）：**
  1. FOLLOW_STUCK：运动窗口内 max|q_d−q_actual| > 0.25 rad 持续 0.5 s → stop；3 s 未消 → 升级 reset
  2. HOLD_DRIFT：state=READY 保持期（窗口已关）末点 vs 实际 > 0.30 rad 持续 1.0 s → stop → 升级 reset
  3. STALE_JS：js stamp 过期 > 1.0 s 且 state ∈ {READY, TRAJ, SAFE_PARK, SERVO, TEACH, AI} → reset（反馈死亡，无法验证，最安全处置）
  4. UNEXPECTED_DISABLE：state ∈ {READY, TRAJ, SAFE_PARK} 但 MotorStates 任一电机 enabled=false 持续 **0.5 s**（2026-09-14 由 1.0 s 下调，**必须短于编排层本地兜底 1.0 s**，否则编排层先转 DISABLED 会让本节点持续窗清零、真故障永远报不出来——F51）→ **仅报告**（电机已失能，不动作——避免 F40 park 在失能电机上失败进 FAULT）
  5. TEMP_UNRESPONSIVE：ArmStatus.temperatures ≥95°C 且 state ∉ {COOLING, SAFE_PARK, FAULT} 持续 3 s → reset（F44 失灵的兜底）
- **抑制（防误报，LL-020 教训）：** mode ∈ {ZERO_TORQUE, GRAVITY_COMP} 跳过 1/2（零力矩臂悬浮是设计行为）；state ∈ {IDLE, INIT, DISABLED, COOLING, FAULT} 跳过 1/2/3（失能期 js 冻结合法）；无 MotorStates 数据跳过 4（仿真早期/桥未起）；启动 3 s 宽限（订阅发现）；触发后 5 s cooldown 防刷屏；条件消失 + 2 s clear_hold 回 OK。
- **输出：** /a3/monitor/status（msg a3_msgs/MonitorStatus：status/**pending_faults**/fault/action/tracking_errors/max_tracking_error/last_event，20 Hz）+ WARN/ERROR 日志；MQTT 上行二期。launch：arm_controller.launch.py 追加 `enable_monitor:=true`（默认开，可用 `enable_monitor:=false` 关）。**status 语义（F51 起）**：OK / PENDING（条件已成立但未达持续阈值，瞬时抖动，不动作）/ TRIGGERED（已确认），`fault` 只表示已确认故障。
- **模式锁存守卫（轻量告警·只告警不动作，2026-09 增）**：`control_mode` 停留 `TRAJ_RUNNING` 且看门狗自建轨迹窗口已关闭（`_traj=None`）持续 `mode_stuck_s`（默认 3 s）→ WARN 日志 + `last_event=MODE_LATCHED`，不进入 `status/fault`、不动作——F29 类「轨迹结束未回收模式」回归检测（编排层中途崩溃 / 漏发 READY/IDLE 时，夹爪力控、FJT、新轨迹会被互锁全部拒绝的早期发现）。SERVO/ZERO_TORQUE/GRAVITY_COMP 各有超时或显式启停语义，不检查。参数 `mode_stuck_s` / `mode_stuck_cooldown_s`（arm_monitor.yaml）。
- **验收标准：**
  1. 仿真：健康 move_to 全程零触发（无误报）
  2. 仿真：SIGSTOP sim_motor → FOLLOW_STUCK → /a3/motor/stop 被调用（服务返回 success）
  3. 仿真：停 sim_motor → STALE_JS → /a3/motor/reset 被调用
  4. 仿真：ZERO_TORQUE 模式下注入轨迹不触发；失能态 js 冻结不触发（抑制生效）
  5. 仿真：sim 电机 reset 而 state 仍 READY → UNEXPECTED_DISABLE 事件上报
  6. 真机：挂载观察（与 F49 真机采集同场），零误报；阈值按真机跟踪误差微调
- **关联：** F42/F44/F40（保留原位的分层保护）；LL-030（js QoS）；LL-020（js 冻结判据）；LL-009（control_mode 语义）；F49（重力标定采集期间依赖本看门狗）
- **状态：** `implemented`（2026-09-13 仿真验收通过：健康 move_to 零触发 max err 0.0004 rad；斜坡注入 → FOLLOW_STUCK → stop → +3 s 升级 reset；SIGSTOP 冻结 → STALE_JS → reset；sim reset 而 READY → UNEXPECTED_DISABLE 报告；ZERO_TORQUE 抑制生效。5 次触发全为注入诱导、零误报，验收标准 1–5 全过。实现踩坑 LL-034：回调内 sync 服务死锁 + 时间纪元混用。**真机观察（验收 6）进行中**：2026-09-14 F49 采集全程（约 1 h，68 点链式轨迹 + 2 次降温）fault **动作**零次（日志无任何 `[monitor] <fault>: success=...` 行 ⇒ 无 stop/reset），仅 19:32:02 出现一次 `FOLLOW_STUCK recovered -> OK` 的**状态抖动**。**已知语义**：MonitorStatus 的 status/fault 曾反映**瞬时**条件（`active` 非空即置 `_fault`），动作才要求持续过 sustain（0.25 rad/0.5 s）；故该抖动未触发任何动作，但 status 通道会短暂显示 FOLLOW_STUCK——对消费 status 的上层（web/MQTT）是噪声。**2026-09-14 已按 F51 第 6 条改为「确认后置位」**（status 分 OK/PENDING/TRIGGERED，`fault` 只表示已确认，瞬时量见 `pending_faults` 与 tracking_errors）——该抖动今后显示为 PENDING 而非 TRIGGERED。真机长跑（≥30 min 含 move_to/goto/零力矩）与阈值微调仍未做）

## F51 使能安全门禁（三层修复：保持抑制 latch / 使能重锚 / 意图边界重基准）

- **说明：** LL-039 真机事故（2026-09-14，甩断 L6 打印关节）的三层修复。**执行层**（`motor_protocol_node.cpp`）：(1) `hold_suppressed_[motor_id]` **保持抑制 latch**——`/a3/motor/stop` 与 reset/set_zero 置位，refresh 播种分支从此只发**零增益保活帧**（p=反馈位、kp=kd=τ=0），停止后不得再把目标锚回旧位姿；显式新意图才解除（新轨迹 / MIT 直驱 / 零力矩退出 / 使能 / park）。(2) **使能 = 保当前位置**：`command==1` 逐电机校验反馈 finite + 新鲜（`require_fresh_feedback_on_enable`），任一缺失/陈旧则**整体拒绝使能**（`success=false`，不发任何帧）；通过则把 MIT 目标无条件重锚到反馈位，陈旧目标丢弃并 WARN，丢弃距离回传在响应里（`F51 重锚 N 电机…最大丢弃目标距离 X rad`）。(3) **使能软起步**：`enable_kp_ramp`（默认 on）+ `enable_ramp_duration_s`（0.8 s），使能后 kp/kd 从 0 线性升到额定（τ_ff 重力前馈不受影响）。(4) 失能态下 `|目标−反馈| > enable_reanchor_tolerance_rad`(0.15) 时限频 WARN（事故时该差值 1.956 rad 静默保留 2 分钟）。**看门狗**（`arm_monitor_node.py`）：(5) `_maybe_rebaseline()` 在「意图边界」（control_mode 转出 ZERO_TORQUE/GRAVITY_COMP、整臂 none→all 使能沿）把 `_last_goal ← 当前实际位姿`、清 `_traj`、`hold_rebaseline_grace_s`(2.0 s) 宽限——示教拖动改写实际位姿后执行层 F38 会重锚目标，看门狗参照必须同步；(6) `MonitorStatus` status 分 OK/**PENDING**/TRIGGERED + 新增 `pending_faults`，`fault` 只表示**已确认**（持续达阈值）——消除 2026-09-14 记录的状态抖动噪声（见 F50 状态栏）；(7) `unexpected_disable_sustain_s` 0.5 s **必须短于编排层本地兜底 1.0 s**，否则编排层先转 DISABLED 会让看门狗持续窗清零、真故障永远报不出来。**编排层**（`arm_controller.py`）：(8) 消费 `MonitorStatus`（只认 TRIGGERED，PENDING 不动作）→ READY/TRAJ 下转 DISABLED；另有**不依赖看门狗在跑**的本地兜底（fresh 电机全部报关闭持续 `unexpected_disable_sustain_s` 1.0 s）。参数：`monitor_status_topic`、`unexpected_disable_guard`、`unexpected_disable_sustain_s`。
- **验收标准：**
  1. 事故回归 `./scripts/a3_test/a3_test.sh incident` 两段全绿：执行层（mock 电机，不经 SocketCAN）stop 后零增益保活、使能重锚 + kp 软起步、关节不被甩动；看门狗/编排层示教拖动退出零触发、带外失能必被抓到
  2. **回归有牙**：把源码换回 HEAD 旧版重编，两个脚本必须失败（旧执行层 stop 后仍发 p=1.98/kp=80、使能甩到 1.98；旧看门狗示教退出 1.4 s 触发 FOLLOW_STUCK→stop→reset）
  3. 真机：使能前执行层对任何反馈陈旧电机拒绝使能；使能后 0.8 s 内 kp 单调升到额定、无甩动；stop 后总线上不再出现 kp>0 的目标帧
  4. 带外失能（`/a3/motor/reset` 直调）→ 看门狗 UNEXPECTED_DISABLE + 编排层 DISABLED（两个通道都要，模拟真机事故前置）
- **真机验收（2026-09-14 夜，事故后 5J 档整栈重启，`scripts/a3_test/f51_real_arm_acceptance.py` P0–P8 全绿）：** 判据 3——`/a3/motor/enable` 响应 `F51 重锚 5 电机到反馈位, 最大丢弃目标距离 0.000000 rad`、使能帧只覆盖档位内 1..5、5 电机 mode=2；**使能后命令位 vs 反馈位最大偏差 0.0004 rad、实际沉降 0.0004 rad（等价于使能瞬间臂没动）**；kp 逐电机 0.35→80.0、0.73–0.74 s 达 90%（软起步生效）；`/a3/motor/stop` 后 1.2 s 内 570 帧（5 电机合计）**全为零增益保活**、无 kp>0。判据 4——带外 `/a3/motor/reset` → 看门狗 `TRIGGERED/UNEXPECTED_DISABLE`（0.61 s，action=report）+ 编排层 `DISABLED`（0.71 s）双通道。另：P7 保持期 3 s 无 HOLD_DRIFT 误报（LL-039 假阳性回归）。**真机验收前排查出并修掉一处同类漏洞**（使能不清 Named 轨迹回退表 `latest_input_champ_rad_`）→ [LL-040](../../lessons_learned/LL-040-enable-resurrects-unnamed-joint-input.md)
- **关联：** [LL-039](../../lessons_learned/LL-039-teach-exit-reanchor-false-trip-enable-snap.md)（事故复盘与判据教训）；[LL-040](../../lessons_learned/LL-040-enable-resurrects-unnamed-joint-input.md)（使能重锚漏回退表）；[LL-041](../../lessons_learned/LL-041-rest-pose-outside-urdf-limit-f48-refuses-enable.md)（姿态越 URDF 限位时编排层 F48 拒使能）；F38（示教退出重锚——本需求第 5 条与之配套）；F50（看门狗）；F42（力矩钳位是防撞不防甩）；[shared/SAFETY.md](../shared/SAFETY.md)（使能前置校验条款）
- **状态：** `implemented`（2026-09-14 仿真回归验收通过：`incident_regression_test.py` T0–T4 与 `incident_monitor_regression_test.py` M1–M4 全绿，且两脚本对 HEAD 旧版均失败——修复前执行层 stop 后仍续发 `(p=1.98, kp=80)`、使能后命令位置偏离反馈位 1.950 rad、mock 关节被拖到 1.98；修复前看门狗示教退出 1.4 s 触发 FOLLOW_STUCK→stop→3 s→reset、带外失能无人报。**真机验收（第 3/4 条）已完成**——2026-09-14 夜事故后按 F52 5J 档（L1–L5）整栈重启，`f51_real_arm_acceptance.py` P0–P8 全绿：见上方「真机验收」段；L6/L7 缺失下不触碰档位外电机）

## F52 缺电机降级档（N 关节运行：只对在线电机做校验/比对/遥测）

- **说明：** 2026-09-14 事故后 can1 上物理只剩 L1–L5（L6/L7 腕部无新鲜反馈），而整栈到处按 7 关节写死（`control_gains.yaml` 的关节/量程/钳位数组、`/joint_states` 名字表、F48 限位校验范围、看门狗逐关节比对、遥测点位）——**后果不是「少两个关节」，而是整臂不可用**：F51 的使能门禁逐电机校验反馈新鲜度，L6/L7 永远陈旧 → **整体拒绝使能**，连存活的 5 个电机都没法调试；同时 L6/L7 的**陈旧缓存值**（`fresh=false`，位置/模式仍是拽脱前最后一帧）会像 LL-020 那样作为「看起来正常的假值」参与限位校验与看门狗比对。本需求把「哪些电机关节参与」收敛成一个**档位清单**（joint profile），各节点按清单长度自适应：清单外电机不参与使能新鲜度校验、限位校验、看门狗比对、F42 钳位与遥测点位，也不再发布其缓存值（避免假值污染）。**7J 档为默认，行为不得改变。**
- **验收标准：**
  1. 5J 档（L1–L5）起栈：`/joint_states`/`MotorStates` 只含清单内关节（不出现 L6/L7 的陈旧缓存值、无 NaN）；F48 硬/软校验只查清单内关节；看门狗不因缺电机报任何故障
  2. 5J 档 `/a3/motor/enable` 与 `/a3/arm/enable` 只看清单内电机的反馈新鲜度 → L1–L5 正常使能到 READY；任一**清单内**电机反馈陈旧仍整体拒绝使能（F51 语义不放松）
  3. 7J 档回归不受影响：`./scripts/a3_test/a3_test.sh incident` 三段全绿，既有仿真/真机路径行为不变
  4. 真机：5J 档 enable → READY，`/a3/arm/move_to` 小幅运动闭环正常、结束后可正常 disable
- **关联：** F51（使能新鲜度校验是本需求的直接动因）、F48（限位校验范围）、F50（看门狗逐关节比对）、F42（钳位数组）、[LL-020](../../lessons_learned/LL-020-bridge-no-seed-stale-js.md)（陈旧值当新鲜用）、[LL-039](../../lessons_learned/LL-039-teach-exit-reanchor-false-trip-enable-snap.md)（事故硬件后果）
- **实现：** 档位清单 = `motor_map*.yaml` 的 `joint_names` + `motor_ids_by_index`（等长，下标即轨迹关节位），`motor_protocol_node`/`power_sequence_node` 启动时读入 `ArmMapper::Configure()`；默认（不配或配错）退回**编译期 7J 档** `kChampJointNames`/`kTemporaryIndexMap`，配错时 ERROR 明示「F52 档位配置非法」而非静默降级。全栈按 `NumArmJoints()` 自适应：`/joint_states` 名字表、`MotorStates`/`tx_stats` 条数、广播 id 展开、F51 使能新鲜度门禁范围、逐关节参数长度校验。配套档位文件：`motor_map_5j.yaml`、`control_gains_5j.yaml`（逐关节数组取 7J 前 5 项）、`arm_controller_5j.yaml`。
- **状态：** `completed`（2026-09-14 夜）
  - **仿真：** `incident` 阶段八c 三用例全绿——5J 档使能+命名轨迹逐关节到位、7J 档遇缺电机整体拒绝使能且零帧、非法档位退回 7J 默认档
  - **真机（5J 档整套起栈，`--bridge-log` 核对执行层日志）：** 判据 1——执行层启动日志 `F52 档位：5 关节 [L1_joint→motor1 … L5_joint→motor5]`，`/joint_states` 恰好 5 个名字（无 L6/L7 陈旧值）、`MotorStates` 5 条全 `fresh`；判据 2——`/a3/motor/enable` 响应 `F51 重锚 5 电机到反馈位`、使能帧**只覆盖电机 1..5**、5 电机 mode=2，`/a3/arm/enable` 到 `READY`；判据 4——`/a3/arm/move_to`（L1 +0.10 rad / 3.0 s，增量经 `safety_limits.clamp_delta`）到位误差 0.0029 rad、原路返回误差 0.0004 rad，随后带外 `/a3/motor/reset` 双通道失能（看门狗 `TRIGGERED/UNEXPECTED_DISABLE` 0.61 s + 编排层 `DISABLED` 0.71 s），收尾 5 电机全部 mode=0、温度 31–34 °C
  - 真机验收脚本：`scripts/a3_test/f51_real_arm_acceptance.py`（`A3_REAL_ARM_ACCEPT=1 … --move`；姿态越 URDF 限位时加 `--nudge-delta`，见 [LL-041](../../lessons_learned/LL-041-rest-pose-outside-urdf-limit-f48-refuses-enable.md)）
  - **未覆盖项（有意）**：判据 4 的 disable 走的是带外 `/a3/motor/reset`（紧急失能，也是 P8 双通道判据本身），**没有**走 `/a3/arm/disable` 的 F40 park——该臂当时 L4≈-1.0 rad、home_L4≈+0.334，park 是约 1.4 rad 的单次无人监护运动，超出当晚授权范围，留待有人在场或新结构到位后单独验证

## F53 编排层「不罢工」补齐：指令×状态×模式 全组合显式拒绝（F40 零力矩 disable 死路径）

- **说明：** 2026-09-16 零力矩手感测试暴露链路缺口：`/a3/arm/disable`（F40 park 路径）在 `mode==ZERO_TORQUE`（外部 `/a3/zero_torque/start` 直打执行层，编排层状态仍 READY）下**不拒绝**——safe park 的回家轨迹被执行层 OnTrajectory **静默丢弃**（`zero_torque_active_ || SERVO`），臂在悬浮中干等 ~4–5 s 后看门狗 `FOLLOW_STUCK → stop → 3s → reset` 阶梯把臂中途切断，编排层误报「safe park aborted: 电机带外失能」→ DISABLED；且此路径下 `reset` 会**撤掉重力补偿让臂垂落**，reset/set_zero/enable 都**不退出** `zero_torque_active_`（重使能后 kp 仍 0、臂保持悬浮，「伪成功」）。原则收束为：**编排层任何指令×状态×模式组合不得静默接受或含糊失败——要么执行，要么 `success=false` + 消息给出可执行的下一步**。修复 5 处（`arm_controller.py`）：
  1. `_disable_cb`（主缺口）：`self._mode in BLOCKED_MODES`（ZERO_TORQUE/GRAVITY_COMP）时拒绝，消息提示 `先 /a3/zero_torque/stop 恢复闭环再 disable`；紧急失能 `/a3/motor/reset`（但注明此后需 zero_torque/stop 才能正常重使能）
  2. `_init_cb`：拒绝组合补 `SAFE_PARK` busy 态 + `BLOCKED_MODES`（零力矩下 set_zero 打碎零位帧；enable 不退出 zero_torque → init 伪成功）
  3. `_enable_cb`：补 `BLOCKED_MODES` 拒绝（使能=重锚当前位但 kp 仍 0，臂继续悬浮 → 伪成功）
  4. `_can_move`：拒绝表补 `FAULT`（此前静默接受轨迹、臂不动——电机已复位关断）
  5. `_enter_ai_cb`：补 `BLOCKED_MODES` 拒绝（AI 发的轨迹会被执行层丢弃，静默无动作）
- **验收标准：**
  1. 零力矩下 `/a3/arm/disable`、`enable`、`init` → `success=false` 且消息含 `先 /a3/zero_torque/stop …`（不再走进 park→看门狗切臂死路径）
  2. `/a3/zero_torque/stop` 后再 disable/enable/init 全部正常（退出后执行层恢复 kp 锚定当前位）
  3. `GRAVITY_COMP` 模式同 1（进入该模式的 set_mode 路径同样拒绝）
  4. FAULT 态下 goto/move_to/playback/jog 被拒（消息含 state）；AI 态零力矩下 enter_ai 被拒
  5. 常规路径零回归：READY 态 goto/move_to/回放、READY/TRAJ 带外失能、IDLE 直达 disable、SAFE_PARK 拒绝、F44 降温门禁行为不变；现有 `scripts/a3_test/a3_test.sh` 用例不受影响
- **配套文档：** [docs/edge/STATE_MACHINE.md](STATE_MACHINE.md)（11 态 stateDiagram + 指令×状态矩阵 + 跨层模式丢弃矩阵）；中文消息改为「动作/原因/下一步」三段式，全部服务拒绝路径均给出 `/a3/zero_torque/stop` 或 `/a3/motor/reset` 逃生
- **关联：** F40（disable park 是死路径入口）、F45（11 态状态机）、F51（使能重锚/模式共享；本需求堵住其「模式不清零」的组合缺口）、F50（看门狗阶梯是死路径的误报源）、[LL-045](../../lessons_learned/LL-045-disable-in-zero-torque-dead-path.md)
- **状态：** `implemented`（2026-09-16，代码 5 处 + 文档；构建验证通过；**真机重启后生效**——修复时臂处于零力矩测试中，未重启正在跑的旧栈，待用户跑完退出流程后手动重启加载）

## F54 示教停止自动保存 + 回放默认最新（空名 ≡ latest 槽位）

- **说明：** 示教回放旧流程「停止示教 → 手动 `save_trajectory` 取名 → `playback` 指名」多一步保存指令，录了就忘 / 忘了存 / 名字记错都麻烦。本需求收成两个默认：
  1. `stop_teach` 后**自动保存**当前录制：写 `latest.yaml`（滚动最新槽）+ 时间戳备份 `teach_YYYYmmdd_HHMMSS.yaml`（防覆盖丢失历史）；样本低于 `teach_auto_save_min_samples`（默认 10，@50Hz≈0.2s）视为 start 后立刻 stop 的**误触发**——跳过自动保存、不覆盖已有 latest；
  2. **空名 ≡ `latest` 槽位**：`save_trajectory {name:""}` 与 `playback {name:""}` 都读写 `latest.yaml`（对既有「保存完了叫 latest / 回放不指名」心智的收敛）；非空名行为完全不变（`{name}.yaml`）。`playback` 空名且无 `latest.yaml` → `success=false` + 消息提示「先 start_teach→拖动→stop_teach 录制,或显式指定 name」。
- **验收标准：**
  1. 仿真：`start_teach` →（模拟移动）→ `stop_teach`，`~/.a3/trajectories/` 出现 `latest.yaml` 且 points 数 = 录制样本数，同时有 `teach_*.yaml` 备份；响应消息含 `auto-saved`
  2. `playback {name:""}`：无任何录制时 `success=false` + 消息指明先录制；有 latest 时回放 latest.yaml（日志 `playback latest`）
  3. 命名路径零回归：`save_trajectory {name:"x"}` → `x.yaml`，`playback {name:"x"}` → `x.yaml`；`save {name:"latest"}` 与空名等价（同一槽）
  4. 误触发示教（start 后立刻 stop，样本 < 阈值）：不写 latest.yaml、不覆盖已有 latest，响应消息注明 `samples < 阈值, keep previous latest`
  5. `_record` 内存缓冲在 stop_teach 后仍保留——自动保存后仍可用命名 `save_trajectory` 另存一份
- **关联：** F38（示教/回放）、F41（ramp 插值口径）、[LL-047](../../lessons_learned/LL-047-playback-smoothing-accel-spike.md)（回放平滑）、[LL-030](../../lessons_learned/LL-030-return-args-free-instruction.md)（安全取参/默认值语义）
- **状态：** `implemented`（2026-09-16，代码 + 文档；仿真验收通过；真机需在场拖臂）

## F55 PS4 全功能映射：示教/执行/使能/初始化一键化（短按+长按双义）

- **说明：** 手柄遥控是示教/回放的主交互入口，但编排层服务（init/enable/disable/start_teach/stop_teach/playback）此前一个键都没接。本需求把 arm_controller 服务全部映射到 PS4 空闲键，并为映射引入短按边沿——一个键「短按=功能 A、长按=功能 B」的双义（Options：短按停示教 / 长按 3s 调零）。示教主流程定键：**Share=开始、Options=结束、Circle=执行**。
- **映射（`src/a3_teleop_ps4/config/mappings/default.yaml`，完整表见该包 `README.md`）：**

  | 键 | 边沿 | 动作 |
  |----|------|------|
  | Share | 短按 | `start_teach`（进示教，0/mode 拖动 + 记录） |
  | Options | 短按 | `stop_teach`（停记录 + F54 自动保存） |
  | Options | 长按 3 s | `power_set_zero`（调零；原 2 s 上调，降误触） |
  | Circle | 短按 | `playback` 空名（回放定义最新） |
  | Touchpad | 短按 | `arm_init`（0/mode，F51 越限只 WARN） |
  | L3 | 短按 | `arm_enable`（使能安全门禁：重锚 + 软起步） |
  | R3 | 短按 | `arm_disable`（F40：离 home 先 safe park） |
  | Triangle | 长按 1 s | `power_shutdown`（**唯一**手柄急停） |
  | Square / Cross / D-pad / L1 / R1 / R2 | 不变 | 上电 / 立即停 / 命名位姿 / deadman / boost / 力控夹爪 |

- **「三键组合急停」废弃：** 原 SAFETY 契约的「L1+R1+Share = 关机」需要三键同时操作，误用风险高且真按三键反而不如单键可靠。急停收敛为 **Triangle 长按 1 s**（单键、防误触、无组合需求）；`L1+R1+Share` 从 SAFETY.md 移除。
- **验收标准：**
  1. 映射含上述 7 个新绑定；`buttons.options` 为**两条绑定的列表**（短按/长按各自独立边沿跟踪）；mapper 启动时 `validate_mapping` 对列表逐条校验、零报错
  2. 短按语义：按下起计时、**释放时**持续时长 `< hold_s`（3 s）才触发一次；按住超 `hold_s` 再释放该次不触发、也不能连带触发同一键的另一条绑定
  3. sim（domain-55 隔离栈 + fake `/joy`）：Share → `arm_status.state=TEACH`；Options 短按 → `stop_teach success + auto-saved latest.yaml`；Circle → 日志 `playback latest`；Options 按住 ≥3 s 再放 → `/power_sequence/command` 收到 `set_zero`（且**不**触发 teach_stop）；L3/R3 → enable/disable 成功
  4. 既有按键零回归：D-pad 4 位姿、Cross 立即停、Square/Triangle 电源、L1 deadman、R1 boost、R2 力控保持原绑定
  5. 文档三处可见：包 `README.md` 完整表 + `default.yaml` 文件名即可改（约零代码） + `QUICKSTART.md` 指向
- **关联：** F54（自动保存/空名=latest）、F38（示教回放）、F51（使能安全）、F40（失能保护）、F36（力控扳机）；[shared/SAFETY.md](../shared/SAFETY.md)、`src/a3_teleop_ps4/README.md`
- **状态：** `implemented`（2026-09-17，代码 + 文档；仿真验收通过；真机需用户在场按手柄）

## F56 MIT 帧目标速度前馈（帧 V 域填每 tick 目标速度）

- **说明：** 轨迹帧的 V 域此前恒为 `default_velocity_=0.0`——伺服只能靠 kp 追位置，kd=2 又按 `kd·(0−v_act)` 对 2 帧（20 ms）尺度的速度纹波**主动拖刹车**（实测基线 rms|v_act−v_cmd|≈0.146 rad/s、max 0.81）。修复：执行层 `SendMitFrame` 内用上一 tick 的 `champ_smoothed` 差分除以真实 dt 得到目标角速度（逐 tick 与位置目标一致，不与 `SmoothJointCommand` 的限速/启动平滑打架），指数滤波（α=0.3）后乘 `joint_signs` 写进帧 V 域，并按电机型号速度量程钳位（`SpeedRangeRadSFor`）。kd 语义从「拖刹车」变成「朝目标速度阻尼」，20 ms 纹波应显著下降。参数：`trajectory_vel_ff_enable: true`、`trajectory_vel_ff_gain: 1.0`（三份 `control_gains*.yaml` 同步）。dt 上限硬守（>4 tick 视为续流不连续 → 清零防尖峰）；更新与位置逐 tick 同拍，保持位路径 V=0 不变。
- **验收标准：**
  1. 仿真：回放期 `tx_stats` 帧速率无回退；无异常日志
  2. 真机：capture 对比同类回放，rms|v_act−v_cmd| 较基线 0.146 rad/s 显著下降；速度纹波谱 20 ms 峰消除
- **关联：** F38（回放）、F57（回放时间重排）、[LL-053](../../lessons_learned/LL-053-f56-f57-f58.md)（根因：V=0 → kd 拖刹）
- **状态：** `implemented`（2026-09-17，代码 + 配置；仿真/真机验收待做）

## F57 回放匀速重排 + 保存时轻量平滑

- **说明：** 回放「一顿一顿」除 F56 的 20 ms 纹波外，第二层根因是**时间轴 = 手拖起-停节奏的忠实复刻**——位置波形平滑动不了时间轴，第 7 点 MA 治不了。修复分两端：
  1. **回放端**：`arm_controller._time_warp_points` 等速重排——位置不动，时间轴按「逐段最大关节位移」标度匀速化：`v_eff = min(总路径/原时长, vmax)`（只压缩不拉伸），每段 `dt = max(段位移/v_eff, dt_min)`（零位移段取地板 → 点严格递增），保留几何路径、压掉停顿。参数 `playback_time_warp: true`、`playback_warp_vmax_rad_s: 0.6`（实测 0.6 rad/s 跟踪干净，≤1.5 command 限速）、`playback_warp_dt_min_s: 0.02`（@50Hz 网格地板）。在 `_smooth_points`（仍开，压尖峰）之后执行。
  2. **保存端**：`_dump_recording` 保存时对 positions 做轻量平滑（复用 `_smooth_points` 凸组合不越包络），`teach_save_smooth_samples: 5`（0/1/2 关闭）——latest.yaml / teach_*.yaml 落盘即干净，`latest` 槽与时间戳备份任何消费方受益；时间轴不变。
- **验收标准：**
  1. 纯函数：喂起-停人工点阵，停顿段被压缩、time 单调递增、位置不动、总时长 ≤ 原时长
  2. 仿真回放：日志出现 `playback time-warp: v_eff<=0.6, duration Xs`；`/joint_group_effort_controller/joint_trajectory` 时间轴严格递增且匀速；monitor 不误报
  3. 真机：回放手感顺（无起-停抖动）；`teach_save_smooth_samples` ≥3 时保存的 yaml 位置已平滑、时间轴不变
- **关联：** F38（回放）、F54（自动保存 latest）、F56（速度前馈）、[LL-053](../../lessons_learned/LL-053-f56-f57-f58.md)
- **状态：** `implemented`（2026-09-17，代码 + 配置；验收待做）

## F58 stop 处置改「重力支撑保持」+ 自动 reset 姿态门禁

- **说明：** 用户追问「回放追不上 → stop 裸卸力 → 3s 后 reload 会不会掉臂」——**会**。原 stop 路径：monitor `FOLLOW_STUCK→stop` 发 kp=kd=tau=0 裸卸力帧（`HandleMotorStopService`），且 refresh 的 `hold_suppressed_` 分支随后以零增益 keepalive 续发——整条 stop→3s ladder→reset 窗口内 7 个电机无约束力矩，重力敏感位形（L3/L4 水平轴）下臂直接垂落（9/14 真机事故甩断 L6/L7 同根）。修复两层：
  1. **执行层（`motor_protocol_node`）**：stop 处置帧在**反馈新鲜**时发「重力支撑保持」——目标锚定反馈位（静止无误差）+ `stop_hold_kp=25`/`stop_hold_kd=2` 中低刚度 + 活重力前馈 `ComputeMitTorqueFf`；refresh 的 `hold_suppressed_` 分支对 `mode ∉ {0,-1}` 且反馈新鲜的电机同样发保持帧（不再零增益盖掉）。反馈陈旧（不能信任保持）/ 失能期才退回零增益卸力帧。`hold_suppressed_` 语义不变（仍不重锚旧位姿）。参数：`stop_hold_kp: 25.0`、`stop_hold_kd: 2.0`（三份 `control_gains*.yaml` 同步）。
  2. **监视端（`arm_monitor_node`）**：自动 reset 前查重力门禁——订阅 `/a3/gravity_torque`（BestEffort，JointState），样本新鲜（`reset_gravity_fresh_s: 1.0` 内）且 `max|τ_grav| > reset_max_gravity_torque_nm: 5.0` → 拒绝 reset，`_last_event = RESET_DENIED_gravity_unsafe (保持中)`，只保持不重置（记录 `act_done="reset_denied"` 防 ladder 3 s 重试刷屏）。样本陈旧/缺失 → 按旧行为放行（无 gravity 节点 / F51-F52 回归不破坏）。失败路径仍可人工 `/a3/arm/disable`。
- **验收标准：**
  1. 真机人为制造大误差触发 `FOLLOW_STUCK→stop`：stop 后臂**停在原地不垂落**（重力保持），对比旧版裸卸力垂落
  2. 危险位形（超阈）下自动 reset 被拒（日志 `RESET_DENIED_gravity_unsafe`），安全位移回后条件消失可再触发；无 gravity 节点时行为同旧版
  3. F52 缺电机档（5J）enable + stop 路径不回归（stop_hold 各关节有效）
  4. 反馈陈旧/失能期的 stop 仍发零增益卸力帧（不信任保持）
- **关联：** F50（看门狗处置阶梯）、F42（力矩钳位）、[shared/SAFETY.md](../shared/SAFETY.md)（stop 语义更新）、LL-039（裸卸力掉臂事故谱系）、[LL-053](../../lessons_learned/LL-053-f56-f57-f58.md)
- **状态：** `implemented`（2026-09-17，代码 + 配置；真机验收待做）

## F59 回放 warp 平滑化重写（弧长均匀重采样 + 加速度限幅时间膨胀）

- **说明：** F57 的 `_time_warp_points` 有确认缺陷（真机 2026-09-17 三次回放暴露）：`dt = max(段位移/v_eff, dt_min)` **保留全部点**，零位移停顿段每点吃 0.02 s 地板 → 停顿被重新充气回时间轴（16.6 s 示教 → 30.9 s 回放，违反自己「只压缩不拉伸」的契约），且停顿↔运动交界处速度单 tick 内 0→0.31 rad/s 阶跃（加速度尖峰 11.4 rad/s²）——叠加 F56 速度前馈后 V 域忠实携带阶跃下伺服，真机上同一轨迹 3 次回放 2 次 FOLLOW_STUCK（L2 重力负载爬升段，断点位置各异 = 边际力矩）。重写为两段式：
  1. **弧长均匀重采样**：按「逐段最大关节位移」累计弧长 S，`v_eff = min(S/原时长, vmax)`，以 `ds = v_eff×dt_min` 在弧长上等步行走、位置在段上线性插值（不新增路径、只做弧长参数化）。停顿段弧长为 0 不占时间 → **停顿天然折叠**，输出时长 ≈ S/v_eff ≤ 原时长，修掉膨胀缺陷。
  2. **粗网格加速度限幅时间膨胀（LL-057 终版）**：每 5 个细点取锚点（0.1 s 窗），粗网格上跑「前后窗口分工」二次恰解 `amax·dt²+vp·dt−Δq=0`（disc<0 走有界 ×1.5 兜底，≤40 轮），只拉 dt 不动位置，窗口内 dt 均匀回填 50 Hz 细网格。**不在 20 ms 细网格上逐点恰解**——细步 |Δv| 混有「最大关节换帅」离散伪影（真实数据 top-20 |Δv| 全部精确 =v_eff=0.274 rad/s）与 ±1 mrad 编码器噪声（伪加速度 ~5 rad/s²），逐点恰解被噪声带飞、前后向往返震荡（21.6 s 输入膨胀到 38.8 s）；粗网格噪声底降到 ~0.4 rad/s²（对 amax 2.0 有 5× 裕量），只响应真实拐角。限幅口径对准故障时间尺度：FOLLOW_STUCK 是持续力矩饱和（0.5 s 窗），单步 kink 只造成 mrad 跟随暂态。
  - 参数：`playback_warp_accel_max_rad_s2: 2.0`（0 = 关闭膨胀 pass）。
- **验收标准：**
  1. 纯函数：起-停人工点阵 → 停顿段折叠（**带/不带停顿的同一几何路径 warp 输出逐点全同**）、时间严格递增、dt ≥ 网格地板、位置全部落在原轨迹段上（插值不越界）。✅ 2026-09-18 `/tmp/f59_unit.py`：case1 151 pts / 3.09 s
  2. 实测回归（teach_20260917_214930.yaml，全管线 ramp5s→MA7→warp）：输出时长 ≈ 原时长（F57 版 31.9 s 膨胀不再）；粗网格（0.1 s 窗）max|accel| ≤ amax。✅ 实测：1080 pts / **21.77 s**（输入 21.59 s，v_eff=0.274），stride5-accel **0.98 ≤ 2.0**，位置不越平滑后包络
  3. 仿真回放：日志 duration 接近原时长；时间轴严格递增；monitor 不误报。✅ 2026-09-18 edge_web_sim 回放同文件：success，1080 pts / 21.9 s，无 FOLLOW_STUCK/fault
  4. 真机：重放同一轨迹——全程无 FOLLOW_STUCK（对比 F57 版同文件 3 次 2 断）、无停顿膨胀、手感顺。⏳ 待用户上电（任务 #18，同时抓 L2 力矩饱和证据与 F56 V 域真值）
- **关联：** F57（warp 初版，本项重写其 `_time_warp_points`）、F56（速度前馈——V 域携带 warp 输出）、F38（回放）、[LL-057](../../lessons_learned/LL-057-warp-reinflation-marginal-l2-torque.md)
- **状态：** `implemented`（2026-09-18 代码终版=粗网格恰解 + 纯函数单测 + 仿真验收过；真机验收待做）

## F60 PS4 键位重设计（一键使能 / 单键硬急停 / 示教三键）

- **说明：** F55 键位在真机联调中暴露三个操作问题：(1) 上电与使能分属 Square 长按 + L3 两键两层，操作员分不清「已开机但未 READY」；(2) 急停是 Triangle 长按，与 Triangle 命名位姿的通用助记冲突；(3) 示教/回放分散在 Share/Options/Circle。重新设计 `config/mappings/default.yaml`（`simple.yaml` 保留作无 deadman 调试档）：

  | 按键 | 手势 | 动作 |
  |---|---|---|
  | L3 | 短按 | `arm_power_enable`：执行层 `power start` → 等 gate_open+Running → 编排层 `/a3/arm/enable`，一键到 READY（非阻塞轮询，5 s 超时） |
  | R3 | 短按 | `arm_disable`（F40：离 home 先 SAFE_PARK 再失能） |
  | Cross(X) | **长按 1 s** | `power_shutdown` 硬急停：门禁关、电机失能；恢复需重新 L3 |
  | Triangle | 短按（rising） | goto 命名位姿 `ready` |
  | Circle | 短按（rising） | goto 命名位姿 `home`（非 zero——机械零位 servo IK 奇异，LL-007） |
  | Share | 短按 | `teach_start` |
  | Options | 短按 / 长按 3 s | `teach_stop`（自动保存 latest，F54）/ `power_set_zero` |
  | Square | 短按 | `playback_latest`（空名 ≡ latest 槽位） |
  | PS | 短按 | `arm_init`（set_zero + 到位校验 + 自动 enable） |
  | L1 | 按住 | deadman，**仅门控摇杆平移/偏航轴**（applies_to=analog_n11） |
  | R2 | 模拟量 | 夹爪力控，**不经 L1 门控**（kind=analog_01，F36 迟滞/力矩映射不变） |
  | R1 | 按住 | 速度档 0.35 ↔ 1.0 |
  | 左摇杆 | 模拟量 | servo 平移 Y（left_x）/ Z（left_y），需 READY + L1 |
  | 右摇杆 | 模拟量 | servo 平移 X（right_y）/ 偏航 Z 预留（right_x），需 READY + L1 |

  预留不绑定：D-pad 四键、触摸板键（蓝牙 js0 无此键事件，LL-052）、L2。命名位姿改走编排层服务 `/a3/arm/goto_named_pose`（a3_msgs/GotoNamedPose），不再由 teleop 本地插值直发 JointTrajectory——状态机进入 TRAJ（紫灯可见）并获得 F53 拒绝语义；删除 teleop 内 `/a3/goto_named_pose`（String）订阅（无其他发布者）。真机 launch 默认映射由 `simple` 改为 `default`。
- **验收标准：**
  1. mapper 启动 `validate_mapping` 零报错；options 双键列表各自独立边沿跟踪（短按释放判定不连带长按绑定）
  2. 合成 /joy 仿真（F62，domain 45）：12 场景全部 PASS——PS init→READY；Triangle/Circle 收敛容差 0.08 rad 且过程 state=TRAJ；L3 在已 READY 时幂等；R2 不按 L1 可开合 L7；摇杆不按 L1 不动、按住 L1 才动；Share/Options/Square 示教回放链保存 latest.yaml；R3 SAFE_PARK→DISABLED；X 长按 1 s gate 关闭；L3 可从关机/失能两态恢复 READY；Options 长按 3 s 发 set_zero
  3. teleop `/joint_states` 订阅为 BEST_EFFORT（真机 SensorDataQoS 兼容，LL-059）
  4. 文档：包 README 全键表 + SAFETY.md 急停/失能/deadman 段更新 + 操作员手册 PS4_OPERATOR_GUIDE.md
- **关联：** 取代 F55 键位表；F40（R3 失能保护）、F48（enable 越限拒绝）、F51（使能重锚）、F54（示教自动保存）、F36（R2 力控）、F61（灯/震反馈）、F62（合成验证）；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented-pending-sim`（2026-09-20 代码 + 配置；待 F62 合成验收 + 用户实操）

## F61 DS4 灯带 + 震动反馈（状态五色系）

- **说明：** 操作员持手柄无任何状态反馈，不知道臂处在哪一层。新增 `ds4_feedback_node`（a3_teleop_ps4）：hidraw 直接写 DS4 HID 输出报告（USB 0x05/32 B；蓝牙 0x11/78 B + 0xC0 控制字节 + CRC32 LE seed 0xA2；报告构造移植自 `scripts/ps4/deep_dog_ds4_hid.py`，设备发现/2 s 热插拔移植自 `ds4_hid_node.py`，反馈 fd 以 O_RDWR 独立打开，与只读 ds4_hid_node 并存）。灯效 10–20 Hz 节拍定时器驱动，仅状态变化/呼吸节拍/震动变更时写 hidraw，rumble 定时自动清零。

  | 派生状态 | 灯带 | 震动 |
  |---|---|---|
  | FAULT（arm FAULT / temp_warn / monitor TRIGGERED） | 红色双闪 | 进入时双震 |
  | 关机/硬急停（shutdown 边沿，或 Idle+gate 关） | 红色闪 | 强震 600 ms |
  | INIT / 上电序列中（Precheck/EnableInit/SoftStand）；gate 开但 IDLE/DISABLED | 橙色常亮 | — |
  | init 完成边沿（→READY 首次） | 白色闪一次 | — |
  | READY / SERVO（jog 中仍 READY） | 绿色常亮 | READY/DISABLED 转换沿弱震 120 ms |
  | TEACH | 蓝色呼吸 | — |
  | TRAJ（命名位姿/回放/SAFE_PARK） | 紫色常亮 | — |

  订阅 `/a3/arm_status`（ArmStatus，默认 QoS）、`/power_sequence/state` + `/power_sequence/gate_open`（TRANSIENT_LOCAL）、`/a3/monitor/status`、`/power_sequence/command`（急停归因边沿）。无 DS4 设备时 WARN 一次但节点存活，降级为只发诊断话题。诊断话题 `/a3/ds4/feedback`（std_msgs/String，JSON：state/gate/fault/color/rumble/reason，volatile depth 10），供无手柄的仿真/CI 断言。参数 `enable`（默认 true）、`bus`（auto/usb/bt）。
- **验收标准：**
  1. 无设备：节点不崩，持续发 `/a3/ds4/feedback`，状态迁移与上表颜色/rumble 字段一致（F62 合成场景断言）
  2. 有设备（USB 先行；BT CRC 路径真机补验）：五色/闪烁/呼吸/三类震动在对应状态沿可见可闻；热插拔 2 s 内恢复
  3. 反馈写 hidraw 不影响 ds4_hid_node 只读事件流（两 fd 并存）
  4. 同状态不重复写设备（变化/节拍才写），rumble 到时自动清零
- **关联：** F60（状态来源键位）、F50（monitor TRIGGERED）、F44（温度预警）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（/a3/ds4/feedback）
- **状态：** `implemented-pending-sim`（2026-09-20 代码；无设备逻辑随 F62 仿真验收，USB/BT 真机灯效待手柄重连补验）

## F62 合成 /joy 全功能仿真验证（无手柄自动化）

- **说明：** 真机联调前先用脚本模拟手柄输出逐键验证，避免「人感觉按键失效」的不可重复排查。新增：
  1. `a3_bringup/launch/edge_teleop_full_sim.launch.py`：include `edge_web_sim`（use_rviz:=false, use_target_ghost:=true, use_gripper:=true）+ 内联 MoveIt Servo 块（复用 a3_bringup.launch.py 的 servo_mode_bridge + servo_node_main 写法，禁止叠加会双 RSP/双 /joint_states 的 servo.launch）+ ps4_teleop（use_joy_node:=false, mapping:=default, enable_feedback:=true）+ 双模型 RViz（软件渲染，LL-027）。
  2. `scripts/a3_test/ps4_sim_test.py`：50 Hz 合成 `sensor_msgs/Joy`（axes[8]/buttons[14]，ds4_linux 索引；开场 ≥25 拍全零基线满足 mapper alive 计数，每步回零）；BEST_EFFORT 订阅 /joint_states；纯 rclpy 计时（LL-050，不用 ros2 CLI 做时序断言）；Reporter 风格逐步打印 PASS/FAIL + 关节位移证据。
  3. 隔离域 ROS_DOMAIN_ID=45。
  12 场景：基线橙 → PS init → Triangle ready → L3 幂等 → R2 脱离 L1 开合 L7 → 摇杆 deadman 逐轴方向表 + R1 速度档 → Circle home → 示教三件套（断言 latest.yaml mtime）→ R3 失能 → L3 恢复 → X 长按硬急停+恢复 → Options 长按 set_zero。
  仿真与真机已知差异**只记录不断言**：sim_power shutdown 无 SoftProne 动画/无 F51 联动；sim 不校验 set_zero 状态前提；sim /joint_states 为 RELIABLE（F60 的 QoS 修复在仿真不可复现，仅代码审查）；sim 不强制门禁。
- **验收标准：**
  1. `ros2 launch a3_bringup edge_teleop_full_sim.launch.py`（DOMAIN 45）起栈无致命错误，双模型 RViz 可见
  2. `python3 scripts/a3_test/ps4_sim_test.py` 12 场景全绿，输出每步关节/状态/灯效证据；摇杆方向表用于标定各轴 invert
  3. 全绿后用户实操（真手柄）复测一轮作为最终签收
- **关联：** F60（被测键位）、F61（被测灯效诊断话题）、LL-027（软件渲染）、LL-050（纯 rclpy 时序）、LL-059（仿真 QoS 画像掩盖真机不兼容）
- **状态：** `implemented-pending-sim`（2026-09-20 launch + 脚本；验收执行随本次任务）

## F63 joynet Linux DS4 独立布局（pygame/SDL 6 轴 + hat，对齐 joy_node 协议）

- **说明：** joynet 在 RK3588（Ubuntu 22.04）上经 pygame/SDL 读取 DS4，真机实测（2026-09-20，蓝牙 `Wireless Controller`）与 Windows 开发环境不同：`axes=6`（0/1 左摇杆、2=L2、3/4 右摇杆、5=R2；摇杆上/左为负、右/下为正；扳机静息 −1、按满 +1），`buttons=13`（0 cross、1 circle、2 triangle、3 square、4 l1、5 r1、8 share、9 options、10 ps、11 l3、12 r3；6/7 是扳机数字点击），十字键是 **hat 0** 不是轴。原 `DS4_LINUX` 表把十字键放在轴 6/7（该 8 轴形态只存在于 joydev/`joy_node` 侧）且 `detect_layout` 对 6 轴手柄错判为 `ds4_sdl`，导致用户实测键位全错。修正：`DS4_LINUX` 改 `dpad="hat"`、摇杆 polarity +1（SDL 已是右/下正），扳机仍走 `(v+1)/2`；`detect_layout` 对「Wireless Controller + 6 轴 + 轴 2 静息 −1（扳机）/ 1 hat」选 `ds4_linux`。joynet 以 **TCP client** 接入 `a3_teleop_ps4` 现成的 `ds4_tcp_joy_node`（127.0.0.1:8890），由其发布标准 ds4_linux 协议 `/joy`（8 轴 / 14 键），ROS 侧不新增代码。
- **验收标准：**
  1. `layout: auto` 时本机手柄自动选中 `ds4_linux`
  2. `ds4_linux` 抽象快照：摇杆推上/左输出负、右/下正，扳机静息 0 / 按满 1，十字键四方向具名按钮正确
  3. joynet 自带 pytest 全绿；端到端 `ds4_tcp_joy_node` 发布的 `/joy` 与 `a3_teleop_ps4/config/ds4_linux.yaml` 契约一致（axes `[LX,LY,L2,RX,RY,R2,DPAD_X,DPAD_Y]`、buttons `[cross,circle,triangle,square,l1,r1,l2,r2,share,options,ps,l3,r3,touch]`）
- **关联：** F60（PS4 键位）、F62（合成 /joy 仿真）；LL-060（SDL 6 轴 + hat vs joydev 8 轴、ds4_sdl 误判、venv pytest 污染）
- **状态：** `in-progress`（2026-09-20，代码+pytest 61 全绿、auto 检测/快照极性/dump 均已实测；evdev 原始事件证实 joydev 侧契约上=−1/下=+1（LL-061）。TCP 桥端到端仅完成分段验证+空闲帧，剩一次带按键的实时帧采集）

## F64 PS4 双模式死人开关（L1 平移 / R1 旋转）+ D-pad 分通道调速

- **说明：** F60 的 R1「全速档」语义含混（按住 R1 把线速度从 0.35 拉满，旋转也同比例放大），且单速度档无法分别精细调节平移与旋转。用户 2026-09-21 提出并确认改为**双模式死人开关**：

  | 操作 | 动作 |
  |---|---|
  | **L1 按住** + 摇杆 | 末端**平移**：左摇杆左右=Y、上下=Z；右摇杆上下=X |
  | **R1 按住** + 摇杆 | 末端**旋转**：右摇杆左右=偏航 Z（首批仅绑 yaw；roll/pitch 待实操后按需加绑） |
  | **D-pad 上 / 下** | 平移速度档位 **+ / −** 步进（仅改线速度比例） |
  | **D-pad 左 / 右** | 旋转速度档位 **− / +** 步进（仅改角速度比例） |

  实现要点：
  1. 删除 `held.r1`（set_speed_scale 全速绑定）；删除 `deadman` 单开关块，轴改逐轴 `gates: [按钮…]` 列表——轴任一 gate 按住才输出，平移三轴 gates=[l1]、偏航轴 gates=[r1]。
  2. ActionExecutor 单一 `speed_scale` 拆为 `linear_scale` / `angular_scale`（默认均 0.35），tick_end 线速度通道乘 linear_scale、角速度通道乘 angular_scale；新增 `step_linear_scale` / `step_angular_scale`（kwargs delta，clamp 0.1..1.0，步进 0.15）。
  3. mapper 新增 `dpad` 配置块，对 dpad_x/dpad_y hat 轴（−1/0/+1）做 0→±1 边沿检测（持续按住不连发，回中后才能再步），映射到上述 step 动作。
  4. R2 夹爪、命名位姿/示教/电源键不受 L1/R1 门控影响（维持 F60 语义）。
  5. 松开 L1/R1：被门控轴立即归零，servo_mode_bridge 0.5 s 超时兜底停。
- **验收标准：**
  1. 不按 L1/R1 推任何摇杆：1.2 s 内关节 Δ<0.03 rad
  2. 按住 L1：平移三轴动、偏航轴不动；按住 R1：偏航动、平移三轴不动（合成场景逐断言）
  3. D-pad 上/下：linear_scale 步进且仅改变平移速度；D-pad 右/左：angular_scale 步进且仅改变旋转速度；持续按住不连发
  4. F62 合成脚本扩展场景在 domain 45 全绿；真机保持断电，用户先以真手柄在仿真栈实操验收
- **关联：** 修订 F60 的 L1/R1 语义（键位表其余部分不变）；F61（灯效仍按 READY 绿，jog 中不换色）、F62（合成验证扩展）；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `in-progress`（2026-09-21）

### F40 — 失能保护（disable → 自动回 home → 失能）

- **说明：** `/a3/arm/disable` 不再是「无条件直接失能」——不在 home 容差内时先平滑回 home 再失能，防止 ready 位直接掉臂。新增参数：`disable_home_pose_name: "home"`、`disable_home_tol_rad: 0.15`、`disable_home_duration_s: 3.0`、`disable_home_confirm_s: 0.5`、`disable_park_timeout_s: 8.0`。新辅助 `_at_home()`（全关节 |q−home| ≤ tol）与 `_safe_park_then_disable()`（同步阻塞：发布 home 轨迹抢占活跃轨迹——执行层 OnTrajectory 天然支持替换，无需排队 → `SAFE_PARK`（期间拒绝新运动指令）→ 轮询连续 `confirm_s` 收敛 → reset → `DISABLED`）。服务语义（同步阻塞返回，`success=true ⟺ 已失能`）：READY/TRAJ 容差内 → 直达 reset → DISABLED；容差外 → safe park → reset → DISABLED（message 含耗时）；IDLE → 直达 reset；DISABLED/COOLING → 幂等不动电机；FAULT → 紧急直达 reset；INIT/TEACH/SERVO/AI → 拒绝 busy；SAFE_PARK → 拒绝「already safe parking」。park 超时 → **FAULT 不 reset**（保持使能、停在半途，人工介入）；reset 被 gate 拒 → park 前 `success=false` + 原文 + "(stop power sequence first)"，park 完成后被拒 → 回 READY（已在 home 位，安全）；`_have_js==False` → 直达 reset + WARN（保持旧行为）。`/a3/motor/reset` 直达保留作紧急失能。
- **验收标准：**
  1. 容差外 disable：`READY → SAFE_PARK`（`_traj_done_at` 清零防旧 TRAJ 时间戳误回 READY）→ 收敛连续 0.5 s → reset → `DISABLED`，最终位姿全部在 home ±0.15 rad 内
  2. 容差内 disable：跳过 park 直达 reset；park 超时：FAULT 且电机保持使能、不 reset
  3. disable 后 7 电机 mode_status=0；DISABLED 下运动命令被拒（`_can_move` 拒绝表，F45）
- **关联：** F45（SAFE_PARK/DISABLED 状态）、F41（park 轨迹复用统一插值）；[shared/SAFETY.md](../shared/SAFETY.md)（失能保护）；[LL-026](../../lessons_learned/LL-026-txstats-window-and-torque-latch-release.md)（park 被残留力矩 latch 钉死风险）
- **状态：** `completed`（2026-09-13 真机验收：READY 位 disable → `[state change] READY -> SAFE_PARK (safe park -> home (3.0s, 150 pts))` → `disable -> success=True 'safe park -> disabled (0.9s)'` → `SAFE_PARK -> DISABLED`；最终位姿 [-0.0044, -0.0006, -0.0171, +0.3336, +0.0125, +0.0056, -0.0002] 全 ≈ home、7/7 失能。实测 0.9 s 即返回——tol 判据在 park 中途即满足、提前 reset、重力把臂荡回平衡位，符合设计）

### F41 — move_to 时长兜底 + 插值密度（≥50 Hz）

- **说明：** move_to 允许 0.05 s 极短时长时插值仅 ~21 点（≈8 Hz，阶跃感明显）——新增参数 `move_to_min_duration_s: 3.0`、`move_to_points_hz: 50.0`、`move_to_max_points: 5000`；新辅助 `_traj_point_count(duration_s)` = max(goto_waypoints, min(ceil(duration×hz), max_points))，三处统一：`_move_to_cb`（duration=max(duration, min) 再 clamp 0.05..60，3 s → 150 点）、`_goto_cb`（goto_duration_s）、`_playback_cb` F38b ramp 段（2.5 s → 125 点）。150 点×7 关节 reliable QoS 无压力。
- **验收标准：**
  1. move_to 请求 0.5 s → 响应回显 `(3.0s, 150 pts)`
  2. goto 3.0 s → 150 点；playback ramp 2.5 s → 125 点；全部 ≥50 Hz
- **关联：** F40（park 轨迹复用同一插值）、F38b（ramp 段）；F46（帧率实测同轨迹）
- **状态：** `completed`（2026-09-13 真机验收：0.5 s 请求回显 3.0s/150 pts；goto/ramp 点数达标；同轨迹 F46 帧率实测 195 Hz/关节）

### F42 — 力矩方向钳位（执行层 latch，碰撞保护）

- **说明：** 执行层（`motor_protocol_node`，权威，插在 ClampMotorCommand 之后）按**方向性判据**做碰撞保护：反馈力矩 |τ| ≥ 阈值（`torque_protection_limit_nm: [5,5,5,3,3,3,3]`，RS00 5 / EL05 3，与 LL-024 codec 量程同源）→ trip 并 latch τ 符号；冻结 = 把目标钉在反馈位（τ>0 → target=min(target, fb)；τ<0 → target=max(target, fb)，MIT 约定 τ≈kp×(target−actual)）；释放 = (mapped−fb)×latch_sign < 0 且 |mapped−fb| > `release_margin`(0.02 rad)——即「增矩方向冻结、反向放行」（无 latch 会形成 0→3 Nm 周期极限环）。kp≤0.01（zero_torque/播种）、反馈不新鲜时清 latch 不钳位；**收到新轨迹时清空全部 latch**（新轨迹 = 新意图；保护不减弱——阻力仍在时钳位会在一个 tick 内按反馈力矩重新 trip）。WARN 日志关键字 `F42 torque clamp trip: motor=%u tau=%.2f limit=%.2f fb=%.4f`（1000 ms 节流）。与 refresh 流天然兼容：refresh 持有钳位后的 `last_commanded_mit_rad_` → 轨迹结束后自动保持冻结位；zero_torque/stop 重锚定不冲突。不做「持续超限 → FAULT」升级（执行层无状态机；编排层可观测 `mtq_L{n}` 扩展）。
- **验收标准：**
  1. 手扶顶住关节（反馈力矩 ≥ 阈值）时该关节目标冻结在反馈位、不再朝阻力方向推进；反向目标放行
  2. 阻力消失后关节继续跟踪轨迹；新轨迹不受残留 latch 影响
  3. kp≤0.01（zero_torque 拖动）或反馈不新鲜时不钳位
- **关联：** [shared/SAFETY.md](../shared/SAFETY.md)（力矩方向钳位）；LL-024（阈值按型号）；固件 0x700B 硬钳兜底
- **状态：** `completed`（2026-09-13 真机验收：L4 手扶受控 trip −3.01 Nm 冻结在反馈位、力矩塌陷后释放继续走完；L6 自然 trip −3.01 同语义（WARN 日志两条与 ~/.a3/stats/torque_stats.yaml 时间戳吻合）。**期间发现并修复 LL-026 缺陷**——慢速跟踪滞后仅 ~0.006 rad < release margin 0.02，残留 latch 把新轨迹钉死（回程 L4/L6 各只动 ~0.03 rad 即停、日志无新 trip）→ 新轨迹清 latch 修复后同一回程完整走完（L4→0.1605、L6→+0.0002））

### F43 — 最大力矩持久化 + MQTT mtqmax

- **说明：** 编排层新增订阅 `/a3/motor/states`（QoS 复用 js_qos best_effort），记录每关节历史最大力矩（`max_abs`/`max_pos`/`max_neg` + 时间戳，仅 fresh+finite 更新），dirty 且 ≥ `torque_stats_save_interval_s`(10 s) 节流落盘 `~/.a3/stats/torque_stats.yaml`（启动恢复、destroy 落盘，复用 gripper_overrides 模式）。`ArmStatus.msg` 追加 `float64[] max_torques`（无数据 0.0，不用 NaN）。MQTT（bridge.yaml）：`/a3/arm_status` 第二条 rule，`flatten: joint_state, fields: [max_torques], prefixes: [mtqmax]` → `mtqmax_L1..L7`（同话题多 rule 支持）。
- **验收标准：**
  1. 运动/保位后 yaml 有值且节点重启恢复
  2. telemetry `mtqmax_L1..L7` 上行与 yaml 一致
- **关联：** F42（trip 事件的持久化证据）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（ArmStatus 字段）
- **状态：** `completed`（2026-09-13 真机验收：当日两次 F42 trip 已持久化——L4 max_abs 2.995@13:09:05、L6 max_abs 2.997@13:09:11 与桥日志 WARN 时间戳吻合；MQTT mtqmax 全键上行验证通过）

### F44 — 温度管理（warn / protect / COOLING）

- **说明：** 参数 `temp_protect_enabled: true`、`temp_warn_c: 90.0`、`temp_protect_c: 95.0`（2026-09-13 由 65 上调——官方电机自带 130°C 兜底，65 使 ready 位保位发热几分钟即误触，LL-023）、`temp_hysteresis_c: 5.0`。warn（≥90，fresh 门控）：置 `temp_warn` + WARN 日志 + `arm_temp_warn` 遥测，不动状态机。protect（≥95 任一 fresh 关节）：READY/TRAJ → 复用 F40 流程 safe park → **COOLING**（message 带关节与温度）；IDLE/DISABLED/COOLING → 直接 COOLING；SAFE_PARK 进行中不打断；reset 被 gate 拒 → **FAULT**(overtemp reset refused)（温度保护不可放弃）。重新使能：COOLING 下 `_enable_cb`/`_init_cb` 先查全 fresh 关节 < protect−hysteresis 才放行；无 fresh 关节不阻碍 + WARN。顺带电机故障监视：fault_mask≠0（含固件过温锁存 bit3）→ reset 广播 + FAULT。`ArmStatus.msg` 追加 `float64[] temperatures`、`bool temp_warn`；MQTT scalar rule fields 扩展 `temp_warn` → `arm_temp_warn`。
- **验收标准：**
  1. 超保护阈 → 自动回 home → 失能 → COOLING；降温至保护阈−迟滞前 enable 被拒
  2. warn 级仅告警不打断运动
  3. 无反馈（fresh=false）时温度判读不生效——温度=0.0 不是 NaN，断连不得被误判「已冷却」放行使能（LL-011 教训）
- **关联：** F40（复用 safe park）、F45（COOLING 态）；[shared/SAFETY.md](../shared/SAFETY.md)（温度策略）；LL-023（阈值放宽）
- **状态：** `completed`（2026-09-13：阈值上调后两 yaml + 代码默认值同步、重编重启回读 95.0/90.0 确认；保护路径（overtemp → safe park 回 home 落点误差 <0.003 rad → 7/7 失能 → COOLING → L3 卸力快速降温）已于 65°C 旧阈值时代真机触发并二次复现，状态转移与遥测一致）

### F45 — 状态机增强（11 态）

- **说明：** 状态全集扩为 11 态：`IDLE/INIT/READY/TRAJ/SERVO/TEACH/AI/SAFE_PARK/DISABLED/COOLING/FAULT`。转移：disable 非 home → SAFE_PARK→DISABLED；温度保护 → SAFE_PARK→COOLING；park 超时/reset 被拒/motor fault → FAULT(reason)；enable（COOLING 已降温）→ READY。`_can_move()`/`_set_joint_positions_cb` 拒绝表加 SAFE_PARK/DISABLED/COOLING（disable 后 move_to 被拒，原 IDLE 允许的语义混乱消除）；`_publish_mode` 兜底：SAFE_PARK→TRAJ_RUNNING、DISABLED/COOLING→IDLE（防互锁锁存，F29 教训）；`_publish_status` 填 temperatures/max_torques/temp_warn。IDLE 语义收窄为「上电未初始化」，DISABLED =「曾使能已失能须显式 enable」。
- **验收标准：**
  1. 完整转移链实测：READY→SAFE_PARK→DISABLED→enable→READY；READY→SAFE_PARK→COOLING→降温→enable→READY
  2. DISABLED/COOLING/SAFE_PARK 下运动命令被拒；arm_state 遥测与状态一致
- **关联：** F40/F44；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（11 态转移表）
- **状态：** `completed`（2026-09-13 真机：F40 路径 READY→SAFE_PARK→DISABLED 与 F44 路径 overtemp→SAFE_PARK→COOLING 实测转移、MQTT arm_state 遥测一致；`_publish_mode` 兜底经 F29 路径回归）

### F46 — TX 帧率监视（TxStats + MQTT txhz）

- **说明：** 新 msg `a3_can_bridge/msg/TxStats`（CMake 注册）；`motor_protocol_node` 按电机计数发送帧（SendMitFrame/OnTxRefreshTimer/mit_hold 路径递增），5 s 窗口定时器**先发布 `/a3/motor/tx_stats`（SensorDataQoS）再清零**（`window_s` 用实测 tick 间隔）：`tx_traj_total`/`tx_refresh_total`（及 can0/can1 拆分）、跳过计数 `skip_max_rate`/`skip_bus_disabled`/`skip_power_gate`（定位帧丢失）、`float64[] tx_hz`（count/window_s）、`bool tx_rate_ok`（仅 `tx_traj_total>0` 时校验 `tx_hz[i] ≥ tx_rate_ok_ratio(0.9)×min(200, max_tx_rate_per_motor_hz)`——静止期只有 refresh 属正常）、`string[] joint_names`。参数 `publish_tx_stats: true`、`tx_stats_topic`、`tx_rate_ok_ratio: 0.9`。MQTT：两条 rule（joint_state 展平 `txhz_L1..L7`；scalar 展平 `tx_window_s`/`tx_traj_total`/`tx_refresh_total`/`tx_rate_ok`）。不做 `ip -s link` berr 计数（can_transport 职责，二期）。
- **验收标准：**
  1. 3 s/150 点 move_to 期间 `tx_hz ≈ min(200, max_rate)` 且 `tx_rate_ok=true`；静止保持期 `tx_hz ≈ refresh 频率`（对照 ~50 Hz/关节）且不误报 `tx_rate_ok=false`
  2. `ros2 topic echo /a3/motor/tx_stats` 可读全部字段
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（/a3/motor/tx_stats + MQTT 展平）；F22 示教卡顿定位（skip 计数器）；[LL-026](../../lessons_learned/LL-026-txstats-window-and-torque-latch-release.md)（窗口旋转误读）
- **状态：** `completed`（2026-09-13 真机验收：3 s/150 点轨迹 tx_traj=4098 帧 ≈195 Hz/关节（99% 交付、限速丢弃 46 帧 0.7%）、refresh 1652/5s 并行、静止对照 47.2 Hz/关节、`tx_rate_ok=true`；**注意 tx_stats 5 s 窗口旋转会把轨迹尾巴切到下一窗口**（首次读 tx_traj_total=7 误导），读帧率须对照同时段桥日志 TX window 行，LL-026）

## 非功能需求

| 指标 | 要求 |
|------|------|
| 板内控制环延迟 | &lt; 10 ms（ROS 节点到 CAN 发送） |
| 关节状态频率 | 50 Hz 稳定发布 |
| CAN 比特率 | 1 Mbps |
| CAN 发送上限 | 200 Hz / 电机（可限流） |
| 操作系统 | Ubuntu 22.04 + ROS 2 Humble |
| 启动时间 | 电源序列完成后 &lt; 5 s 可接受轨迹 |

## 硬件依赖

- RK3588 开发板（已验证 LubanCat-4-V1）
- CAN 收发器（40PIN TX/RX 或板载 CAN）
- Device tree overlay 启用 `can2-m0`（注册为 `can1`，板载收发器，控臂总线）；可选 `can0-m0` 及多臂 `can1`..`can3`
- 控臂总线由 `a3_can_bridge/config/motor_map.yaml` 的 `arm_bus` 单一参数决定（默认 `can1`），切换只改此处并重启栈
- 7× MIT 协议电机，ID 1..7，主机 ID 0xFD

## 边界与不做事项

- 不在 WSL2 上调试板载 SocketCAN 真电机
- 不与 MotorBridge 同时占用同一 `can1`
- 不把 Windows 编译产物直接部署到 ARM 板
- CloudEdge 薄边缘形态不在本产品线范围

## 验收标准

1. `can-up.service` 启动后 `can1` 为 UP，1 Mbps
2. `ros2 launch a3_bringup a3_bringup.launch.py` 无致命错误
3. PS4 启动后 `/power_sequence/gate_open` 为 `true`
4. 测试轨迹（见 [QUICKSTART.md](QUICKSTART.md)）在 2 s 内完成运动
5. F60：Cross(X) 长按 1 s 硬急停后 gate 关闭；R3 失能（F40）；L3 一键 start+enable 到 READY
6. MoveIt demo 可规划（mock 或真机模式）
7. F6–F9：Wave A 见 [dev/WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)
8. F10–F15：见 QUICKSTART Wave B / [dev/WAVE_B_SIM_NOTES.md](../dev/WAVE_B_SIM_NOTES.md) / [dev/WAVE_B_SIM_TEST_REPORT.md](../dev/WAVE_B_SIM_TEST_REPORT.md)
9. F16：`ros2 launch a3_bringup edge_teleop_sim.launch.py use_rviz:=true`；先 `joy_dump` 核对轴序

## 关联文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [QUICKSTART.md](QUICKSTART.md)
- [PLATFORM_CAN.md](PLATFORM_CAN.md)
- [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- [shared/SAFETY.md](../shared/SAFETY.md)
- [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
