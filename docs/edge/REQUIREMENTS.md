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
- **状态：** `implemented`（`servo.launch.py` + mode bridge；依赖 `moveit_servo`；**真机电机入环见 F65**——2026-09-21 前真机在 SERVO 模式互锁丢弃 servo 轨迹）

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
- **状态：** `completed`（2026-09-21 仿真：全新栈 F62 合成 46/0；含 L1/R1 通道隔离、松手 Δ<0.03、D-pad 步进 0.15/clamp 0.1、持续按住不连发；真手柄真机验收待办）

## F65 真机 Servo 入环（独立 servo 话题）+ L3 使能幂等

- **说明：** 2026-09-21 真机「L1+摇杆完全无响应」根因：MoveIt Servo 的 50 Hz 单点帧原本与编排层多点轨迹共用 `/joint_group_effort_controller/joint_trajectory`，而真机 `motor_protocol_node` 自 8ce7f58 起在 `control_mode=SERVO` 时**主动丢弃该话题全部消息**（模式互锁），故仿真能动、真机被静默丢弃。同时 F60 L3 一键使能在电源序列 `EnableInit` 已使能电机（`send_enable_in_startup_=true`）后，再调 `/a3/motor/enable` 被 F32 gate 互锁拒绝，臂停在 DISABLED 看起来「按键没反应」。修复方案（通道分离而非放开互锁）：
  1. Servo 输出改走**新独立话题** `/a3/servo/joint_trajectory`（`servo_config.yaml command_out_topic` + 三处 launch remap），高频单点帧与编排层多点轨迹物理隔离、互不抢占。
  2. `motor_protocol_node` 新增 `servo_trajectory_topic` / `servo_target_timeout_s`（0.3 s）订阅：仅在 **gate 开 + 非零力矩 + `SERVO` 模式**时缓存最新单点，200 Hz 插值 tick 走 SERVO 早分支复用既有 `ApplyPositionTargets()`（同一套平滑/ClampJointCommand/力矩钳/MIT 发送）；不满足条件 WARN_THROTTLE 丢弃并计入 `servo[cb/apply/drop]` 窗统计；目标 0.3 s 过期则不刷新（MIT 保持最后目标），松摇杆由 servo halt/bridge 回 IDLE 兜底。**主轨迹话题在 SERVO 模式下的丢弃互锁原样保留**。
  3. `a3_arm_monitor` 同订阅新话题（`servo_traj_topic` 参数），jog 中同步更新保持参照，避免 HOLD_DRIFT 在 SERVO 模式误跳。
  4. `ps4_mapper` tick_end：检测到移动 twist 且 servo 未启动时**按需调用** `/servo_node/start_servo`（服务未就绪静默失败、下一 tick 重试）——真机 launch 保持 `auto_start_servo:="false"`，servo 不再常驻。
  5. L3 使能幂等：`arm_controller` 从 `/a3/motor/states` 跟踪每电机使能位与数据新鲜度；F48 等检查通过后，若 7 电机均已使能且状态新鲜，则直接回 READY（message `motors already enabled by power sequence EnableInit -> READY`），不再调用会被 F32 拒绝的 enable 服务。
  6. 仿真 `sim_motor_node`/`sim_executor` 同加第二订阅（不做 gate/SERVO 互锁，仿真有意简化）。
- **验收标准：**
  1. 仿真全新栈 F62 合成脚本 46 PASS / 0 FAIL（含 S5a-d servo jog 走新话题、S9/S10 恢复链）
  2. `auto_start_servo:=false` 下：不推摇杆时新话题 0 帧；按住 L1+摇杆后 mapper 日志出现 `called /servo_node/start_servo`，新话题 ~50 Hz 出帧，非奇异 ready 位关节随动（实测 6 s L2 +0.225 / L3 −0.179 rad）
  3. 话题拓扑：`/a3/servo/joint_trajectory` 1 publisher（servo_node），订阅者 motor_protocol_node + a3_arm_monitor；主轨迹话题在 SERVO 模式仍被互锁
  4. 电源序列 EnableInit 后按 L3：直接 READY 绿灯，无 gate 拒绝日志
  5. **真机待验（断电未测）**：上电开机 → L3 绿 → 先按 Circle 回 home（避开零位奇异 LL-007）→ L1+摇杆运动；motor 日志窗 `servo[cb= apply=]` 计数增长、`drop=0`
- **关联：** 修订 F14（真机入环路径）；F60（L3 语义）、F32（gate 互锁保留）、F62（仿真验证）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（新话题与仿真分歧说明）；[shared/SAFETY.md](../shared/SAFETY.md)；LL-007（零位 servo 奇异）
- **状态：** `in-progress`（2026-09-21 代码完成、编译通过、仿真 46/0 + 按需启动链路验证；真机板测待用户上电确认）

## F66 使能意图安全（gate 关作废全部意图 / 任意使能沿强制重锚 / 恢复死锁解锁）

- **说明：** 2026-09-22 真机第二次甩断 L6/L7（LL-039 同类复发）：X 长按硬急停由 `power_sequence_node` 直发裸 CAN disable，L3 上电由 EnableInit 直发裸 CAN enable（`SendCmdAll(0x03)`），**两条路径都绕过 `/a3/motor/enable` 服务**——F51 的「新鲜反馈重锚 + 软起步」只挂在服务路径；且真机三份 `control_gains.yaml` 把 `enable_mode_rising_smoothing` 置 false。人工搬臂回 home 后裸使能，执行层 200 Hz refresh 立刻以陈旧目标（事故值 L2=1.0839 rad）+ kp=80 续发，臂被甩回失能前位姿。其后看门狗在使能逐电机传播窗口内误报 UNEXPECTED_DISABLE（READY 后 19 ms），编排层跳 DISABLED 而 gate 仍 Running、电机仍锁存使能（LL-042），再按 L3 的 enable 服务又被 F32 拒绝 → 橙灯死锁。修复：
  1. **gate 关闭沿 = 全部运动意图作废**（`motor_protocol_node::OnPowerGate` 新增 `InvalidateAllMotionIntent()`）：清活动插值轨迹、servo 单点缓存、全部 7 电机 `last_commanded_mit_rad_`→NaN、置 `hold_suppressed_`、`has_latest_input_`=false（LL-040 回退表同源副本）、清 enable ramp；gate 关期间 refresh 只发零增益保活帧。
  2. **使能模式上升沿无条件 F51 重锚**（反馈处理，0/未知→使能态）：以当前新鲜反馈重锚目标、清回退缓存、重置平滑锚点、启动 kp/kd 软起步；陈旧目标偏差超 `enable_reanchor_tolerance_rad`(0.15) 打 ERROR。不再受 `enable_mode_rising_smoothing` 开关控制，服务使能与裸 CAN 使能同等保护；桥启动时电机已锁存 mode=2（LL-042）同样在首帧反馈沿重锚。
  3. **refresh 防甩兜底**（新参数 `stale_target_snap_guard_rad: 0.25`）：无活动轨迹/servo、电机使能反馈新鲜，但有限目标与实测偏差 >0.25 rad 且无软起步在身 → 拒绝拉拽，重锚实测位并启动软起步（正常保持残差 <0.02 rad）。
  4. **F32 恢复通道**：gate Running + `control_mode=IDLE` + 无活动轨迹时允许 `/a3/motor/enable`（command=1，走完整 F51 重锚+软起步），解锁「编排层 DISABLED/橙灯 但电源序列仍 Running」的 L3 恢复；reset/set_zero/save_param 仍严格拒绝。
  5. **电源序列使能帧补发**：EnableInit 保持期内每 50 ms 重发 0x03（幂等），消除与 EPScan 参数写同窗口竞争导致的漏帧/逐电机使能空洞。
  6. **看门狗使能建立宽限**：`none/partial→all` 使能沿（partial 也认）后 `hold_rebaseline_grace_s`(2.0 s) 窗口内豁免 UNEXPECTED_DISABLE；真实失能在宽限 + 0.5 s sustain 后仍必捕获。
- **验收标准：**
  1. `scripts/a3_test/f66_gate_enable_regression.py`（真 motor_protocol_node + mock CAN，精确复刻事故时序：旧位姿建目标→gate 关+裸失能→搬回 home→带外裸使能→gate 重开）通过：重开后所有 MIT 帧目标 ≤0.10 rad 贴 home、kp 从 0 软起步、mock 关节不被甩向旧位姿；旧二进制同脚本必失败（已验证：目标 1.090、首帧 kp=80、mock 被甩到 1.090 = 真阳性）
  2. `./scripts/a3_test/a3_test.sh incident` 全套通过（F51/F66/F50 看门狗/F52）
  3. PS4 全新栈 F62 46/0（含 S9「gate 仍开 L3 恢复 READY」）
  4. **真机待验（L6/L7 机械修复后）**：X 失能→人工挪臂→L3，臂不跳回旧位姿，电机日志每个电机出现一条 `F66 enable rising edge ... 重锚`；gate 关日志出现 `F66 motion intent invalidated`；看门狗不再在使能瞬间报 UNEXPECTED_DISABLE
- **关联：** F51（使能安全三要素，本需求把覆盖范围从服务路径扩到全部使能路径）、F32（gate 互锁恢复语义）、F60/F65（L3 幂等）、F58（stop 重力保持语义，回归判据同步更新）；[shared/SAFETY.md](../shared/SAFETY.md)；LL-039（首起甩臂事故）、LL-040（回退表缓存）、LL-042（电机锁存最后命令）、LL-043（启动禁满增益）、LL-070（本次事故，真机复测后补录）
- **状态：** `implemented-pending-hw`（2026-09-22 代码 + 编译 + mock-CAN 真桥回归 + 46/0 仿真；待 L6/L7 机械修复后真机复测）

## F67 goto/move_to 走 move_group + TOTG（工业轨迹，起止零速）

- **说明：** 现状 `_goto_cb`/`_move_to_cb` 用 `alpha=i/(n-1)` 线性插值：速度方波（起止瞬间加速度无限大）、点上无速度/加速度，电机跟随表现为起步/停止顿挫，即用户反馈的"不丝滑"。改为工业标准路径：编排层作 MoveGroup action 客户端（`move_action`，goal `MoveGroup.Goal`），`MotionPlanRequest` 给关节空间目标（`JointConstraint` 逐关节 = 目标位），group=`arm`（L1–L6）；规划管线 `default_planner_request_adapters/AddTimeOptimalParameterization`（TOTG）已在 `ompl_planning.yaml` 配置——几何路径 + `joint_limits.yaml` 的 v/a 限位自动算出时间参数化轨迹（梯形速度、起止速度为 0、连续加速度），执行经 MoveIt 控制器管理 → FJT action → `/joint_group_effort_controller/joint_trajectory`（执行层 200 Hz 插值不变）。L7（夹爪）不在 arm group：目标含 L7 时由编排层另行夹爪命令/直通保持（回放见 F68），goto 不改变 L7。新参数：`goto_use_moveit: true`（false = 旧线性插值兜底）、`moveit_goto_timeout_s: 15.0`、`moveit_action_name: move_action`。move_group 不可用/规划被拒/超时 → 自动回退本地线性插值并打 WARN（服务不报错，语义保持"尽力到位"）。
- **验收标准：**
  1. 全栈（`use_moveit:=true`）Triangle→ready：move_group 规划成功，执行轨迹首末点速度 ≈ 0（|v_end| ≤ 0.02 rad/s），`/joint_states` 数值微分的速度曲线无方波跳变、峰值受 max_velocity 限幅
  2. Circle→home 同标准；最终关节误差 ≤ 0.02 rad
  3. `use_moveit:=false`（move_group 不在）时自动走本地插值，服务仍 success、WARN 日志记录回退
  4. `use_servo:=true` 与 move_group 进程共存：goto 期间 Servo 输出不被消费（状态机仲裁），goto 结束后 servo jog 正常
- **关联：** F68（回放重定时）、F65（servo 独立话题/模式仲裁）、F38（goto 服务门面）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；LL-071
- **状态：** `done（仿真）`（2026-09-22，scripts/a3_test/f67_f68_sim_acceptance.py，14/14 ALL PASS：goto ready/home 走 move_group、首末速度≈0、限位内；线性兜底验证后恢复；真机验收待上电）

## F68 示教回放走 Ruckig/TOTG 在线重定时（保几何、退役手搓平滑）

- **说明：** 现状回放对点列做中心滑动平均（`_smooth_points`）+ 弧长重采样/加速度限时长（`_time_warp_points`）——手搓链路复杂、仍非连续加加速度，用户评价"各种折腾，效果反而不好"。改为工业标准：**几何保持的重定时（re-timing）**。50 Hz 稠密录制点不得交 OMPL 重规划（几何路径会变），只重算时间/速度/加速度：新增 C++ 包 `a3_trajectory_processing`（moveit_core `trajectory_processing` Python 不可绑定，无 moveit_py），服务 `/a3/arm/retime_trajectory`（`a3_msgs/srv/RetimeTrajectory`：输入 `trajectory_msgs/JointTrajectory`（仅位置）+ backend `ruckig|totg` + v/a 缩放；输出重定时 JointTrajectory，含 velocities/accelerations/稠密 time_from_start）。默认 backend=**Ruckig**（jerk-limited，起止零速、加加速度有界，最丝滑；系统已装 ros-humble-ruckig），TOTG 备选。限位取自 `joint_limits.yaml`（launch 注入 v/a map，jerk 默认 5×accel）。**关于录制速度：不需要保存**——Ruckig/TOTG 只吃位置点 + 关节限位，速度/加速度/加加速度全部由算法重算；已存 yaml 的 `time_from_start_sec` 时间戳在重定时中丢弃。回放流程：载入点列 → 末端拼接「当前位→首点」短 ramp（同样送重定时，保证整段连续）→ retime 服务 → 下发；L7 随 7 关节点列一同重定时（限位同源）。新参数：`playback_retime: true`（false 时保留旧 smooth/time_warp 作兜底，默认走新链路；旧参数保留不删）。
- **验收标准：**
  1. 示教一段任意轨迹（含变速/停顿）→ Square 回放：逐关节几何路径与录制点一致（重采样后位置偏差 ≤ 0.01 rad），但时间曲线重排：起止速度 ≈ 0、|a| ≤ joint_limits×缩放、jerk 有界
  2. retime 服务单测（ros2 service call）：输入线性/折线点列 → 返回带 velocities/accelerations 的轨迹，总时长随 v_scaling 单调变化
  3. ruckig 失败自动降级 totg；retime 节点/服务不可用 → 回退旧链路（WARN），回放不中断
  4. 录制 yaml 仍只存 positions + time_from_start_sec（格式不变，旧文件可直接回放）
- **关联：** F67（goto 同体系）、F22（示教/回放门面）、F38b（ramp 段）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；LL-071
- **状态：** `done（仿真）`（2026-09-22，14/14 ALL PASS：Ruckig 回放 4.41s vs 录制 4.0s，101/101 几何点匹配、最大偏差 0.0034 rad、端点误差 0.0001 rad，无超速/超加速；旧链路几何 0.0045 rad 作回归对照；Ruckig 单步 jerk 绑定致 29.6s 拉伸的根因与修复见 LL-071 坑 5；真机验收待上电）

## F69 ready 点位改为非腕奇异形（修复伺服 L1 驱动整臂变软下坠）

- **说明：** 真机 READY 下按住 L1 推摇杆，臂无视指令方向在重力下缓慢下坠、且可被外力自由拖动。bag 证据（2026-09-22）：伺服全程 kp=80/kd=2 未变（common SendMitFrame），根因是当前运行时 ready 来自用户覆盖 `~/.a3/poses.yaml`（2026-09-13 手动抬臂按反馈保存 `[0.07,1.0839,-0.5649,-0.4617,0.0831,-0.0497,0]`），L5≈0.083/L6≈−0.05 腕关节近共线 → MoveIt Servo 奇异速度缩放（threshold 17/30）全程触发 status 1/6，twist 被缩放近零 → 每 20 ms 目标≈反馈（|Δ|≈0.003 rad）→ kp 回复力 ≈0.24 Nm ≪ 该位形抗重力所需 ~3.4 Nm，重力获胜（L3 每集下坠 +0.36~0.42 rad，与指令方向无关）。修复：把 ready 覆盖为包内/SRDF 既有 ready `[0,0.785,-1.57,0,0.785,0,L7 保持]`——L5=0.785 将腕折出共线，且 L3=−1.57 前臂近竖直、重力力臂小。home（L5=L6=0）仍腕奇异不可用。改动仅 `~/.a3/poses.yaml` 数据，不改代码/契约。
- **验收标准：**
  1. 仿真闭环：goto ready 到位后按住 L1 推各方向摇杆 ≥3 s，/servo_node/status 全程 0（无 1/6），关节反馈无下坠（|Δ| ≤ 0.03 rad/松杆后回位）
  2. 真机：named pose ready 到位（各关节与目标 ≤0.03 rad），L1+摇杆各方向无重力下坠、外力不可自由拖动；bag 复核 status=0、|target−fb| 正常
  3. F40 失能保护回 home 不受影响（home 数据不改）
- **关联：** F65（servo 入环）、F64（L1 平移死人开关）、F42（力矩钳，本次无 trip）；[shared/SAFETY.md](../shared/SAFETY.md)；LL-069（实现验收后补录）
- **状态：** `in-progress`（2026-09-22）

## F70 ros2_control 标准栈仿真激活（JTC/JSB/controller_manager 取代手搓 FJT/插值）

- **说明：** 审计结论（用户：「自己折腾总会出错，尽量复用现有工具」）：现行执行链是手搓三件套——`a3_fjt_action`（自研 FJT action 桥）+ `motor_protocol_node` 内 200 Hz 插值 + 自研 /joint_states 反馈，MoveIt Execute 实际落到自研 action 再转 effort 话题。工业标准路径是 **ros2_control 标准栈**：`controller_manager`（200 Hz update loop）+ `joint_state_broadcaster`（标准 /joint_states）+ `joint_trajectory_controller`（官方 JTC，样条插值、容差监控、FJT action 原生暴露）；MoveIt `moveit_simple_controller_manager` 的 `arm_controller`/`gripper_controller` 命名空间与 JTC 默认 action（`<ns>/follow_joint_trajectory`）天然一致——move_group 可**直连**官方 JTC action，自研 `a3_fjt_action` 整条删除。仿真用 `mock_components/GenericSystem`（xacro 已具备 `use_mock_hardware:=true` 分支，`mock_sensor_commands=false`）替代三个自研 sim 节点。**本需求为增量式**：新增 `edge_ros2_control_sim.launch.py`（与旧栈并存，不删旧节点/旧 launch），真机迁移（SocketCAN/MIT 协议的 `hardware_interface::SystemInterface` 插件）另立需求。控制器配置 `el_a3_controllers.yaml` 已就绪但从未激活；**不加载其中 zero_torque_controller**（a3_can_bridge 自研插件，属旧栈）。
- **验收标准：**
  1. 新 launch 启动无致命错误：controller_manager 加载 joint_state_broadcaster + arm_controller（JTC，L1–L6）+ gripper_controller（JTC，L7）三个控制器且状态 active
  2. joint_state_broadcaster 以稳定频率发布 /joint_states（7 关节 name/position/velocity 齐全）
  3. 直接向 `/arm_controller/follow_joint_trajectory` 发 action 目标（home→ready→home）：样条轨迹平滑、到位（per-joint goal 容差内），action 成功返回；gripper_controller 同理可动 L7
  4. move_group plan+execute 端到端：经官方 JTC 完成 goto（全程无 a3_fjt_action 进程）
  5. 数值验收脚本：起止速度≈0、v/a 不超限、目标误差 ≤0.02 rad；结论 ALL PASS
- **关联：** F67/F68（同 MoveIt 体系，本次替换执行底座）、审计任务（自研→标准工具）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；真机 SystemInterface 插件（后续需求）
- **状态：** `completed（仿真）`（2026-09-22 仿真验收 11/11 ALL PASS，ROS_DOMAIN_ID=58：JTC 直连 home→ready→home（31 点五次 S 曲线），vmax ≤0.52、amax ≤2.77、目标误差 0.0000；夹爪开合 vmax ≤0.78、amax ≤2.57；move_group plan+execute 经官方 JTC（9 点 TOTG 轨迹），vmax ≤2.09、amax ≤7.31、目标误差 ≤0.0093；栈内无自研 FJT 节点。踩坑见 [LL-072](../lessons_learned/LL-072-ros2-control-mock-jsb-order-jtc-single-point.md)。真机迁移 = SocketCAN/MIT SystemInterface 插件，另立需求）

## F71 arm_monitor 标准诊断通道（diagnostic_updater → /diagnostics）

- **说明：** 审计结论（用户：「自己折腾总会出错，尽量复用现有工具」）：`a3_arm_monitor` 只发布自定义 `MonitorStatus`，标准 ROS 诊断工具链（`diagnostic_aggregator`、`rqt_robot_monitor`、robot monitor 大屏）无法接入；故障分级（OK/PENDING/TRIGGERED）是工业诊断里现成的 `DiagnosticStatus.level`（OK/WARN/ERROR）。**增量改造，不动看门狗故障判定/处置阶梯逻辑**（LL-039/LL-053/LL-070 真机教训全部原位保留），仅新增官方包 `diagnostic_updater`（4.0.7）发布 `/diagnostics`，两个组件：`a3_arm_monitor: Monitor`（fault=ERROR / pending=WARN / OK，键值含 fault、pending_faults、action、last_event）、`a3_arm_monitor: Tracking`（各关节跟随误差 + max，超 follow 阈值 WARN，FOLLOW_STUCK/HOLD_DRIFT 触发时 ERROR）。组件名由 diagnostic_updater 自动加节点名前缀，add() 只给裸名。`MonitorStatus` 话题保留不变。参数 `publish_diagnostics`（默认 true）、`diagnostics_period_s`（默认 1.0）。
- **验收标准：**
  1. 健康态：`/diagnostics` 周期性出现两个组件且 level=OK，键值齐全；`/a3/monitor/status` 仍正常（OK）
  2. 故障态（js 停发 → STALE_JS 确认）：Monitor 组件 level=ERROR 且 fault 键值为 STALE_JS；恢复（js 复发并持续 clear_hold）后回 OK
  3. pending（条件成立未达 sustain）期间 level=WARN
  4. 数值验收脚本：上述 1–3 全自动，结论 ALL PASS
- **关联：** F50（看门狗本体）、审计任务（自研→标准工具）；F70（同标准栈体系）
- **状态：** `completed（仿真）`（2026-09-22 全自动验收 9/9 ALL PASS，ROS_DOMAIN_ID=59：健康态两组件 level=OK + MonitorStatus OK；js 停发 → pending WARN → STALE_JS TRIGGERED，Monitor level=ERROR fault=STALE_JS（319 条）；js 恢复 clear_hold 后两组件回 OK、MonitorStatus OK。看门狗判定/处置逻辑零改动。踩坑见 [LL-073](../../lessons_learned/LL-073-diagnostic-updater-name-prefix-byte-level.md)。真机随栈上电另验。F82 联验回归修复后复验 9/9 ALL PASS：修掉 b671ff9 的组件名双前缀回归（恢复裸名 add），并补「监控节点冷启动无参照 → Tracking 恒 STALE」缺口（启动播种保持参照），见 [LL-085](../../lessons_learned/LL-085-monitor-startup-baseline-seed-volatile-discovery-race.md)）

## F72 真机 SystemInterface 插件（MIT/SocketCAN 直驱，ros2_control 标准栈真机化）

- **说明：** F70 的真机迁移：新建 ament_cmake 包 `a3_hardware_interface`，提供 `hardware_interface::SystemInterface` 插件 `a3_hardware_interface/A3MITHardwareInterface`——controller_manager 直接打开 SocketCAN（默认 `can1`）、收线程解 MIT 反馈帧（类型 2 / 0x18），`write()` 直接打 MIT 控制帧；F70 的 JTC/JSB/move_group 配置**原样复用**，真机路径上 `motor_protocol_node`（3361 行）+ 200 Hz 插值器 + `/can_tx_frames` 话题中转整体被旁路。MIT 编解码**复用本仓真机验证过的 ProtocolCodec**（motor_model.hpp + protocol_codec.hpp 两份头文件 vendored 进新包，仅换命名空间；不重写协议常量），SocketCAN 传输按官方 el_a3_hardware 结构写单总线实现（SOCK_RAW、SO_RCVTIMEO、send mutex + ENOBUFS 重试、CAN_RAW_FILTER 只收反馈帧）。关节→电机映射取 URDF 既有参数：每关节 `motor_id`、`direction`（L1..L7 = −1,+1,−1,+1,−1,+1,+1，与 control_gains.yaml joint_signs 一致）、`position_offset=0`；硬件参数 `can_interface`、`kp`（默认 80）/`kd`（默认 2）、`command_rate_hz`（默认 200）、力矩/速度量程按 motor_id（1–3 RS00 ±14 Nm/±33 rad/s，4–7 EL05 ±6 Nm/±50 rad/s，LL-024）。生命周期：`on_configure` 打开 CAN 并起收线程；`on_activate` 先发 reset（清故障）再发 enable、随后以反馈位重锚指令（F51 语义）；`on_deactivate` 发零增益续流帧后 reset 失能。xacro 新增 `use_real_hardware` 分支（mock 分支不动）。**增量并存：不删旧栈、不改 F70 仿真栈。**
- **验收标准：**
  1. `vcan0` + 电机反馈仿真器（收到 MIT 指令帧→一阶跟随→回类型 2 反馈）下，真机 launch 启动无致命错误：插件 loaded/active，JSB 发布 7 关节 /joint_states（position/velocity 非全零）
  2. 直连 `/arm_controller/follow_joint_trajectory`（31 点五次 S 曲线 home→ready→home）与夹爪 JTC 开合：action 成功、到位（误差 ≤0.02 rad）
  3. move_group plan+execute 端到端经插件完成（无 motor_protocol_node 进程）
  4. 数值验收脚本：起止速度≈0、v/a 不超限（与 F70 同判据），结论 ALL PASS；CAN 线侧抓包确认指令帧位置=direction×joint+offset
- **关联：** F70（仿真标准栈，本需求真机化）、F51（使能重锚语义）、LL-024（量程按型号）；[shared/SAFETY.md](../shared/SAFETY.md)；官方 el_a3_hardware（结构蓝本，协议常量以本仓为准）
- **状态：** `仿真验收通过`（2026-09-22，vcan0 + `f72_ros2_control_vcan_acceptance.py` ALL PASS：JTC/夹爪/move_group 到位 ≤0.01、CAN 指令与反馈映射偏差 0、kp=80/kd=2；真机上电验收另约，见 LL-074）

## F73 力矩指令模式 + 标准重力补偿自由拖动控制器（对标官方 ZeroTorqueController，switch_controllers 切换示教）

- **说明：** 对标官方参考仓 `el_a3_hardware/ZeroTorqueController` 的工业级示教路径，取代手搓的 zero-torque 示教：
  1. `A3MITHardwareInterface` 增加力矩指令模式：覆写 `prepare_command_mode_switch` / `perform_command_mode_switch`，按接口名 `effort` 跟踪 `effort_mode_`；`write()` 在力矩模式下发 MIT 控制帧 **kp=0、kd=effort_kd（硬件参数，默认 2.0）、torque_ff=cmd_eff×direction**（关节空间力矩→电机轴：motor τ = joint τ / direction，direction=±1 等价乘；与官方一致）、位置域填当前实测电机角（保持）、速度域 0；力矩超量程由 codec 按 ±torque_max 钳位（RS00 ±14、EL05 ±6）。位置模式行为与 F72 完全不变。
  2. 同包新增 `controller_interface::ControllerInterface` 插件 `a3_hardware_interface/GravityCompensationController`：claim 各关节 `/effort` 命令接口 + position/velocity 状态接口；`on_configure` 优先从 controller_manager 节点 `robot_description` 参数取 URDF（兜底参数 `urdf_path`），`pinocchio::buildModelFromXML` 建模；`update()` 以实测 q 跑 RNEA `rnea(q,0,0)` 得重力矩写入 effort 命令，每 10 拍发布 `~/gravity_torque`（sensor_msgs/JointState）供观测。插件名/参数按官方 ZeroTorqueController 语义（joints 数组），不做自研惯性标定（URDF 惯性参数为准，后续可加标定文件）。
  3. `el_a3_controllers.yaml` 增加 `zero_torque_controller` 条目（L1–L6）；vcan launch 以 `--inactive` 预生成；示教进入/退出全部走标准 `ros2 control switch_controllers`（与 arm_controller 互斥），不再有自定义模式门。**增量并存：不改位置模式任何既有行为、不删旧示教代码路径。**
- **验收标准：**
  1. vcan0 栈启动后 `zero_torque_controller` 存在且 inactive；`switch_controllers --deactivate arm_controller --activate zero_torque_controller` 成功，控制器 active
  2. CAN 抓包：切换后每帧 kp≈0、kd≈effort_kd；torque_ff 与验收脚本用独立 Pinocchio（python bindings）RNEA 计算的重力矩（×direction）一致（误差 ≤0.02 Nm，仅量化误差）；多姿态（home/ready/中间位）逐一核对
  3. 切回 arm_controller 后 CAN 帧恢复 kp=80/kd=2，JTC home→ready→home 运动验收仍 ALL PASS（到位 ≤0.02）
  4. 全过程无节点崩溃、无自定义 FJT/插值节点；力矩帧值不超出 ±torque_max
- **关联：** F72（真机插件，本需求补全 effort 命令通路）、F70（标准控制器体系）；官方 `el_a3_hardware/ZeroTorqueController`（直接蓝本）；[shared/SAFETY.md](../shared/SAFETY.md)（示教安全）
- **状态：** `completed`（2026-09-22 vcan0 仿真验收，机械臂保持断电：`scripts/a3_test/f73_gravity_comp_vcan_acceptance.py` 41 项 ALL PASS——home/ready/mid 三姿态标准 switch_controllers 互斥切换全部 ok；CAN 帧 kp=0、kd=2.0、vel≈0、位置字段=实测位；torque_ff 对独立 Python-RNEA×direction 静态姿态最大偏差 0.0003 Nm；外力注入（motor3 +0.6 Nm ×1.5 s）L3 同号位移 −0.437 rad、10/10 步单调、全程 kp=0、运动中 torque_ff 偏差 ≤0.0013 Nm；撤力后稳定窗口间漂移 0.0008 rad；切回后 kp=80/kd=2/t_ff=0，JTC home→ready→home 落位 0.0003。坑见 LL-075）；真机验收待上电

## F74 编排层标准执行后端（control_msgs/FollowJointTrajectory action → JTC，参数门控）

- **说明：** F70–F73 建成 ros2_control 标准栈后，编排层兜底轨迹（goto/move_to 线性兜底、set_joint_positions jog、playback、safe-park）仍只往旧栈话题 `/joint_group_effort_controller/joint_trajectory` 直发，在标准栈上没有接收者。本需求统一执行入口，不重写任何轨迹生成逻辑：
  1. 新增参数 `control_backend`（默认 `"topic"`，旧行为零变化；`"fjt_action"` = 标准栈）、`arm_fjt_action`（默认 `/arm_controller/follow_joint_trajectory`）、`gripper_fjt_action`（默认 `/gripper_controller/follow_joint_trajectory`）。
  2. 新增 `_dispatch_trajectory(traj)`：topic 后端保持话题直发；fjt_action 后端把 7 关节轨迹按 JTC claim 集拆分投影（L1–L6 → arm FJT goal，L7 → gripper FJT goal；velocities/accelerations/effort 字段随点投影，时间戳不变），经两个标准 `control_msgs/FollowJointTrajectory` action 客户端异步发送；action 不可用/goal 拒绝/非 SUCCESSFUL 结束码均有 ERROR/WARN 日志。新 goal 抢占同 server 旧 goal，与旧栈话题替换语义一致；服务回调不阻塞，完成判定仍走既有时长调度与状态轮询。
  3. 五处 `self._traj_pub.publish(traj)` 全部改走 `_dispatch_trajectory`；move_group 成功路径（goto/move_to/safe-park 主路径）不经过本入口，行为不变。**增量并存：不删旧栈、默认参数不变。**
- **验收标准：**
  1. 默认（topic）后端：旧仿真栈行为零回归
  2. fjt_action 后端 + mock 标准栈（无 CAN/无电机）：jog（set_joint_positions）连续多次下发均到位（≤0.02），L7 同步运动；goto 线性兜底（关闭 moveit）到位；playback（含 retime）到位；safe-park 轮询收敛正常
  3. 全程轨迹只经标准 FJT action（无话题直发），无节点崩溃，抢占语义正确（连续 jog 不排队、不报错）
- **关联：** F70（JTC/标准栈）、F72/F73（真机/力矩栈）、F67/F68（上层规划/重定时，本需求是其兜底路径的执行落地）；审计任务 #10；LL-076（组外 L7 补发、单向关节夹紧）
- **状态：** `completed`（2026-09-22 仿真验收，F70 mock 双 JTC 栈，ROS_DOMAIN_ID=60，`scripts/a3_test/f74_fjt_backend_mock_acceptance.py` 12/12：显式 enable→READY；jog ×3（含 L7）落点误差 ≤0.0192；goto 线性兜底 ready/home 到位；playback 61 点正弦经 ruckig retime → 拆 arm/gripper FJT 执行，落 home err=0.0079；连续 jog 抢占语义正确（P1 dist=0.502 未跟踪）；旧话题全程零消息；disable safe-park move_group 后补发 L7 gripper 轨迹→全 7 关节收敛 home、reset 调用 1 次→DISABLED。L2/L3 单向限位目标必须在界内（JTC 越限静默夹紧）

## F75 全产品 mock-hardware 标准栈 bringup（零自研 sim 节点：控制器 inactive 启动 + 编排层 switch_controller 使能）

- **说明：** F70–F74 建成标准执行/规划栈后，「整体启动」仍走 `edge_web_sim.launch.py` 的三个自研模拟节点（sim_motor_node / sim_power_sequence_node / gravity_torque_node），且真机栈的使能语义（`/a3/motor/enable|reset|set_zero` 服务）在标准栈上没有对应物。本需求统一产品级 bringup 拓扑，与真机 F72 栈同构：
  1. 编排层新增参数 `motor_service_backend`（默认 `"can_service"`，旧行为零变化；`"controller_switch"` = 标准栈）：enable → `/controller_manager/switch_controller` STRICT 激活 `arm_controller` + `gripper_controller`（硬件插件 `on_activate` 内 reset→enable，见 F72）；reset → 同服务去激活两控制器（`on_deactivate` 零增益刷新 + MIT stop）；set_zero → 标准栈绝对编码帧无此操作，返回成功跳过。switch 是幂等操作，`_enable_cb` 在本后端不再依赖 `/a3/motor/states` 的使能镜像；`_init_cb` 在本后端跳过 set_zero 与零位确认，直接激活控制器（在当前反馈位重锚）。
  2. 新增 `edge_full_mock.launch.py`：xacro `use_mock_hardware:=true` → `mock_components/GenericSystem`；两个 JTC 经 spawner `--inactive` 启动（上电不使能，等待显式 enable），JSB 保活提供 `/joint_states`；move_group + retime + 编排层（`control_backend=fjt_action`、`motor_service_backend=controller_switch`、`require_gate=false`）+ 夹爪产品节点（`traj_topic:=/gripper_controller/joint_trajectory`——JTC 原生话题入口，位置类命令直入标准控制器）+ MQTT 桥 + arm_monitor（可选）。**全程无 sim_motor_node / sim_power_sequence_node / gravity_torque_node。**
- **验收标准：**
  1. 启动后节点清单无任何自研 sim 节点；boot 态两个 JTC `inactive`、FSM `IDLE`
  2. enable → 两 JTC `active`、FSM `READY`；jog / goto(move_group) / playback(retime) 落点 ≤0.02，`/joint_states` 速度字段有效（LL-072 顺序不回退）
  3. 夹爪 `/a3/gripper/command` 位置命令 → L7 经标准 JTC 话题实际运动
  4. disable safe-park → 两 JTC `inactive`、FSM `DISABLED`；MQTT 桥节点存活无崩溃
- **关联：** F70（标准栈）、F72（硬件 on_activate/deactivate 使能语义）、F74（FJT action 后端）；LL-072（JTC/JSB 启动顺序与单点语义）、LL-076、LL-077（goto L7 补发 + 位置/速度双落定）；审计任务 #10
- **状态：** `completed`（2026-09-22 仿真验收，ROS_DOMAIN_ID=61，`scripts/a3_test/f75_full_mock_acceptance.py` 15/15：boot 两 JTC inactive、FSM IDLE、零自研 sim 节点、产品节点齐；enable→controllers activated→READY、两 JTC active；jog ×3（含 L7）落点 err ≤0.0199、速度字段有效 max_vel=0.429；goto move_group ready/home err ≤0.0094（L7 同步补发）；playback 61 点正弦经 retime+双 JTC 落 home err=0.0118；夹爪位置命令经标准 JTC 话题驱动 L7（0→1.780→0）；disable safe-park 位置/速度双落定后两 JTC inactive、FSM DISABLED；MQTT 桥全程存活）

## F76 Pilz 工业运动规划器（PTP / LIN / CIRC + Sequence 混合，替代手写笛卡尔节点）

- **说明：** 审计任务 #10 发现：笛卡尔直线/圆弧运动一直靠自研节点（`move_to_pose_ik_node.py` 手写 IK + 直线插值、`draw_rectangle_demo.py` 四点拼矩形），与工业现场的标准指令语义（PTP 点到点、LIN 空间直线、CIRC 圆弧、带 blend_radius 的顺序程序）不一致，且没有速度规划。MoveIt 官方的 Pilz 工业运动规划器（`ros-humble-pilz-industrial-motion-planner`，`pilz_industrial_motion_planner/CommandPlanner`）提供这四类标准能力，move_group 以第二条规划管线（`planning_pipelines: [ompl, pilz]`）并存加载：
  - PTP：关节/位姿点到点；LIN：末端空间直线（在线求解保持直线几何 + 速度规划）；CIRC：以 center 或 interim 辅助点定义的圆弧（经 MotionPlanRequest.path_constraints，约束名 `center`/`interim`）。
  - Sequence：`pilz_industrial_motion_planner/MoveGroupSequenceAction`（+ `MoveGroupSequenceService`）能力，`moveit_msgs/action/MoveGroupSequence`，多条指令 + blend_radius 平滑混合（矩形/多边形程序一次下发）。
  - move_group 暴露统一服务 `/plan_kinematic_path`（GetMotionPlan，按请求内 `pipeline_id`/`planner_id` 选管线与 PTP/LIN/CIRC）、`/plan_sequence_path`（GetMotionSequence）与 `/sequence_move_group` action；执行仍经标准 JTC FJT。
- **接线：** `a3_moveit_config/config/pilz_industrial_motion_planner.yaml`（CommandPlanner + 笛卡尔速度上限；关节速度/加速度复用 joint_limits.yaml）；`edge_full_mock.launch.py` 的 move_group 改为双管线并加载 sequence 能力。OMPL 管线与现有 goto 默认路径不受影响。
- **验收标准：**
  1. move_group 启动后同时存在 ompl / pilz 两管线（`/plan_kinematic_path` + `/plan_sequence_path` 服务）与 `/sequence_move_group` action
  2. PTP：关节目标（ready）规划+执行落点 ≤0.02 rad
  3. LIN：两点位姿目标规划成功，执行中末端实际轨迹对直线的最大偏离 ≤ 2 mm
  4. CIRC：带 center 约束规划成功，轨迹点到圆心距离恒定（偏差 ≤ 2 mm）
  5. Sequence：3 段 LIN + blend_radius 的三角形程序经 action 一次执行成功，运动连续无停顿
  6. 全程经标准栈（mock GenericSystem + JTC），零自研笛卡尔节点参与
- **关联：** F67（move_group）、F75（全产品 mock 栈）；任务 #10、#12（reBot 手写 IK/demo 退役由本需求提供标准替代）
- **状态：** `completed`（2026-09-22 仿真验收，ROS_DOMAIN_ID=62，`scripts/a3_test/f76_pilz_acceptance.py` 12/12：规划/序列端点齐（`/plan_kinematic_path`、`/plan_sequence_path`、`/sequence_move_group`），boot 两 JTC inactive；enable→READY 两 JTC active；OMPL zero→ready 落点 err=0.0009；Pilz PTP ready→home→ready err ≤0.0002；LIN 末端直线 max_dev=0.97 mm、end_err=1.32 mm；CIRC 半径恒定 max_rdev=0.60 mm、end_err=0.93 mm；3 段 LIN + blend_radius 三角形一次执行连续通过，dev=1.46 mm、拐角通过速度 179.9 mm/s；全程零自研笛卡尔节点）

## F77 PS4 D-pad 单关节点动改走 MoveIt Servo JointJog（替代手写单点轨迹）

- **说明：** 审计任务 #10 发现：PS4 D-pad 的单关节点动（`joint_jog_L1..L6` → `tick_end` 直接把 `q + jog×speed×dt` 包成单点 JointTrajectory 发到旧式 `/joint_group_effort_controller/joint_trajectory`）是 F70 标准栈之前的遗留路径——① 单点轨迹绕过 FSM 模式门禁，无速度规划/限幅/奇异点与碰撞保护；② 该旧话题在 F75/F76 标准栈上没有任何消费者，点动实际是死指令。标准替代是 MoveIt Servo 已内置的 `control_msgs/JointJog` 输入（`~/delta_joint_cmds`，速度单位）：servo 内部统一做缩放、关节限位余量、奇异点保护与碰撞检查，输出标准 JointTrajectory 经 JTC 执行。PS4 侧改为持续发布 `velocities` 非零的 JointJog（松开 D-pad → 零速度，伺服自然停住保位），与现有 TwistStamped 笛卡尔遥操走同一伺服、同一 FSM `mode=SERVO` 语义。L7（夹爪）不属 move_group 臂组，继续走夹爪指令/标准 JTC 路径，不进 servo。
- **接线：** `a3_teleop_ps4/actions.py`（删 tick_end 的单点轨迹点动块，改发 JointJog；按需 start_servo）；`a3_bringup/servo_mode_bridge.py`（新增 JointJog 订阅，关节点动同样断言 SERVO 模式）；`edge_full_mock.launch.py`（标准栈内常驻 servo_node，输出 → `/arm_controller/joint_trajectory`）。
- **验收标准：**
  1. D-pad 按住单关节：JointJog 持续发到 `/servo_node/delta_joint_cmds`，对应关节按速度方向实际运动；松开后停止且无越限
  2. 旧式 `/joint_group_effort_controller/joint_trajectory` 在点动全程零消息
  3. 点动期间 `/a3/control_mode` 为 SERVO，停止 0.25 s 后回到 IDLE；FSM 状态保持 READY
  4. 点动轨迹经 servo → 标准 JTC（mock GenericSystem）闭环，零自研点动节点；L7 点动仍走夹爪路径不受影响
- **关联：** F64（D-pad 双死人开关/单通道调速）、F75（标准 mock 栈）、F76；审计任务 #10；LL-079
- **状态：** `completed`（2026-09-22 仿真验收 ROS_DOMAIN_ID=63，`scripts/a3_test/f77_joint_jog_acceptance.py` 8/8：JointJog +/−0.2 rad/s 各 1.5 s → L1 位移 ±0.121 rad、实测最大速度 0.34 rad/s，松开即停零漂移；点动全程 control_mode=SERVO→停止后 IDLE、FSM 恒 READY；旧 `/joint_group_effort_controller/joint_trajectory` 零消息；L7 位置服务仍正常驱动（0→1.780→0）。调试中修复 servo 输出话题嵌套参数 `moveit_servo.command_out_topic`（LL-079），并统一指令链路 SensorDataQoS）

## F78 统一 bringup 单入口（hardware:=mock|can，mock/can 产品拓扑完全一致）

- **说明：** 审计任务 #10 发现真机入口 `a3_bringup.launch.py`（旧 can_bridge C++ 栈：can_transport/motor_protocol/power_sequence + trajectory_bridge + 手搓 FJT action）与 F75 起的产品标准栈 `edge_full_mock.launch.py`（ros2_control 标准栈：controller_manager + JTC/JSB + move_group 双管线 + servo + 编排层标准后端）是两套互不相同的拓扑——真机/仿真行为分叉、节点与话题契约不一致，真机验收需另写脚本。F78 将 `a3_bringup.launch.py` 重写为唯一产品入口，以 `hardware:=mock|can` 切换硬件插件：mock 为 `mock_components/GenericSystem`（无 CAN/无电机可直接起），can 为 `a3_hardware_interface/A3MITHardwareInterface`（SocketCAN + MIT，接口名 `can_interface`，真机先起 can-up / vcan 验收用 vcan0）；两种模式下 controller_manager、JTC（inactive 启动）、JSB、move_group（OMPL+Pilz 双管线）、retime、servo_node（JointJog/TwistStamped 双输入）、servo_mode_bridge、编排层（fjt_action + controller_switch 后端）、夹爪产品节点拓扑完全一致。旧 can_bridge C++ 栈、trajectory_bridge、旧 FJT action 节点不再由该入口加载（文件保留，历史 launch/脚本可继续引用）。
- **接线：** `a3_bringup/launch/a3_bringup.launch.py`（重写；`hardware`/`can_interface` 参数 + 条件 xacro 命令；组件开关 `use_mqtt/use_teleop/use_rviz/use_monitor`）。
- **验收标准：**
  1. `hardware:=mock`：无需 CAN 设备即可起栈，F75/F76/F77 三套既有验收脚本对该入口全部通过（拓扑与 edge_full_mock 一致）
  2. `hardware:=can can_interface:=vcan0`：配合 `vcan_motor_sim.py`，F72 既有 vcan 验收通过（真机 MIT 插件闭环、CAN 帧映射正确、栈内无旧 can_bridge/旧 FJT 节点）
  3. 非法 `hardware` 值不得静默走真机；mock 模式不触碰任何 CAN socket
- **关联：** F72（MIT 真机插件 + vcan 验收）、F75–F77（标准栈拓扑）；审计任务 #10；AGENT.md §3
- **状态：** 已完成（2026-09-22，仿真验收）。mock 路径：F75 15/15、F76 12/12、F77 8/8 全部对统一入口通过；can 路径：domain 59 + vcan0 + vcan_motor_sim，F72 16/16 ALL PASS（CAN 指令/反馈映射偏差 0、kp=80/kd=2、栈内无 legacy 节点）；非法 hardware 值被 launch 硬拒（exit 1）；/proc/net/can/rcvlist 审计确认 mock 栈在 can0/can1/vcan0 零 socket（模拟器接收计数不增长）。旧拓扑保留为 `edge_legacy_stack.launch.py`（deprecated，仅供历史回归）。

## F79 产品验收接入 colcon test / launch_testing（标准 CI 测试门）

- **说明：** 审计任务 #10 发现：F75/F76/F77 验收脚本只能人工「先起栈、再开一个终端跑脚本、肉眼看 PASS」，无标准退出码收集、无 JUnit XML、不能进 CI/门禁——工业工程要求验收可重复、自动化、可归档。F79 新增独立测试包 `a3_acceptance_tests`（ament_cmake），用 ROS 官方 `launch_testing` 框架把每个验收封装为一个 launch test：测试描述内拉起统一产品入口（`a3_bringup.launch.py hardware:=mock`，固定 ROS_DOMAIN_ID、PYTHONNOUSERSITE=1）并执行既有验收脚本（单一事实源，脚本经 CMake install 进包内 share/harness，测试不复制检查逻辑）；断言脚本进程退出码 0；测试结束 launch_testing 自动 SIGINT 拆栈。结果经 `add_launch_test` 自动产出 JUnit XML（`colcon test-result` 可读），与 ament 生态标准测试完全同构。F72 vcan 验收因需 sudo 建 vcan，保留人工/脚本入口，不纳入本测试包。
- **接线：** 新包 `src/a3_acceptance_tests/`（package.xml + CMakeLists.txt + `test/test_f75_full_mock.py`、`test/test_f76_pilz.py`、`test/test_f77_joint_jog.py`）。
- **验收标准：**
  1. `colcon build --packages-select a3_acceptance_tests` 后 `colcon test --packages-select a3_acceptance_tests` 三个测试全部通过，退出码 0
  2. `colcon test-result --all` 可见三个 launch test 的 JUnit 结果；任一验收项失败时测试失败（失败可注入验证）
  3. 测试结束无残留 ROS 进程；F75 测试栈配置与人工验收一致（含 MQTT，broker 不可达时该测试按失败计），F76/F77 栈 `use_mqtt:=false`
- **关联：** F75–F78（被封装的验收与统一入口）；审计任务 #10
- **状态：** 已完成（2026-09-22，仿真验收）。证据：`colcon test --packages-select a3_acceptance_tests` 三个 launch test 全绿（合计 ~85 s），`colcon test-result` Summary 6 tests / 0 errors / 0 failures（3 个 ctest 包装 + 3 个 launch 内用例）；F75 15/15、F76 12/12、F77 8/8 均经 colcon 门通过。失败传播已实证：首跑 F77 因使能早于 controller spawner（6/8 FAIL，退出码 1），JUnit 记录 failure、colcon test 非零退出；修复方式为 F77 脚本内等待 `joint_state_broadcaster` active（spawn 完成标志）再 enable（见 LL-081）。JUnit XML 位于 `build/a3_acceptance_tests/test_results/a3_acceptance_tests/test_test_f7*.xunit.xml`。测试结束 domain 61/62/63 无残留进程。

## F81 单电机反馈断线看门狗（按电机 last-rx 超时 → 内部锁存 + 整臂冻结保持）

- **说明：** 审计发现 `arm_monitor` 的 STALE_JS 只监视整个 `/joint_states` 话题静默；真实故障更常见的形态是**单个电机反馈 TX 通道死掉、其余 6 个照常上报**——此时 JSB 持续发布、话题不静默，整机对「盲关节」无感知，控制器会带着冻结的状态继续指令运动。F81 在真机硬件插件 `A3MITHardwareInterface` 内按电机记录最近反馈时刻（RxLoop 每解码一帧更新），`read()` 中检测：active 后任一电机反馈年龄超过 `feedback_timeout_s`（默认 0.2 s；正常 200 Hz 控制帧应答，容忍约 40 帧丢失）→ 置 per-motor stale 标志与整臂 stale latch、ERROR 日志（节流，含 motor_id / 关节名 / 年龄）。**关键：`read()` 始终返回 `return_type::OK`，绝不返回 ERROR。** 已对 ros2_control 2.54.0 源码核实：`System::read()` 一旦收到插件 ERROR 就调 `error()`，默认 `on_error` 返回 SUCCESS，组件被强制转到 **unconfigured**；而 `System::write()` 在 unconfigured 态直接早退返回 OK——插件的安全写根本不执行，CAN TX 全灭，臂失保后静默掉臂。正确路径是 read() 内部锁存、组件保持 active，由 `write()` 执行整臂保护性 freeze-hold：丢弃控制器新指令，全部 7 路按最后已知关节位置发位置保持帧（kp/kd 维持）。stale 状态同时通过标准通道上报：`/diagnostics`（`a3_hardware:feedback_watchdog`，含每电机年龄 KeyValue，ERROR/OK）与锁存话题 `/a3/hardware/feedback_stale`（`std_msgs/Bool`，TRANSIENT_LOCAL）；FSM 订阅后在 stale 期间经 `_can_move()` 拒绝一切新运动（goto/playback/web slider jog），disable 走 stale 快路径（在 home 位直接 reset；不在 home 位拒绝并提示恢复反馈或紧急 reset，避免 safe-park 在 freeze-hold 下必超时进 FAULT）。反馈恢复后 latch 自动清除、freeze-hold 解除。
- **启动门（fail-fast）：** `on_activate` 重排为 **reset-all → 500 ms 内验证 7/7 应答 → 才允许 enable-all**。ros2_control 2.54.0 中 on_activate 返回 ERROR 是致命的：资源管理器抛 `Failed to set the initial state of the component ... to active`，controller_manager 进程 abort，spawner 永远联系不上 CM——故障形态是整机停机。因此必须在发送任何 enable 帧**之前**发现暗电机：reset（MIT 0x04）使电机进入失能/coast 态，未通过应答验证即返回 ERROR，所有电机安全地留在 reset 态，绝不能出现「6 个已使能 + 激活失败进程死亡」。
- **接线：** `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（JointMapping 加 `has_feedback/stale/last_fb_time`；RxLoop / read / write / on_activate / on_cleanup；health 节点 + `/diagnostics` + `/a3/hardware/feedback_stale` 发布）；`src/a3_hardware_interface/package.xml` + `CMakeLists.txt`（新增 `diagnostic_msgs` / `std_msgs` 依赖）；`src/a3_arm_controller/a3_arm_controller/arm_controller.py`（参数 `feedback_stale_topic`、订阅、`_can_move` 门禁、disable stale 快路径、slider jog 拒绝）；`scripts/a3_test/vcan_motor_sim.py` 的 `--silence-file`（JSON `{"motor": 4}`：该电机停止发送反馈帧但仍执行控制，模拟反馈 TX 断线；`{}` 恢复）；验收 `scripts/a3_test/f81_feedback_stale_acceptance.py`。
- **验收标准：**
  1. vcan 全栈（`a3_bringup.launch.py hardware:=can`）使能后 7 路反馈正常、READY；silence 文件掐断 motor 4 反馈后，≤ `feedback_timeout_s + 0.5 s` 内插件 ERROR 日志含 motor 4
  2. 断链期间整机进入保护性 freeze-hold（对标 ISO 10218 protective stop）：全部 7 路 CAN 指令为最后已知位置的位置保持帧（kp/kd 维持），实测 10 s 量级零位移；此时下发的 FJT 轨迹被接受但**不得报成功**（JTC 即使 open_loop 仍在末点后按实际状态校验 goal tolerance，goal_time=0 不 abort、持续挂起），无任何关节位移
  3. 清除 silence 文件后反馈恢复、latch 与 FSM 门禁清除；disable/enable 正常，重新使能后 7 关节（含 4 与 7）可运动；最终 disable 干净拆栈
  4. 反馈从启动即缺失的电机被 on_activate 启动门拦截：插件未发任何 enable 帧（日志 `no enable frames sent`），CM 激活失败/进程死亡，arm_controller 从未 active
- **关联：** F72（真机插件/vcan 基建）、F71（诊断通道）、F74（标准执行后端）；[shared/SAFETY.md](../shared/SAFETY.md)（反馈断线策略）；LL-083（read ERROR 导致组件 unconfigured、write 静默的框架行为）；arm_monitor STALE_JS（整话题静默，互补）
- **状态：** 已完成（2026-09-22，仿真验收，9/9）。证据（`ROS_DOMAIN_ID=72 F81_CAN_IF=vcan1 F81_CAN_IF_B=vcan3 python3 scripts/a3_test/f81_feedback_stale_acceptance.py 72`，phase A domain 72/vcan1、phase B domain 75/vcan3，日志 /tmp/f81_harness_run2.log）：1a FSM=READY 且 arm/gripper JTC active、JSB active；1b age4=0.00183 s；① 静默 motor 4 → detected=0.264 s（门限 0.7 s 内）、age 持续增长；② freeze-hold：err=pending/pending（FJT 目标接受但不报成功）、min_frames=895 保持帧、max_span=0.0000、max_drift=0.000；③ 恢复 age4=0.0011 s、latch 清除 → disable→DISABLED → 重新使能运动 err=0/0、span4=0.250(676 帧)/span7=0.150 → 干净失能；④ 启动即缺 motor：ever_active=False、日志含 "no enable frames sent"（log_no_enable=True）、spawner 无法联系 CM（spawner_failed=True），ros2_control_node SIGABRT 退出、无一路电机被使能。验收结束无残留进程（pgrep 核实）。

## F82 诊断聚合器接入（diagnostic_aggregator / GenericAnalyzer → /diagnostics_agg + toplevel state）

- **说明：** F71（`arm_monitor` diagnostic_updater）与 F81（硬件插件 `a3_hardware:feedback_watchdog`）均已按标准向 `/diagnostics` 发布 DiagnosticStatus，但全栈没有聚合节点：`rqt_robot_monitor`/`rqt_runtime_monitor` 看到的是裸流，无分组层级、无整机单一健康状态、无 STALE 超时判定。F82 引入标准包 `diagnostic_aggregator`（4.0.7，已装）：`aggregator_node` 加载 `GenericAnalyzer` 分组配置（`src/a3_bringup/config/diagnostics.yaml`），AnalyzerGroup 路径 `A3`，下分 `Hardware`（startswith `a3_hardware:`，含 F81 看门狗）与 `Arm Monitor`（startswith `arm_monitor:`/`a3_arm_monitor:`，含 F71 Monitor/Tracking）两组，分析器 `timeout: 5.0`（输入消失后转 STALE）。输出标准话题 `/diagnostics_agg`（分组树，路径形如 `/A3/Hardware/a3_hardware:feedback_watchdog`）与 `/diagnostics_toplevel_state`（`diagnostic_msgs/DiagnosticStatus`，取 `level` 字段：0 OK / 1 WARN / 2 ERROR / 3 STALE，`name=/A3`），供 HMI/CI 一键判定整机健康。节点经 `a3_bringup.launch.py` 参数 `use_diagnostics`（默认 true，mock/can 两模式均含）启动；纯增量，不改变任何看门狗的判定与处置逻辑。
- **接线：** `src/a3_bringup/config/diagnostics.yaml`（GenericAnalyzer YAML）；`src/a3_bringup/launch/a3_bringup.launch.py`（aggregator_node + 参数声明）；`src/a3_bringup/package.xml`（exec_depend diagnostic_aggregator）；验收 `scripts/a3_test/f82_diagnostic_aggregator_acceptance.py`。
- **验收标准（仿真）：**
  1. aggregator_node 按本仓 diagnostics.yaml 启动后订阅 `/diagnostics`；注入两组 OK 状态（`a3_hardware:feedback_watchdog`、`a3_arm_monitor: Monitor`）后 `/diagnostics_agg` 出现 `/A3/Hardware/...` 与 `/A3/Arm Monitor/...` 两条聚合路径，`/diagnostics_toplevel_state` 数据为 0（OK）
  2. 将 `a3_hardware:feedback_watchdog` 置为 ERROR（模拟 F81 freeze-hold）后 ≤ 2 s 内 toplevel 变为 2（ERROR）；恢复 OK 后 ≤ 2 s 回到 0
  3. 停止发布后 ≤ `timeout + 2 s` 聚合项转 STALE（toplevel=3），恢复发布后回到 OK
  4. 全栈烟雾：`a3_bringup.launch.py hardware:=mock use_mqtt:=false use_teleop:=false` 启动无错误，`ros2 node list` 含 `/diagnostic_aggregator`、`/diagnostics_agg` 与 `/diagnostics_toplevel_state` 话题存在
- **关联：** F71（arm_monitor 标准诊断）、F81（硬件看门狗诊断）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（聚合话题契约）；对标工业 HMI 单一健康指示（ISO 10218 状态可见性）
- **状态：** 仿真验收通过（2026-09-22，`ROS_DOMAIN_ID=90 python3 scripts/a3_test/f82_diagnostic_aggregator_acceptance.py 90` 4/4：phase1 domain 90 分组路径齐 / ERROR 1.0 s 升级并恢复 / 停发 5.0 s 转 STALE 并恢复；phase2 domain 91 mock 全栈 `/diagnostic_aggregator` 节点与两话题在；真机待加电回归）

## F83 使能流程对齐厂商标准（清故障 → 整数写运行模式 → 使能 → 软启动阻尼接管）

- **说明：** 对比官方参考仓库 EDULITE_A3（`el_a3_sdk/interface.py` `EnableArm`）发现，插件当前 `on_activate` 仅 reset-all → enable-all，缺少厂商标准使能编排的两个关键步骤：①使能前先发 **Type 4 失能帧且 data[0]=1 清除故障锁存**（`clear_fault=True`），防止带历史故障位直接使能被电机拒入或带病运行；②使能前必须用 **Type 18 参数写把运行模式 0x7005 写为目标模式的整数值**（MOTION_CONTROL=0）——该参数是 uint8，走 float 编码会把低字节写成 0x00 导致模式损坏（厂商 `write_parameter_int` 专用整数路径）。F83 将 `on_activate` 重排为厂商顺序，在 F81 启动门（reset-all → 7/7 应答证明）通过后，**逐电机**执行：清故障（0x04, data[0]=1）→ 间隔 30 ms → 整数写 RUN_MODE=0（0x12, 0x7005, value=0）→ 间隔 30 ms → 使能（0x03）→ 间隔 30 ms（每电机约 90 ms，7 路合计 < 1 s 一次性成本）。任一电机未应答则保持 F81 fail-fast 语义，绝不部分使能。
- **软启动（我们的加固，非厂商行为，须如实标注）：** 厂商 `EnableArm(startup_kd=4.0)` 的 `startup_kd` 在其代码库中**声明但从未使用**（全库 grep 仅签名与文档串）。F83 落地该参数的语义：使能后前 `soft_start_cycles`（默认 10 个写周期 ≈ 50 ms @ 200 Hz）个 `write()` 周期，全部 7 路发**纯阻尼接管帧**——位置取实测位、kp=0、kd=`startup_kd`（默认 4.0，clamp 0–5）、速度/力矩前馈 0——先以阻尼「接住」机械臂再过渡到正常位置刚度（kp/kd 默认 80/2），避免使能瞬间弹簧阶跃。F81 stale freeze-hold 优先级仍最高（软启动期反馈断线照常冻结）；软启动结束后正常控制帧恢复。全部 7 个关节（含 L7 夹爪）统一 MOTION_CONTROL 模式——厂商给夹爪用 POSITION_PP，是因其夹爪走梯形 profile 接口；本仓夹爪栈依赖 MIT 运动控制帧（含力控特性），此处为有意偏差并记录。
- **接线：** `src/a3_hardware_interface/include/a3_hardware_interface/protocol_codec.hpp`（新增 `BuildClearFaultFrame`：Type 4 data[0]=1；新增 `kParamCanTimeout=0x7028` 备 F86；同步注释镜像到 `src/a3_can_bridge` 同名 codec）；`src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（on_init 新参数 `startup_kd` 默认 4.0 / `soft_start_cycles` 默认 10；on_activate 厂商编排；write() 软启动倒计时；on_deactivate 复位）；验收 `scripts/a3_test/f83_enable_choreography_acceptance.py`（CAN 原始帧序 + 软启动增益 + 运动回归）。
- **验收标准（仿真，vcan 原始帧嗅探）：**
  1. 使能时每路电机（id 1–7）在总线上**按序**出现：Type 4 且 data[0]=1（清故障）→ Type 18 且 param=0x7005、data[4]=0（MOTION_CONTROL）→ Type 3（使能），各类帧之间间隔 ≥ ~20 ms；F81 启动门仍在编排之前（先 reset + 7/7 应答）
  2. Type 3 之后每个电机的前 ≥3 个 Type 1 控制帧 kp 解码 < 1、kd 解码在 3.5–4.5（纯阻尼），随后 ≤ 0.5 s 内过渡到正常增益（kp 解码 70–90、kd≈2）
  3. 软启动结束后运动正常：小幅度 quintic FJT（L1–L6 0.10 rad / L7 0.05 rad，3 s）两 JTC 均 error_code=0，嗅探到位移 span；disable 干净（on_deactivate 零增益帧 + reset）
  4. 启动即缺电机时 F81 语义不回退：无任何清故障/模式/使能帧之外的部分使能（暗电机情况下整个编排不执行），on_activate 返回 ERROR
- **关联：** F81（启动门 / freeze-hold / read 永远 OK）、F72（硬件插件）、F74（ros2_control 执行后端）；官方 EDULITE_A3 `EnableArm`；[shared/SAFETY.md](../shared/SAFETY.md)（使能安全）
- **状态：** 仿真验收通过 9/9（2026-09-22，vcan1/vcan3；另含 boot→enable 零帧检查与 F83/LL-086 组件生命周期显式编排）。真机验收待上电。

## F84 温度/故障码完整解码（temperature state interface + motor_health 诊断 + 复用 F44 FSM 门禁）

- **说明：** 对比官方参考仓库 EDULITE_A3（`el_a3_hardware/src/robstride_can_driver.cpp` 反馈解码：`mode_state=(can_id>>22)&0x03`、`fault_code=(can_id>>16)&0x3F`、温度 raw/10）发现：F72 插件的 `DecodeFeedback` 虽然解码出温度与五个故障 bool，但 RX 循环**只复制 pos/vel/effort，温度/mode/故障字全部丢弃**；插件也没有温度 state interface。更关键的断链：FSM 既有的 **F44** 温度保护（90 warn / 95 protect / 迟滞 5 / safe park → COOLING）与电机故障监视（fault_mask≠0 → reset → FAULT）只订阅 `a3_can_bridge/msg/MotorStates`（`/a3/motor/states`），而 ros2_control `hardware:=can` 栈不再启动 a3_can_bridge——**F44 在新栈里静默失效**（无错误、无日志，超温/故障都不会触发）。F84 不重造保护逻辑，只把插件解码出的状态接到 F44 既有入口。
- **改动：**
  1. `motor_model.hpp`（两份镜像同步）：`MotorFeedback` 新增 `uint8_t fault_code`（6 位完整故障字，五个 bool 保留）；两份 `protocol_codec.hpp` `DecodeFeedback` 填 `fault_code=(can_id>>16)&0x3F`。
  2. 插件 `JointMapping` 新增 `hw_temp / hw_mode / hw_fault`；RX 循环复制；`export_state_interfaces` 每关节新增 `temperature` state interface（pos/vel/effort/temperature ×4）。URDF 中该接口**仅真机条件声明**（`use_real_hardware`），mock GenericSystem 不导出 temperature，避免 mock 栈校验失败。
  3. `PublishHealth` 新增第二条诊断 `a3_hardware:motor_health`：每电机键 `motorN_temp_c / motorN_mode / motorN_fault`；level：fresh 关节 fault_code≠0 或温度 ≥ `temp_protect_c`(95) → ERROR，≥ `temp_warn_c`(90) → WARN，否则 OK（与 F44 参数同源；F82 聚合 `startswith: ["a3_hardware:"]` 自动收录）。
  4. 插件 health 节点新增 publisher 以 50 Hz 发 `a3_can_bridge/MotorStates`（包新增对 a3_can_bridge 的消息依赖）：7 条全发，`fresh` = 有反馈且 age ≤ feedback_timeout，`fault_mask=fault_code`，`enabled`=组件 active，温度/mode 填实测值——FSM F43/F44 代码零改动即恢复工作。
  5. `vcan_motor_sim.py` 新增 `--health-file`（默认 `/tmp/f84_health.json`）：`{"motor":3,"temp_c":92.0,"fault":4,"mode":1}`，温度/6 位故障字/mode 注入反馈帧；`{}` 清除。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f84_thermal_fault_acceptance.py`）：**
  1. `ros2 control list_hardware_interfaces` 可见 7 路 `<joint>/temperature [state]`（未 claim）；常温 30 °C 时 `/a3/motor/states` 7 条 fresh=true、temperature_c≈30、fault_mask=0、enabled=true；`a3_hardware:motor_health` level OK、键齐全
  2. 注入 92 °C（motor 3）：motor_health WARN、`ArmStatus.temp_warn=true`；warn 不打断——小幅 quintic FJT 两控制器 error_code=0
  3. 注入 96 °C：FSM 走**既有 F44** 路径（READY → SAFE_PARK → 失能 → COOLING）；降温至 protect−hysteresis(90 °C) 前 enable 被拒；改注 80 °C 后 enable 成功回 READY
  4. 注入故障位（motor 5 `fault=4`）：FSM 走既有 fault_mask≠0 路径（紧急 reset → FAULT），motor_health ERROR
  5. mock 栈（GenericSystem，无 temperature 接口）回归正常：hardware:=mock bringup 不因缺 temperature 接口失败
- **关联：** F44（温度保护/COOLING，本项只补数据源不重写逻辑）、F43（力矩统计同源 MotorStates）、F72/F74（ros2_control 栈）、F82（聚合自动收录）、F83（使能编排后电机才 enabled=true）；官方 EDULITE_A3 `robstride_can_driver.cpp`；LL-087（ros2_control 栈切换后旁路话题静默断链）
- **状态：** 仿真验收通过 10/10（2026-09-22，vcan4；含 mock 栈回归）。真机验收待上电。

## F85 自由拖动速度自适应 Kd（Lorentzian 速度曲线 + EMA；对标官方 el_a3_hardware computeAdaptiveKd）

- **说明：** F73 的自由拖动在插件 EFFORT 写帧路径里用**固定** `effort_kd=2.0`：静止时阻尼偏大、手感「黏」，快速拖动时又像在「搅蜂蜜」。对比官方 EDULITE_A3 发现其 `el_a3_hardware/src/el_a3_hardware.cpp` 在同一位置实现了 `computeAdaptiveKd()`：静止高阻尼稳位、高速低阻尼跟手，Lorentzian 曲线给出中点平滑过渡，再做一阶 EMA 抑制帧间跳变。更关键：本仓 xacro（`el_a3_ros2_control.xacro`，2026-08-21 起）**早已声明** `adaptive_kd_enabled / zero_torque_kd_min / zero_torque_kd_max / kd_velocity_ref / kd_smoothing_alpha / zero_torque_kd` 六个参数，但插件从未解析（插件只认 `effort_kd`）——参数是死的。F85 把官方实现接到既有参数上，不引入新参数名、不改 F73 重力补偿控制器（仍发纯 RNEA 力矩）。
- **改动：**
  1. 插件 `on_init` 解析硬件参数：`adaptive_kd_enabled`（bool，默认 false）、固定兜底 `zero_torque_kd`（缺省回退 `effort_kd`，再缺省 2.0；与旧路径完全兼容）、`zero_torque_kd_min=0.001`、`zero_torque_kd_max=0.15`、`kd_velocity_ref=1.0 rad/s`、`kd_smoothing_alpha=0.15`（min/max 合法性校验 min≤max、α∈(0,1]）。
  2. 新增每关节 EMA 状态 `adaptive_kd_[i]`，`on_init`/进入 EFFORT 模式时播种为 kd_max（与参考实现一致，避免模式切换瞬间阻尼塌陷）。
  3. `write()` EFFORT 分支按关节计算：`kd_raw = kd_min + (kd_max−kd_min)/(1+(|hw_vel|/v_ref)²)`，`kd = α·kd_raw + (1−α)·prev_kd`，clamp [0,5]；adaptive 关闭时沿用固定 `zero_torque_kd`。
  4. xacro：把六个参数从内层 GenericSystem 兜底块（永远到不了真机插件）移到真机硬件块并补 `adaptive_kd_enabled` xacro/launch 透传，取值保持 `true / 0.001 / 0.15 / 1.0 / 0.15`。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f85_adaptive_kd_acceptance.py`）：**
  1. 静止切 zero_torque_controller：6 路臂 CAN 指令帧 kp≈0、位置字段=当前测量位、vel=0；L1–L3 kd∈[0.12,0.16]（≈全局 kd_max 0.15），L4–L6 kd ≈ 各自 per-joint kd_max（0.10/0.05/0.05，±0.02）
  2. 注入外力（motor 3 持续 0.6 Nm）使其转动：motor 3 高速段（|v|≥1.5 rad/s）kd∈[0.001,0.05]，明显低于静止值；未被推动的 L1/L2 kd 仍 ∈[0.12,0.16]，L4–L6 保持各自 kd_max±0.02
  3. EMA 平滑：高速采样段相邻 200 Hz 帧 kd 跳变 ≤ 0.03（α=0.15 理论单步最大变化 0.0224）
  4. 撤去外力：kd 在 3 s 内回升至 ≥0.10
  5. 切回 arm_controller：位置帧 kp≈80、kd≈2，小幅 quintic FJT error_code=0、运动正常
  6. 固定阻尼兜底回归：以 `adaptive_kd_enabled:=false`（vcan 专用 launch 参数透传）启动时 kd 恒为 `zero_torque_kd`(0.3±0.05)，不随速度变化
- **关联：** F73（力矩模式 + 重力补偿控制器，本项只改插件阻尼）、F72（插件）；官方 EDULITE_A3 `el_a3_hardware.cpp::computeAdaptiveKd`；LL-088（xacro 声明的参数长期无人解析——死参数比缺参数更难发现）
- **状态：** 仿真验收通过（2026-09-22，f85 harness 29/29，domain 85 / vcan5，断电）。真机验收待上电。

## F86 电机侧通信超时配置（0x7028 Type-18 写入；与主机看门狗独立的纵深防御）

- **说明：** F81 的反馈 staleness 看门狗运行在主机上：主机掉电、系统卡死、CAN 线缆脱落时，看门狗本身随主机一起失效。MIT 固件内置 CAN 通信超时参数 `0x7028`（**uint32，单位约 50 µs，20000≈1 s；0=关闭**，出厂默认 0；Type-18 写入为易失性、掉电丢失）：超时未收到任何帧时电机自行进入 RESET 模式（失能/阻尼态，等同 Type 4）。官方 EDULITE_A3 在每次使能时**主动写 0 关闭**它、只依赖主机看门狗；F86 从纵深防御角度有意偏离官方——主机看门狗负责可恢复的瞬时冻结保持（F81），电机侧超时负责「主机已死」时最后一道独立卸力。默认 0.2 s（4000 counts）：远大于 200 Hz 指令环抖动（正常帧间隔 5 ms），又能在主机失死后快速切断力矩。
- **改动：**
  1. 插件 `on_init` 解析硬件参数：`motor_can_timeout_enabled`（bool，默认 true）、`motor_can_timeout_s`（double 秒，默认 0.2，clamp [0.05,10]）；换算 `counts = round(seconds * 20000)`，以 uint32 LE 经 `BuildSetParamRawFrame`（Type 18）下发——不能按 float 编码（旧 codec 注释「float, s」是错的，F86 修正）。
  2. `on_activate` 厂商编排每电机：clear-fault → 整数写 RUN_MODE=0 → **写 0x7028**（enabled 写配置 counts，disabled 显式写 0，与官方一致且状态确定）→ enable，各步保持 30 ms 间隔。
  3. `on_deactivate` 每电机在 reset 前补写一次 0x7028=0 解除武装，避免同一次上电周期内残留。
  4. xacro 真机插件块新增两参数；bringup（can）与 vcan launch 透传 `motor_can_timeout_enabled`（默认 true）；`motor_can_timeout_s` 走 xacro 默认。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f86_can_timeout_acceptance.py`）：**
  1. 默认使能：raw socket 抓到 7 路 Type-18 帧，`data[0..1]=28 70`、`data[2..3]=00 00`、`data[4..8]` LE uint32 ≈ 4000（±10%）；帧序位于 RUN_MODE 写（0x7005）之后、enable（Type 3）之前
  2. Type-17 读回：harness 发读参数帧，电机应答 0x7028 值 ≈ 4000
  3. 端到端跳闸：杀掉整栈后，sim 状态文件显示 7/7 电机在 [0.18, 0.7] s 内进入 tripped（RESET），无一提前误跳
  4. `motor_can_timeout_enabled:=false` 重启：使能编排仍写 0x7028 但 counts=0；杀栈后 1 s 内无电机 tripped
  5. 使能后小幅 quintic FJT error_code=0、运动正常（F83/F85 编排回归）
- **关联：** F81（主机侧 staleness 看门狗，本项补电机侧独立防线）、F83（使能编排，本项在其中插步）；官方 EDULITE_A3 `can_driver.py::write_parameter_int` / `interface.py`（使能写 0）；LL-089（寄存器类型/量纲须以协议手册为准——0x7028 是 uint32 计数不是 float 秒；易失写入与 flash 保存的边界）
- **状态：** 仿真验收通过（vcan `f86_can_timeout_acceptance.py` 29/29，2026-09-22：布防帧/读回/无误跳/SIGKILL 后 7/7 在 0.202 s 跳闸/撤防重启无跳闸）。真机验收待上电。

## F87 电机侧力矩限制布防（0x700B Type-18 float 写入；使能编排内每电机逐路下发）

- **说明：** 统一栈（F78）下真机只走 `a3_hardware_interface` 插件；原 `a3_can_bridge` 的 `/a3/motor/set_param` 服务无人启动。F24 夹爪节点仍向该服务写固件力矩上限，服务永不就绪 → 重试静默失败，固件力矩限制在统一栈下从未被任何节点显式布防（出厂值虽为各型号峰值，状态不可验证；URDF 的 `torque_max` 逐关节下调也没有固件落地路径）。F87 把力矩上限布防收进使能编排：插件按已解析的逐关节 `torque_max`（JointMapping；RS00=14 Nm、EL05=6 Nm，可用 URDF 参数 `torque_max` 逐关节下调）用 Type-18 写 `0x700B`。协议手册五件事已核实：**float32、单位 Nm、范围 0~型号峰值（EL05 6 / RS00 14）、写入易失（掉电回出厂峰值）、运控模式下立即作为输出力矩钳位生效**。
- **改动：**
  1. `on_activate` 厂商编排每电机在 RUN_MODE=0 之后、0x7028 之前插入：`BuildSetParamFrame(0x700B, j.torque_max)`（IEEE754 float，不能复用 raw/u8 构造器），保持 30 ms 间隔；每次 activate 重写（易失，且 reset 后状态不确定）。
  2. `vcan_motor_sim.py` 参数存储泛化：0x700B 按 raw uint32 存储并在 Type-17 读回应答（与 0x7028 同一存储/应答路径）。
  3. 验收脚本 `scripts/a3_test/f87_torque_limit_acceptance.py`：raw socket 断言逐电机写帧的存在/编码/数值/帧序，Type-17 读回对照，编排回归（clear-fault/RUN_MODE/0x7028/enable 仍齐）。
  4. 夹爪节点对已退役 `/a3/motor/set_param` 的依赖处理放在 F87 第二步（L7 原生力控）；本提交不改变夹爪节点行为。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f87_torque_limit_acceptance.py`）：**
  1. 默认使能：raw socket 抓到 7 路 Type-18 帧，`data[0..1]=0b 70`、`data[2..3]=00 00`、`data[4..8]` LE 解释为 IEEE754 float：motor 1–3 = 14.0 Nm、motor 4–7 = 6.0 Nm（±1%）；每路恰一帧，无重复无跳电机
  2. 帧序：每电机的 0x700B 写位于该电机 RUN_MODE 写（0x7005）之后、0x7028 写之前、enable（Type 3）之前
  3. Type-17 读回：harness 发读参数帧，7 路应答的 raw 值与对应 float 位模式逐位一致
  4. 编排回归：clear-fault（Type 4 data[0]=1）/ RUN_MODE / 0x7028=4000 / enable 帧计数与顺序仍满足 F83/F86
  5. 使能后小幅 quintic FJT error_code=0、运动正常（F83/F85/F86 回归）
- **关联：** F83（使能编排，本项在其中插步）、F86（同一 Type-18 编排，float vs uint32 编码对照）、F24（夹爪固件硬限；本项先布全臂，第二步退役夹爪节点的死服务依赖）；LL-089（寄存器类型/量纲以协议手册为准）
- **状态：** 仿真验收通过（vcan `f87_torque_limit_acceptance.py` 47/47，2026-09-23：7 路 float 写帧 14/14/14/6/6/6/6±1%、帧序 0x7005<0x700B<0x7028<enable、Type-17 读回逐位一致、编排回归、FJT error_code=0）。真机验收待上电。

### F87（第二步）— L7 走标准 GripperActionController（per-goal max_effort；退役夹爪节点死服务依赖）

- **说明：** 第一步把固件 `0x700B` 收进使能编排后，Python 夹爪节点（F24/F25/F26）的两条外部依赖在统一栈下都是死的：`/a3/motor/set_param`（旧 `a3_can_bridge` 服务，无人启动，力矩上限写不进去 → 静默重试）和 `/mit_gains_cmd`（旧执行层增益话题，无人订阅 → 力/位增益切换全部空发）。节点内部还自维护 50 Hz 力环状态机、接触/打滑判定和归一化开合，均与 ros2_controllers 的标准实现重复。第二步按工业路径替换：L7 控制器改为 `gripper_controllers` 的 `effort_controllers/GripperActionController`（ROS 2 Humble 官方包 `ros-humble-gripper-controllers`），对外是标准 `control_msgs/action/GripperCommand`；力控由控制器内置 PID（位置/速度误差 → effort 命令）完成，per-goal `max_effort` 在控制器内逐目标钳位，固件 `0x700B`（第一步）仍是最终硬件钳位——双层限力。position GAC 变体只透传位置命令、本身不执行力上限，故不采用。
- **改动：**
  1. `el_a3_controllers.yaml`：`gripper_controller` 类型改为 `effort_controllers/GripperActionController`，参数 `joint: L7_joint`、`goal_tolerance`、`max_effort`（默认值）、`action_monitor_rate`、`allow_stalling/stall_velocity_threshold/stall_timeout`，`gains.L7_joint: {p, i, d, i_clamp}`；控制器要求 L7 同时导出 effort 命令接口与 position/velocity 状态接口（确认 `A3MITHardwareInterface` 已对 L7 导出 effort+position command）。
  2. 编排层：不再把 7 关节 FJT 中的 L7 投到 `/gripper_controller/follow_joint_trajectory`（该 action 已不存在）；L7 段转成标准 GripperCommand goal（position + max_effort）经 ActionClient 下发；纯位姿路径（goto/move_to/park）的 L7 同步段同样走 gripper action 并等待结果。
  3. PS4：开合/切换/半程改发 GripperCommand goal（position）；R2 力控扳机改发带 `max_effort` 的 goal（扳机行程映射为 max_effort，档位语义保留），删除对 `a3_msgs/GripperCommand` 服务的调用与重试状态机。
  4. MQTT：现有 `gripper_*` op 改走标准 action；`gripper_set_max_torque` 的固件写不再可用（0x700B 由使能编排统一布防、易失寄存器不接受运行期单路改写），语义改为「本次会话 per-goal max_effort 上限」或明确返回不可用（不保留静默失败）。
  5. 产品 launch 默认不再启动 Python 夹爪节点；节点代码保留（不删文件），死服务依赖在本提交内从默认路径摘除。
  6. 验收脚本 `scripts/a3_test/f87b_gripper_action_acceptance.py`（编号沿用任务序号 F87 第二步，脚本独立命名）：vcan 上断言标准 action 存在、位置 goal 收敛、带 max_effort 的抓取 goal 产生的 effort 命令不超过上限（raw Type-1 帧 t_ff/kp 解析 + sim 接触注入）、stall 检测、固件 0x700B 仍是最终钳位、FSM/PS4 路径无对死服务的调用。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f87b_gripper_action_acceptance.py`）：**
  1. `gripper_controller` 加载为 `effort_controllers/GripperActionController`，`/gripper_controller/gripper_cmd`（标准 `control_msgs/GripperCommand` action）可连；无 `/gripper_controller/follow_joint_trajectory`
  2. 位置 goal（开/合）在 goal_tolerance 内收敛、返回 `reached_goal=true`；L7 行程与标定一致（开 0.0、合 ≈1.79 rad）
  3. 力控 goal：sim 注入接触后，raw Type-1 帧解析出的实际输出 effort（t_ff 与 kp×err 合成）在稳态不超过该 goal 的 `max_effort`（±5%）；换不同 max_effort 重复验证；控制器 stall 判定按 `stall_velocity_threshold/stall_timeout` 触发并正常保持
  4. 双层限力：固件 `0x700B`（第一步编排）仍为 6 Nm；控制器命令即使配置错误也不会超过固件钳位（sim 固件钳位生效证据）
  5. 默认 launch 不启动 `a3_gripper_controller` 节点；栈内无对 `/a3/motor/set_param`、`/mit_gains_cmd` 的请求（日志/话题级证据）；FSM goto/park/disable 中的 L7 段经标准 action 执行且全链路回归通过
- **关联：** F87 第一步（固件 0x700B 布防，本项的最终硬件钳位）、F24/F25/F26（Python 夹爪节点力/位/配置能力，由标准控制器取代）、F78（统一产品栈）、F81（反馈失鲜看门狗对 L7 同样有效）
- **状态：** 仿真验收通过（vcan `f87b_gripper_action_acceptance.py` 23/23，2026-09-23：控制器类型/状态、gripper_cmd 可连、FJT action 已消失；自由空间开/合收敛 reached_goal=true（1.779 / −0.016 rad）；接触注入后 0.5/1.0 Nm 两档 raw Type-1 稳态 effort 中位 0.500/1.000（±5%），stall 契约 stalled=true/reached_goal=false（allow_stalling→succeeded）并持续保持；max_effort=20 被固件/插件钳在 6.000 Nm；0x700B 读回 6.0 Nm；Python 夹爪节点与死服务端点全部缺席；混合模式 arm FJT+L7 action、FSM goto ready、safe park→DISABLED 全链路回归）。另：F32 `/a3/motor/*` 九个旧 can_bridge 调试 op 在统一栈下无服务端，本次一并从 MQTT 下行白名单摘除（改为显式 unknown op，不再静默挂死）。真机验收待上电。

### F88 — 两点标准轨迹替代手搓密集线性插值（JTC splines 控制器侧插值；retime ramp 同步稀疏化）

- **说明：** F41 起，goto/move_to 本地兜底、safe-park 纠偏、`/a3/arm/set_joint_positions` web jog、回放 ramp 都在编排层生成 ≥50 Hz 的稠密点列（线性等距），再整条发给执行后端。这套手搓插值有三个问题：① 标准 JTC 自身按 `interpolation_method: splines`（变量次数样条，已在 `el_a3_controllers.yaml` 配置）在 200 Hz 更新环内插值，喂稠密线性点等于用「折线段」覆盖掉控制器的平滑样条，起停速度不连续（三角速度曲线）；② 稠密点经 FJT action 投影/序列化是无谓负载，回放 ramp 的近重复点还曾导致 Ruckig 求解失败；③ 点密度由三个自研参数（`goto_waypoints/move_to_points_hz/move_to_max_points`）控制，属于重复造轮。工业路径：编排层只发**起点（t=0，当前位）+ 终点（t=duration，目标位）两个点**，L1–L6 由 JTC 样条插值生成 rest-to-rest 平滑运动，L7 仍取末点转 GripperCommand goal；旧栈 topic 后端的 `motor_protocol_node` 自带 200 Hz 插值，两点同样合法。回放 ramp 的几何输入也稀疏化为两点（线性段几何不变，Ruckig/TOTG 重定时结果保几何）。
- **改动：**
  1. 新辅助 `_two_point_trajectory(q0, q1, duration, joint_names=None)`：恰好 2 个点（α=0/1），两点均显式盖 `velocities=0` 且 `accelerations=0`——JTC VARIABLE_DEGREE_SPLINE 按点上可用导数选次数（位置-only→线性匀速、+速度→cubic、+速度+加速度→quintic），必须显式零 v/a 才会得到 quintic rest-to-rest S 曲线；替代 `_linear_trajectory` 与 `set_joint_positions` 内联 11 点循环；`_linear_trajectory` 删除。
  2. `_dispatch_l7_linear`：topic 后端的 L7 轨迹改两点（fjt_action 后端本就只取末点）。
  3. `_safe_park_then_disable` fallback 与纠偏轨迹、`_goto_cb`/`_move_to_cb` 本地兜底、`/a3/arm/set_joint_positions` jog 全部改走两点辅助。
  4. `_playback_cb` ramp（retime 几何输入与 legacy 链路）改两点；删除 `_traj_point_count` 及参数 `goto_waypoints`、`move_to_points_hz`、`move_to_max_points`（声明 + 三份 FSM yaml）。
  5. 验收脚本 `scripts/a3_test/f88_two_point_trajectory_acceptance.py`：mock-hardware 标准栈（无需 vcan）下直接对 JTC 发两点 FJT goal，采样 `controller_state` 参考轨迹断言样条曲线（起/止速度≈0、α=0.1 处速度远低于线性常数速度、α=0.5 处峰值比 ≥1.5）、末点收敛；再经编排层验证 set_joint_positions jog / goto / move_to / safe-park→disable 全链路回归。
- **验收标准（仿真，mock-hardware；脚本 `scripts/a3_test/f88_two_point_trajectory_acceptance.py`）：**
  1. 两点 FJT goal 被 JTC 接受（error_code=0），末点在 goal 容差内收敛；全程无 PATH/GOAL_TOLERANCE 违约
  2. `controller_state` 采样证据：起点与终点速度 ≈0（≤0.02 rad/s）；quintic 钟形速度剖面 `v/(Δq/T)=30α²(1−α)²`——α=0.1 处比值 ≤0.50（理论 0.243）、α=0.5 附近出现峰值且峰值比 ≥1.5（理论 1.875），证明控制器侧 quintic 样条在工作，而非线性折线段（注：原拟在 α=0.25 判 ≤0.95 与 quintic 数学不符——该点理论比值 1.055，故改在 α=0.1 判据 + 中点峰值）
  3. 编排层 set_joint_positions jog（连续抢占 3 次）、goto/move_to 兜底路径全部成功；实际下发轨迹点数 = 2（抓 `/arm_controller/follow_joint_trajectory` goal 证据）
  4. safe-park→disable 全链路回归绿（回 home 收敛 → DISABLED）
  5. 栈内无 `goto_waypoints`/`move_to_points_hz`/`move_to_max_points`/`_traj_point_count` 残留引用；topic 后端（motor_protocol 200 Hz 插值）两点轨迹回归通过
- **关联：** F41（原 ≥50 Hz 稠密插值，本项取代其默认路径）、F74（FJT 标准后端）、F68（retime 服务消费 ramp 几何）、F76/F77（同样的「标准工具替代手写」路线）
- **状态：** 已完成（2026-09-23，仿真 22/22：phase 1 mock-hardware fjt_action 后端 + phase 2 vcan topic 后端；脚本退出码 0，LL-091～LL-096）。真机验收待通电。

### F89 — pinocchio 重力模型标定/核验（静态多姿态测量 → 逐关节重力矩比例因子 + R²；对标 EDULITE_A3 pinocchio_gravity_calibration.py）

- **说明：** 参考仓库 `el_a3_ros/scripts/pinocchio_gravity_calibration.py` 提供了一套轻量「重力模型核验」流程：多姿态静止、测电机维持力矩、与 pinocchio RNEA 预测做逐关节最小二乘，给出比例因子 + RMSE/R²。我们已有更重的 F49 十二参数惯性标定（mass+CoM，L-BFGS-B，真机小时级）和 C++ RNEA 自由拖动控制器，但缺三样：① 快速、可重复的**模型-实物一致性核验**（调试/出厂 commissioning 标准动作）；② F49 之后残余误差的廉价单参数修正；③ 修正因子被运行时消费的闭环。本项按工业 commissioning 路径补齐（与参考脚本同一测量原理：位置控制器保持静态姿态时电机维持力矩即重力负载，平衡态 τ_motor = g，无需切换控制器——早期「每姿态切自由驱动」方案在低惯量 sim 下残差重力导致姿态漂移，且 SIGTERM 中断会留下互换的控制器，已弃用）：标定工具走 F88 两点 FJT 到位，**保持 arm_controller 抱位**，静止窗口直接采样 `/joint_states` 电机反馈力矩（电机域，按 JOINT_SIGNS 转 URDF 域），预测力矩在**同一实测位姿**上用独立 pinocchio RNEA 计算（默认套用 F49 标定惯量），逐关节 LSQ 比例（clip [0.5,2.0]）+ RMSE/R²；结果直接写成 controller_manager 可用的 ParameterFile，由 bringup 加载后自由拖动控制器按比例缩放重力矩。vcan 仿真器同步增加 URDF 重力物理模型（可注入真实比例 + 测量噪声），使全流程可在断电条件下闭环验收。
- **改动：**
  1. 新增产品工具 `scripts/gravity_scale_calibration.py`（ROS 2 节点）：测试构型 = home + L2/L3 单关节扫掠 + L2×L3 网格 + L4/L5/L6 扰动（去重 atol、按 URDF 限位 clamp 并留安全余量，`--quick` 缩减）；到位用 F88 两点 FJT（两点显式零 v/a）；每构型到位后保持 arm_controller 抱位 → settle → 30 样本均值/标准差（不切换控制器）；启动时校验 arm_controller active。
  2. 拟合：`scale = Σ(g·τ)/Σ(g²)`（τ 为 URDF 域实测、g 为同位姿 RNEA），clip [0.5,2.0]，逐关节 RMSE 与 R²；输出 `~/.a3/gravity_scales.yaml`（格式 = CM ParameterFile：`zero_torque_controller.ros__parameters.tau_scale`，元数据另置顶层 key），终端打印汇总；`--out/--quick/--no-calibrated-inertia` 等参数。
  3. C++ `GravityCompensationController` 增参 `tau_scale`（double 数组，默认全 1，尺寸不符回退全 1），update() 内按比例缩放 RNEA 力矩；configure 时另经 yaml-cpp 解析 `a3_description/config/inertia_params.yaml`，把 F49 fitted mass/CoM 套用到 5 个 link（`use_calibrated_inertia` 默认 true，可用 `inertia_params_file` 覆盖路径）——运行时 RNEA 模型必须与标定工具/仿真真值同模型，否则比例因子相对另一个模型回归（首版残差最大 0.40 Nm）。a3_description 已 exec-depend 本包，反向不声明 package depend（循环），改由 ament_index 运行时定位 share。
  4. `a3_bringup.launch.py` 增参 `gravity_scales_file`（默认 ""；为空且 `~/.a3/gravity_scales.yaml` 存在则自动采用，语义同 named-pose 用户覆盖），存在时作为追加 ParameterFile 供 ros2_control_node 加载。
  5. `vcan_motor_sim.py` 增 `--gravity-model urdf`（+ `--gravity-scales` 六值、`--gravity-noise`、`--gravity-urdf`）：effort 模式下按当前姿态实时 RNEA 计算重力负载（电机域），物理净力矩 = 施加力矩 − 重力负载 + 推力 − 黏性，反馈力矩 = 施加力矩 + 高斯噪声；position 模式下内层刚性伺服一阶跟随到位，静态平衡时反馈力矩 = 电机域重力负载 + 高斯噪声（F89 抱位测量配对；运动中动态力矩不模拟，测量只在 settle 后进行）。
  6. 验收脚本 `scripts/a3_test/f89_gravity_scale_acceptance.py`：A 段注入已知比例跑标定工具，断言恢复精度与 R²；B 段以产出文件重启栈，经 `~/gravity_torque` 话题断言自由拖动力矩 = scale×独立 RNEA（≤0.03 Nm，实测 0.0000），切模式 settle 1 s 后测稳态漂移（1 s 内 ≤0.02 rad，实测最差 L3 0.0038 rad——模式切换瞬态在 settle 前，见 LL-099）；C 段无 scales 文件时日志确认 tau_scale 回退 1.0。
- **验收标准（仿真；机械臂保持断电，vcan0 闭环；脚本 `scripts/a3_test/f89_gravity_scale_acceptance.py`）：**
  1. A 段：工具全流程成功（两点 FJT 到位、arm_controller 全程抱位不切换、静态采样），产出 yaml 含 6 个比例与逐关节 R²/RMSE
  2. 注入比例（如 0.92/1.08/0.95/1.10/1.00/1.05，噪声 0.01 Nm）恢复误差 ≤ 0.05；被激发关节 R² ≥ 0.95（重力幅度过小的关节允许标记 low-excitation 豁免）
  3. B 段：加载产出文件后，zero_torque_controller 在 home/ready 姿态下发（`~/gravity_torque`）的力矩与 `scale × 独立 RNEA` 一致（≤0.03 Nm，实测 0.0000）；切模式 settle 1 s 后稳态 1 s 位姿漂移 ≤ 0.02 rad（实测最差 0.0038）
  4. 默认（无 scales 文件）行为不变：C++ 控制器 tau_scale=1.0、bringup 不追加文件；legacy 栈 gravity_torque_node 不受影响
- **关联：** F49（十二参数惯性标定，本项默认套用其结果并修正残余）、F73（RNEA 自由拖动控制器）、F88（两点 FJT 规约）、F85（自由拖动阻尼）；参考 `EDULITE_A3/el_a3_ros/scripts/pinocchio_gravity_calibration.py`
- **状态：** 已完成（2026-09-23，仿真 20/20：`scripts/a3_test/f89_gravity_scale_acceptance.py` A/B/C 三段全绿，vcan89 闭环，脚本退出码 0；LL-097～LL-099）。真机验收待通电。

### F89b — FSM 自由拖动走标准控制器切换（switch_controller：arm_controller ↔ zero_torque_controller；PS4 示教经同一 FSM 服务）

- **说明：** F89 证明标准栈的 STRICT `switch_controller`（停 arm_controller、激活 zero_torque_controller）是可靠的自由拖动路径，但 FSM 的 `start_teach/stop_teach` 仍在调 legacy 执行层服务 `/a3/zero_torque/start|stop`（motor_protocol_node / sim_motor_node 提供，标准栈不存在该服务）。结果：产品标准栈上按 Share 进入示教必然失败（service unavailable），操作员只能像 F89 验收 B 段那样手工拼 switch_controller——「自己折腾总会出错」。本项按工业路径把切换收进 FSM：标准后端（`motor_service_backend=controller_switch`）下，start/stop teach 各发**一次原子 STRICT 切换**（同一请求内 activate+deactivate，失败保持原控制器不变），模式语义与 legacy 对齐（`/a3/control_mode` 发 ZERO_TORQUE / 退出后发 READY，BLOCKED_MODES 门禁照旧生效）；stop 先恢复位置闭环再做自动保存，切换失败则留在 TEACH 态由操作员重试（不允许「报成功但臂还在自由态」）。PS4 侧 Share/Options 早已指向 `/a3/arm/start_teach|stop_teach`，无需改映射。
- **改动：**
  1. `arm_controller.py` 新增参数 `freedrive_arm_controller`（"arm_controller"）、`freedrive_controller`（"zero_torque_controller"）、`freedrive_switch_timeout_s`（5.0）；新增 `_cm_freedrive_switch(enter: bool)`：经现有 `/controller_manager/switch_controller` 客户端发原子 STRICT 请求（enter=deactivate arm + activate zero_torque；exit 反之），超时/拒绝返回 (False, msg)。
  2. `_start_teach_cb`：controller_switch 后端走 `_cm_freedrive_switch(True)`，成功后 `_publish_mode("ZERO_TORQUE")`（回环 echo 置 `_mode`，BLOCKED_MODES 语义不变）；legacy 后端保持调 `/a3/zero_torque/start`，零行为变化。
  3. `_stop_teach_cb`：controller_switch 后端**先** `_cm_freedrive_switch(False)`——失败则不切状态、不保存、返回 success=false（臂仍有重力补偿，重试 stop_teach 即可）；成功后 `_publish_mode("READY")` 再走既有自动保存（latest.yaml + 时间戳备份）。
- **验收标准（仿真；断电；脚本 `scripts/a3_test/f89b_freedrive_switch_acceptance.py`，vcan89b 标准栈，ROS_DOMAIN_ID=92）：**
  1. enable→READY 后调 start_teach：状态 TEACH；`list_controllers` 显示 zero_torque_controller active、arm_controller inactive；`~/gravity_torque` 持续发布；settle 1 s 后稳态 1 s 漂移 ≤ 0.02 rad
  2. 调 stop_teach：状态 READY；arm_controller active、zero_torque inactive；响应消息含 auto-saved；`/a3/control_mode` 恢复 READY
  3. 拒绝路径：DISABLED 下 start_teach 被拒；TEACH 中重复 start_teach 被拒；非 TEACH 下 stop_teach 被拒
  4. 退出切换失败时状态保持 TEACH、success=false（运行时经 set_parameters_atomically 把 freedrive_arm_controller 改为不存在的名字，STRICT 拒绝；LL-100）
- **关联：** F89（同一 STRICT 切换与漂移判据）、F73/F85（zero_torque 控制器与阻尼）、F75（后端分流范式）、F54（停止自动保存）
- **状态：** 已完成（2026-09-23，仿真 20/20：`scripts/a3_test/f89b_freedrive_switch_acceptance.py` 全绿，vcan89b 闭环，脚本退出码 0；稳态漂移实测最差 0.0023 rad；LL-100）。真机验收待通电。

### F90 — 故障触发有界 rosbag2 黑匣子（snapshot-mode 循环缓冲，FAULT 边沿自动落盘）

- **说明：** 工业控制器标配故障黑匣子（事件前后一段历史自动留存供复盘）。不手搓录制器：ROS 2 官方 `rosbag2_transport` 提供 snapshot mode——常驻进程只把消息留在固定大小的内存循环缓冲（不落盘、无磁盘增长），被调 `/rosbag2_recorder/snapshot`（**服务类型 `rosbag2_interfaces/srv/Snapshot`，不是 std_srvs/Trigger**）时把当前缓冲写成分片 bag（每次触发一个独立分片文件，天然有界）。产品 bringup 默认经 `ros2 bag record --snapshot-mode` 起该录制器（`use_rosbag` 可关），FSM 在**进入 FAULT 的边沿**（电机故障/安全停车超时/过热保护失败等所有 FAULT 入口都汇聚于 `_set_state`）异步触发一次 snapshot：fire-and-forget，不阻塞故障处置路径；录制器不在（use_rosbag:=false / 尚未发现）则静默跳过。录制话题最小集：`/joint_states`、`/a3/arm_status`、`/a3/control_mode`、`/diagnostics`、`/diagnostics_toplevel_state`、`/arm_controller/joint_trajectory`。
- **改动：**
  1. `a3_bringup.launch.py` 增参 `use_rosbag`（默认 true）、`bag_dir`（默认 `~/.a3/blackbox`）；`ExecuteProcess` 起 `ros2 bag record --snapshot-mode --max-cache-size 33554432 --max-bag-size 67108864 --storage mcap -o <bag_dir>/blackbox_<启动时间戳>`（时间戳在 launch 生成期取本地时间，每次启动唯一目录）。
  2. `arm_controller.py` 新增 `rosbag2_interfaces/srv/Snapshot` 客户端 `/rosbag2_recorder/snapshot`；`_set_state` 检测到旧态≠FAULT、新态=FAULT 且 `service_is_ready()` 时 `call_async`（响应回调仅记日志），不满足就绪条件直接跳过。
- **验收标准（仿真；断电；脚本 `scripts/a3_test/f90_blackbox_acceptance.py`，vcan90 标准栈，ROS_DOMAIN_ID=90）：**
  1. 启动后 `~/.a3/blackbox` 下出现本次启动的 mcap 目录；故障前目录内无消息分片（snapshot 未触发，磁盘不增长）
  2. enable→READY 后经 `/tmp/f84_health.json` 注入电机故障（motor 5 fault=4）：FSM 进入 FAULT；无需任何手工调用，新分片自动出现且包含 FAULT 前/后的 `/joint_states`、`/a3/arm_status` 等录制话题消息（`ros2 bag info` 可读，消息数 > 0，时长跨故障时刻）
  3. 有界性：循环缓冲 32 MiB / 分片 64 MiB 参数生效（bag info / 文件大小验证）；重复故障每次只多一个分片
  4. `use_rosbag:=false` 时无录制器进程，FSM 照常进 FAULT，不报错不阻塞
- **关联：** F44/F84（电机故障→FAULT）、F81（冻结保持，事件源之一）、F82（诊断话题）；对标工业控制器事件黑匣子
- **状态：** 已完成（2026-09-23，仿真 17/17：`scripts/a3_test/f90_blackbox_acceptance.py` 全绿，vcan90 闭环，脚本退出码 0；LL-101）。真机验收待通电。

### F92 — 全仓质量门禁接线（lint 真跑 + 一键 CI 门脚本）

- **说明：** 现状审计：5 个包在 `package.xml` 声明了 `ament_lint_auto`/`ament_lint_common`，但除 `a3_can_bridge` 外 C++ 包 CMakeLists 无 `BUILD_TESTING → ament_lint_auto_find_test_dependencies()` 块、Python 包 `setup.py` 无 `tests_require` 且无 `test/test_*.py`——lint 声明全是死的，`colcon test` 什么都不查。工业基线是「每次构建后一条命令跑全部静态检查与集成验收，非零即不合格」。
  - **门禁 linter 集合（本任务选定的 ament_lint 子集）**：C++ 包 `ament_cmake_lint_cmake` + `ament_cmake_xmllint` + `ament_cmake_cppcheck`；Python 包 `ament_flake8` + `ament_pep257`（经标准 `test/test_copyright` 同款 pytest 入口接线）；外加已有 launch_testing 产品验收（`a3_acceptance_tests` F75/F76/F77）。
  - **明确不纳入本任务（存量专项，见 LL-103）**：`cpplint`/`uncrustify`（`a3_can_bridge` 实测 cpplint 3454 项、uncrustify 5 文件——全量修复是独立格式化专项，产生巨型 blame churn，且多个相关文件正被队友在工作区修改）；`copyright`（全部存量文件缺版权头，机械添加同样需要避开队友在改文件）。门禁必须真的过——把 3500 项失败留在门上等于没有门。版权头/格式化作为后续任务，新文件起执行规范。
  - 声明了 `ament_lint_common` 的 5 个包改为上述精选 `test_depend`（不再由 common 间接拉入 cpplint/uncrustify/copyright），使门禁诚实。
  - 新增 `scripts/a3_test/a3_ci_gate.sh`：`colcon build --symlink-install` → `colcon test`（全仓 lint + launch_testing，顺序执行器）→ 结果 XML 强执行（JUnit errors/failures + ctest `<Test Status="failed">` 双格式），任一失败非零退出；脚本内 source 规则、`PYTHONNOUSERSITE=1`、离线 xmllint catalog 固化。本机 `colcon test-result` 无 `--enforce`，强执行内置于脚本（LL-103）。
  - F76 门禁化暴露的起栈竞态一并修复：高负载下 rmw 丢失 `load_controller` 响应，spawner 重试撞上 "already loaded" FATAL 退出。`a3_bringup.launch.py` 改 JTC→zero_torque→JSB 严格顺序链 + spawner 非零退出自动重启（spawner 启动先查 is_controller_loaded，重启幂等），`--service-call-timeout 5.0`（内部固定 3 次重试，恢复 ≤15 s；30 s 时最长 90 s 超出验收窗口）。
  - 门禁固定 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`：Cyclone 严格按 DDS 规范判 QoS，借此发现真实缺陷——PS4/F77 的 JointJog 发布端是 BEST_EFFORT，而 Humble moveit_servo 订阅实测 RELIABLE（Fast-DDS 宽松放行一直掩盖），已统一改 RELIABLE（LL-104）。
- **改动：**
  1. C++ 包（`a3_msgs`、`a3_description`、`a3_hardware_interface`、`a3_trajectory_processing`、`a3_moveit_config`、`a3_acceptance_tests`、`a3_cloud_edge`）：`package.xml` 加精选 test_depend；CMakeLists 加 `BUILD_TESTING` 块（`a3_cloud_edge` 仅补 test_depend）
  2. Python 包（`a3_arm_controller`、`a3_gripper_controller`、`a3_mqtt_bridge`、`a3_teleop_ps4`、`a3_bringup`）：`package.xml` 加 `test_depend ament_lint_auto/ament_flake8/ament_pep257`；`setup.py` 加 `tests_require`；新增标准 `test/test_flake8.py`、`test/test_pep257.py`
  3. `a3_can_bridge`：`ament_lint_common` 换精选 test_depend（接线已存在）
  4. 修复门禁报出的全部真实问题（cppcheck/flake8/pep257/lint_cmake/xmllint）
  5. 新增 `scripts/a3_test/a3_ci_gate.sh`、`scripts/a3_test/catalog.xml`、`scripts/a3_test/schema/package_format3.xsd`（离线 xmllint）
  6. `a3_bringup.launch.py`：spawner 顺序链 + 非零自动重启 + service-call-timeout 5.0 s（修 F76 起栈竞态）
  - 不动：`a3_lerobot_config`、`lerobot_robot_a3`（待批准删除，不接门禁）、`third_party`
- **验收标准（仿真；脚本可在无 vcan 条件下运行；mock hardware 栈不要求真机）：**
  1. `./scripts/a3_test/a3_ci_gate.sh` 退出码 0；输出中每包 lint 测试 0 failures
  2. `colcon test --packages-select a3_acceptance_tests`：F75/F76/F77 launch_testing 全绿
  3. 故意在任一 Python 文件引入一行 PEP8 违规 → 该包 flake8 FAIL、脚本非零（验收后还原）
- **关联：** F79（colcon test 产品验收，本任务把它串进统一门）；对标工业 CI 静态门；[LL-103](../lessons_learned/LL-103-lint-gate-curated-subset-and-honest-wiring.md)、[LL-104](../lessons_learned/LL-104-cyclone-qos-strictness-servo-reliable-and-setup-set-u.md)
- **状态：** 已完成（2026-09-23，仿真；验收 1：`./scripts/a3_test/a3_ci_gate.sh` 退出码 0——5 个 JUnit 包 10 tests、ctest 23 tests 全 0 failures，含每包 lint；验收 2：F75 15/15、F76 13/13、F77 8/8 全绿；验收 3：注入 W291 → a3_gripper_controller flake8 FAIL、门禁非零，已还原复绿。新增 apt 依赖 `ros-humble-rmw-cyclonedds-cpp`）。真机验收待通电。

### F93 — 产品栈 systemd 托管（版本化开机单元 + 崩溃自动重启；默认禁用，显式启用）

- **说明：** 现状产品栈靠登录后手工 `ros2 launch`：进程崩溃无人拉起、断电恢复不会自启、日志只在终端。工业现场标配是 init 系统托管：单元文件随仓库版本化、安装到 `/etc/systemd/system/`、输出进 journald；**默认 disabled**——未经现场人工确认不得让机械臂开机自动使能，由运维显式 `systemctl enable`。
  - `After=can-up.service network-online.target` + `Wants=can-up.service`：CAN 接口先行就绪，网络就绪供 MQTT/EMQX
  - `Restart=on-failure` + `RestartSec=5` + `StartLimitIntervalSec=60` / `StartLimitBurst=3`：崩溃自动拉起；60 s 内连续快速失败 3 次即停止重启（故障状态下避免反复使能），转人工处置
  - `KillSignal=SIGINT` + `TimeoutStopSec=20` + `KillMode=mixed`：SIGINT 给 launch 主进程优雅关停全栈，超时再 SIGKILL 残留
  - `Environment` 固化 `PYTHONNOUSERSITE=1`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（LL-104）；支持 `EnvironmentFile=-/etc/default/a3-arm` 现场覆盖而不改单元
- **改动：**
  1. 新增 `systemd/a3-arm.service`、`systemd/a3-arm.default`（现场环境文件范例）
  2. 新增 `scripts/setup/install_a3_service.sh`：拷贝单元 + daemon-reload；**不 enable、不 start**
  3. 新增 `scripts/a3_test/f93_systemd_acceptance.py`：mock 模式全自动验收（需 sudo，密码经 `A3_SUDO_PASS` + askpass；`hardware:=mock` 不触碰 CAN）
- **验收标准（仿真；机械臂保持断电）：**
  1. `systemd-analyze verify systemd/a3-arm.service` 退出码 0、无 error
  2. 安装后 `systemctl is-enabled a3-arm` = `disabled`、`is-active` = `inactive`
  3. 临时 mock drop-in 启动：90 s 内 `/joint_states` 恢复且 3 s 窗口 ≥100 条（实测 mock 200 Hz）；调 `/a3/arm/enable`（mock 安全）后 arm_controller / gripper_controller / joint_state_broadcaster 均 active
  4. `systemctl kill -s SIGKILL a3-arm` 模拟崩溃 → 自动重启，重启后 120 s 内 joint_states 恢复、NRestarts ≥ 1
  5. `systemctl stop` 后 10 s 内无残留 controller_manager / ros2 launch 进程；删除临时 drop-in；最终状态 disabled/inactive（全程未 enable）
- **关联：** `systemd/can-up.service`；对标工业现场开机自启 + 进程看门狗；LL-104（Cyclone 固化）、LL-105（systemd 249 键位 / Humble echo 无 --timeout / ANSI 解析）。真机急停/断电链路在 systemd 下的优雅关停需通电时补验
- **状态：** `completed`（2026-09-23，mock 全链路验收 17/17：verify 干净、安装保持 disabled/inactive、起栈后 joint_states 3 s 收到 600 条、enable 后三控制器 active、SIGKILL 后 NRestarts 0→1 且 120 s 内恢复、stop 无残留；LL-105）。真机验收待通电

### F96 — 主机资源标准诊断（diagnostic_common_diagnostics：CPU/内存/磁盘 → /diagnostics 与 /diagnostics_agg/Host）

- **说明：** F82 的 aggregator 只聚合机械臂子系统（硬件/monitor），RK3588 板载控制器本身的健康没有标准监测：黑匣子分片、MQTT 缓存、内存压力（参见真实栈被内存回收整栈杀死的教训）与磁盘写满都是嵌入式现场最常见的故障源。ROS 标准包 `ros-humble-diagnostic-common-diagnostics` 已提供现成 Python 监控节点（可执行 `cpu_monitor.py` / `ram_monitor.py` / `hd_monitor.py`），不应自研。
  - **实际发布名带节点名前缀（易踩）**：4.0.7 任务名虽为 `CPU Information` / `RAM Information` / `<hostname> HD Usage`，但 `diagnostic_updater` 强制 `stat.name = <node_name> + ': ' + stat.name`（除非任务名以 `/` 开头）。launch 中 `name=cpu_monitor/ram_monitor/hd_monitor` 覆盖后，`/diagnostics` 实际收到 `cpu_monitor: CPU Information`、`ram_monitor: RAM Information`、`hd_monitor: <hostname> HD Usage`（实测；详见 LL-108）
  - `a3_bringup.launch.py` 新增三个标准节点（`use_host_diagnostics:=true` 默认开；且受 `use_diagnostics` 总门控），参数：hd_monitor 监控根分区（`path:=/`）、内存/磁盘阈值走包默认（RAM WARN 90%；磁盘 WARN 5% free/ERROR 1%）
  - `config/diagnostics.yaml` 新增 GenericAnalyzer 分组 `host`（path `Host`，timeout 10.0，`contains` 匹配 `CPU Information`/`RAM Information`/`HD Usage`，前缀不影响子串匹配），聚合后路径 `/A3/Host/...`
  - 监控只读零运动；ntp_monitor 不接入（板上无 ntpd 守护，NTP 源待现场确定后单列需求）
- **改动：**
  1. 安装 `ros-humble-diagnostic-common-diagnostics` 4.0.7（apt；用户已授权「该装的装」）
  2. `config/diagnostics.yaml`：host analyzer 组
  3. `a3_bringup.launch.py`：三节点 + `use_host_diagnostics` 参数
  4. 新增 `scripts/a3_test/f96_host_diagnostics_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f96_host_diagnostics_acceptance.py`）：**
  1. mock 全栈：`/diagnostics` 出现三条主机 status（`cpu_monitor: CPU Information` / `ram_monitor: RAM Information` / `hd_monitor: <hostname> HD Usage`），level 均 ≤1（WARN 可接受、ERROR 不可；实测全 0）
  2. `/diagnostics_agg` 出现 `/A3/Host/<同名>` 三项；`/diagnostics_toplevel_state` 不被主机项拉到 ERROR（注意该话题类型是裸 `diagnostic_msgs/DiagnosticStatus`、name 固定 `toplevel_state`，不是 DiagnosticArray）。mock 栈用 `mock_components/GenericSystem` 不发 `a3_hardware:` 诊断，Hardware 分组按 F82 设计为 STALE（toplevel=3）是既有行为；Host 分组不被点名即通过
  3. `use_host_diagnostics:=false` 起栈：10 s 内 `/diagnostics` 不出现任何主机 status
- **关联：** F82（diagnostic_aggregator；本需求补主机侧数据源）、F90（黑匣子磁盘占用是 hdd_monitor 的保护对象）、[[real-arm-stack-not-in-agent-background]]（内存压力曾杀死整栈）
- **状态：** `completed`（2026-09-23，mock 全栈验收 8/8：/diagnostics 三项全 level=0；/diagnostics_agg `/A3/Host/...` 三项；toplevel=3(STALE，仅 Hardware 点名、Host 健康)；use_host_diagnostics:=false 零泄漏；LL-108）。真机验收待通电

### F97 — JTC 轨迹容差工业级配置（goal_time 超时必 abort + 逐关节 trajectory 跟踪容差；堵住卡死轨迹永久挂起）

- **说明：** `arm_controller`（joint_trajectory_controller 2.53.x）现有 `constraints` 只有逐关节 `goal: 0.03` 与 `stopped_velocity_tolerance: 0.1`，存在两个工业现场不可接受的缺口：
  1. **`goal_time: 0.0` 禁用目标时间约束**：JTC 语义中 goal_time ≤ 0 时检查直接跳过——轨迹段结束后若实际状态始终进不了 goal 容差，FollowJointTrajectory 目标**永远不结束**。F81 已实证此行为：单电机反馈失联触发插件 freeze-hold（写指令冻结在最后位置），运动中的轨迹 JTC 目标保持 pending 永不返回。工业控制器必须在轨迹结束后给定有限收敛窗口，超时即 abort
  2. **无逐关节 `trajectory:` 跟踪容差**：运动过程中实际位置与指令的偏差不做任何检查，电机堵转/被拽偏/严重滞后时轨迹照常「成功」走完时间轴
  - FSM 执行后端（arm_controller.py）下发的 FJT 目标不携带逐目标 path/goal_time 容差，控制器默认约束即对所有产品路径（F88 两点轨迹、goto、playback、jog）生效，无需改应用代码
- **改动（仅标准 JTC 参数，零自研代码）：**
  1. `src/a3_description/config/el_a3_controllers.yaml` `arm_controller.constraints`：
     - `goal_time: 1.0`（轨迹最后一点之后 1.0 s 内必须进入 goal 容差，否则 abort → GOAL_TOLERANCE_VIOLATED）
     - L1–L6 逐关节增加 `trajectory: 0.05`（运动中位置偏差 > 0.05 rad 即 abort → PATH_TOLERANCE_VIOLATED）；保留 `goal: 0.03`
  2. 新增 `scripts/a3_test/f97_jtc_tolerance_acceptance.py`（vcan 注入，不触真机）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f97_jtc_tolerance_acceptance.py`）：**
  1. **回归**：vcan 栈（vcan_motor_sim 一阶跟随）使能后下发正常两点轨迹，结果 `SUCCESSFUL`(0)
  2. **跟踪/超时 abort**：运动前用 `/tmp/f97_silence.json` 冻结电机 4 反馈（插件 freeze-hold），下发 2 s 轨迹：目标必须在 `轨迹时长 + goal_time + 1.0 s` 内返回（不再永久 pending），error_code 为 `PATH_TOLERANCE_VIOLATED`(-4) 或 `GOAL_TOLERANCE_VIOLATED`(-5)
  3. **恢复**：清除 silence 后再发正常轨迹，恢复 `SUCCESSFUL`（容差 latch 不残留）
- **关联：** F81（freeze-hold 是本需求要兜底的故障形态）、F88/F94（产品轨迹均经同一 JTC）、F74（FSM 执行后端）
- **状态：** `completed`（2026-09-23，vcan 验收 7/7：`scripts/a3_test/f97_jtc_tolerance_acceptance.py`。A 正常两点轨迹 SUCCESSFUL（2.98 s）；B 冻结电机 4 反馈后轨迹在运动 0.90 s 处即被逐关节跟踪容差中止，error_code=PATH_TOLERANCE_VIOLATED(-4)（远早于 2+1+1 s 上限，不再永久 pending）；C 清除 silence 后恢复 SUCCESSFUL（3.01 s）。验收脚本 in-process 驱动节点的 domain/RMW 环境坑见 LL-109）。真机验收待通电

### F99 — JTC 指令超时 cmd_timeout（话题接口陈旧指令在轨迹结束后确定性切 hold；堵住末速指令长期驻留）

- **说明：** F97 给 JTC 配了 `constraints.goal_time: 1.0`，但**只对 action goal 生效**——goal_time 触发时 goal 被 abort、控制器切 hold。JTC 同时在话题 `~/joint_trajectory` 上接受轨迹（工业旁路/工具链常用接口，无 action 生命周期管理）：一条轨迹插值到末点后，如果没有新指令到来，控制器会一直停留在该轨迹的末段采样上。配合本栈 `allow_nonzero_velocity_at_trajectory_end: true`（open_loop + position 接口，末点速度非零也不报错），陈旧的末段指令没有任何确定性的「到期」边界。工业惯例（ros2_controllers JTC 官方参数）是配置 `cmd_timeout`：**从轨迹最后一点计时**，超过该秒数仍未收到新轨迹，控制器主动告警并切到当前位置的 hold 点（`Aborted due to command timeout`）。硬约束：`cmd_timeout` 必须 **严格大于 `constraints.goal_time`，否则该参数在 on_configure 里被静默置 0（仅一条 WARN）**；取 `2.0`（goal_time 1.0 + 1.0 s 裕量）。action goal 永远先在 goal_time=1.0 处拿到 SUCCESSFUL/GOAL_TOLERANCE_VIOLATED，不会走到 cmd_timeout，故对产品 FJT 路径零行为变化。
- **改动（仅配置 + 验收脚本）：**
  1. `src/a3_description/config/el_a3_controllers.yaml` arm_controller：`cmd_timeout: 2.0`
  2. 新增 `scripts/a3_test/f99_jtc_cmd_timeout_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f99_jtc_cmd_timeout_acceptance.py`）：**
  1. **参数生效**：栈启动后 `ros2 param get /arm_controller cmd_timeout` = 2.0（控制器参数挂在各控制器自己的节点 `/arm_controller` 上；`/controller_manager` 只有 `arm_controller.type`），且栈日志无「Command timeout must be higher than goal_time tolerance」警告（出现即参数被忽略）
  2. **action 无回归**：标准两点 action 轨迹仍 SUCCESSFUL（f97 正常路径）
  3. **话题接口兜底**：向 `/arm_controller/joint_trajectory` 发一条短轨迹（L1 小幅移动），末点之后约 2 s，栈日志出现 `Aborted due to command timeout`，之后关节位置保持稳定（无漂移/继续运动）
- **关联：** F97（goal_time / 跟踪容差；本项是其话题接口侧的对称兜底）、F70（JTC 标准栈）、F94（速度限幅）
- **状态：** `completed`（2026-09-24，vcan 验收 5/5）

### F98 — L7 夹爪限位与实测标定统一（URDF / ros2_control / MoveIt 三处 [0.0, 1.78]；堵住模型与实物 12% 偏差）

- **说明：** L7 夹爪 2026-09-13 在新电机上完成标定（`src/a3_gripper_controller/config/gripper_config.yaml`）：全开位设零、闭合为正，实测机械止位 **1.7825 rad**，运行钳位取 **1.78 rad**（不顶止位、无驻留力矩）。但三处模型限位仍是 reBot 参考值 **±1.5708**：
  1. `src/a3_description/urdf/el_a3.urdf.xacro:279`（L7 joint limit lower/upper）
  2. `src/a3_description/urdf/el_a3_ros2_control.xacro`（L7 position command_interface min/max）
  3. `src/a3_moveit_config/config/joint_limits.yaml:59-66`（MoveIt L7 min/max_position）

  工业现场不能接受「控制器模型与实际机构不一致」：MoveIt 对夹爪的任何规划/展示最多只能闭合到 1.57 rad（比实际行程少 0.21 rad ≈ 12%，夹爪永远合不拢到模型可知的全行程）；负方向在机械上无对应行程（开位即零位），模型却允许 -1.57 rad 的位置指令，存在反向驱动、扯线风险。修复取**与标定钳位一致的单一真值 [0.0, 1.78]**（can_bridge 侧 `control_gains.yaml` 早已是 [0, 1.8]，本次不改）。L5/L6 的 ±1.5708 是真实关节限位，不动。
- **改动（仅配置，零代码逻辑）：**
  1. `el_a3.urdf.xacro` L7 `<limit>`：`lower="0.0" upper="1.78"`
  2. `el_a3_ros2_control.xacro` L7 position command_interface：`min 0.0 / max 1.78`
  3. `joint_limits.yaml` L7：`min_position 0.0 / max_position 1.78`
  4. 新增 `scripts/a3_test/f98_l7_limits_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f98_l7_limits_acceptance.py`）：**
  1. **模型一致性**：解析 xacro 展开后的 URDF、ros2_control xacro、MoveIt joint_limits.yaml，L7 三处 lower=0.0、upper=1.78（且 L5/L6 仍为 ±1.5708 未误伤）
  2. **全行程可达**：vcan 栈激活标准 `gripper_controller`（effort_controllers/GripperActionController），GripperCommand position=1.78 → `/joint_states` L7 在时限内到达 ≥1.65 rad（超过旧上限 1.5708，证明全行程打通）
  3. **回零**：GripperCommand position=0.0 → L7 回到 ≤0.05 rad
- **关联：** F87（GripperActionController 标准执行后端）、L7 零点标定记录、[shared/SAFETY.md](../shared/SAFETY.md)；a3_can_bridge `control_gains.yaml` 的 L7 [0, 1.8] 已是同方向约束
- **状态：** `completed`（2026-09-23，vcan 验收 13/13：`scripts/a3_test/f98_l7_limits_acceptance.py`。A 模型一致性 8/8：xacro 展开后 URDF L7=[0,1.78]、ros2_control position 接口 [0,1.78]、MoveIt joint_limits L7=[0,1.78]，L5/L6 ±1.5708 未误伤；B GripperCommand 1.78 reached_goal，实测 pos=1.770（越过旧上限 1.5708，全行程打通）；C GripperCommand 0.0 回到 pos=0.009。f87b 回归 7/7。多限位源同步踩坑见 LL-110）。真机验收待通电

### F95 — 标准自检服务（ros-humble-self-test / diagnostic_msgs/SelfTest；开机/维护一键只读自检）

- **说明：** 工业驱动惯例（URBK/ABB、ROS-I 驱动）提供**标准自检服务**：现场上电后或维护后，一条服务调用跑完「连通 → 子系统状态 → 故障检查」并给出逐项结果，而不是让人手工 `topic echo` 一个个查。ROS 标准实现是 `ros-humble-self-test`（`self_test::TestRunner`，头文件库；服务类型 `diagnostic_msgs/srv/SelfTest`，响应 `bool passed` + `DiagnosticStatus[]`；约定 level≥2 ERROR 判失败、WARN 不判失败；其中一项须 `setID` 上报设备标识）。现状 F71 的 `arm_monitor` 只有周期 `diagnostic_updater` 上报表，没有按需自检通道。
  - 新增 ament_cmake 包 `a3_self_test`（唯一可执行 `a3_self_test`；self_test 在 Humble 无 Python 绑定，必须 C++）：内部持续缓存（subscriber + 2 Hz timer，任务回调只读缓存、不阻塞）：
    1. **连通** `Connection`：`/joint_states` 最近 1 s 消息数 ≥ `min_joint_states_rate`（默认 40 Hz，50/200 Hz 两栈均过）且 7 个关节名齐全；缺失/掉速 → ERROR
    2. **控制器** `Controllers`：`/controller_manager/list_controllers` 缓存（超时 5 s 未响应 → ERROR）；`joint_state_broadcaster` 必须 active；期望控制器列表（默认 `arm_controller,gripper_controller`）存在且状态 ∈ `inactive,configured,active`（Humble 生命周期：spawn 后未 activate 为 `inactive`；未 enable 也通过）→ 缺失 = ERROR
    3. **故障** `Faults`：最近 2 s `/diagnostics` 无 level≥2 状态（无诊断消息本身不算失败——arm_monitor 可选部署）；有 ERROR → 失败并在 message 点名
    4. `ID`：`setID(<hostname>)`（标识自检针对的设备）
  - 板载 `ros-humble-diagnostic-msgs 4.9.1`（定制源）把 `DiagnosticStatus.level` 与 `SelfTest.passed` 均定义为 `octet`，rclpy 侧拿到单字节 `bytes`（`b'\x00'`/`b'\x01'`），Python 客户端须按 `x[0]` 归一化（与 C++ 线上语义一致；详见 LL-107）
  - 全部检测**只读、零运动**（不调 enable、不发轨迹）；mock / can 栈通用
  - 接入 `a3_bringup.launch.py`：`use_self_test:=true`（默认开），硬件/模拟都启动；可独立运行（栈未起时自检应判失败，这本身就是故障检测能力）
- **改动：**
  1. 新包 `src/a3_self_test/`（package.xml、CMakeLists.txt、src/a3_self_test_node.cpp）
  2. `a3_bringup.launch.py`：`use_self_test` 参数 + 条件节点
  3. 新增 `scripts/a3_test/f95_self_test_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f95_self_test_acceptance.py`）：**
  1. 独立启动（无产品栈）调 `/a3_self_test/self_test` → `passed=false`，Connection 项 level=2
  2. mock 全栈（不 enable）调自检 → `passed=true`，状态项 ≥3；Connection 报告速率 ≥40 Hz（mock 实测 200 Hz）；Controllers 项含 joint_state_broadcaster active、arm_controller inactive；id 非空（实测 hostname `lubancat`）
  3. enable 后再调 → 仍 `passed=true`，arm_controller 状态变 active 且体现在 message
  4. disable safe-park 回归正常
- **关联：** F71（diagnostic_updater 周期上报；本需求补按需自检）、F82（/diagnostics_agg）、F75（mock 标准栈）
- **状态：** `completed`（2026-09-23，mock 全栈验收 13/13：无栈 passed=false 且 Connection ERROR；未 enable passed=true、rate 200 Hz、JSB active、arm_controller inactive、id=lubancat；enable 后 arm_controller active；disable safe-park 后位姿偏差 worst=0.009 rad；LL-107）。真机验收待通电

### F94 — 两点轨迹统一速度限幅（URDF velocity 地板时长；堵住绕过规划器的无限速 jog/park）

- **说明：** MoveIt goto / playback 路径由 OMPL + TOTG/Ruckig 强制速度/加速度限幅，但编排层所有「两点轨迹」（`_two_point_trajectory`：web set_joint_positions jog、set_jog、disable safe-park、F40 park 等 6 处）是直接把 2 点（端点 v=a=0）丢给 JTC spline，**没有任何速度上限**：web 端 duration 最小 0.05 s，若 Δq 大（如 1 rad / 0.05 s = 20 rad/s），JTC VARIABLE_DEGREE_SPLINE 生成的峰值速度可超过关节 URDF `velocity` 限（33/50 rad/s 是电机空载极限，不是安全作业速度）。工业做法：**限幅只有一个权威来源（URDF joint limit velocity，MoveIt joint_limits.yaml 与之同源）**，所有下发路径（规划路径由 TOTG 保证、原始两点路径由编排层地板时长保证）都不得超过同一限值。
  - `_load_joint_limits()` 同时解析 URDF 每关节 `velocity`（新增 `self._joint_vel_limits: Dict[str,float]`；零/缺失不采用，保持原有位置限加载行为）
  - 新参数 `joint_velocity_scale: 1.0`：现场收紧余量的统一旋钮（0.5 = 全路径减半；规划路径如需同步收紧调 MoveIt joint_limits.yaml），默认 1.0 = 不改变规划路径现状、只堵「无限制」漏洞
  - 新辅助 `_velocity_floor_duration(q0, q1, joint_names)` = `_SPLINE_PEAK_FACTOR × max_i |Δq_i| / (vmax_i × scale)`（未知关节名跳过；Δq=0 为 0）。**形状系数来源**：JTC VARIABLE_DEGREE_SPLINE 两点（端点 v/a=0）实测峰值/平均速度≈2.0–2.09（mock 全栈、T=0.5/1.0 s、200 Hz 微分数得），取 `_SPLINE_PEAK_FACTOR=2.2` 留 ~5% 余量；topic 线性后端按此只更保守
  - **集中在 `_two_point_trajectory()` 内强制** `duration = max(duration, floor)`——6 个调用点全覆盖、零漏网；扩展时 WARN 节流日志（关键字 `F94 duration extended`，含原值/地板值/最慢关节）
  - 响应语义：jog 服务响应回显实际采用时长（`/a3/arm/set_joint_positions` message 已含 duration；jog 定时器在 jog 调用点同样用地板后的时长，避免 back-to-ready 提前触发）
- **改动：**
  1. `src/a3_arm_controller/a3_arm_controller/arm_controller.py`：URDF velocity 解析、`joint_velocity_scale` 参数、`_velocity_floor_duration()`、`_two_point_trajectory()` 集中强制 + 日志；jog 调用点计时同步
  2. 新增 `scripts/a3_test/f94_velocity_limit_acceptance.py`：mock 全自动验收（不触碰 CAN）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f94_velocity_limit_acceptance.py`）：**
  1. 7 关节 URDF velocity 全部加载（L1–L3=33、L4–L7=50 rad/s）
  2. 超限请求：L1 Δq=3.0 rad、duration=0.05 s（vmax=33 → 地板 2.2×3.0/33=0.20 s）→ 响应回显 duration ≥ 0.20 s；从 `/joint_states` 中心差分实测运动窗口内 L1 峰值速度 ≤ vmax×scale（容差 5%；中心差分容忍 DDS 成批投递的早到帧，前向差分实测有 2 倍假峰）
  3. 保守请求（Δq=0.1 rad、duration=2.0 s）→ duration 不被修改（=2.0 s）
  4. `joint_velocity_scale:=0.5` 重测同超限请求：地板翻倍（0.40 s）、实测 L1 峰值速度 ≤ 0.5×33（+容差）
  5. 回归：safe-park（disable 回 home）正常完成、最终位姿在 home 容差内
- **关联：** F88（两点标准轨迹取代手搓稠密插值）、F40/F41（park/move_to 时长语义）、[shared/SAFETY.md](../shared/SAFETY.md)；MoveIt joint_limits.yaml 与 URDF 必须同源（本需求不改二者数值）
- **状态：** completed（2026-09-23，mock 全栈验收 11/11：`scripts/a3_test/f94_velocity_limit_acceptance.py`。实测：7 关节速度限加载（L1–L3=33、L4–L7=50）；L1 Δq=3.0/0.05s → 回显 duration=0.200s、中心差分峰值 28.87 rad/s；保守 Δq=0.1/2.0s → 2.000s 不变；scale=0.5 → 0.400s、峰值 14.94 rad/s；disable safe-park 成功、最终 worst=0.010 rad。形状系数 2.2 与中心差分的踩坑见 LL-106）

### F91 — 电机零点/参数维护产品化（独立维护节点 + 控制器活动联锁；退役手柄长按调零死映射）

- **说明：** 产品栈 SystemInterface 插件不暴露任何维护服务：协议层 `BuildSetZeroFrame`（0x06）/`BuildSaveParamFrame`（0x16）在产品栈不可达；L7 零点重标定（断电丢多圈计数）目前只能切 deprecated `can_bridge` legacy 栈。PS4 Options 长按经 default 映射发 `/power_sequence/command` "set_zero"，产品栈无消费者（死映射）。对标 EDULITE_A3 SDK（`SetZeroPosition`/`SaveParameters`：先停控制环、逐电机发送、50 ms 间隔）与工业现场维护惯例（维护工具独立运行、控制环停止后执行），新增**独立维护节点** `motor_maintenance`（不做进 controller_manager 插件——SystemInterface 不承载服务）：
  - 服务 `/a3/maintenance/set_zero`、`/a3/maintenance/save_parameters`，类型新 `a3_msgs/srv/MotorIdCommand`（`uint8 motor_id`，255=全部电机；响应 `bool success` + `string message`）
  - **安全联锁**：查询 `controller_manager/list_controllers`；`arm_controller`/`gripper_controller`/`zero_torque_controller` 任一 active 即拒绝且不发任何 CAN 帧；controller_manager 不存在（栈已停、节点独立启动）放行；`enforce_controller_interlock` 可关
  - 发送顺序：逐电机 `BuildSetZeroFrame`/`BuildSaveParamFrame`，帧间隔 50 ms（`inter_command_delay_ms` 可配）；零点后不做目标重同步（SDK 的 `_sync_command_targets_from_feedback` 服务于其进程内控制环；产品栈停机维护，无目标可同步）
- **改动：**
  1. `a3_msgs` 新增 `srv/MotorIdCommand.srv`
  2. `a3_hardware_interface` 新增可执行 `motor_maintenance`（`src/motor_maintenance_node.cpp`，复用 `SocketcanTransport` + `ProtocolCodec`；参数 `can_interface`/`motor_ids`/`inter_command_delay_ms`/联锁参数；新增依赖 `a3_msgs`、`controller_manager_msgs`）
  3. `a3_bringup` 新增 `motor_maintenance.launch.py`（仅维护节点；`can_interface` 参数，默认 can1）
  4. 退役死映射：`default.yaml` 移除 Options 长按 `power_set_zero`（保留短按 teach_stop）；`action_registry.yaml`/`actions.py` 移除 `power_set_zero`
  5. `vcan_motor_sim.py` 建模 0x06（角度归 0、速度/力矩清零、`set_zero_count++`）与 0x16（仅数据域为 01..08 时 `save_param_count++`，LL-019）；状态文件输出两计数
- **验收标准（仿真；断电；脚本 `scripts/a3_test/f91_maintenance_acceptance.py`，vcan91，ROS_DOMAIN_ID=91）：**
  1. 独立维护：`set_zero` 单机与 255 全发 → sim 状态文件计数正确、被标定电机角度归 0
  2. `save_parameters` 单机与 255 全发 → flash 计数正确
  3. `motor_id` 不在 `motor_ids`（如 9）→ `success=false`，无任何计数增加
  4. 联锁：产品栈 enable→READY 后两个维护服务均 `success=false`（message 指出 active 控制器）且计数不增；停栈（controller_manager 消失）后恢复放行
- **关联：** L7 零点标定（断电丢多圈计数）、LL-019（save 数据域须 01..08）、F83（使能编排）、F40（停机维护语义）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- **状态：** `completed`（2026-09-23，vcan91 仿真验收 16/16：单机/255 广播 zero+save 计数与角度归零正确、越界 id 拒绝无副作用、READY 态联锁拒绝并列出 active 控制器、停栈后放行；LL-102）。真机验收待通电

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
