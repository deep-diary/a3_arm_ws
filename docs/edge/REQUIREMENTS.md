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

### F7 — 命名姿态 zero→work 仿真闭环（C1）

- **说明：** 支持命名姿态 `zero`（上电全零）与 `work`（目标工作位）；无真机 CAN 时可通过仿真执行器完成 Plan/下发与 `/joint_states` 跟踪
- **验收标准：**
  1. SRDF/`named_poses.yaml` 含 `work`（L2≈51°, L3≈-57°）
  2. 仿真 launch 下从 `zero` 运动到 `work`，最终关节误差在容差内
  3. 文档化 launch/脚本命令（见 QUICKSTART / WAVE_A 测试报告）
  4. `use_rviz:=true` 启动 `el_a3_view.rviz` 可视化模型（默认 `false`，无屏验收不启 GUI）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C1；[shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)
- **状态：** `implemented`（仿真验收，见 [WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)）

### F8 — 重力力矩计算（C3 仿真可验部分）

- **说明：** 基于模型（Pinocchio 优先，否则标定惯量解析近似）按当前 `joint_states` 计算重力补偿力矩并发布；提供启停服务骨架，与轨迹模式互斥标志
- **验收标准：**
  1. 在 `zero` 与 `work` 稳态可采样到有限力矩
  2. `work` 下 L2/L3 重力力矩量级大于腕部小关节（抬臂）
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
  5. D-pad 上/下/左/右分别到 `work` / `zero` / `home` / `ready`
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
  4. 10 个动作可下发：初始化/使能/失能、示教开始/结束、保存/回放（带轨迹名输入）、goto（zero/home/ready/work 下拉）、进入/退出 AI；点击先弹二次确认，确认后才 publish
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
- **状态：** `implemented`（仿真软/硬物体闭环 ±10% 且 GRASPED、超力/看门狗 FAULT 均通过；真机物体抓取板测中）

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
5. Triangle shutdown 后电机 disable，gate 关闭
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
