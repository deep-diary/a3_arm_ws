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

### F24 — LeRobot robot 插件（lerobot_robot_a3，A0 数据地基）

- **说明：** 新增独立可 `pip install` 的 LeRobot 插件包 `lerobot_robot_a3`（包名前缀 `lerobot_robot_`，被 LeRobot 自动发现，`--robot.type=a3`），把 EDULITE A3 作为 7-DOF follower 臂接入 LeRobot 数据采集/回放工具链。插件**内部只用 ROS 2 接口，不碰 CAN**：`get_observation()` 订阅 `/joint_states`（`sensor_msgs/JointState`，按 `L1_joint`..`L7_joint` 排序取弧度位置）；`send_action()` 把 7 维关节位置动作按软限位裁剪后，组单点 `trajectory_msgs/JointTrajectory` 发布到执行层主话题 `/joint_group_effort_controller/joint_trajectory`；`connect()`/`disconnect()` 以 best-effort 方式调用门面服务 `/a3/arm/enter_ai`、`/a3/arm/exit_ai`（F21，服务缺失/超时时不致命）。ROS 侧收发一律**弧度 + URDF 关节系、不乘 joint_signs**（对齐 TOPIC_CONTRACT）；L7 夹爪有效区间 0(闭)~1.5708(开)。关节名/限位/默认话题镜像 `src/a3_lerobot_config/config/a3_robot.yaml`。A3 回零走 `/a3/arm/init`，插件提供 passthrough/identity 校准，使 `lerobot-calibrate` 不构成阻塞。本期 `cameras` 为空（无图像观测）。
- **验收标准：**
  1. `lerobot_robot_a3` 可 `pip install -e .`（普通 pip 包，无 package.xml，colcon 自动跳过）；安装后 LeRobot 能自动发现并以 `--robot.type=a3` 实例化（robot registry 含 `a3`）
  2. 仿真栈（`edge_moveit_execute.launch.py use_sim:=true` + `arm_controller.launch.py require_gate:=false`，均 `use_rviz:=false`）下：插件 `connect()` 后 `get_observation()` 返回 7 维弧度状态且与 `/joint_states` 一致
  3. `send_action()` 下发一个合法 7 关节目标后，`/joint_states` 经 sim_executor 跟随到位（关节名/顺序/弧度正确，软限位外的目标被裁剪）
  4. `connect()` 时 `/a3/arm/enter_ai` 被调用（`/a3/arm_status` 进入 AI 态）；门面服务不存在时插件不报错、可继续；`disconnect()` best-effort 调 `exit_ai`
  5. 插件 README 含可复制的安装与仿真验证命令；动作不绕过门控/软限位（SAFETY）
- **关联：** [shared/AI_ROADMAP.md](../shared/AI_ROADMAP.md) A0 / F24；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（关节状态/轨迹主话题/门面服务契约）；[shared/SAFETY.md](../shared/SAFETY.md)；F21（编排节点 enter_ai/exit_ai）、F10（FJT Action）；[`a3_lerobot_config`](../../src/a3_lerobot_config)（`a3_robot.yaml` 关节定义）；插件包 [`src/lerobot_robot_a3`](../../src/lerobot_robot_a3)（含 README 仿真验证步骤）
- **状态：** `implemented`（仿真链路实测通过：LeRobot 可发现 `--robot.type=a3`；`get_observation` 读 7 关节弧度、`send_action` 驱动 `/joint_states` 到位且软限位裁剪生效、enter_ai/exit_ai best-effort 成功；真机 episode、相机与训练待硬件/GPU 服务器）

### F25 — LeRobot 相机观测 + 数据采集（lerobot-record，仿真先行）

- **说明：** 在 F24 的 `lerobot_robot_a3` 插件上补齐**相机观测**与**数据采集落盘**能力，使 A3 能用 `lerobot-record` 录制包含 `observation.state`(7 关节)、`action`(7 关节)、`observation.images.<cam>`(RGB) 的 LeRobotDataset v2 数据集（parquet + mp4）。本期**无硬件仿真先行**：插件新增一个 LeRobot 相机类型 `ros_topic`（`@CameraConfig.register_subclass`），内部用 rclpy 订阅 `sensor_msgs/Image`，用 numpy 按 `rgb8`/`bgr8` 直接解析为 **uint8 / HWC / RGB** 帧（系统 ROS 未装 cv_bridge，故不依赖 cv_bridge）；关节特征由 F24 的向量 `{"state":(7,)}` 修正为 LeRobot 录制管线要求的**逐关节 `float` 标量**（`L1.pos`…`L7.pos`，否则 tuple 会被误判为相机）。另提供一个虚拟遥操作 `a3_auto`（`@TeleoperatorConfig.register_subclass`，输出限内正弦动作），使真实 `lerobot-record --robot.type=a3 --teleop.type=a3_auto` CLI 可人工按键录制；自动化测试用程序化 `LeRobotDataset.create(...)` 脚本（避开 record 的键盘门控），视频编码用板上 ffmpeg 可用的 `mpeg4`（无 libx264/libsvtav1）。仿真侧新增 `a3_bringup/sim_camera` 节点，纯 numpy 发布 `/camera/color/image_raw`（`rgb8`，画面随 `/joint_states` 关节角调制），并在 `edge_moveit_execute.launch.py use_sim` 条件下拉起；预留 `/camera/depth/image_raw`(16UC1) 与 `/camera/color/camera_info`。真机 Gemini 2 由 OrbbecSDK_ROS2 发布同一 `/camera/color/*` 话题，插件零改动切换。深度图本期不落盘（LeRobot 0.4.4 录制管线仅接受 len==3 的图像 tuple）。
- **验收标准：**
  1. `lerobot_robot_a3` 安装后，LeRobot 可发现机器人 `a3`、相机 `ros_topic`、遥操作 `a3_auto`（`register_third_party_plugins()` 后 registry 均含）
  2. 仿真栈（`edge_moveit_execute.launch.py use_sim:=true` 含 `sim_camera` + `arm_controller`）下，`/camera/color/image_raw` 以 `rgb8` 持续发布；`RosTopicCamera.read()` 返回 `(480,640,3)` uint8 RGB 帧且帧间有变化
  3. `robot.observation_features` 含 7 个逐关节 `float`（`L1.pos`…`L7.pos`）与 1 个 `(480,640,3)` 图像特征；`get_observation()` 返回对应关节标量与相机图像 key
  4. 程序化录制脚本跑通 ≥2 个短 episode：`meta/info.json` 含 `observation.state(7)`/`action(7)`/`observation.images.head(480,640,3)`；`data/**/*.parquet` 帧数正确；`videos/**/*.mp4` 存在且非空；读回数据集图像为 HWC、state/action 为 7 维
  5. F24 回归：`send_action` 仍驱动 sim_executor `/joint_states` 到位并保持，软限位裁剪生效，`enter_ai`/`exit_ai` best-effort 成功
  6. 真实 `lerobot-record --robot.type=a3 --teleop.type=a3_auto --dataset.push_to_hub=false --dataset.vcodec=mpeg4 ...` 配置可解析（CLI 可发现，人工按键录制命令写入插件 README）
  7. 仿真与真机共用 `/camera/color/image_raw` 契约（见 TOPIC_CONTRACT）；`sim_camera` 纯 numpy 不依赖 cv2/cv_bridge
- **关联：** [shared/AI_ROADMAP.md](../shared/AI_ROADMAP.md) A0/A1 / F25；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（新增「相机」话题契约：`/camera/color/image_raw`、`/camera/color/camera_info`、`/camera/depth/image_raw`）；F24（robot 插件）、F21（enter_ai/exit_ai）；插件包 [`src/lerobot_robot_a3`](../../src/lerobot_robot_a3)（`ros_camera.py`/`auto_teleop.py`/`record_sim.py`）；仿真节点 [`src/a3_bringup`](../../src/a3_bringup)（`sim_camera.py`）；真机驱动待硬件（OrbbecSDK_ROS2 Gemini 2）
- **状态：** `implemented`（仿真链路实测通过：`ros_topic` 相机/`a3_auto` 遥操作/`a3` 机器人三类插件均可被 LeRobot 发现；`sim_camera` 发布 `rgb8` `/camera/color/image_raw`；程序化录制落盘 `observation.state(7)`/`action(7)`/`observation.images.head(480,640,3)` → parquet + mp4，读回 10/10 断言通过；F24 关节回归通过；真机 Gemini 2 驱动、手眼标定、深度落盘待硬件）

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
