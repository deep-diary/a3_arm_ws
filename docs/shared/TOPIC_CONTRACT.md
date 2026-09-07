# ROS 话题契约

> **Status:** active

A3 Edge 与 A3 CloudEdge 必须遵守的统一消息契约。实现位置不同（板载 ROS vs micro-ROS），话题名与消息类型保持一致。

## 消息类型

| 用途 | 类型 | 说明 |
|------|------|------|
| 轨迹命令 | `trajectory_msgs/JointTrajectory` | `joint_names`、`points[]`：`positions` 必填；`velocities` 由规划时间参数化填入（MoveIt TOTG）；`accelerations` 可选（执行层五次插值）；`effort` 为开环重力补偿（Nm），与 `positions` 同一 URDF 关节系，**不**乘 `joint_signs`；另有 `time_from_start` |
| 轨迹执行 Action | `control_msgs/action/FollowJointTrajectory` | MoveIt Execute / 标准控制器入口 |
| 关节反馈 | `sensor_msgs/JointState` | position、velocity、effort（部分字段可为 NaN） |
| 电源门控 | `std_msgs/Bool` | `true` 时允许轨迹执行 |
| 电源命令 | `std_msgs/String` | `start` / `shutdown` / `set_zero` |
| 电源状态 | `std_msgs/String` | 电源序列当前状态（可选订阅） |
| 控制模式 | `std_msgs/String` | `IDLE` / `TRAJ_RUNNING` / `GRAVITY_COMP` / `ZERO_TORQUE` / `SERVO` |
| 笛卡尔 IK | `a3_msgs/srv/MoveToPoseIK` | 位姿 → 关节解 |
| 笛卡尔运动 | `a3_msgs/action/MoveToPose` | IK + 轨迹执行 |
| Servo 速度 | `geometry_msgs/TwistStamped` | MoveIt Servo 输入 |
| 手柄 | `sensor_msgs/Joy` | `joy_node` 轴/按键 |
| 命名姿态 | `std_msgs/String` | `zero` / `work` / `home` / `ready` |
| 夹爪开合 | `std_msgs/Float32` | 0 闭合 … 1 张开（POSITION 模式输入） |
| 夹爪力控命令 | `a3_msgs/srv/GripperCommand` | `mode`：`position`/`force`/`release`/`stop`；`position` 0–1；`torque_nm` 目标握力；`timeout_s` |
| 夹爪配置 | `a3_msgs/srv/GripperSetConfig` | 键值下发（`max_torque_nm` 等），返回是否接受与原因 |
| 夹爪状态 | `a3_msgs/msg/GripperStatus` | 模式、目标/实际力矩、目标/实际位置、接触/抓稳标志、错误码、时间戳 |
| DS4 IMU | `sensor_msgs/Imu` | 可选 hidraw（陀螺/加速度） |
## 标准话题（单臂，无 namespace）

### 轨迹输入（订阅侧 / 执行层监听）

执行层（`motor_protocol_node` 或 ESP32）默认监听：

| 话题 | 优先级 | 说明 |
|------|--------|------|
| `/joint_group_effort_controller/joint_trajectory` | 主 | 执行层默认输入（含 `effort` 重力补偿后） |
| `/a3/planned_joint_trajectory` | 规划输出 | CloudEdge：MoveIt / 测试发布器 → 重力补偿节点 |
| `/a3/joint_trajectory` | 桥接 | 测试与外部集成 |
| `/rebotarm/joint_trajectory` | 桥接 | reBot 工具链输出 |

`a3_bringup/trajectory_bridge` 将上述话题统一转发到 `/joint_group_effort_controller/joint_trajectory`。

### FollowJointTrajectory Action（执行层）

| Action | 说明 |
|--------|------|
| `/arm_controller/follow_joint_trajectory` | `a3_bringup` FJT Server → 转发轨迹话题；MoveIt `moveit_controllers.yaml` 默认指向此处 |

话题桥 **不等于** Action：仅转发 `JointTrajectory` 话题，无 goal/feedback/result。真机 MoveIt Execute 需要本 Action。

### 控制模式与高级服务

| 接口 | 类型 | 说明 |
|------|------|------|
| `/a3/control_mode` | `std_msgs/String` | 模式互锁广播 |
| `/a3/gravity_compensation/start\|stop` | `std_srvs/Trigger` | 重力补偿 |
| `/a3/zero_torque/start\|stop` | `std_srvs/Trigger` | 零力矩/拖动（软 kp + 重力 FF） |
| `/a3/move_to_pose_ik` | `a3_msgs/srv/MoveToPoseIK` | 仅 IK |
| `/a3/move_to_pose` | `a3_msgs/action/MoveToPose` | IK + 执行 |
| `/a3/gravity_torque` | `sensor_msgs/JointState` | URDF 系重力力矩（effort） |
| `/a3/goto_named_pose` | `std_msgs/String` | 命名姿态（`zero`/`work`/`home`/`ready`） |
| `/a3/gripper_cmd` | `std_msgs/Float32` | 夹爪归一化 0–1（POSITION 模式，web 直驱与遗留路径；由 `gripper_controller_node` 订阅执行）。PS4 R2 自 F36 起不再走此话题，改走 `/a3/gripper/command` 力控服务 |
| `/a3/gripper_status` | `a3_msgs/msg/GripperStatus` | 夹爪力控状态快照（模式/目标与实际力矩/目标与实际位置/接触标志/错误码），默认 10 Hz，力控期间 50 Hz |
| `/a3/gripper/command` | `a3_msgs/srv/GripperCommand` | 夹爪命令：`position`（开合 0–1）/ `force`（按 `torque_nm` 抓取）/ `release` / `stop` |
| `/a3/gripper/set_config` | `a3_msgs/srv/GripperSetConfig` | 握力参数下发（如 `max_torque_nm`）；越界（超硬上限/±6 Nm）拒绝并返回原因 |
| `/joy` | `sensor_msgs/Joy` | PS4 轴与按键 |
| `/a3/ds4/imu` | `sensor_msgs/Imu` | 可选 DualShock 4 HID 惯性 |
| `/a3/ds4/battery` | `std_msgs/Float32` | 可选电量 0–1 |

**插值语义（执行层）：** 仅 positions → 线性；+velocities → 三次；+accelerations → 五次；effort 始终线性（JTC 对齐）。参数 `trajectory_interpolation_method`（默认 `auto`）。

### 关节状态（发布侧）

| 话题 | 说明 |
|------|------|
| `/joint_states` | 电机反馈汇总，默认 50 Hz |
| `/rebotarm/joint_states` | `trajectory_bridge` 镜像输出，供 reBot 消费 |

### 电源序列

| 话题 | 方向 | 说明 |
|------|------|------|
| `/power_sequence/gate_open` | 发布 | `true` 后轨迹方可下发 CAN |
| `/power_sequence/command` | 订阅 | `start` / `shutdown` / `set_zero` |
| `/power_sequence/state` | 发布 | 序列状态 |
| `/power_sequence/set_zero_event` | 发布 | 调零事件（可选） |

配置见 [power_sequence.yaml](../../src/a3_can_bridge/config/power_sequence.yaml)。

## 机械臂编排（a3_arm_controller）

统一对外交互门面（需求 [F21](../edge/REQUIREMENTS.md)），底层复用 `/a3/motor/*`、`/power_sequence/*`、`/arm_controller/follow_joint_trajectory`、`/servo_node/*`、`/a3/zero_torque/*`、`/a3/gravity_compensation/*`，自身只做状态机、生命周期、示教与模式仲裁。

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/a3/arm/init` | `std_srvs/Trigger` | 设零 → 异步确认 7 电机到位 → 使能；`message` 携带 `n/7` |
| `/a3/arm/enable` | `std_srvs/Trigger` | 使能 7 电机，状态 → `READY` |
| `/a3/arm/disable` | `std_srvs/Trigger` | 失能 7 电机，状态 → `IDLE` |
| `/a3/arm/goto_named_pose` | `a3_msgs/srv/GotoNamedPose` | `pose_name` 按 `named_poses.yaml` 插值下发 |
| `/a3/arm/set_joint_positions` | `a3_msgs/srv/SetJointPositions` | 设 7 关节目标位置（`positions[7]` + `duration`），限位 clamp 后短插值下发；节流连续下发以覆盖语义衔接 |
| `/a3/arm/start_teach` | `std_srvs/Trigger` | 切零力矩拖动 + 开始记录 |
| `/a3/arm/stop_teach` | `std_srvs/Trigger` | 停止记录 + 退出拖动 |
| `/a3/arm/save_trajectory` | `a3_msgs/srv/SaveTrajectory` | `name` → 保存为本地轨迹文件 |
| `/a3/arm/playback` | `a3_msgs/srv/PlaybackTrajectory` | `name` → 读取并回放 |
| `/a3/arm/enter_ai` | `std_srvs/Trigger` | 状态 → `AI`（LeRobot 采集/回放） |
| `/a3/arm/exit_ai` | `std_srvs/Trigger` | 状态 → `READY` |

### 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/a3/arm_status` | `a3_msgs/msg/ArmStatus` | 聚合状态快照（`state` + `mode` + 7 关节位置 + 时间戳），默认 10 Hz，作为前端唯一状态入口 |

### MQTT 下行指令与回执（a3_mqtt_bridge ↔ Web，需求 F23）

桥接节点 `a3_mqtt_bridge` 订阅 `<prefix>/cmd`（`prefix = deep-trace/HOME-DEMO/RK3588`），按白名单 op 调用上节 `/a3/arm/*` 服务，并把结果回发到 `<prefix>/cmd_result`。指令通道为浏览器 mqtt.js 直连 EMQX（WS 8083），Django 后端不经手。

下行 `cmd`（JSON，QoS 建议 1）：

```json
{ "op": "goto", "args": { "pose": "home" } }
```

| op | args | 对应服务 |
|----|------|----------|
| `init` | `{}` | `/a3/arm/init`（Trigger） |
| `enable` | `{}` | `/a3/arm/enable` |
| `disable` | `{}` | `/a3/arm/disable` |
| `goto` | `{"pose": "zero\|home\|ready\|work"}` | `/a3/arm/goto_named_pose`（`pose_name=args.pose`） |
| `set_joints` | `{"positions": [7 个 rad], "duration": 0.3}` | `/a3/arm/set_joint_positions`（`SetJointPositions`，滑动条 jog 直驱，限位 clamp + 短插值） |
| `teach_start` | `{}` | `/a3/arm/start_teach` |
| `teach_stop` | `{}` | `/a3/arm/stop_teach` |
| `save` | `{"name": "<轨迹名>"}` | `/a3/arm/save_trajectory`（`name=args.name`） |
| `playback` | `{"name": "<轨迹名>"}` | `/a3/arm/playback` |
| `enter_ai` | `{}` | `/a3/arm/enter_ai` |
| `exit_ai` | `{}` | `/a3/arm/exit_ai` |

回执 `cmd_result`（JSON）：

```json
{ "op": "goto", "ok": true, "message": "goto home done", "ts": "2026-09-02T13:30:00+08:00" }
```

- `op`：回显指令 op；`ok`：服务 `success`（未知 op / 服务不可用 / 调用异常均为 `false`）；`message`：服务返回文本或错误原因（如 `init` 的 `n/7`）；`ts`：本地 ISO8601 时间戳。
- 未知 op 不抛异常，回 `ok=false, message="unknown op ..."`；服务未就绪回 `ok=false, message="<op> service unavailable"`。
- `/a3/arm_status` 经 `bridge.yaml` 的 `scalar` 展平上报为 telemetry `points.arm_state`（=`state`）、`points.arm_mode`（=`mode`）、`points.arm_message`（=`message`），前端据此渲染状态，不另开话题。

**遥测发布速率（F35）：** telemetry 按源话题速率聚合不发布——各话题回调只更新缓存，由桥内 flusher 按 `telemetry_min_interval_sec`（默认 0.2 s = 5 Hz 上限）统一发布全量 points，**最新值胜出**。`cmd_result`/`device/status`/`device/info` 不受节流、永不丢（遥测队列满时只丢遥测）；发布经独立线程，broker 慢不影响 cmd 处理。前端绘制曲线无需 50 Hz 原始采样，若曲线不平滑属正常（降频预期）。

### 状态机与仲裁

- 状态：`IDLE → INIT → READY`；`READY ↔ TRAJ / SERVO / TEACH / AI`；`READY → FAULT`。
- 运动类命令（`goto_named_pose` / `playback`）在 `mode ∈ {ZERO_TORQUE, SERVO, GRAVITY_COMP}` 或 gate 关闭（`require_gate:=true` 时）拒绝。

## 夹爪力控（a3_gripper_controller，需求 F24–F27）

独立 Python 节点 `gripper_controller_node`（50 Hz 力外环），输出 L7 单关节 `JointTrajectory` 走现有执行层；不改动 C++ CAN 实时路径。力环只在 Edge 本地运行（见 [SAFETY.md](SAFETY.md)）。

### 服务与话题

| 接口 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/a3/gripper/command` | `a3_msgs/srv/GripperCommand` | 服务 | `mode=position`（`position` 0–1）/ `force`（`torque_nm` 目标握力，`timeout_s`）/ `release` / `stop` |
| `/a3/gripper/set_config` | `a3_msgs/srv/GripperSetConfig` | 服务 | 下发 `max_torque_nm` 等；校验 `≤ max_grasp_torque_nm` 且在 ±6 Nm 内，越界 `success=false` |
| `/a3/gripper_cmd` | `std_msgs/Float32` | 订阅 | 归一化开合（0 闭…1 开），POSITION 模式输入 |
| `/a3/gripper_status` | `a3_msgs/msg/GripperStatus` | 发布 | 状态快照，默认 10 Hz，力控期间 50 Hz |
| `/joint_states` | `sensor_msgs/JointState` | 订阅 | 取 `L7_joint` 的 `effort` 作力反馈（MIT 力矩，±6 Nm 量程） |
| `/a3/control_mode`、`/power_sequence/gate_open` | `std_msgs/String`/`Bool` | 订阅 | 互锁：gate 关闭或臂在 `TRAJ_RUNNING`/`SERVO`/`ZERO_TORQUE`/`GRAVITY_COMP` 时拒绝 `force` |

`GripperStatus` 字段语义：`state`（`IDLE`/`POSITION`/`FORCE_CLOSING`/`GRASPED`/`RELEASING`/`FAULT`）、`mode`（最近命令模式）、`target_torque_nm`、`actual_torque_nm`、`position`（0–1 实测开合）、`target_position`（0–1，最近 position/release 命令目标；未命令前跟随实测，需求 F31；力控期间为 PI 实时位置目标，需求 F33）、`contact`（接触/抓稳）、`error_code`（0 无；1 互锁拒绝；2 抓取超时；3 反馈看门狗；4 超硬限；5 配置越界；6 固件硬限写入失败）。

### MQTT 下行指令（a3_mqtt_bridge ↔ Web，需求 F26/F27）

同一 `<prefix>/cmd` 与 `<prefix>/cmd_result` 通道，白名单追加 5 个 op：

| op | args | 对应服务 |
|----|------|----------|
| `gripper_grasp` | `{"torque": <Nm>}` 或 `{"preset": "weak"\|"medium"\|"strong"}` | `/a3/gripper/command`（`mode=force`） |
| `gripper_release` | `{}` | `/a3/gripper/command`（`mode=release`） |
| `gripper_stop` | `{}` | `/a3/gripper/command`（`mode=stop`） |
| `gripper_set_max_torque` | `{"value": <Nm>}` | `/a3/gripper/set_config`（`max_torque_nm=args.value`） |
| `gripper_set_position` | `{"position": <0..1>}` | `/a3/gripper/command`（`mode=position`，需求 F31） |

- `torque`/`value` 越界、`position` 非有限或越界 [0,1]、缺失 preset 时回 `ok=false`，服务端不执行；回执 JSON 格式与臂指令一致（`op`/`ok`/`message`/`ts`）。
- 桥接层对全部下行载荷做非有限浮点清洗（NaN/±Inf → null）且 `json.dumps(allow_nan=False)` 硬兜底，保证线上 JSON 恒合法（[LL-011](../../lessons_learned/LL-011-nan-poisons-json-telemetry.md)）。
- `/a3/gripper_status` 经 `bridge.yaml` 的 `scalar` 展平上报 telemetry points：

| points key | 来源字段 | 类型 |
|------------|----------|------|
| `grip_state` | `state` | discrete |
| `grip_mode` | `mode` | discrete |
| `grip_target_torque` | `target_torque_nm` | float（Nm） |
| `grip_actual_torque` | `actual_torque_nm` | float（Nm） |
| `grip_position` | `position` | float（0–1，实测开合） |
| `grip_target_position` | `target_position` | float（0–1，目标开合，F31） |
| `grip_contact` | `contact` | discrete（0/1） |
| `grip_error` | `error_code` | discrete |

信号 code 须与 deep-trace 设备 YAML `HOME-DEMO.RK3588.yaml` 的 `points[].code` 完全对齐。

### PS4 扳机力控契约（F36）

`a3_teleop_ps4` 的 `ps4_mapper` 把 R2 扳机（`trigger_01` 归一化，0=松开…1=按满）经 `/a3/gripper/command` 服务映射到夹爪力控（`simple.yaml` 与 `default.yaml` 同改，L2→L6 不变）：

- 迟滞：按过 **0.22** 进入力控；松开低于 **0.15** 才发一次 `release`（全开），避免抖动来回切换。
- 力矩映射：扳机 0.2..1 → 目标力矩 **0.1..1.0 Nm** 线性（下限 0.1：目标 ≤0 触发「直接全开」硬逻辑、接触判定下限 0.1 Nm；上限对齐 `max_grasp_torque_nm` 硬上限）。
- 持按期间目标变化 **≥0.1 Nm** 才重发 `force`（频繁重发会重置力环积分退化成纯 P）；`timeout_s=15`（与节点默认一致，GRASPED 后持续持握）。
- 服务未就绪或互锁拒绝（臂在 `TRAJ_RUNNING`/`SERVO`/`ZERO_TORQUE`/`GRAVITY_COMP`）→ 每 **0.5 s** 重试，松手即停；`default.yaml` 仍受 L1 死人开关门控（松 L1 而 R2 按住 → mapper 停发 → 保持当前夹持，再按 L1 恢复跟随）。
- 遗留位置路径（`set_gripper`/`set_joint_L7`/Square/Circle 按键）常量已对齐 2026-09-06 标定（开=0/闭=1.79）。

## 电机调试（motor_protocol_node，需求 F32）

Web 端单电机调试页（deep-trace `rk3588_motor` 模块）依赖的 ROS 侧契约：逐电机结构化遥测 + CAN 扫描聚合 + 单电机 MIT 直驱。全部接口由 C++ `motor_protocol_node` 提供（无 CAN 仿真下由 `sim_motor_node` 等效实现）。

### 话题与消息

| 接口 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/a3/motor/states` | `a3_can_bridge/msg/MotorStates` | 发布 | 7 条 `MotorState`（`header` + `states[]`），50 Hz，SensorDataQoS（best_effort） |

`MotorState` 字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `motor_id` | uint8 | CAN_ID 1..7 = L1..L7 |
| `master_id` | uint8 | 反馈帧 master id（默认 0xFD，无反馈时保持默认值） |
| `position_rad` / `speed_rad_s` / `torque_nm` / `temperature_c` | float32 | MIT 域原始值；**无反馈时发 0 而非 NaN**（避免 MQTT JSON 污染，用 `fresh`/`has_feedback` 表无效） |
| `mode_status` | uint8 | 固件 bit22-23：0=复位 1=标定 2=闭环 |
| `error_status` | uint8 | 聚合故障 0/1 |
| `fault_mask` | uint32 | bit0=综合 bit1=霍尔 bit2=磁编 bit3=过温 bit4=过流 bit5=电压 |
| `has_feedback` / `fresh` | bool | 是否收到过反馈 / 最近反馈在 `feedback_fresh_timeout_s` 内 |
| `enabled` | bool | `mode_status` 为闭环（尽力而为） |

### 服务（4 个新增）

| 服务 | 类型 | 说明 |
|------|------|------|
| `/a3/motor/scan_and_collect` | `a3_can_bridge/srv/MotorScanCollect` | 对 `[id_min,id_max]` 发 0x00 探针并聚合 device_id 回复直至 `timeout_s`（默认 1.5 s）；返回 `ids[]`/`uids[]`（同序，UID 为大端字节序 uint64）。busy 时拒绝；限时返回，永挂起 |
| `/a3/motor/mit_command` | `a3_can_bridge/srv/MotorMitCommand` | 单电机 MIT 直驱（`motor_id` 禁止 0 广播）：`hold_duration_s<=0` 单发一帧；`>0` ROS 侧定时保持（`hold_hz` 上限 `min(200, max_tx_rate_per_motor_hz)`，时长上限 `max_hold_duration_s` 默认 30 s；新保持替换旧保持）。数值越界自动 clamp |
| `/a3/motor/stop` | `a3_can_bridge/srv/MotorStop` | 取消该电机（0=全部）MIT 保持，并逐电机发一帧 `kp=kd=t=0` 卸力帧（p=最近反馈角） |
| `/a3/motor/set_mode` | `a3_can_bridge/srv/MotorSetMode` | 写 0x7005 运行模式：`mit`=0/`position`=1/`speed`=2；position 追加写 0x7016（目标位置）+0x7017（限速），speed 追加写 0x700A（目标速度）。切换取消该电机保持 |

**保持（hold）语义**：`hold_duration_s>0` 时由 `motor_protocol_node` 内部 5 ms tick 定时发帧（前端不做 setInterval 流式发送）；到时长自动停；`motor_stop` 手动取消；**`/power_sequence/gate_open` 由关→开的瞬间自动取消**（并记 WARN）。

### 互锁（gate 关闭才可调试写）

`gate_open=true`（电源序列运行中）时，以下**调试写操作**被 C++ 侧拒绝并返回带 gate 文案的 `success=false`：`/a3/motor/enable`（command=1）、`/a3/motor/reset`（command=2）、`/a3/motor/set_zero`（command=3）、`/a3/motor/set_param`、`/a3/motor/mit_command`、`/a3/motor/set_mode`、`/a3/motor/set_can_id`。**扫描、读类（get_device_id/request_version）、`/a3/motor/stop` 永不拦截**。a3_mqtt_bridge 不做前置 gate 预检（有意为之）：C++ 是权威拦截点，拒绝文案经 `cmd_result` 回传；预检会破坏 sim 闭环（`sim_power_sequence_node` 的 gate 恒 true，仿真有意不实现互锁）。

### MQTT 下行指令（a3_mqtt_bridge ↔ Web，需求 F32）

同一 `<prefix>/cmd` 与 `<prefix>/cmd_result` 通道，白名单追加 9 个 op：

| op | args | 对应服务 |
|----|------|----------|
| `motor_scan` | `{id_min?, id_max?, bus?, timeout_s?}`（默认 1/127/1/1.5） | `/a3/motor/scan_and_collect` |
| `motor_enable` / `motor_reset` / `motor_set_zero` | `{motor}`（int 1..127） | `/a3/motor/enable|reset|set_zero`（command=1/2/3） |
| `motor_mit` | `{motor, p, v?, kp, kd, t?}` | `/a3/motor/mit_command`（`hold_duration_s=0` 单发） |
| `motor_hold` | `{motor, p, v?, kp, kd, t?, duration_s, hz?}` | `/a3/motor/mit_command`（`hold_duration_s=duration_s`） |
| `motor_stop` | `{motor}`（0=全部） | `/a3/motor/stop` |
| `motor_set_mode` | `{motor, mode, position?, limit_spd?, speed?}`（mode∈mit\|position\|speed） | `/a3/motor/set_mode` |
| `motor_set_param` | `{motor, index, value}`（index 支持 int 或 `"0x7005"`） | `/a3/motor/set_param` |

- 校验失败（`motor` 非 int 1..127、浮点非法、mode 非法）时回 `ok=false` 且不调用服务；回执 JSON 格式与臂指令一致（`op`/`ok`/`message`/`ts`）。
- `motor_scan` 成功时 `cmd_result.message` 重编码为 JSON：`{"motors":[{"id":<int>,"uid":"<16位大写十六进制>"}, ...]}`（前端直接 `JSON.parse`）。

### 遥测展平（flatten=motor_state）

`/a3/motor/states` 经 `bridge.yaml` 的 `motor_state` 展平上报 telemetry points（6 前缀 × L1..L7 = 42 个）：

| points key（n=1..7） | 来源字段 | 类型 |
|----------------------|----------|------|
| `temp_Ln` | `temperature_c` | float（°C） |
| `err_Ln` | `fault_mask` | int（位掩码） |
| `mode_Ln` | `mode_status` | int（0 复位/1 标定/2 闭环） |
| `online_Ln` | `fresh` | 0/1 |
| `mtq_Ln` | `torque_nm` | float（Nm） |
| `mp_Ln` | `position_rad` | float（rad，MIT 原始角） |

信号 code 须与 deep-trace 设备 YAML `HOME-DEMO.RK3588.yaml` 的 `points[].code` 完全对齐。

## 关节名

轨迹与 `JointState` 中的 `joint_names` 必须使用：

```
L1_joint, L2_joint, L3_joint, L4_joint, L5_joint, L6_joint, L7_joint
```

## 多臂 namespace

多臂时每臂带前缀，由 [multi_arm_config.yaml](../../src/a3_description/config/multi_arm_config.yaml) 定义：

```yaml
arm1:
  prefix: "arm1_"
  can_interface: "can0"
arm2:
  prefix: "arm2_"
  can_interface: "can1"
```

话题示例（臂 1）：

- `/arm1/joint_states`
- `/arm1/joint_group_effort_controller/joint_trajectory`
- `/arm1/power_sequence/gate_open`

CloudEdge 服务器通过 micro-ROS Agent 为每臂分配独立 namespace，契约与上表相同。

## CAN 内部话题（仅 Edge）

Edge 主线在 ROS 层还有 CAN 帧桥接话题，CloudEdge 在 ESP32 固件内完成等效逻辑，不暴露到 ROS：

| 话题 | 说明 |
|------|------|
| `/can_tx_frames` | 待发 CAN 帧 |
| `/can_rx_frames` | 接收 CAN 帧 |

## 测试命令

```bash
ros2 topic pub --once /a3/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint], \
    points: [{positions: [0,0.5,-0.5,0,0,0,0], time_from_start: {sec: 2}}]}"
```

须在 `/power_sequence/gate_open` 为 `true` 后执行（Edge 主线）。
