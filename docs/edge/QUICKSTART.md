# A3 Arm — 快速上手（Edge 主线）

> **Status:** active  
> **产品线：** A3 Edge（`edge`）

0. **本机 WSL2 开发环境（推荐先做）：** 见 [dev/WSL2_SETUP.md](../dev/WSL2_SETUP.md)（Ubuntu 22.04 + Humble + mock/MoveIt）
   - 板载终端环境：`source ~/a3_arm_ws/scripts/a3_shell_env.sh`（`~/.bashrc` 已接入则新开终端自动生效）。看板载 HDMI 用 `DISPLAY=:0`，SSH **不要** `-X`/`-Y`。
1. Platform CAN（RK3588 真机）：见 [PLATFORM_CAN.md](PLATFORM_CAN.md)
2. Build packages listed in [../../README.md](../../README.md)
3. `ros2 launch a3_bringup a3_bringup.launch.py`（统一入口，默认起编排层 + MQTT 桥 + MoveIt + 夹爪力控 + PS4；按需关组件见下方）
   - 重力补偿 MIT 前馈（默认关）：`use_gravity_compensation:=true`
   - 或运行时：`ros2 param set /motor_protocol_node enable_gravity_compensation true`
   - 组件开关（默认值见 [ARCHITECTURE.md](ARCHITECTURE.md)「Launch 链」）：`use_arm_controller` / `use_mqtt` / `use_moveit` / `use_gripper` / `use_teleop` 默认开，`use_servo` / `use_rviz` / `use_gravity_compensation` 默认关。例：缺 L7 的 5J 档加 `use_gripper:=false`
4. PS4 电源：Square 长按 = start；Options 长按 = set_zero；Triangle = shutdown。笛卡尔/夹爪见第 10 节。
5. Send test trajectory:
   ```bash
   ros2 topic pub --once /a3/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint], points: [{positions: [0,0.5,-0.5,0,0,0,0], time_from_start: {sec: 2}}]}"
   ```
6. reBot shell: clone in `a3_arm_vendor`; remap via `ros2 run a3_bringup rebot_remap_info`
7. **Wave A 无 CAN 仿真（zero→ready + Pinocchio 重力）：**
   ```bash
   sudo apt install -y ros-humble-pinocchio   # 一次
   ./scripts/verify_wave_a_sim.sh             # 推荐全量验收
   # 或：
   ros2 launch a3_bringup edge_sim_wave_a.launch.py duration_s:=3.0 use_rviz:=true
   # 无屏验收省略 use_rviz（默认 false）
   ./scripts/dual_domain_zero_to_ready.sh      # Edge=10 / CloudEdge=20 均为 ROS_DOMAIN_ID
   ```
   报告：[dev/WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)

   **官方重力方向（EDULITE）：** Pinocchio `τ_g` 在 URDF 关节系；发 MIT 时再乘 `joint_signs=[-1,+1,-1,+1,-1,+1,+1]`（只乘一次）。

8. **Wave B 仿真（样条 / FJT / IK / 零力矩 / Servo）：**
   ```bash
   sudo apt install -y ros-humble-moveit-servo   # 一次
   ./scripts/verify_wave_b_sim.sh                # 推荐全量验收（含 Wave A 回归）
   # 或手动：
   ros2 launch a3_bringup edge_moveit_execute.launch.py use_sim:=true use_gravity:=true run_demo:=true use_rviz:=true
   ros2 launch a3_bringup servo.launch.py
   ```
   细节：[dev/WAVE_B_SIM_NOTES.md](../dev/WAVE_B_SIM_NOTES.md)  
   插值参数：`trajectory_interpolation_method:=auto`（线性/三次/五次按航点字段）

9. **MoveIt 拖动球 Plan & Execute（mock，非 Wave A/B）：**
   ```bash
   sudo apt install -y wmctrl ros-humble-controller-manager ros-humble-ros2-control ros-humble-pick-ik ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager ros-humble-moveit-ros-visualization
   export DISPLAY=:0
   ros2 launch a3_moveit_config demo.launch.py use_rviz:=true
   ```
   Interact → 拖末端球 → 面板 **Plan** / **Execute**。橙色半透明 = 目标模；Scene Robot = `/joint_states` 实际模。不发 CAN。

10. **PS4 映射遥操作（仿真，F16）：**
    ```bash
    ls /dev/input/js*                    # 确认手柄节点
    ros2 launch a3_teleop_ps4 ps4_teleop.launch.py dump:=true
    # 另开终端：摇遍轴/键，核对索引后写入 src/a3_teleop_ps4/config/ds4_linux.yaml

    export DISPLAY=:0
    # 重复 launch 前先清僵尸进程：pkill -f 'edge_teleop|a3_sim_executor|ps4_mapper|servo_node'
    ros2 launch a3_bringup edge_teleop_sim.launch.py use_rviz:=true
    # 默认 mapping:=simple（无 L1 组合键）；恢复 L1 死人开关：mapping:=default
    ```
    - **simple（默认）**：右摇杆基座系左右/上下；左摇杆 Y 前后；L2→L6、R2→夹爪力控（F36：松开全开，按过 0.2 后 0.2..1 → 0.1..1.0 Nm）；Square/Circle 夹爪开/合
    - D-pad 上/下/左/右：`ready` / `zero` / `home` / `ready`（上键暂与右键同，work 已并入 ready）；Cross 急停
    - 改映射只编 `config/mappings/*.yaml`；轴序校准见 `ds4_linux.yaml`
    - 真机：统一入口默认已含 PS4 mapper（`teleop_mapping:=default` 切换 L1 死人开关映射）；笛卡尔 jog 加 `use_servo:=true` 一起起（勿另开 `servo.launch.py`，会双 RSP + sim_executor 冲突；板测待办）

11. **机械臂编排节点（F21，a3_arm_controller）：**
    ```bash
    # 真机：统一入口默认已含编排层（use_arm_controller:=true，含 F50 看门狗），直接调服务即可；
    # 若需单独起（如用非默认 config_file）：a3_bringup 加 use_arm_controller:=false，再：
    # ros2 launch a3_arm_controller arm_controller.launch.py config_file:=...

    # 初始化（设零 + 确认 7 电机到位 + 使能）：
    ros2 service call /a3/arm/init std_srvs/srv/Trigger
    # 使能 / 失能：
    ros2 service call /a3/arm/enable std_srvs/srv/Trigger
    ros2 service call /a3/arm/disable std_srvs/srv/Trigger
    # 运行到预设点（zero/home/ready；work 已并入 ready）：
    ros2 service call /a3/arm/goto_named_pose a3_msgs/srv/GotoNamedPose "{pose_name: ready}"
    # 状态聚合（唯一状态入口）：
    ros2 topic echo /a3/arm_status
    # 示教（拖动）→ 保存 → 回放：
    ros2 service call /a3/arm/start_teach std_srvs/srv/Trigger
    # ...手动拖动机械臂...
    ros2 service call /a3/arm/stop_teach std_srvs/srv/Trigger
    ros2 service call /a3/arm/save_trajectory a3_msgs/srv/SaveTrajectory "{name: demo}"
    ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: demo}"
    # AI 模式（LeRobot 采集/回放）：
    ros2 service call /a3/arm/enter_ai std_srvs/srv/Trigger
    ros2 service call /a3/arm/exit_ai std_srvs/srv/Trigger
    ```
    - 编排层复用 `/a3/motor/{set_zero,enable,reset}`、`/a3/zero_torque/*`、`/joint_states`，自身不做 CAN/插值/规划。
    - 运动命令在 `ZERO_TORQUE`/`SERVO`/`GRAVITY_COMP` 时被拒；`require_gate:=true` 时还需 gate 打开。
    - 前端控制：`a3_mqtt_bridge` 订阅 MQTT `.../cmd`（`{"op":"goto","args":{"pose":"ready"}}` 等），回发 `.../cmd_result`。

12. **单电机分层回归测试套件（F22，can1 / ID7 空载）：**
    ```bash
    # 前置：can1 接 1 个空载电机（CAN_ID=7），24V 供电；EMQX 192.168.3.73 可达
    ./scripts/a3_test/a3_test.sh env        # 环境自检（can1 / EMQX / ROS / paho）
    ./scripts/a3_test/a3_test.sh hw         # 真机底层：scan/设零/使能/小角度运动/零力矩/失能
    ./scripts/a3_test/a3_test.sh telemetry  # MQTT 上行：转动电机，断言 pos_L7 变化
    ./scripts/a3_test/a3_test.sh mqtt_cmd   # MQTT 下行：mock 编排层，10 个 op 全链路
    ./scripts/a3_test/a3_test.sh servo      # 仿真：MoveIt Servo 六方向直线 jog
    ./scripts/a3_test/a3_test.sh gripper    # 夹爪力控闭环（默认 sim 无硬件；hw 见第 14 节）
    ./scripts/a3_test/a3_test.sh incident   # LL-039 事故回归（纯仿真/mock 电机，无需硬件/root，F51）
    ./scripts/a3_test/a3_test.sh web        # 启动 deep-trace 网页，人工确认曲线/3D（操作清单）
    ./scripts/a3_test/a3_test.sh force_web  # web 路径力控阶梯验收 0.3→0.5→0（真机+泡棉+生产 MQTT 桥，F34）
    ./scripts/a3_test/a3_test.sh all        # 顺序跑 env→gripper→hw→telemetry→mqtt_cmd→servo→motor_debug→incident
    ```
    - **跑 `mqtt_cmd` 前先停真机 `a3_mqtt_bridge`**（`pkill -f bridge.launch.py`）：测试桥与真机桥共享 `deep-trace/HOME-DEMO/RK3588/cmd` 话题，两边的 `cmd_result` 会互相覆盖（2026-09-07 实测 15/18 串扰，且测试指令会被真机夹爪执行）；跑完重启真机桥。
    - 真机直连底层 `/a3/motor/*`（`motor_id:=7`），**不**走 `/a3/arm/init`（需 7 电机齐全）；脚本以 `use_power_sequence:=false` 起 can_bridge。
    - 所有真机运动经安全限幅（目标 ≤0.30 rad、时长 ≥2.5 s），结束自动失能；零力矩步骤需人工在旁。
    - 详见 [`scripts/a3_test/README.md`](../../scripts/a3_test/README.md)。

13. **Web 端机械臂控制面板（F23，跨仓 deep-trace）：**
    浏览器经 MQTT 直连 EMQX 下发机械臂指令，无需 Django 经手；编排状态实时回显。
    ```bash
    # 设备侧：统一入口默认已含编排节点 + MQTT 桥接（bridge.yaml 已含 /a3/arm_status 展平）
    ros2 launch a3_bringup a3_bringup.launch.py   # use_mqtt:=true 默认
    ```
    - 前端（外部仓 `/home/cat/deep-trace`，分支 `rk3588`）：RK3588 详情页 → 「机械臂编排节点」(`a3_arm_controller`) 卡片 → 点击展开控制面板。
    - 面板含状态区（`arm_state`/`arm_mode`/`arm_message`，随 `/a3/arm_status` 刷新）、10 个动作按钮（初始化/使能/失能/示教起止/保存/回放/goto/进入退出 AI）、操作消息列表。
    - 所有动作 **二次确认** 后才下发 `.../cmd`（`{"op","args"}`），回执 `.../cmd_result`（`{op,ok,message,ts}`）以 toast + 消息列表反馈；MQTT 未连接时按钮禁用。op↔服务映射见 [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)。
    - 改设备 YAML 后须在 deep-trace 后端 `python manage.py load_device_config` 合入 `edge_config`。
    - 单电机现状下 `init`/`goto` 等会如实回 `ok=false`（message 带 `n/7` 等原因），失败信息显示在面板，正好验证回执链路；真机完整动作需 7 电机齐全。

14. **夹爪力控（F24–F28，L7 自适应抓取）：**
    夹爪（第 7 电机）支持「PI 力外环 + 电机位置内环」：读 `eff_L7` 反馈，力不足继续闭合、超力回退，自适应抓取软硬物体；握力可经配置文件或 web 设定，固件 `0x700B` + 软件 clamp 双保险限力。

    **L7 零点标定（2026-09-06 台架实测）：** 全开位（丝杠硬限位）设零，闭合为正，实测行程 1.7945 rad。软件约定 `gripper_config.yaml` open=0 / close=1.79，URDF L7 限位 `[0, 1.8]`、轴 `0 0 -1`（正转=闭合，与 web 3D 一致），`control_gains.yaml` L7 joint_cmd `[0, 1.8]`。设零：电机通电后 `ros2 service call /a3/motor/set_zero a3_can_bridge/srv/MotorCommand "{motor_id: 7, command: 3}"`。**断电与多圈计数**：短时断电（分钟级 24V 断电重上）实测多圈计数保留（2026-09-06 事故后重启，硬止位读数 0.0056 rad 未漂）；但更早一次长时间断电曾丢计数（读数跳变）。机制不明，**重上电后先读硬止位读数校验零点**：与标定值（全开≈0）偏差大才 set_zero，不要盲信也不盲设。
    ```bash
    # (a) 无硬件闭环回归（任意机器可跑，自起节点 + 假「电机+物体」植物）：
    ./scripts/a3_test/a3_test.sh gripper          # 13 项 PASS：配置校验/开合/软硬物体收敛±10%/超力/看门狗

    # (b) 起夹爪节点（独立节点，不改 C++ CAN 实时路径；力控仅在 Edge 本地）：
    ros2 launch a3_gripper_controller gripper_controller.launch.py require_gate:=false

    # (c) 服务/话题验证：
    ros2 service call /a3/gripper/set_config a3_msgs/srv/GripperSetConfig "{key: max_torque_nm, value: 1.0}"
    ros2 service call /a3/gripper/command a3_msgs/srv/GripperCommand "{mode: force, preset: medium}"
    ros2 service call /a3/gripper/command a3_msgs/srv/GripperCommand "{mode: release}"
    ros2 topic echo /a3/gripper_status           # state/mode/target+actual_torque/target+actual position/contact/error_code
    ros2 topic pub --once /a3/gripper_cmd std_msgs/msg/Float32 "{data: 1.0}"   # POSITION 开合 0..1

    # (d) web 下发（经 a3_mqtt_bridge 白名单 5 op，F31 起）：
    #   {"op":"gripper_grasp","args":{"preset":"medium"}}  或  {"op":"gripper_grasp","args":{"torque":0.6}}
    #   {"op":"gripper_release"}  {"op":"gripper_stop"}  {"op":"gripper_set_max_torque","args":{"value":0.8}}
    #   {"op":"gripper_set_position","args":{"position":0.5}}   # 0..1，0 闭 1 开（位置模式直驱，F31）
    # web 夹爪面板（F37）：「停止」= gripper_stop + motor_reset{motor:7} 顺序下发（失能、夹持物会掉落）；
    #   「使能」= motor_enable{motor:7}；「设置零位」= motor_set_zero{motor:7}（仅限全开硬止位）；
    #   恢复流程：使能 → 释放 → 设置零位；曲线合并为单图双轴（左 Nm / 右 0–1，图例点击隐藏）。
    ```
    - 参数：`a3_gripper_controller/config/gripper_config.yaml`（最大握力 `max_grasp_torque_nm` 出厂硬上限 1.0 Nm（2026-09-07 由 2.0 下调，见 LL-014）、弱/中/强档位、PI（2026-09-07 减半为 kp=0.25/ki=0.3）、接触阈值、超时/看门狗；超硬限 FAULT 阈值 = 1.0 × `overtorque_ratio`(1.5) = 1.5 Nm 瞬态带，固件 0x700B 仍硬钳 1.0）；下发的最大握力落盘 `~/.a3/gripper/gripper_overrides.yaml`，重启保留，越界（超硬上限/±6 Nm）拒绝。
    - 遥测：`grip_target_position` 为最近 position/release 命令目标（未命令前跟随实测）；web 面板曲线已合并为单图双轴（F37：`group_by_unit` 轴模式，力矩对左轴 Nm、位置对右轴 0–1，图例点击隐藏）。桥接层对非有限浮点（NaN/±Inf）统一清洗为 null 并以 `allow_nan=False` 兜底，telemetry JSON 恒合法（见 [LL-011](../../lessons_learned/LL-011-nan-poisons-json-telemetry.md)）。
    - 真机：`A3_GRIPPER_TEST_MODE=hw ./scripts/a3_test/a3_test.sh gripper` 做服务/配置/开合安全检查；力控阶跃需人工在夹爪放置海绵（软）/木块（硬阻挡），观察 `grip_actual_torque` 收敛到目标 ±10% 且 `GRASPED`，握力不超硬上限。2026-09-13 真机软泡棉 0.3 N 实测：~10 s GRASPED、稳态 0.30±0.01 Nm。抓取超时只约束「进入 GRASPED 前」，抓稳后滑脱振荡不会再触发 FAULT（[LL-021](../../lessons_learned/LL-021-grasp-timeout-kills-established-grasp.md)）；命令传 `timeout_s` 建议 ≥ 默认 15 s。
    - 力控与臂运动互锁：gate 关闭或臂处于 `TRAJ_RUNNING`/`SERVO`/`ZERO_TORQUE`/`GRAVITY_COMP` 时拒绝力控；安全条款见 [shared/SAFETY.md](../shared/SAFETY.md)「夹爪力控安全」。

15. **单电机调试页（F32，跨仓 deep-trace）：**
    web 端 `motor_protocol_node` 卡片 → 「调试页 →」进入电机详情页：CAN 扫描选择电机、状态卡片（温度/模式/故障位）、使能/复位/设零、MIT+位置+速度三模式、MIT 单发/定时保持、话题信号下拉 + 实时曲线。
    ```bash
    # (a) 仿真闭环（任意机器可跑；sim 不实现 gate 互锁，见 TOPIC_CONTRACT）：
    ros2 launch a3_bringup edge_web_sim.launch.py use_gripper:=true
    ros2 topic echo /a3/motor/states --field states   # 7 条，fresh=true

    # (b) 无 CAN 单节点（真机台架 use_power_sequence:=false）：
    ros2 launch a3_can_bridge can_bridge.launch.py use_power_sequence:=false require_gate:=false
    ros2 topic echo /a3/motor/states --field states   # 7 条 fresh=false（无反馈发 0 而非 NaN）
    ros2 service call /a3/motor/scan_and_collect a3_can_bridge/srv/MotorScanCollect \
      "{id_min: 1, id_max: 127, bus: 1, timeout_s: 1.5}"
    ros2 service call /a3/motor/mit_command a3_can_bridge/srv/MotorMitCommand \
      "{motor_id: 7, position_rad: 0.2, velocity_rad_s: 0.0, kp: 20.0, kd: 1.0, torque_ff_nm: 0.0, hold_duration_s: 0.0, hold_hz: 0.0}"
    ros2 service call /a3/motor/mit_command a3_can_bridge/srv/MotorMitCommand \
      "{motor_id: 7, position_rad: 0.2, velocity_rad_s: 0.0, kp: 20.0, kd: 1.0, torque_ff_nm: 0.0, hold_duration_s: 2.0, hold_hz: 20.0}"
    ros2 service call /a3/motor/stop a3_can_bridge/srv/MotorStop "{motor_id: 7}"

    # (c) 回归测试（纯 MQTT 驱动 sim 闭环，9 个 motor op 全链路）：
    ./scripts/a3_test/a3_test.sh motor_debug
    ```
    - MIT 保持语义：`hold_duration_s<=0` 单发一帧；`>0` 由 `motor_protocol_node` 内部定时发帧（前端不做流式发送），到时长自动停，`/a3/motor/stop` 手动取消；`gate_open` 由关→开瞬间自动取消。
    - 互锁：真机 `gate_open=true` 时使能/复位/设零/MIT/模式/参数写入被拒（拒绝文案经 `cmd_result` 回传），扫描/读类/停止不受限；仿真有意不实现（sim gate 恒 true）。
    - 前端（外部仓 `/home/cat/deep-trace`）：节点卡片进入 `rk3588_motor` 模块；`motor_scan` 回执 message 为 JSON 电机列表（`{"motors":[{"id","uid"}]}`）；状态与曲线来自 `temp/err/mode/online/mtq/mp_L{n}` 42 个遥测点（改 YAML 后须 `load_device_config`）。
    - 真机台架：单电机 CAN_ID=7 空载，扫描见真实 UID，MIT hold 用小角度 ±0.3 rad、2 s，结束自动停 + `motor_stop` 卸力；整机 gate 测试见 [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)「互锁」节。
    - 安全条款见 [shared/SAFETY.md](../shared/SAFETY.md)「单电机调试（MOTOR_DEBUG）」；契约细节（服务/op 表/42 遥测点）见 [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)。

16. **通用 N 关节臂验证流程（F38，can1 接非 A3 臂、URDF 无关）：**
    在 can1 换接无夹爪/异 URDF 机械臂时，用专用配置跑 init → 挪臂确认 → 示教 → 回放全流程，无需改代码、无 URDF/前端依赖：
    ```bash
    # (a) 探测总线电机（纯标准库，无使能）：
    python3 scripts/mit_noenable_stream.py probe --iface can1 --ids "1 2 3 4 5 6"

    # (b) 执行层：专用 gains（kp=40/kd=2、限位放宽 ±12.57、关 startup smoothing）
    ros2 launch a3_can_bridge can_bridge.launch.py use_power_sequence:=false \
      gains_file:=src/a3_can_bridge/config/control_gains_generic.yaml

    # (c) 编排层：N 关节名参数化（6 关节臂去 L7_joint）
    ros2 launch a3_arm_controller arm_controller.launch.py \
      config_file:=src/a3_arm_controller/config/arm_controller_6j.yaml

    # (d) 初始化：set_zero 广播 → 按 /joint_states 计数确认 → enable 广播
    ros2 service call /a3/arm/init std_srvs/srv/Trigger "{}"   # 期望 "6/6 confirmed"
    ros2 topic echo /joint_states                               # 6 关节 |p| < 0.05

    # (e) 零力矩挪臂：start 后自由拖动，stop 后保持释放位（F38a 防弹回，臂不弹回旧目标）
    ros2 service call /a3/zero_torque/start std_srvs/srv/Trigger "{}"
    ros2 service call /a3/zero_torque/stop std_srvs/srv/Trigger "{}"

    # (f) 示教 5s → 保存 → 回放（F38b：回放先按 playback_ramp_duration_s 插值到首记录点，无跳变）
    ros2 service call /a3/arm/start_teach std_srvs/srv/Trigger "{}"
    ros2 service call /a3/arm/stop_teach std_srvs/srv/Trigger "{}"
    ros2 service call /a3/arm/save_trajectory a3_msgs/srv/SaveTrajectory "{name: teach6}"
    ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: teach6}"
    ```
    - init 前后顺序：先 set_zero 后 enable；6 关节配置下 `arm_controller` 只数 `joint_names` 内的关节，缺席/多余电机不影响确认计数。
    - 无运动来源需「播种」：init 后由零力矩/示教/回放路径自动播种 refresh 目标（`last_commanded_mit_rad_`），无命令前 refresh 不发帧。
    - 安全：全部运动来自用户拖动（kp=0）与回放自录轨迹 + 插值段；急停 = Ctrl+C 后 `ros2 service call /a3/motor/stop` + `/a3/arm/disable`。
    - 命名点位与平滑移动（F39）：`/a3/arm/save_named_pose`（positions 留空 = 当前位姿，写 `~/.a3/poses.yaml`，同名覆盖包内点位、重启保留）；`/a3/arm/move_to`（任意目标 + duration_s，多点插值，不 clamp）；失能 = `/a3/arm/disable`。典型序列：
      ```bash
      ros2 service call /a3/arm/save_named_pose a3_msgs/srv/SaveNamedPose "{name: ready}"          # 当前位姿
      ros2 service call /a3/arm/save_named_pose a3_msgs/srv/SaveNamedPose "{name: home, positions: [0,0,0,0,0,0]}"
      ros2 service call /a3/arm/move_to a3_msgs/srv/MoveToJointPositions "{positions: [0,0,0,0,0,0], duration_s: 5.0}"
      ros2 service call /a3/arm/goto_named_pose a3_msgs/srv/GotoNamedPose "{pose_name: ready}"
      ros2 service call /a3/arm/disable std_srvs/srv/Trigger "{}"
      ```
    - 失能后 refresh 流仍按最后锚定目标发帧（电机 mode 0 忽略力矩）；重新使能前若臂被挪过，先 `zero_torque/start→stop` 重锚定目标再 `enable`，避免拉回旧位姿。

### 断电多圈窗口 zero_sta 标定（F47，真机 7 关节臂）

断电后手动转动关节再上电，默认 `zero_sta=0`（0~2π 重建）会把负向转动 +2π 环绕（实测 -22° 读 +338°，见 LL-019）。出厂标定一次：工装摆 URDF 零位 → set_zero → zero_sta=1 → save；之后 ±180° 内断电转动读数连续。上电后必须先 probe 校验 7 关节读数全部落在 URDF 限位内，**读数超限严禁使能**。

```bash
# 1) 读当前 zero_sta（0x7029=28713）：7 电机应答 values_u8 全 0 = 出厂默认
ros2 service call /a3/motor/get_param a3_can_bridge/srv/GetMotorParam \
  "{motor_id: 0, param_id: 28713, timeout_s: 0.5}"

# 2) 广播置 1 → 保存 flash → 读回校验（缺帧时对个别 motor_id 单发补写）
ros2 service call /a3/motor/set_param_u8 a3_can_bridge/srv/SetMotorParamU8 \
  "{motor_id: 0, param_id: 28713, value: 1}"
ros2 service call /a3/motor/save_param a3_can_bridge/srv/MotorCommand "{motor_id: 0, command: 5}"
ros2 service call /a3/motor/get_param a3_can_bridge/srv/GetMotorParam \
  "{motor_id: 0, param_id: 28713, timeout_s: 0.5}"

# 3) 断电 → 各关节 ± 转动（<180°）→ 上电 → probe 校验
python3 scripts/mit_noenable_stream.py probe --iface can1 --ids 1..127
```

- **窗口仅上电重建多圈时生效**：运行中改 zero_sta 不会重推导当前读数。
- **保存帧数据域**：类型 22 数据域固定 `01 02 03 04 05 06 07 08`，全零不触发保存（实测断电参数回退）。协议原文：`~/EDULITE_A3/el_a3_sdk/docs/电机通信协议汇总.md`。
- **固件版本分裂**（2026-09-13 实测）：RS00 `0.0.3.4`（L1/L3）**读回恒 0 但写+保存实际生效**（断电行为验证：L1/L3 断电负转读数连续）——判断以断电行为为准，别信读回；RS00 `0.0.3.19`（L2）与 EL05 `10.5.0.1`（L4–L7）读写正常。
- **不要用「每次上电 set_zero 代替」**：会把零位定义成当次上电姿态，poses/FK 每次漂移（只适合无绝对位姿的机器狗）。
- L2（0~3.67）/L3（-4.01~0）行程超 ±π：这两个关节断电转动超 ±180° 仍会环绕。

### 开机零位校验（F48，真机 7 关节臂）

上电后、使能前校验读数（两层）：编排层 `/a3/arm/enable` 内置限位门禁（读数越限/无 /joint_states 一律拒绝，环绕时 kp×误差会瞬间猛拉）；独立脚本做完整检查。

```bash
# 开机后（硬件栈已起）：
python3 scripts/a3_check_zero_frame.py        # exit 0 = 限位内，可 enable
ros2 service call /a3/arm/enable std_srvs/srv/Trigger "{}"
```

- **硬检查（决定 exit code）**：7 关节读数全部落在 URDF 限位内（默认裕量 0.001 rad 吸收编码器量化噪声 ±0.0002）**且消息新鲜**（stamp 距今 ≤1 s）；越限 = 疑似断电多圈环绕（LL-019）；陈旧 = 桥异常/refresh 停发（LL-020，读旧值校验形同虚设）。
- **软检查（仅 WARN，默认容差 ±20°）**：L2/L3/L5/L6/L7 ≈ 0、L4 ≈ 0（URDF 零位）或 ≈ 0.34（折叠下垂）、L1 自由（±178° 机械限位）——模板外说明臂被留在其它位姿；判断是否环绕的唯一标准是硬检查，软检查不阻断。
- **越限恢复**：把臂摆回 URDF 零位（工装/泡沫垫）→ `/a3/arm/init`（set_zero 重建零位帧；init 是恢复路径，越限只 WARN 不阻断，见 REQUIREMENTS F48）。
- **enable 门禁参数**：`enable_position_check: true`（默认）、`position_check_margin_rad: 0.001`、`js_max_stale_s: 1.0`（arm_controller.yaml / arm_controller_6j.yaml）。
- **桥保活播种（LL-020）**：桥重启后即使从未下发轨迹，refresh 流也会以零增益保活帧（kp=kd=tau=0）保持总线帧流与反馈上送——开机后 js 应当持续更新（可用 `candump can1` 看到 ~350 帧/s）。

### 安全保护与状态监控验证（F40–F50，真机 7 关节臂）

F40–F46 于 2026-09-13 在 can1 真机 7 关节臂（含夹爪）全部验收通过（需求与验收标准见 [REQUIREMENTS.md](REQUIREMENTS.md)，安全语义见 [shared/SAFETY.md](../shared/SAFETY.md)）。验证命令与结论：

```bash
# F41 时长兜底：0.5s 请求 → 响应回显 "(3.0s, 150 pts)"
ros2 service call /a3/arm/move_to a3_msgs/srv/MoveToJointPositions "{positions: [0,0,0,0,0,0,0], duration_s: 0.5}"

# F46 帧率：3s/150 点 move_to 期间读 tx_stats（注意 5s 窗口旋转，对照同时段桥日志）
ros2 topic echo /a3/motor/tx_stats --once

# F40 失能保护：READY 位 disable → 自动 safe park 回 home → DISABLED
ros2 service call /a3/arm/disable std_srvs/srv/Trigger "{}"

# F43 持久化证据（MQTT mtqmax_L1..L7 同步上行）
cat ~/.a3/stats/torque_stats.yaml
```

- **F40 失能保护**：容差外 disable → `safe park -> disabled (0.9s)`，最终位姿全 ≈ home、7/7 失能；park 超时 → FAULT 不 reset。参数：`disable_home_tol_rad: 0.15`、`disable_home_duration_s: 3.0`、`disable_home_confirm_s: 0.5`（arm_controller.yaml）。
- **F41 move_to**：最短 3 s + ≥50 Hz 插值（`move_to_min_duration_s`/`move_to_points_hz`）；goto/playback ramp 同口径。
- **F42 力矩方向钳位**：手扶顶关节（L4 力臂长，~20 N 手力可顶 3 Nm）→ trip 冻结在反馈位、反向放行；**新轨迹自动清 latch**（LL-026）。阈值 `[5,5,5,3,3,3,3]` Nm（RS00 5 / EL05 3）。
- **F43 最大力矩**：当日两次 trip 已写入 torque_stats.yaml 并上行 mtqmax（与桥日志时间戳吻合）。
- **F44 温度**：warn=90/protect=95/迟滞 5（官方电机 130°C 兜底）；超限自动 park → COOLING，降温至保护阈−迟滞才可 enable；无反馈（fresh=false）温度判读不生效。
- **F45 状态机**：11 态（IDLE/INIT/READY/TRAJ/SERVO/TEACH/AI/SAFE_PARK/DISABLED/COOLING/FAULT）；disable/温度保护路径转移实测，arm_state 遥测一致。
- **F46 帧率**：轨迹期 195 Hz/关节（4098 帧/3 s ≈99% 交付、限速丢弃 0.7%）、静止对照 47.2 Hz/关节、`tx_rate_ok=true`；**tx_stats 5 s 窗口旋转会切分轨迹尾巴，读帧率须对照同时段桥日志**（LL-026）。
- **F50 故障监视看门狗**（`a3_arm_monitor`，随 arm_controller.launch.py 默认启动，`enable_monitor:=false` 可关）：跨源比对 js/轨迹/电机状态/编排状态，故障走 stop → 升级 reset 阶梯（阈值与抑制规则见 [TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)「故障监视看门狗」）。2026-09-13 仿真验收（ROS_DOMAIN_ID=55 sim 闭环 + 故障注入）：健康 move_to 零触发（max err 0.0004 rad）；斜坡轨迹注入 → FOLLOW_STUCK → stop → +3 s 升级 reset；SIGSTOP sim_motor → STALE_JS → reset；ZERO_TORQUE 模式注入跳变零触发（抑制生效）。**5 次触发全为注入诱导，零误报**；真机观察随 F49 重力采集同场进行（`ros2 topic echo /a3/monitor/status`）。

### 使能安全与事故回归（F51，仿真 + 真机已验证）

来源：2026-09-14 真机事故（示教退出后看门狗假触发 → stop 的 NaN 被 refresh 播种覆盖 → 失能期陈旧目标满增益使能 → **甩断 L6**）。完整复盘见 [LL-039](../lessons_learned/LL-039-teach-exit-reanchor-false-trip-enable-snap.md)，条款见 [shared/SAFETY.md](../shared/SAFETY.md)「使能安全」。使能语义现在是：

- **使能 = 保当前位置**：`/a3/motor/enable`、`/a3/arm/enable` 逐电机校验反馈新鲜度（缺失/陈旧 → **整体拒绝**），并把 MIT 目标无条件重锚到反馈位；响应里回传「F51 重锚 N 电机…最大丢弃目标距离 X rad」。
- **使能软起步**：kp/kd 在 `enable_ramp_duration_s`（0.8 s）内线性升到额定，使能瞬间残余误差不会被满增益放大成甩动（τ_ff 重力前馈不受影响）。
- **stop/reset 有保持抑制 latch**：卸力后 refresh 只发零增益保活帧（p=反馈位、kp=kd=τ=0），**不再**把旧目标锚回去续推；只有新轨迹 / MIT 直驱 / 零力矩 / 使能 / park 才解除。
- **失能期陈旧目标告警**：`!enabled && |cmd−fb| > 0.15 rad` 时限频 WARN（事故时该差值 1.956 rad 静默保留了 2 分钟）。
- **看门狗**：意图边界（示教/零力矩退出、整臂使能沿）重基准保持参照 + 2 s 宽限；`status` 分 OK/PENDING/TRIGGERED（只对 TRIGGERED 动作）；编排层消费 `UNEXPECTED_DISABLE`，另有不依赖看门狗的本地兜底。

```bash
# 仿真回归（有牙：旧代码必失败）——两个脚本各自起被测栈，独立 ROS_DOMAIN_ID=57 与真机栈 DDS 隔离
./scripts/a3_test/a3_test.sh incident
#   八a 执行层：mock 电机 ↔ 真 motor_protocol_node，断言 stop 后无 kp>0 帧、使能命令位≈反馈位、kp 斜坡
#   八b 看门狗：sim 栈 + 真 arm_controller/arm_monitor，断言示教退出 4 s 零触发（真阳性另验：带外失能必须被抓到）
```

- **真机验收（2026-09-14 夜完成，事故后 5J 档 L1–L5）**：`scripts/a3_test/f51_real_arm_acceptance.py`，P0–P8 全绿——使能重锚（`最大丢弃目标距离 0.000000 rad`、命令位 vs 反馈位 0.0004 rad、沉降 0.0004 rad）、kp 0.35→80.0/0.74 s 软起步、stop 后 570 帧全零增益保活、带外 reset 双通道（看门狗 `TRIGGERED/UNEXPECTED_DISABLE` 0.61 s + 编排层 `DISABLED` 0.71 s）、保持期无 HOLD_DRIFT 误报。运行方式见下方「缺电机降级档（F52）5J 档起栈」。
- **测试必须跑在独立 `ROS_DOMAIN_ID`**：mock 会伪造全部反馈，而同名 `/a3/motor/*` 服务在真机栈上存在——同域运行等于把测试的 enable 打到真电机上（LL-039 判据教训 3）。

### 缺电机降级档（F52，5J 档起栈 / 真机已验证）

can1 上只剩部分电机时（事故后只剩 L1–L5），用**档位**整套替换——`motor_map_file` 与 `gains_file` **必须配对**（逐关节数组长度 = `joint_names` 长度），否则执行层报「F52 档位配置非法」并退回 7J 默认档（不静默降级）：

```bash
# 真机 5J 档（L6/L7 缺失）：停掉 7J 栈后；统一入口显式关夹爪（缺 L7）与 MoveIt（SRDF 7 关节，
# FJT 若执行会把 7 关节轨迹打到 5 关节执行层），编排层换 5J 配置
source scripts/a3_shell_env.sh
CFG=$HOME/a3_arm_ws/install/a3_can_bridge/share/a3_can_bridge/config
ros2 launch a3_bringup a3_bringup.launch.py use_power_sequence:=false use_teleop:=false \
    use_gripper:=false use_moveit:=false \
    gains_file:=$CFG/control_gains_5j.yaml motor_map_file:=$CFG/motor_map_5j.yaml \
    arm_controller_config:=$HOME/a3_arm_ws/install/a3_arm_controller/share/a3_arm_controller/config/arm_controller_5j.yaml
# 起栈自检：执行层日志 "F52 档位：5 关节 [...]"、/joint_states 恰好 5 个名字、MotorStates 5 条、
#           控制器日志 "a3_arm_controller ready: joints=5"、看门狗 /a3/monitor/status = OK
```

- **真机验收脚本**（只驱动档位内电机；唯一运动项是 `--move` 时的小幅 `move_to`，增量经 `safety_limits.clamp_delta` ≤0.30 rad、≥3 s，结束必定失能）：
  ```bash
  A3_REAL_ARM_ACCEPT=1 python3 scripts/a3_test/f51_real_arm_acceptance.py \
      --bridge-log /tmp/a3_hw_5j.log --move        # 不带 A3_REAL_ARM_ACCEPT=1 会拒绝运行
  ```
- **若 P5（`/a3/arm/enable`）被 F48 拒**（`position check failed: L4_joint=… limit=…`）：说明某个关节静置在 URDF 限位外，这是设计行为，**不要**放宽限位或关 `enable_position_check`——按 [LL-041](../lessons_learned/LL-041-rest-pose-outside-urdf-limit-f48-refuses-enable.md) 处置：失能态手动抬回限位内，或加 `--nudge-delta 0.15`（执行层先使能 + F51 保护下 2.5 s 小幅插值把 L4 挪回限位内，随后在使能态调 `/a3/arm/enable`）。
- **起栈方式**：真机栈要长时间无人监护时**别挂在 agent 会话的后台任务上**——2026-09-15 00:21 系统内存告紧，harness 把 5J 整栈 6 个进程一次性回收（当时臂失能，无损失）。用 `setsid nohup … &` 脱离会话，或做成 systemd unit（同 `can-up.service`）。**臂使能中被杀 = 看门狗与轨迹层同时消失**（执行层进程死亡后电机是保持最后一条 MIT 命令还是超时失能，尚未实测）。
- **7J 档默认行为不变**：不传这两个参数即 `control_gains.yaml` + `motor_map.yaml`。
- 注意 `/a3/motor/enable`（执行层）**不查 URDF 限位**，只有编排层 `/a3/arm/enable` 查——「一个拒一个过」不是 bug。

### 重力标定（F49，真机 7 关节臂）

替换 `a3_description/config/inertia_params.yaml`（Pinocchio 重力前馈的参数源）。2026-09-14 真机 full 模式 68 点完成，RMSE 0.1592 / R² 0.9835（旧的官方 6J 参数在同批数据上 0.2036，URDF 默认 0.705）。**误差地板与验收偏差见 [LL-038](../lessons_learned/LL-038-static-gravity-calibration-hardware-floor.md)**。

```bash
# 0) 前置：硬件栈在跑、臂 enable 到 READY、周围清空、手边可断电；量夹爪全开
#    统一入口默认已含编排层（READY 门禁 + 温度守护数据源）；重力采集不需要 MoveIt/Servo，
#    可加 use_moveit:=false use_servo:=false 减负载
ros2 launch a3_bringup a3_bringup.launch.py use_power_sequence:=false use_teleop:=false

# 1) 先看要动的点位与时长（不动臂）
python3 scripts/gravity_calibration.py --dry-run --no-rest-anchor \
  --anchors 0.07,1.0839,-0.5649,-0.4617,0.0831,-0.0497,-0.0002

# 2) 采集 + 拟合（断点续采：重跑同命令；清数据：--restart；只拟合：--optimize-only）
python3 scripts/gravity_calibration.py --no-rest-anchor \
  --anchors 0.07,1.0839,-0.5649,-0.4617,0.0831,-0.0497,-0.0002

# 3) 生效：重启重力节点（install 下 yaml 是 symlink，不必重编）
ros2 run a3_bringup gravity_torque_node --ros-args -r __node:=a3_gravity_torque -p enabled:=True
#   日志出现 "Applied calibrated inertia to 5 links" 即为加载成功
```

关键参数与坑：

- **`--effort-domain`**：真机 `/joint_states.effort` 是**电机域**（LL-037），默认 `motor`（拟合前 ×`joint_signs`）；仿真数据文件用 `urdf`。域随数据文件 meta 留档，混用会直接报错退出。
- **`--temp-pause/--temp-resume`**（默认 85/75 °C）：超限回折叠位降温再续采（3D 打印外壳散热差，85 留 5 °C 余量给 F44 warn 90；实测被动降温 ~2 °C/min，单次约 5 min）。
- **`--no-rest-anchor` + `--anchors`**：折叠 home 位搭在支撑上 τ 恒≈0，不是重力样本（LL-035）——用 ready 等自由位姿做外推锚定。
- **安全**：所有移动 ≤0.25 rad/步链式分段（3 s/段），每点前用当前模型预测 τ（>3.5 Nm 跳过），F42/F44 仍是最后防线；异常时 Ctrl+C，数据逐点留盘。
- **yaml 旧版留档**：`inertia_params.official-6J-20260318.bak.yaml`（官方 6J 标定值，回退用）。
- **验收位姿用 ready**（自由悬空位）：home 是折叠支撑位，实测 τ≈0 判不了模型（LL-035）。

```bash
# 4) ready 位复核模型 vs 实测（臂会移动到 ready；实测 τ 需 ×joint_signs）
ros2 service call /a3/arm/goto_named_pose a3_msgs/srv/GotoNamedPose "{pose_name: ready}"
#   2026-09-14 结果：max|Δτ| 0.302 Nm（L3）、RMSE 0.145；旧 6J 参数 0.403 / 0.212
```

### URDF 方向校验（RViz 双模型，臂不失能）

用两个模型对照 URDF 关节方向与真机反馈：**目标模型**（半透明青蓝色 ghost，默认静止在 home）与**实际反馈模型**（原色实体，由电机反馈 /joint_states 驱动）。全程臂失能（电机 mode=0，轨迹帧只收不执行，LL-018）；手转各关节，对比实际模型运动方向与真机是否一致。

```bash
# 1) 硬件栈（can1 已 UP）
ros2 launch a3_can_bridge can_bridge.launch.py
# 2) RViz 双模型（HDMI :0；use_sw_render 默认开，LL-027 必须软件渲染）
DISPLAY=:0 ros2 launch a3_bringup urdf_dir_check.launch.py
# 无窗口环境只跑数据流：use_rviz:=false
```

- 显示内容：`ArmActual_实际反馈`（TF Prefix 空，**原色橙/深棕**）+ `ArmTarget_目标`（TF Prefix `target`，**青蓝/深蓝色系**——换色 URDF 走独立 `/target_robot_description` 话题）+ TF 坐标轴（Marker Scale 0.1）；RViz 打开 4 s 后自动最大化（wmctrl，无则跳过）。
- 目标默认**静止在 home**（固定参照，手转关节看实际模型偏离即可判方向）；需摆动对比时给 `urdf_dir_check_pub` 传 `osc_amplitudes_rad`（如 `[0.15,0,0,0.15,0.2,0.2,0]`，L2/L3/L7 限位不对称保持 0）；发布器护栏：任一电机 mode_status=2 即暂停轨迹下发（目标 ghost 照发）。
- 校验对象：URDF 关节轴方向/旋转正负与电机实际方向一致（手转关节看实际模型是否同向转动、幅度是否吻合）。
- 测试后注意：读数偏离 home 属正常（手转过）；重新使能会先回 last_commanded，且 F48 门禁要求读数在 URDF 限位内——测试期间勿断电。

## 相关文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [REQUIREMENTS.md](REQUIREMENTS.md)
- [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
- [文档索引](../README.md)
