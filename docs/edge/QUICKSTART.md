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
4. PS4 电源（F60）：L3 短按 = 一键开门禁 + 使能；R3 短按 = safe-park 后失能；Cross 长按 1 s = 硬急停（断电关闸，恢复需重新 L3）。完整键位/灯效见第 10 节与 [PS4_OPERATOR_GUIDE.md](PS4_OPERATOR_GUIDE.md)。
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

10. **PS4 映射遥操作（F60/F64 键位 / F61 灯带震动 / F62 合成验证）：**

    **(a) 全功能仿真闭环 + 双模型 RViz（无手柄、无 CAN，推荐逐键验收入口）：**
    ```bash
    source scripts/a3_shell_env.sh        # 含 PYTHONNOUSERSITE=1；勿 source install/setup.bash
    export ROS_DOMAIN_ID=45
    export DISPLAY=:0
    ros2 launch a3_bringup edge_teleop_full_sim.launch.py
    # 另开终端（同域）：合成 /joy 12 场景 46 项自动验收，逐项打印 PASS/FAIL + 关节证据：
    python3 scripts/a3_test/ps4_sim_test.py
    ```
    - RViz 双模型：实体色 = 实际反馈（`/joint_states`），半透明 = 目标 ghost（`target/` TF）。
    - 本机 RViz 两个必备前缀已由 launch 自动加：`LIBGL_ALWAYS_SOFTWARE=1`（LL-027）、`LD_PRELOAD=~/.a3/hide_randr/libhide_randr.so`（LL-065）。
    - S0 基线检查只能在干净栈上通过；重复跑须重启栈（速度档/位姿有记忆），首次 46/46 为准。
    - 合成全绿后，手柄实操把启动命令换成 `ros2 launch a3_bringup edge_teleop_full_sim.launch.py use_joy_node:=true`（域不变）。

    **(b) 真机 / 校准：**
    ```bash
    ls /dev/input/js*                    # 确认手柄节点（USB 无节点见 LL-032）
    ros2 launch a3_teleop_ps4 ps4_teleop.launch.py dump:=true   # 轴索引校准
    # 统一入口默认已含 mapper + ds4_feedback_node，默认 mapping:=default：
    ros2 launch a3_bringup a3_bringup.launch.py use_servo:=true # jog 需 servo 一起起
    ```
    - **default（真机生产映射，F60+F64）**：L3 一键开门禁+使能、R3 safe-park 失能、Cross 长按 1 s 硬急停；Triangle=ready、Circle=home；示教三步 **Share=开始 / Options=结束(自动保存) / Square=回放(latest)**；PS=init、Options 长按 3 s=set_zero；**L1=平移死人开关、R1=旋转死人开关（F64）、R2 夹爪力控不需要死人开关**；D-pad 上下调平移速度、左右调旋转速度（独立、步长 0.15、范围 0.10–1.0、按住不连发）；左摇杆平移 Y/Z，右摇杆 right_y 平移 X、right_x 偏航。touchpad/L2 预留不绑。
    - **灯带五色（F61）**：红闪=失电/硬急停、红双闪=FAULT、橙=已上电未使能、绿=READY/SERVO、蓝呼吸=TEACH、紫=TRAJ（goto/回放/safe-park）、白闪一次=init 完成。震动：使能/失能 120 ms 弱震，硬急停 600 ms 强震，FAULT 双震。无手柄时逻辑帧看 `/a3/ds4/feedback`（JSON）。
    - **操作员手册：[PS4_OPERATOR_GUIDE.md](PS4_OPERATOR_GUIDE.md)；完整参考表：`src/a3_teleop_ps4/README.md`。**
    - 改键位只编 `config/mappings/default.yaml`（零代码）；轴索引校准见 `config/ds4_linux.yaml`。
    - 勿另开 `servo.launch.py`（双 RSP + sim_executor 冲突）。

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
    # 示教（拖动）→ 回放（F54：stop_teach 自动保存；空名 ↑ 空名 = latest）：
    ros2 service call /a3/arm/start_teach std_srvs/srv/Trigger
    # ...手动拖动机械臂...
    ros2 service call /a3/arm/stop_teach std_srvs/srv/Trigger
    #   stop_teach 已自动保存 latest.yaml + teach_TIMESTAMP.yaml 备份
    #   （误触发：样本 < teach_auto_save_min_samples 默认 10 时自动跳过、不覆盖）
    ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: ''}"
    #   save_trajectory 留空名 ≡ 另存/刷新 latest；playback 空名 ≡ 回放 latest，
    #   命名 {name} 仍按名保存/回放；无 latest 时 playback 给出 start_teach 引导消息
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

    # (f) 示教 5s → 回放（F54 自动保存 latest.yaml；F38b：回放先按 playback_ramp_duration_s 插值到首记录点，无跳变）
    ros2 service call /a3/arm/start_teach std_srvs/srv/Trigger "{}"
    ros2 service call /a3/arm/stop_teach std_srvs/srv/Trigger "{}"
    #   stop_teach 即自动保存：latest.yaml + teach_TIMESTAMP.yaml（样本<阈值自动跳过）
    ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: ''}"
    #   命名保存/回放仍可用（空名 ≡ latest 槽位）：
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

### RViz 双模型（实际 vs 目标 ghost，真机主 launch 内置）

真机主 launch（`a3_bringup.launch.py`）的 RViz 默认就是双模型：`ArmActual_实际反馈`（`/joint_states` 电机反馈驱动，**原色橙/深棕**）+ `ArmTarget_目标`（`/target_robot_description` 换色 URDF，TF Prefix `target`，**青蓝半透明 ghost**，由 `/a3/display_target_joint_states` 注入驱动）。**核心场景：臂断电后实际模型停住，仍可手动注入目标角离线调试「应该到哪」。** 目标 ghost 不依赖电机/电源，纯话题通道。

主 launch **默认自带 ghost 三件套**（target rsp + 恒等静态 TF + home 静止注入节点 `urdf_dir_check_pub`，`publish_traj:=false` 只发纯话题、不碰执行层轨迹）——没有注入源时 target rsp 整树不发 TF、ghost 全白（LL-055/LL-056），所以注入不能缺。`edge_web_sim.launch.py` 同样默认带三件套（其默认 RViz 是单模型 `el_a3_view.rviz`，ghost 不可见；换/另开 `el_a3_dual_view.rviz` 才显示）。不需要 ghost 时两 launch 均可 `use_target_ghost:=false` 整体关掉。

```bash
# 1) 真机栈（断电调试只起下面任意一段拿到 rsp_target 即可，无需本段）
sudo systemctl start can-up.service
source scripts/a3_shell_env.sh
ros2 launch a3_bringup a3_bringup.launch.py use_rviz:=true   # HDMI :0；无屏 use_rviz:=false
#    ↑ 启动后 ghost 自动停在 home；无需任何手动注入即正常着色
# 2) 手动注入目标角（7J 档整臂）：注入节点检测到内容不同的外来消息会自动让位
#    （backoff 3 s，--rate 持续发就持续让位）；停发后 3 s 自动恢复 home
ros2 topic pub --rate 10 /a3/display_target_joint_states sensor_msgs/msg/JointState \
  "{name: ['L1_joint','L2_joint','L3_joint','L4_joint','L5_joint','L6_joint','L7_joint'],
    position: [0.0, 0.0, 0.0, -0.6, 0.3, 0.0, 0.0]}"
```

- **关节名必须放齐 7 个**（5J 档 `gains_file:=control_gains_5j.yaml motor_map_file:=motor_map_5j.yaml` 起栈也一样）：物理上 L6/L7 缺失，但 `target` ghost 用的 URDF 恒为 7 关节，`robot_state_publisher` **只对 message 里出现的关节名发布 TF**——名字缺失的关节不会按 URDF 默认位渲染，而是整个分支没有变换，RViz 显示 no transform：`target/l5_l6_urdf_asm`（`L6_joint` 子）、`target/end_effector`（`l5_l6_urdf_asm` 下固定关节的子）、`target/gripper_link`（`L7_joint` 子）消失。5J 档请把 L6/L7 用 0.0 占位（与上面命令同形式，7 名 7 值），ghost 手腕/夹爪停 URDF 零位。
- 目标 ghost 由独立 `target_robot_state_publisher`（`frame_prefix="target/"`，LL-028）+ 恒等静态 TF `base_link→target/base_link` 接入 TF 树。
- 主 launch / sim launch 自带的注入是 **ghost-only 模式**（`urdf_dir_check_pub publish_traj:=false`）：不创建轨迹发布器与电机状态订阅，纯话题不碰执行层，断电也安全。方向校验用的「轨迹 + 摆动」模式只在上一节 `urdf_dir_check.launch.py`（其护栏只认 mode_status=2）。
- 手动注入让位机制：注入节点订阅自己的话题，凡内容与最近一次自发不同（`ros2 topic pub` 等外部来源）即停发让位 3 s 并滚动续期；同名话题多发布者是 last-writer-wins，rsp 只认最后一条。

### goto/回放工业轨迹验收（F67/F68，仿真全闭环）

goto（Triangle→ready、Circle/R3→home）走 MoveIt move_group + TOTG；示教回放走 `/a3/arm/retime_trajectory`（Ruckig 默认，TOTG 备选），只重定时不改几何。move_group/retime 不可用时分别自动回退本地线性插值 / 旧 smooth+time_warp 链路（参数 `goto_use_moveit`、`playback_retime`，默认 true）。

```bash
# 全自动验收（自建域 55 闭环栈，约 3~5 分钟；结束自动收栈）
source scripts/a3_shell_env.sh && export PYTHONNOUSERSITE=1
python3 scripts/a3_test/f67_f68_sim_acceptance.py
#   通过标准：末尾「总体: ALL PASS」（14 项：goto/home move_group、首末零速、限位、
#   线性兜底恢复、playback 时长带 ±25% 与 101/101 几何匹配、旧链路回归、安全 park）
```

- Ruckig 回放若出现时长被放大数倍（如 4s→29.6s），是 Humble 单步 update 的 jerk 绑定所致，已由节点内 jerk 需求代数松弛（`seed_jerk_margin=1.5`）修复，排查见 LL-071 坑 5。
- 录制 yaml 仍只存 positions + time_from_start_sec，**不需要保存速度**；重定时全部重算 v/a/jerk。
- 真机上电后复跑同一脚本（需 can-up + 主 launch）做真机验收。

### ros2_control 标准栈仿真验收（F70，mock 硬件）

工业标准执行底座：`controller_manager`（200 Hz）+ `joint_state_broadcaster` + 官方 `joint_trajectory_controller`（arm L1–L6 / gripper L7，样条插值 + 容差监控 + FJT action 原生），MoveIt move_group 经 `moveit_simple_controller_manager` 直连 JTC，无自研 FJT/插值节点；硬件用 `mock_components/GenericSystem`（`calculate_dynamics:=true`）。与旧栈并存，不影响现有 launch。

```bash
source scripts/a3_shell_env.sh && export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=58
ros2 launch a3_bringup edge_ros2_control_sim.launch.py use_rviz:=false &
sleep 12
python3 scripts/a3_test/f70_ros2_control_sim_acceptance.py
#   通过标准：末尾「总体: ALL PASS」（11 项：JTC home→ready→home、夹爪开合的
#   多点五次 S 曲线平滑到位 + move_group plan+execute 端到端 + 栈内无自研 FJT 节点）
```

- 控制器配置 `src/a3_description/config/el_a3_controllers.yaml`；spawn 顺序必须 JTC 先、JSB 后，否则速度字段恒 0；直连 JTC 测试要发完整多点轨迹，单点只做匀速线性插值——三个坑详见 [LL-072](../lessons_learned/LL-072-ros2-control-mock-jsb-order-jtc-single-point.md)。

### 看门狗标准诊断验收（F71，/diagnostics）

`a3_arm_monitor` 增量发布标准 `/diagnostics`（官方 `diagnostic_updater`），两组件可直接接 `diagnostic_aggregator` / `rqt_robot_monitor`：`a3_arm_monitor: Monitor`（fault=ERROR、pending=WARN、OK）、`a3_arm_monitor: Tracking`（各关节跟随误差 + max，超阈值 WARN，FOLLOW_STUCK/HOLD_DRIFT 触发时 ERROR）。看门狗判定/处置逻辑不变，`MonitorStatus` 话题保留；参数 `publish_diagnostics`（默认 true）、`diagnostics_period_s`（默认 1.0）。

```bash
source scripts/a3_shell_env.sh && export PYTHONNOUSERSITE=1
# 全自动（脚本自行在独立 ROS_DOMAIN_ID=59 起 monitor 并喂数：健康→STALE_JS→恢复）
python3 scripts/a3_test/f71_monitor_diagnostics_acceptance.py
#   通过标准：末尾「总体: ALL PASS」（9 项）
```

- 组件名自动带节点名前缀（add() 只给裸名）、`level` 是 byte 字段 rclpy 收为 bytes、harness 自身进程也要设 ROS_DOMAIN_ID——详见 [LL-073](../lessons_learned/LL-073-diagnostic-updater-name-prefix-byte-level.md)。

### 真机插件 vcan 闭环验收（F72，SystemInterface + SocketCAN/MIT）

F70 的真机化：`a3_hardware_interface/A3MITHardwareInterface` 插件让 controller_manager 直接打开 SocketCAN、收发 MIT 协议，真机路径上 `motor_protocol_node` + 200 Hz 插值 + CAN 话题中转整体被旁路；JTC/JSB/move_group 配置与 F70 完全相同。无真机时在 vcan0 上用 7 电机反馈模拟器闭环验收。

```bash
source scripts/a3_shell_env.sh && export PYTHONNOUSERSITE=1

# 1) 一次性建 vcan0（已存在可跳过）
echo temppwd | sudo -S modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null; sudo ip link set vcan0 up

# 2) 起电机模拟器（独立终端/后台：收到 MIT 指令→一阶跟随→回 type-2 反馈）
python3 scripts/a3_test/vcan_motor_sim.py --interface vcan0 &

# 3) 起标准栈（xacro use_real_hardware:=true can_interface:=vcan0）
export ROS_DOMAIN_ID=59
ros2 launch a3_bringup edge_ros2_control_vcan.launch.py use_rviz:=false &
sleep 12

# 4) 验收：运动学指标 + 独立 CAN socket 抓包双侧核对
python3 scripts/a3_test/f72_ros2_control_vcan_acceptance.py
#   通过标准：末尾「总体: ALL PASS」（JTC home→ready→home、夹爪开合、move_group
#   plan+execute 到位 ≤0.01；CAN：7 电机指令/反馈角=direction×joint+offset、
#   kp=80/kd=2、速度与前馈为 0；栈内无 fjt/can_bridge/motor_protocol 节点）
```

- 真机上电时：`sudo systemctl start can-up.service`，xacro 参数换 `can_interface:=can1`（或后续提供真机 launch），其余不动；先低压低速复测同一脚本。
- Humble 无 xacro:elif、launch Command 的 `"xacro "` 前缀、install launch 是 build 副本等五个接线坑详见 [LL-074](../lessons_learned/LL-074-ros2-control-system-interface-vcan-xacro-humble-pitfalls.md)。

### 重力补偿自由拖动验收（F73，effort 模式 + ZeroTorque 对标控制器）

工业级示教路径，取代手搓 zero-torque：`zero_torque_controller`（`a3_hardware_interface/GravityCompensationController`，RNEA 重力矩写 `/effort`）以 `--inactive` 常驻，进入/退出全部走标准 `ros2 control switch_controllers`，与 arm_controller 互斥；硬件插件在 effort 模式发 kp=0/kd=2/torque_ff=关节重力矩×direction。

```bash
# 沿用 F72 的 vcan0 模拟器 + 标准栈（ROS_DOMAIN_ID=59）
python3 scripts/a3_test/f73_gravity_comp_vcan_acceptance.py
#   通过标准：末尾「总体: ALL PASS」（41 项：home/ready/mid 三姿态互斥切换；
#   CAN kp=0/kd=2/位置字段=实测位，torque_ff 对独立 RNEA 偏差 ≤0.0003 Nm；
#   外力注入 L3 同号跟随、撤力漂移 0.0008 rad；切回后 JTC 回归 0.0003）

# 手动进入/退出自由拖动（真机与 vcan 通用）
ros2 control switch_controllers --deactivate arm_controller --activate zero_torque_controller
ros2 control switch_controllers --deactivate zero_torque_controller --activate arm_controller
# 观测：ros2 topic echo /zero_torque_controller/gravity_torque
```

- Humble `get_name()` 全名语义、旧 shell AMENT_PREFIX_PATH 致 pluginlib 只认 mock 等六个坑详见 [LL-075](../lessons_learned/LL-075-gravity-comp-controller-fullname-stale-ament-pitfalls.md)。

### 编排层标准执行后端验收（F74，control_backend=fjt_action）

编排层全部兜底轨迹（jog / goto 线性兜底 / playback / safe-park）改经标准 `control_msgs/FollowJointTrajectory` action 下发，arm（L1–L6）、gripper（L7）双 JTC 拆分投影、异步抢占；参数 `control_backend`（默认 `topic` 零回归，`fjt_action` 走标准栈）。

```bash
# 1) F70 mock 标准栈（无 CAN）
ROS_DOMAIN_ID=60 ros2 launch a3_bringup edge_ros2_control_sim.launch.py
# 2) 编排 FSM（fjt_action）+ retime 节点
ROS_DOMAIN_ID=60 ros2 launch scripts/a3_test/f74_extra.launch.py
# 3) 验收
ROS_DOMAIN_ID=60 python3 scripts/a3_test/f74_fjt_backend_mock_acceptance.py
#   通过标准：末尾「F74 acceptance: 12/12」（enable/jog×3/goto 兜底 ready+home/
#   playback retime 拆分执行/抢占语义/旧话题零消息/safe-park 含 L7 归位）
```

- move_group 只规划 arm 组、safe-park 须补发 L7 gripper 轨迹；L2/L3 单向限位越限会被 JTC 静默夹紧——详见 [LL-076](../lessons_learned/LL-076-moveit-arm-group-leaves-l7-jtc-clamps-one-sided-limits.md)。

### 全产品 mock-hardware 标准栈验收（F75，零自研 sim 节点）

单一 bringup 拉起与真机 F72 栈同构的产品级拓扑：mock GenericSystem + controller_manager（JSB active、arm/gripper 两 JTC **inactive 启动**）+ move_group + retime 节点 + 编排层（`control_backend=fjt_action`、`motor_service_backend=controller_switch`，enable/disable 走标准 `/controller_manager/switch_controller`）+ 产品夹爪节点（轨迹出口指 JTC 原生话题）+ MQTT 桥。全程不加载 sim_motor_node / sim_power_sequence / gravity_torque。

```bash
source scripts/a3_shell_env.sh && export PYTHONNOUSERSITE=1
# 1) 起栈（无 CAN / 无电机；use_rviz:=true 可加 RViz）
ROS_DOMAIN_ID=61 ros2 launch a3_bringup edge_full_mock.launch.py
# 2) 验收（另一终端）
ROS_DOMAIN_ID=61 python3 scripts/a3_test/f75_full_mock_acceptance.py
#   通过标准：末尾「F75 acceptance: 15/15」（boot 两 JTC inactive/零自研 sim 节点/
#   产品节点齐、enable→READY 两 JTC active、jog×3 含 L7、goto ready/home、
#   playback retime、夹爪位置命令经标准 JTC 驱动 L7、safe-park 双落定后失能、MQTT 存活）
```

- goto/move_to 走 move_group 同样漏 L7（统一补发）；失能落定必须位置 AND 速度双条件；重启栈先按 PID 清全部子进程——详见 [LL-077](../lessons_learned/LL-077-goto-l7-dispatch-settle-pos-vel-double-stack.md)。
- 真机上电后：`edge_full_mock.launch.py` 的 mock 拓扑即真机 bringup 的改造模板（xacro 切 `use_real_hardware:=true`、F72 SystemInterface 已提供使能语义），真机验收待上电。

### Pilz 工业运动规划器验收（F76，PTP / LIN / CIRC + Sequence）

F75 栈的 move_group 改为双规划管线：OMPL（默认）+ Pilz 工业运动规划器。PTP 点到点、LIN 末端直线、CIRC 圆弧、带 `blend_radius` 的 Sequence 混合程序一次下发，执行仍经标准 JTC，替代手写 `move_to_pose_ik_node` / `draw_rectangle_demo`。管线按请求内 `pipeline_id`/`planner_id` 选择（统一服务 `/plan_kinematic_path`，序列 `/plan_sequence_path` + `/sequence_move_group` action）。

```bash
source scripts/a3_shell_env.sh && export PYTHONNOUSERSITE=1
# 1) 起栈（F75 全产品 mock 栈，自动加载双管线；关掉 MQTT 验收更快）
ROS_DOMAIN_ID=62 ros2 launch a3_bringup edge_full_mock.launch.py use_mqtt:=false
# 2) 验收（另一终端）
ROS_DOMAIN_ID=62 python3 scripts/a3_test/f76_pilz_acceptance.py
#   通过标准：末尾「F76 acceptance: 12/12」（端点齐、JTC 生命周期、OMPL/PTP 落点、
#   LIN 直线度 ≤2 mm、CIRC 半径偏差 ≤2 mm、三角形 blend 连续通过、零自研笛卡尔节点）
```

- CIRC center 约束的 region 必须覆盖整弧（小盒必报 Position constraint violated）；升级 MoveIt 后 pick-ik 等第三方插件要同步换同版本构建——详见 [LL-078](../lessons_learned/LL-078-pilz-pipeline-version-coupling-circ-center-region.md)。
- 真机上电后 Pilz 管线随 F72 真机栈同构复用，真机验收待上电。

### 单关节点动走 MoveIt Servo JointJog 验收（F77）

PS4 D-pad 单关节点动不再发旁路 FSM 的手写单点轨迹（旧 `/joint_group_effort_controller/joint_trajectory` 在标准栈上是死话题），改走标准栈常驻的 MoveIt Servo：`control_msgs/JointJog`（速度单位）发到 `/servo_node/delta_joint_cmds`，限位/奇异点/碰撞由 servo 统一保护，输出轨迹直入标准 arm JTC；松开即停保位，FSM 状态保持 READY。L7 夹爪不进 servo，仍走夹爪指令路径。

```bash
source scripts/a3_shell_env.sh && export PYTHONNOUSERSITE=1
# 1) 起栈（标准栈已常驻 servo_node + servo_mode_bridge）
ROS_DOMAIN_ID=63 ros2 launch a3_bringup edge_full_mock.launch.py use_mqtt:=false
# 2) 验收（另一终端）
ROS_DOMAIN_ID=63 python3 scripts/a3_test/f77_joint_jog_acceptance.py
#   通过标准：末尾「F77 acceptance: 8/8」（enable/JTC/servo 端点、JointJog 双向
#   定向运动、停止保位且 control_mode IDLE、FSM 恒 READY、旧话题零消息、L7 不受影响）
```

- servo 输出话题参数是嵌套的 `moveit_servo.command_out_topic`，顶层同名键被静默忽略；「servo 不动」先查输出话题端点——详见 [LL-079](../lessons_learned/LL-079-servo-nested-command-out-topic-param-and-input-qos.md)。
- 真机上电后点动随 servo 真机栈同构复用，真机验收待上电。

## 相关文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [REQUIREMENTS.md](REQUIREMENTS.md)
- [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
- [文档索引](../README.md)
