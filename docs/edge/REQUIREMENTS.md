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


> 需求条目已按「一条一文件」拆分到 [`requirements/`](requirements/) 目录，文件名 `F###-slug.md`。下表为完整索引（按 F# 排序）；新增/修改需求正文请进入对应文件，登记/状态变化按 requirements-first 流程同步更新本表。

| ID | 标题 | 状态 | 分组 | 文档 |
|---|---|---|---|---|
| F1 | 轨迹执行 | — | 功能 | [requirements/F1-trajectory-execution.md](requirements/F1-trajectory-execution.md) |
| F2 | 电源序列与门控 | — | 功能 | [requirements/F2-power-sequence-gating.md](requirements/F2-power-sequence-gating.md) |
| F3 | PS4 遥操作（电源） | — | 功能 | [requirements/F3-ps4-teleop-power.md](requirements/F3-ps4-teleop-power.md) |
| F4 | reBot / MoveIt 集成 | implemented | 功能 | [requirements/F4-rebot-moveit-integration.md](requirements/F4-rebot-moveit-integration.md) |
| F5 | 多臂（可选） | — | 功能 | [requirements/F5-multi-arm-optional.md](requirements/F5-multi-arm-optional.md) |
| F6 | 轨迹时间跟踪（C2） | implemented | 功能 | [requirements/F6-trajectory-time-tracking.md](requirements/F6-trajectory-time-tracking.md) |
| F7 | 命名姿态 zero→ready 仿真闭环（C1） | implemented | 功能 | [requirements/F7-named-pose-zero-ready-sim.md](requirements/F7-named-pose-zero-ready-sim.md) |
| F8 | 重力力矩计算（C3 仿真可验部分） | implemented | 功能 | [requirements/F8-gravity-torque-calc.md](requirements/F8-gravity-torque-calc.md) |
| F9 | 重力补偿 MIT 前馈入环（C3 真机路径） | implemented | 功能 | [requirements/F9-gravity-compensation-feedforward.md](requirements/F9-gravity-compensation-feedforward.md) |
| F10 | FollowJointTrajectory Action（C1） | implemented | 功能 | [requirements/F10-follow-joint-trajectory-action.md](requirements/F10-follow-joint-trajectory-action.md) |
| F11 | 统一 MoveIt Execute launch + 画矩形 demo | implemented | 功能 | [requirements/F11-moveit-execute-launch-rectangle-demo.md](requirements/F11-moveit-execute-launch-rectangle-demo.md) |
| F12 | 笛卡尔 MoveToPose / IK | implemented | 功能 | [requirements/F12-cartesian-movetopose-ik.md](requirements/F12-cartesian-movetopose-ik.md) |
| F13 | 零力矩模式（C5） | implemented | 功能 | [requirements/F13-zero-torque-mode.md](requirements/F13-zero-torque-mode.md) |
| F14 | MoveIt Servo（C4） | implemented | 功能 | [requirements/F14-moveit-servo.md](requirements/F14-moveit-servo.md) |
| F15 | JTC 兼容样条插值（C2 增强） | implemented | 功能 | [requirements/F15-jtc-spline-interpolation.md](requirements/F15-jtc-spline-interpolation.md) |
| F16 | PS4 映射遥操作（笛卡尔 + 夹爪） | implemented | 功能 | [requirements/F16-ps4-mapped-teleop-cartesian-gripper.md](requirements/F16-ps4-mapped-teleop-cartesian-gripper.md) |
| F17 | EL05 电机协议命令集补齐（L0 增强） | implemented | 功能 | [requirements/F17-el05-motor-protocol-commands.md](requirements/F17-el05-motor-protocol-commands.md) |
| F18 | ROS2→MQTT 遥测上报与 Web 实时展示（跨仓） | in progress | 功能 | [requirements/F18-ros2-mqtt-telemetry-web.md](requirements/F18-ros2-mqtt-telemetry-web.md) |
| F19 | 总线扫描服务（通信类型 0 范围探测） | implemented | 功能 | [requirements/F19-can-bus-scan-service.md](requirements/F19-can-bus-scan-service.md) |
| F20 | Web 端 3D 机械臂实时渲染（跨仓） | in progress | 功能 | [requirements/F20-web-3d-arm-render.md](requirements/F20-web-3d-arm-render.md) |
| F21 | 机械臂编排节点（a3_arm_controller） | implemented | 功能 | [requirements/F21-arm-orchestration-node.md](requirements/F21-arm-orchestration-node.md) |
| F22 | 单电机分层回归测试套件（can1 / ID7 空载） | implemented | 功能 | [requirements/F22-single-motor-layered-regression-test.md](requirements/F22-single-motor-layered-regression-test.md) |
| F23 | Web 端机械臂控制面板（MQTT cmd 下行 UI，跨仓） | implemented | 功能 | [requirements/F23-web-arm-control-panel.md](requirements/F23-web-arm-control-panel.md) |
| F24 | 夹爪握力配置与安全限幅 | implemented | 功能 | [requirements/F24-gripper-grip-force-config-limit.md](requirements/F24-gripper-grip-force-config-limit.md) |
| F25 | 夹爪自适应力控（PI 力外环 + 位置内环） | implemented | 功能 | [requirements/F25-gripper-adaptive-force-control.md](requirements/F25-gripper-adaptive-force-control.md) |
| F26 | 夹爪服务/话题契约与 MQTT 桥接 | implemented | 功能 | [requirements/F26-gripper-service-topic-contract-mqtt.md](requirements/F26-gripper-service-topic-contract-mqtt.md) |
| F27 | Web 端夹爪控制面板（MQTT 下行 UI，跨仓） | implemented | 功能 | [requirements/F27-web-gripper-control-panel.md](requirements/F27-web-gripper-control-panel.md) |
| F28 | 夹爪力控真机回归测试 | implemented | 功能 | [requirements/F28-gripper-force-control-hardware-regression.md](requirements/F28-gripper-force-control-hardware-regression.md) |
| F29 | 互锁模式回收（/a3/control_mode 发布缺陷修复，真机实测） | implemented | 功能 | [requirements/F29-interlock-mode-reclaim.md](requirements/F29-interlock-mode-reclaim.md) |
| F30 | 夹爪力控超时以命令参数为准（真机实测） | implemented | 功能 | [requirements/F30-gripper-force-timeout-cmd-param.md](requirements/F30-gripper-force-timeout-cmd-param.md) |
| F31 | Web 端夹爪面板 v2：位置模式直驱 / 双曲线 / NaN 遥测毒化修复（跨仓） | completed | 功能 | [requirements/F31-web-gripper-panel-v2.md](requirements/F31-web-gripper-panel-v2.md) |
| F32 | Web 端电机调试页（CAN 扫描 / MIT 保持 / 实时曲线，跨仓） | in_progress | 功能 | [requirements/F32-web-motor-debug-page.md](requirements/F32-web-motor-debug-page.md) |
| F33 | 夹爪力控真机行为修正（接触位移门 / 默认超时 / 释放柔顺 / 零力矩直开 / 力环目标位置遥测） | completed | 功能 | [requirements/F33-gripper-force-behavior-fix.md](requirements/F33-gripper-force-behavior-fix.md) |
| F34 | web 路径力控阶梯验收（MQTT 模拟按键 0.3 → 0.5 → 0） | completed | 功能 | [requirements/F34-web-path-force-ladder-acceptance.md](requirements/F34-web-path-force-ladder-acceptance.md) |
| F35 | MQTT 遥测降采样 5 Hz（全局） | implemented | 功能 | [requirements/F35-mqtt-telemetry-downsample-5hz.md](requirements/F35-mqtt-telemetry-downsample-5hz.md) |
| F36 | PS4 R2 扳机力控夹爪 | implemented | 功能 | [requirements/F36-ps4-r2-trigger-gripper-force.md](requirements/F36-ps4-r2-trigger-gripper-force.md) |
| F37 | Web 夹爪面板：停止=失能、使能/设置零位按钮、4 曲线合并单图双轴（跨仓） | implemented | 功能 | [requirements/F37-web-gripper-panel-stop-enable.md](requirements/F37-web-gripper-panel-stop-enable.md) |
| F38 | 通用 N 关节臂 init/示教/回放（URDF 无关）+ 回放首点插值 + 示教停止防弹回 | completed | 功能 | [requirements/F38-generic-n-joint-init-teach-playback.md](requirements/F38-generic-n-joint-init-teach-playback.md) |
| F39 | 命名点位保存（用户层覆盖）+ 通用平滑移动指令 move_to | implemented | 功能 | [requirements/F39-named-pose-save-move-to.md](requirements/F39-named-pose-save-move-to.md) |
| F40 | 失能保护（disable → 自动回 idle（原 home，F113）→ 失能） | completed | 功能 | [requirements/F40-disable-protection.md](requirements/F40-disable-protection.md) |
| F41 | move_to 时长兜底 + 插值密度（≥50 Hz） | completed | 功能 | [requirements/F41-move-to-duration-interp-density.md](requirements/F41-move-to-duration-interp-density.md) |
| F42 | 力矩方向钳位（执行层 latch，碰撞保护） | completed | 功能 | [requirements/F42-torque-direction-clamp.md](requirements/F42-torque-direction-clamp.md) |
| F43 | 最大力矩持久化 + MQTT mtqmax | completed | 功能 | [requirements/F43-max-torque-persistence.md](requirements/F43-max-torque-persistence.md) |
| F44 | 温度管理（warn / protect / COOLING） | completed | 功能 | [requirements/F44-temperature-management.md](requirements/F44-temperature-management.md) |
| F45 | 状态机增强（11 态） | completed | 功能 | [requirements/F45-state-machine-11-states.md](requirements/F45-state-machine-11-states.md) |
| F46 | TX 帧率监视（TxStats + MQTT txhz） | completed | 功能 | [requirements/F46-tx-frame-rate-monitor.md](requirements/F46-tx-frame-rate-monitor.md) |
| F47 | 0x7029 zero_sta 参数读写协议（断电多圈窗口选择） | implemented | 功能 | [requirements/F47-zero-sta-parameter-protocol.md](requirements/F47-zero-sta-parameter-protocol.md) |
| F48 | 开机零位校验（读数限位硬检查 + 期望位姿软检查 + 使能门禁） | implemented | 功能 | [requirements/F48-boot-zero-position-check.md](requirements/F48-boot-zero-position-check.md) |
| F49 | 重力标定（7J 真机 inertia_params 重标定，复刻官方 dynamics_calibration） | implemented | 功能 | [requirements/F49-gravity-calibration.md](requirements/F49-gravity-calibration.md) |
| F50 | 故障监视看门狗（arm_monitor：期望 vs 实际偏差 → 升级处置） | implemented | 功能 | [requirements/F50-fault-monitor-watchdog.md](requirements/F50-fault-monitor-watchdog.md) |
| F51 | 使能安全门禁（三层修复：保持抑制 latch / 使能重锚 / 意图边界重基准） | implemented | 功能 | [requirements/F51-enable-safety-gate.md](requirements/F51-enable-safety-gate.md) |
| F52 | 缺电机降级档（N 关节运行：只对在线电机做校验/比对/遥测） | completed | 功能 | [requirements/F52-missing-motor-degradation.md](requirements/F52-missing-motor-degradation.md) |
| F53 | 编排层「不罢工」补齐：指令×状态×模式 全组合显式拒绝（F40 零力矩 disable 死路径） | implemented | 功能 | [requirements/F53-orchestration-full-explicit-rejection.md](requirements/F53-orchestration-full-explicit-rejection.md) |
| F54 | 示教停止自动保存 + 回放默认最新（空名 ≡ latest 槽位） | implemented | 功能 | [requirements/F54-teach-stop-autosave.md](requirements/F54-teach-stop-autosave.md) |
| F55 | PS4 全功能映射：示教/执行/使能/初始化一键化（短按+长按双义） | implemented | 功能 | [requirements/F55-ps4-full-mapping.md](requirements/F55-ps4-full-mapping.md) |
| F56 | MIT 帧目标速度前馈（帧 V 域填每 tick 目标速度） | implemented | 功能 | [requirements/F56-mit-frame-velocity-feedforward.md](requirements/F56-mit-frame-velocity-feedforward.md) |
| F57 | 回放匀速重排 + 保存时轻量平滑 | implemented | 功能 | [requirements/F57-playback-constant-speed-resort.md](requirements/F57-playback-constant-speed-resort.md) |
| F58 | stop 处置改「重力支撑保持」+ 自动 reset 姿态门禁 | implemented | 功能 | [requirements/F58-stop-gravity-hold.md](requirements/F58-stop-gravity-hold.md) |
| F59 | 回放 warp 平滑化重写（弧长均匀重采样 + 加速度限幅时间膨胀） | implemented | 功能 | [requirements/F59-playback-warp-smoothing.md](requirements/F59-playback-warp-smoothing.md) |
| F60 | PS4 键位重设计（一键使能 / 单键硬急停 / 示教三键） | implemented-pending-sim | 功能 | [requirements/F60-ps4-keymap-redesign.md](requirements/F60-ps4-keymap-redesign.md) |
| F61 | DS4 灯带 + 震动反馈（状态五色系） | implemented-pending-sim | 功能 | [requirements/F61-ds4-led-rumble.md](requirements/F61-ds4-led-rumble.md) |
| F62 | 合成 /joy 全功能仿真验证（无手柄自动化） | implemented-pending-sim | 功能 | [requirements/F62-synthetic-joy-sim-validation.md](requirements/F62-synthetic-joy-sim-validation.md) |
| F63 | joynet Linux DS4 独立布局（pygame/SDL 6 轴 + hat，对齐 joy_node 协议） | in-progress | 功能 | [requirements/F63-joynet-linux-ds4-layout.md](requirements/F63-joynet-linux-ds4-layout.md) |
| F64 | PS4 双模式死人开关（L1 平移 / R1 旋转）+ D-pad 分通道调速 | completed | 功能 | [requirements/F64-ps4-dual-mode-deadman.md](requirements/F64-ps4-dual-mode-deadman.md) |
| F65 | 真机 Servo 入环（独立 servo 话题）+ L3 使能幂等 | in-progress | 功能 | [requirements/F65-hardware-servo-integration.md](requirements/F65-hardware-servo-integration.md) |
| F66 | 使能意图安全（gate 关作废全部意图 / 任意使能沿强制重锚 / 恢复死锁解锁） | implemented-pending-hw | 功能 | [requirements/F66-enable-intent-safety.md](requirements/F66-enable-intent-safety.md) |
| F67 | goto/move_to 走 move_group + TOTG（工业轨迹，起止零速） | done（仿真） | 功能 | [requirements/F67-goto-move-to-moveit-totg.md](requirements/F67-goto-move-to-moveit-totg.md) |
| F68 | 示教回放走 Ruckig/TOTG 在线重定时（保几何、退役手搓平滑） | done（仿真） | 功能 | [requirements/F68-teach-playback-ruckig-retime.md](requirements/F68-teach-playback-ruckig-retime.md) |
| F69 | ready 点位改为非腕奇异形（修复伺服 L1 驱动整臂变软下坠） | done（仿真） | 功能 | [requirements/F69-ready-pose-non-singular.md](requirements/F69-ready-pose-non-singular.md) |
| F70 | ros2_control 标准栈仿真激活（JTC/JSB/controller_manager 取代手搓 FJT/插值） | completed（仿真） | 功能 | [requirements/F70-ros2-control-sim-activation.md](requirements/F70-ros2-control-sim-activation.md) |
| F71 | arm_monitor 标准诊断通道（diagnostic_updater → /diagnostics） | completed（仿真） | 功能 | [requirements/F71-arm-monitor-diagnostics.md](requirements/F71-arm-monitor-diagnostics.md) |
| F72 | 真机 SystemInterface 插件（MIT/SocketCAN 直驱，ros2_control 标准栈真机化） | 仿真验收通过 | 功能 | [requirements/F72-hardware-systeminterface-plugin.md](requirements/F72-hardware-systeminterface-plugin.md) |
| F73 | 力矩指令模式 + 标准重力补偿自由拖动控制器（对标官方 ZeroTorqueController，switch_controllers 切换示教） | completed | 功能 | [requirements/F73-torque-command-mode-zero-torque.md](requirements/F73-torque-command-mode-zero-torque.md) |
| F74 | 编排层标准执行后端（control_msgs/FollowJointTrajectory action → JTC，参数门控） | completed | 功能 | [requirements/F74-orchestration-standard-exec-backend.md](requirements/F74-orchestration-standard-exec-backend.md) |
| F75 | 全产品 mock-hardware 标准栈 bringup（零自研 sim 节点：控制器 inactive 启动 + 编排层 switch_controller 使能） | completed | 功能 | [requirements/F75-mock-hardware-standard-bringup.md](requirements/F75-mock-hardware-standard-bringup.md) |
| F76 | Pilz 工业运动规划器（PTP / LIN / CIRC + Sequence 混合，替代手写笛卡尔节点） | completed | 功能 | [requirements/F76-pilz-industrial-planner.md](requirements/F76-pilz-industrial-planner.md) |
| F77 | PS4 D-pad 单关节点动改走 MoveIt Servo JointJog（替代手写单点轨迹） | completed | 功能 | [requirements/F77-ps4-dpad-joint-jog-servo.md](requirements/F77-ps4-dpad-joint-jog-servo.md) |
| F78 | 统一 bringup 单入口（hardware:=mock|can，mock/can 产品拓扑完全一致） | 已完成 | 功能 | [requirements/F78-unified-bringup-single-entry.md](requirements/F78-unified-bringup-single-entry.md) |
| F79 | 产品验收接入 colcon test / launch_testing（标准 CI 测试门） | 已完成 | 功能 | [requirements/F79-product-acceptance-colcon-test.md](requirements/F79-product-acceptance-colcon-test.md) |
| F81 | 单电机反馈断线看门狗（按电机 last-rx 超时 → 内部锁存 + 整臂冻结保持） | 已完成 | 功能 | [requirements/F81-motor-feedback-loss-watchdog.md](requirements/F81-motor-feedback-loss-watchdog.md) |
| F82 | 诊断聚合器接入（diagnostic_aggregator / GenericAnalyzer → /diagnostics_agg + toplevel state） | 仿真验收通过 | 功能 | [requirements/F82-diagnostic-aggregator.md](requirements/F82-diagnostic-aggregator.md) |
| F83 | 使能流程对齐厂商标准（清故障 → 整数写运行模式 → 使能 → 软启动阻尼接管） | 仿真验收通过 9/9 | 功能 | [requirements/F83-enable-flow-vendor-standard.md](requirements/F83-enable-flow-vendor-standard.md) |
| F84 | 温度/故障码完整解码（temperature state interface + motor_health 诊断 + 复用 F44 FSM 门禁） | 仿真验收通过 10/10 | 功能 | [requirements/F84-temperature-fault-decode.md](requirements/F84-temperature-fault-decode.md) |
| F85 | 自由拖动速度自适应 Kd（Lorentzian 速度曲线 + EMA；对标官方 el_a3_hardware computeAdaptiveKd） | 仿真验收通过 | 功能 | [requirements/F85-free-drive-adaptive-kd.md](requirements/F85-free-drive-adaptive-kd.md) |
| F86 | 电机侧通信超时配置（0x7028 Type-18 写入；与主机看门狗独立的纵深防御） | 仿真验收通过 | 功能 | [requirements/F86-motor-comm-timeout-config.md](requirements/F86-motor-comm-timeout-config.md) |
| F87 | 电机侧力矩限制布防（0x700B Type-18 float 写入；使能编排内每电机逐路下发） | 仿真验收通过 | 功能 | [requirements/F87-motor-torque-limit-arm.md](requirements/F87-motor-torque-limit-arm.md) |
| F87（第二步） | L7 走标准 GripperActionController（per-goal max_effort；退役夹爪节点死服务依赖） | 仿真验收通过 | 功能 | [requirements/F87-step2-l7-gripper-action-controller.md](requirements/F87-step2-l7-gripper-action-controller.md) |
| F88 | 两点标准轨迹替代手搓密集线性插值（JTC splines 控制器侧插值；retime ramp 同步稀疏化） | 已完成 | 功能 | [requirements/F88-two-point-standard-trajectory.md](requirements/F88-two-point-standard-trajectory.md) |
| F89 | pinocchio 重力模型标定/核验（静态多姿态测量 → 逐关节重力矩比例因子 + R²；对标 EDULITE_A3 pinocchio_gravity_calibration.py） | 已实现 | 功能 | [requirements/F89-pinocchio-gravity-model-calibration.md](requirements/F89-pinocchio-gravity-model-calibration.md) |
| F89b | FSM 自由拖动走标准控制器切换（switch_controller：arm_controller ↔ zero_torque_controller；PS4 示教经同一 FSM 服务） | 已完成 | 功能 | [requirements/F89b-fsm-free-drive-controller-switch.md](requirements/F89b-fsm-free-drive-controller-switch.md) |
| F90 | 故障触发有界 rosbag2 黑匣子（snapshot-mode 循环缓冲，FAULT/COOLING 边沿自动落盘） | implemented | 功能 | [requirements/F90-fault-triggered-rosbag2-blackbox.md](requirements/F90-fault-triggered-rosbag2-blackbox.md) |
| F91 | 电机零点/参数维护产品化（独立维护节点 + 控制器活动联锁；退役手柄长按调零死映射） | completed | 功能 | [requirements/F91-motor-zero-param-maintenance.md](requirements/F91-motor-zero-param-maintenance.md) |
| F92 | 全仓质量门禁接线（lint 真跑 + 一键 CI 门脚本） | 已完成 | 功能 | [requirements/F92-repo-quality-gate-lint.md](requirements/F92-repo-quality-gate-lint.md) |
| F93 | 产品栈 systemd 托管（版本化开机单元 + 崩溃自动重启；默认禁用，显式启用） | completed | 功能 | [requirements/F93-product-stack-systemd.md](requirements/F93-product-stack-systemd.md) |
| F94 | 两点轨迹统一速度限幅（URDF velocity 地板时长；堵住绕过规划器的无限速 jog/park） | completed | 功能 | [requirements/F94-two-point-velocity-limit.md](requirements/F94-two-point-velocity-limit.md) |
| F95 | 标准自检服务（ros-humble-self-test / diagnostic_msgs/SelfTest；开机/维护一键只读自检） | completed | 功能 | [requirements/F95-standard-self-test.md](requirements/F95-standard-self-test.md) |
| F96 | 主机资源标准诊断（diagnostic_common_diagnostics：CPU/内存/磁盘 → /diagnostics 与 /diagnostics_agg/Host） | completed | 功能 | [requirements/F96-host-resource-diagnostics.md](requirements/F96-host-resource-diagnostics.md) |
| F97 | JTC 轨迹容差工业级配置（goal_time 超时必 abort + 逐关节 trajectory 跟踪容差；堵住卡死轨迹永久挂起） | completed | 功能 | [requirements/F97-jtc-trajectory-tolerance.md](requirements/F97-jtc-trajectory-tolerance.md) |
| F98 | L7 夹爪限位与实测标定统一（URDF / ros2_control / MoveIt 三处 [0.0, 1.78]；堵住模型与实物 12% 偏差） | completed | 功能 | [requirements/F98-l7-gripper-limit-calibration.md](requirements/F98-l7-gripper-limit-calibration.md) |
| F99 | JTC 指令超时 cmd_timeout（话题接口陈旧指令在轨迹结束后确定性切 hold；堵住末速指令长期驻留） | completed | 功能 | [requirements/F99-jtc-cmd-timeout.md](requirements/F99-jtc-cmd-timeout.md) |
| F100 | 实时调度权限硬化（controller_manager 真正跑上 SCHED_FIFO；堵住 200 Hz 控制环被普通负载抢占） | completed | 功能 | [requirements/F100-realtime-sched-hardening.md](requirements/F100-realtime-sched-hardening.md) |
| F101 | 看门狗硬化（硬件 watchdog + systemd 服务 watchdog 双层；控制环挂死/整机死锁确定性恢复） | completed | 功能 | [requirements/F101-watchdog-hardening.md](requirements/F101-watchdog-hardening.md) |
| F102 | MQTT 通信链路健康入标准诊断（断链可观测、10 s 升级 ERROR；Comms 聚合分组；env 覆盖 broker） | completed | 功能 | [requirements/F102-mqtt-link-health-diagnostics.md](requirements/F102-mqtt-link-health-diagnostics.md) |
| F103 | CAN 总线物理层健康入标准诊断（bus-off/错误帧可观测；链路 down/丢失 → ERROR 进现场状态） | completed | 功能 | [requirements/F103-can-bus-health-diagnostics.md](requirements/F103-can-bus-health-diagnostics.md) |
| F104 | 关键话题频率健康入标准诊断（速率退化 WARN、停发 ERROR；Topic Rates 聚合分组） | completed | 功能 | [requirements/F104-topic-rate-health-diagnostics.md](requirements/F104-topic-rate-health-diagnostics.md) |
| F105 | DDS 网络栈硬化（内核套接字缓冲 sysctl + CycloneDDS 标准配置；高负载零丢包） | net.core.rmem_max/wmem_max | 功能 | [requirements/F105-dds-network-hardening.md](requirements/F105-dds-network-hardening.md) |
| F106 | 一次性调试/交付冒烟测试（commissioning smoke test；对标 EDULITE startup_test_demo；一条命令六阶段端到端） | completed | 功能 | [requirements/F106-commissioning-smoke-test.md](requirements/F106-commissioning-smoke-test.md) |
| F107 | 额定负载与占空比静态门禁（rated payload 1.5 kg + duty-cycle；工业机器人额定参数对标） | completed | 功能 | [requirements/F107-rated-payload-duty-gate.md](requirements/F107-rated-payload-duty-gate.md) |
| F108 | 位置模式 pinocchio 重力前馈（对标 EDULITE gravity_feedforward_ratio；消除稳态下垂） | completed | 功能 | [requirements/F108-position-mode-gravity-feedforward.md](requirements/F108-position-mode-gravity-feedforward.md) |
| F109 | ready 点位重定义（折叠竖直、臂重心投影过底座中心；对标官方 zero/home/ready 点位语义） | completed | 功能 | [requirements/F109-ready-pose-redefinition.md](requirements/F109-ready-pose-redefinition.md) |
| F110 | mock 产品栈补齐电源序列（sim_power_sequence；修复 L3 使能超时与 F61 灯效红闪误判） | completed | 功能 | [requirements/F110-mock-power-sequence.md](requirements/F110-mock-power-sequence.md) |
| F111 | 真机（hardware:=can）栈补接 C++ power_sequence_node（修复真机 L3 gate 0 publisher） | completed | 功能 | [requirements/F111-hardware-power-sequence-node.md](requirements/F111-hardware-power-sequence-node.md) |
| F112 | JTC trajectory 跟踪容差按真机实测滞后放宽（0.05→0.15；修复 home↔ready goto 误报 PATH_TOLERANCE_VIOLATED 导致中途停与重复按键起跑-反冲） | completed | 功能 | [requirements/F112-jtc-tolerance-relaxation.md](requirements/F112-jtc-tolerance-relaxation.md) |
| F113 | 失能态电机 0x18 主动上报保活（R3 后 RViz 实时 + L3 立即使能；修复 reset 后总线静默） | completed | 功能 | [requirements/F113-disabled-motor-0x18-keepalive.md](requirements/F113-disabled-motor-0x18-keepalive.md) |
| F113 | 点位改名 home→idle + 弃用 `~/.a3/poses.yaml` 用户覆盖（点位统一收敛到包内 named_poses.yaml） | implemented | 功能 | [requirements/F113-pose-rename-home-idle.md](requirements/F113-pose-rename-home-idle.md) |
| F114 | PS4 一键保存当前位姿为命名点位（L2 短按，保存到包内 named_poses.yaml） | implemented | 功能 | [requirements/F114-ps4-save-pose-one-key.md](requirements/F114-ps4-save-pose-one-key.md) |
| F115 | D-pad 退役 F64 调速，改绑 home 周边 4 个笛卡尔偏移命名点位 | implemented | 功能 | [requirements/F115-dpad-home-offset-poses.md](requirements/F115-dpad-home-offset-poses.md) |
| F116 | 随机命名点位 MoveIt 巡游服务（数量可配，相邻不重，从当前点起） | implemented | 功能 | [requirements/F116-random-pose-moveit-tour.md](requirements/F116-random-pose-moveit-tour.md) |
| F117 | JTC 跟踪容差 0.30/0.05 临时定型（持久化实测热设组合；语义=卡滞检测非防撞） | implemented | 功能 | [requirements/F117-jtc-tolerance-persist.md](requirements/F117-jtc-tolerance-persist.md) |
| F118 | MoveIt Servo 绝对目标（薄桥接锚点节点，抗外力漂移） | implemented | 功能 | [requirements/F118-moveit-servo-absolute-target.md](requirements/F118-moveit-servo-absolute-target.md) |
| F119 | 任意模式软失能（R3：先退出当前模式 → 回 idle → 保电失能） | implemented | 功能 | [requirements/F119-any-mode-soft-disable.md](requirements/F119-any-mode-soft-disable.md) |
| F120 | Triangle 改绑 home（不绑 ready） | implemented | 功能 | [requirements/F120-triangle-rebind-home.md](requirements/F120-triangle-rebind-home.md) |
| F121 | 反馈新鲜度与解析实时性诊断探针（真机基线） | 真机验收通过 | 功能 | [requirements/F121-feedback-freshness-probe.md](requirements/F121-feedback-freshness-probe.md) |
| F122 | 失能态零增益保活解耦 power-gate（gate 关闭仍保反馈实时） | implemented | 功能 | [requirements/F122-disabled-zero-gain-keepalive-decouple.md](requirements/F122-disabled-zero-gain-keepalive-decouple.md) |
| F123 | 0x18 主动上报周期下调 100ms/10Hz（0x7026 接管，on_activate + on_deactivate 双写） | accepted | 功能 | [requirements/F123-0x18-report-period-downshift.md](requirements/F123-0x18-report-period-downshift.md) |
| F124 | 示教回放回首点改走 MoveIt 轨迹规划（替代几何插值 ramp） | accepted | 功能 | [requirements/F124-teach-playback-return-home-moveit.md](requirements/F124-teach-playback-return-home-moveit.md) |
| F125 | 回首点 / 执行示教轨迹 LED 双段异色（cyan→purple） | accepted | 功能 | [requirements/F125-return-home-led-dual-color.md](requirements/F125-return-home-led-dual-color.md) |
| F126 | 密集示教点 → 单条 MoveIt 平滑轨迹评估（复用 F68 Ruckig retime） | accepted | 功能 | [requirements/F126-dense-teach-smooth-trajectory.md](requirements/F126-dense-teach-smooth-trajectory.md) |
| F127 | 示教（free-drive）期间夹爪 L7 同步释放 + 回放仅到终点位 | accepted | 功能 | [requirements/F127-teach-gripper-sync-release.md](requirements/F127-teach-gripper-sync-release.md) |
| F128 | 冷启动未使能即显实际姿态（on_configure 布防 0x18 纯遥测；L3 使能逻辑不变） | implemented | 功能 | [requirements/F128-cold-start-show-actual-pose.md](requirements/F128-cold-start-show-actual-pose.md) |
| F129 | Square 回放立即灯效 + YAML C loader 提速 + 轨迹本体静态力矩 gate | implemented | 功能 | [requirements/F129-square-playback-led-yaml-speed.md](requirements/F129-square-playback-led-yaml-speed.md) |
| F130 | R3 safe-park 容忍 idle 物理残差：稳态无进展判定（独立噪声速度门） | implemented | 功能 | [requirements/F130-r3-safe-park-residual-tolerance.md](requirements/F130-r3-safe-park-residual-tolerance.md) |
| F131 | 路点示教（Waypoint Teach）：Share 长按录制路点 → L2 打点 → Options 保存 → Square/Circle 长按 PTP/LIN 回放 | Share 长按 1.5s 进入零力矩拖动；L2 短按在当前位置记录一个路点 | 功能 | [requirements/F131-waypoint-teach.md](requirements/F131-waypoint-teach.md) |
| F132 | 触摸板手势：触摸板 tap 触发路点 LIN 回放（暂缓启用，Circle 长按兜底） | implemented | 功能 | [requirements/F132-touchpad-gesture.md](requirements/F132-touchpad-gesture.md) |
| F134 | 多点任务整序列一次规划/执行：pilz Sequence + blend，中间点不停车 | implemented | 功能 | [requirements/F134-multi-point-pilz-sequence.md](requirements/F134-multi-point-pilz-sequence.md) |
| F135 | init 键改绑 L1+R1 长按 2 s，PS 解绑防开机误触 | implemented | 功能 | [requirements/F135-init-key-rebind.md](requirements/F135-init-key-rebind.md) |
| F136 | 轨迹平滑度标定脚手架（标准轨迹 vs 录制轨迹 + 指标 J + 参数扫描，先仿真） | in-progress | 功能 | [requirements/F136-trajectory-smoothness-benchmark.md](requirements/F136-trajectory-smoothness-benchmark.md) |
| F137 | 录制轨迹几何去噪 + 平滑重规划（五次 B 样条光顺 + quintic 重定时） | implemented | 功能 | [requirements/F137-recorded-trajectory-denoise.md](requirements/F137-recorded-trajectory-denoise.md) |
| F138 | MIT 位置环 kp/kd 伺服层自动整定（逐关节，安全自循环） | in-progress | 功能 | [requirements/F138-mit-kp-kd-autotune.md](requirements/F138-mit-kp-kd-autotune.md) |
| F139 | 重力补偿平滑退出（MIT ramp-out，消除模式切换 clack/jerk）【P0】 | proposed | backlog | [requirements/F139-gravity-comp-smooth-exit.md](requirements/F139-gravity-comp-smooth-exit.md) |
| F140 | Safe Park & Shutdown（连接捕获休息位姿；shutdown/park 慢速回位再失能）【P0】 | proposed | backlog | [requirements/F140-safe-park-shutdown.md](requirements/F140-safe-park-shutdown.md) |
| F141 | 参数化 pick & place MoveIt demo（规划场景物体 + 全 YAML 参数）【P1】 | proposed | backlog | [requirements/F141-parameterized-pick-place-demo.md](requirements/F141-parameterized-pick-place-demo.md) |
| F142 | 关节状态发布率提升至 100 Hz（带宽评估后默认）【P1】 | proposed | backlog | [requirements/F142-joint-state-100hz.md](requirements/F142-joint-state-100hz.md) |
| F143 | rqt_robot_monitor 诊断值班视图与排障 SOP【P2】 | proposed | backlog | [requirements/F143-rqt-robot-monitor-view.md](requirements/F143-rqt-robot-monitor-view.md) |
| F144 | SE(3) 笛卡尔测地线轨迹 + CLIK 跟踪【P2】 | proposed | backlog | [requirements/F144-se3-cartesian-geodesic-clik.md](requirements/F144-se3-cartesian-geodesic-clik.md) |
| F145 | Pinocchio + MeshCat 运动学/重力可视化工具【P2】 | proposed | backlog | [requirements/F145-pinocchio-meshcat-viz.md](requirements/F145-pinocchio-meshcat-viz.md) |
| F146 | 多厂商电机/多传输抽象层评估（motorbridge 兼容）【P3】 | proposed | backlog | [requirements/F146-multi-vendor-motor-abstraction.md](requirements/F146-multi-vendor-motor-abstraction.md) |
| F147 | 浏览器 MuJoCo 数字孪生（免安装演示/远程验收）【P3】 | proposed | backlog | [requirements/F147-browser-mujoco-digital-twin.md](requirements/F147-browser-mujoco-digital-twin.md) |
| F148 | 编排层单关节调试 passthrough 服务统一【P3】 | proposed | backlog | [requirements/F148-single-joint-debug-passthrough.md](requirements/F148-single-joint-debug-passthrough.md) |
| F149 | LeRobot 原生 A3 follower robot 类 + 采集全链路（A0）【P0】 | proposed | backlog | [requirements/F149-lerobot-a3-follower.md](requirements/F149-lerobot-a3-follower.md) |
| F150 | RGB-D 深度相机接入与 TSAI 手眼标定（A1）【P0】 | proposed | backlog | [requirements/F150-rgbd-tsai-hand-eye.md](requirements/F150-rgbd-tsai-hand-eye.md) |
| F151 | YOLO/YOLOE 目标检测分割与 RKNN 适配（A1）【P1】 | proposed | backlog | [requirements/F151-yolo-rknn.md](requirements/F151-yolo-rknn.md) |
| F152 | GraspNet 6-DoF / OBB 抓取姿态估计与视觉抓取闭环（A1）【P1】 | proposed | backlog | [requirements/F152-graspnet-6dof.md](requirements/F152-graspnet-6dof.md) |
| F153 | ACT / Diffusion 模仿学习训练管线（服务器训练 + action chunk 落地）（A2）【P1】 | proposed | backlog | [requirements/F153-act-diffusion-imitation.md](requirements/F153-act-diffusion-imitation.md) |
| F154 | LeRobot 异步推理（PolicyServer / RobotClient gRPC）（A2/A3）【P1】 | proposed | backlog | [requirements/F154-lerobot-async-inference.md](requirements/F154-lerobot-async-inference.md) |
| F155 | VLA 策略与 PEFT 微调（SmolVLA / Pi0.5 / GR00T）（A3）【P2】 | proposed | backlog | [requirements/F155-vla-peft-finetune.md](requirements/F155-vla-peft-finetune.md) |
| F156 | 自然语言 Embodied Agent 任务编排（对标 WRC）（A4）【P0】 | proposed | backlog | [requirements/F156-nl-embodied-agent.md](requirements/F156-nl-embodied-agent.md) |
| F157 | 语音 ASR/TTS 链路与 LLM 任务桥（A4）【P1】 | proposed | backlog | [requirements/F157-voice-asr-tts-llm.md](requirements/F157-voice-asr-tts-llm.md) |
| F158 | reSpeaker 麦克风阵列与 DoA 空间感知（A4，可选硬件）【P3】 | proposed | backlog | [requirements/F158-respeak-mic-doa.md](requirements/F158-respeak-mic-doa.md) |
| F159 | Isaac Sim USD 数字孪生与 sim-to-real（A5）【P2】 | proposed | backlog | [requirements/F159-isaac-sim-digital-twin.md](requirements/F159-isaac-sim-digital-twin.md) |
| F160 | MIT 位置环速度/加速度前馈（VFF + AFF + 科氏/离心，全模型 computed-torque 前馈） | implemented | 功能 | [requirements/F160-mit-position-velocity-accel-feedforward.md](requirements/F160-mit-position-velocity-accel-feedforward.md) |

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

## reBot-DevArm 对标潜在需求（2026-10-05 盘点 backlog）

> 来源：[reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) Roadmap & Status（2026-10 口径）+ [reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2) v0.3.0 + [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) + 社区 fork。
> 以下条目均为**潜在需求（状态 `proposed`）**，用于把 reBot 已具备、本仓尚缺的功能/技术栈登记入册，后续**逐条立项 → 补验收细节 → 再开发**（见 requirements-first）。优先级 P0>P1>P2>P3 仅为建议排序；立项时可调整。控制栈本仓已反超项（ros2_control 真机 HAL、Servo、编排 FSM、PS4、MQTT）不再登记。


> 上述 backlog 条目（F139–F159）正文已拆至 `requirements/`，索引见上方「功能需求」表（分组 = backlog）。
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
