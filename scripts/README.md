# scripts/ — A3 工作区脚本索引

运维、测试、标定、桥接类脚本。分四组：**栈管理**、**测试验收**、**标定调试**、**桥接/工具**。

## 栈管理

| 脚本 | 功能 | 用法 |
|---|---|---|
| `a3_stack.sh` | **真机/仿真栈一键启停 + 重力标定**。真机：can1 检查→电机探测（7/7→7J，1..6 在线缺 L7→自动切 6J 档，F168）→setsid 起栈→健康验证；`calibrate` 子命令收口 F49 模型/F89 比例标定（F169）；`--face-track` 起栈后顺带拉起人脸跟踪（F183，stop/restart 先停它）；仿真：`edge_teleop_full_sim`（domain 99 隔离真机，默认带 PS4 手柄 joy_node） | `./scripts/a3_stack.sh {start\|stop\|restart\|status\|calibrate} [-f] [--rviz] [--no-probe] [--6j] [--keep-rviz] [--sim] [--no-joy] [--face-track]` · `./scripts/a3_stack.sh calibrate {model\|scale}` |
| `a3_shell_env.sh` | ROS 环境 source 脚本（含 PYTHONNOUSERSITE、DISPLAY 处理、micro-ROS prefix 剥离） | `source scripts/a3_shell_env.sh` |
| `setup_face_track_env.sh` | F183 人脸检测环境：建 `~/.a3/venv/face_track`（`--system-site-packages`）+ 装 mediapipe + 下 `face_landmarker.task` 到 `~/.a3/models/`；解决系统 pyparsing/numpy 抢占问题 | `./scripts/setup_face_track_env.sh [--force]` |
| `setup/install_a3_service.sh` | F93 安装 a3-arm.service 到 systemd（只安装，不 enable） | `sudo scripts/setup/install_a3_service.sh` |
| `setup/setup_realtime.sh` | F100 配置 ros2_control 实时调度权限（pam_limits） | `sudo scripts/setup/setup_realtime.sh [USER]` |
| `setup/setup_dds_network.sh` | F105 DDS 网络栈硬化（sysctl 缓冲 + CycloneDDS XML） | `sudo scripts/setup/setup_dds_network.sh [wlan0\|eth0]` |
| `setup/setup_watchdog.sh` | F101 启用硬件看门狗（systemd 喂狗） | `sudo scripts/setup/setup_watchdog.sh` |

## 测试验收

| 脚本/目录 | 功能 | 用法 |
|---|---|---|
| `a3_test/` | **分层测试套件**（F22）：单电机→全栈仿真→真机验收，含 40+ 个 F 系列验收脚本 | 见 `a3_test/README.md`；入口 `./scripts/a3_test/a3_test.sh {env\|hw\|telemetry\|mqtt_cmd\|servo\|gripper\|incident\|all}` |
| `verify_wave_a_sim.sh` | Wave A 仿真验证（Pinocchio 重力 + zero→ready + 双域） | `./scripts/verify_wave_a_sim.sh` |
| `verify_wave_b_sim.sh` | Wave B 仿真验证（F10–F15 + Servo smoke） | `./scripts/verify_wave_b_sim.sh` |
| `dual_domain_zero_to_ready.sh` | 双域仿真（Edge + CloudEdge executor）zero→ready 轨迹扇出对比 | `./scripts/dual_domain_zero_to_ready.sh` |
| `a3_test/f181_servo_track.py` | **F181 servo 跟踪验收**（真机，默认栈即 `a3_servo_target`，臂 READY）：`sweep`=回 home 后三轴 +10→−20→+10cm；`random`=随机轴/方向/速度方形跟踪（60s 或 home±10cm 边界先到先停）。输出命令CV/纹波/跟随误差/回home/停稳漂移，结果落 `~/.a3/servo_track/` | `python3 scripts/a3_test/f181_servo_track.py sweep` · `... random --vmax 0.05 --duration 60 --bound 0.10` |

## 标定调试

| 脚本 | 功能 | 用法 |
|---|---|---|
| `a3_check_zero_frame.py` | F48 开机零位校验：`/joint_states` 读数 vs URDF 限位（硬）+ 期望位姿（软） | `python3 scripts/a3_check_zero_frame.py` |
| `gravity_calibration.py` | F49 7J 重力标定（复刻官方 dynamics_calibration.py 适配真机栈） | `python3 scripts/gravity_calibration.py` |
| `gravity_scale_calibration.py` | F89 重力模型验证 + 逐关节重力 scale 标定 | `python3 scripts/gravity_scale_calibration.py` |
| `mit_noenable_stream.py` | MIT 电机 CAN 无使能探测 + 零增益流式工具（纯标准库，无 ROS） | `python3 scripts/mit_noenable_stream.py probe --iface can1 --ids 1..7` |
| `traj_smoothness_calibration.py` | F136 轨迹平滑度标定脚手架：标准 quintic 轨迹 vs 录制轨迹 + 指标 J + 参数扫描（离线，无 ROS） | `python3 scripts/traj_smoothness_calibration.py --scan v_scaling` |
| `traj_smooth.py` | F137 录制轨迹几何去噪 + 平滑重规划：逐关节五次 B 样条光顺 + quintic 重定时，三列对比 + ε 扫描 + 报告（离线） | `python3 scripts/traj_smooth.py --dir ~/.a3/trajectories --latest 3 --eps 0.01` |
| `face_landmark_overlay.py` | 人脸 6DoF 离线叠加（F183 前置验证）：mediapipe FaceLandmarker 478 点 + 自带姿态矩阵 + One-Euro 滤波，输出 X/Y/Z(mm) 与 YAW/PITCH/ROLL(deg)、建议 twist，绘制关键点/包围盒/死区/滚动曲线 | `env -u PYTHONPATH ~/.a3/venv/face_track/bin/python scripts/face_landmark_overlay.py <video.avi> --z-ref 0.45 --units-mm 14` |
| `melodion_bake.py` | 口风琴 stream 预烘焙：把逐段 via 关节轨迹算好存 `<name>.yaml.baked.yaml`，playback 直接下发跳过实时 IK+重定时 | `python3 scripts/melodion_bake.py --stream scale` |
| `melodion_impact_eval.py` | 口风琴跨键冲击评估（approach/速度 vs j_rms/a_rms/时长） | `python3 scripts/melodion_impact_eval.py` |
| `melodion_speed_diag.py` | 口风琴跨键 via 关节空间 nominal 时长扫参（approach/vmax/amax） | `python3 scripts/melodion_speed_diag.py` |
| `melodion_pts2stream.py` | F176 口风琴实测定位点（路点录制滚动槽 `trajectories/waypoints/latest.yaml`）→ 4 键按压 stream（写 `trajectories/streams/latest.yaml`，长按 Triangle 回放槽；press 6mm + 起手/收尾悬停「第 1 点上方 40mm」+ 键间 3 过点抬升 40/80/40mm、`mode=ptp_blend`、`hold_orientation=true`、speed 0.2，文件头附 F166 九模式速查）+ 打印 D-pad 命名点片段（纯离线，仅 PyYAML） | `python3 scripts/melodion_pts2stream.py [--mode ptp_blend] [--hold-orientation/--no-hold-orientation] [--lift-src 40 --lift-apex 80 --lift-dst 40] [--exact]` |
| `traj_modes_bench.py` | 多路点轨迹模式基准（离线）：ptp/L1/L2/L3/L1.5 × 速度，命令平滑度 j_rms/a_rms + 时长 + SVG/MD | `python3 scripts/traj_modes_bench.py --config src/a3_arm_controller/config/traj_modes_bench.yaml` |
| `gen_stream_modes.py` | 路点轨迹 → 多模式 stream 生成器（离线，不触发运动）：读 waypoints/latest.yaml 取前 6 点，生成 **9 模式**(ptp/ptp_blend/lin_exact/lin_blend/jnt_g64_quintic/jnt_g64_trapezoid/jnt_spline_bell/circ/jnt_spline_approx)×N 速度的 stream（段间自动回起点） | `python3 scripts/gen_stream_modes.py [--speeds 0.4 0.8] [--modes ptp circ] [--out <path>]` |
| `traj_mode_recommend.py` | F165 轨迹模式离线推荐器：给定轨迹算各模式时长/a_rms/j_rms/过冲 + 推荐（最快/最平滑 + 场景提示） | `python3 scripts/traj_mode_recommend.py --poses home_up,home_front,...` |
| `traj_plan_auto.py` | F166 离线 auto 规划器：读含逐点 `exact` + `mode:auto` 的规划配置 → 按 exact 筛可行模式 → 时长/平滑加权择优 → 写最终 stream + 报告 | `python3 scripts/traj_plan_auto.py --config <plan.yaml> [--w-dur 0.5 --w-j 0.5]` |
| `a3_test/traj_modes_tcp_capture.py` | 9 模式 TCP 轨迹在线采集：逐模式回放 + 录 `/joint_states` → JSON | `python3 scripts/a3_test/traj_modes_tcp_capture.py --speed 0.4` |
| `a3_test/traj_modes_tcp_report.py` | 9 模式 TCP 对比报告：FK 到末端 → 位置（等比例 y-z）/速度/加速度 3 张 SVG + 距直线折线偏差 MD | `python3 scripts/a3_test/traj_modes_tcp_report.py --run /tmp/traj_modes_tcp.json` |
| `traj_retime_toppra.py` | F167.2 toppra 离线重定时：速度/加速度 + **力矩约束**（Pinocchio `rnea`）+ 自定义凸约束 `\|W·q̈\|≤c`；输出指标 + baked 轨迹（可回放）。**依赖 toppra**（本机 `sudo pip3 install --no-deps --no-build-isolation toppra`，先装 Cython；顶层 import matplotlib 已内置 stub） | `python3 scripts/traj_retime_toppra.py --waypoints trajectories/waypoints/latest.yaml --out <name>.yaml.baked.yaml` |
| `a3_test/f167_retime_ab.py` | F167.1 A/B：4 软件模式 `goto_via_time_backend`=software vs totg 的时间律指标对比（几何一致性判定） | `python3 scripts/a3_test/f167_retime_ab.py --software <sw.json> --totg <totg.json>` |
| `a3_test/f167_toppra_acceptance.py` | F167.2 验收：toppra 求解 + 力矩在限内 +（`--sim`）baked 仿真回放 | `python3 scripts/a3_test/f167_toppra_acceptance.py --sim` |
| `a3_test/f167_otg_report.py` | F167.3 报告：读 `otg_demo` CSV → 同步/改目标/急停指标 + SVG（numpy-only） | `python3 scripts/a3_test/f167_otg_report.py --dir /tmp --out-dir docs/dev` |

## 桥接/工具

| 脚本 | 功能 | 用法 |
|---|---|---|
| `camera_capture_ui.py` | **F182 相机采集小 UI**（独立于 ROS）：PyQt5 窗口实时预览 USB 相机（直接 V4L2，by-id 设备自动选），拍照（JPEG）+ 开始/停止录制（MJPG AVI），产物落 `capture/face/{images,videos}/`（已 gitignore），供人脸算法取素材。与真机栈 `usb_camera_node` 争用同一相机，二选一 | `python3 scripts/camera_capture_ui.py [--out capture/face/<name>] [--width 1280 --height 720 --fps 30]`；`Space` 拍照、`R` 录制 |
| `ps4/deep_dog_ds4_hid.py` | DS4 HID 报告解析器（deep-dog I06 touchpad + I07 motion） | 库文件，被其他 ps4 脚本引用 |
| `ps4/deep_dog_ds4_touchpad_probe.py` | DS4 touchpad + buttons 并发解析探针（I06 stage-1） | `python3 scripts/ps4/deep_dog_ds4_touchpad_probe.py` |
| `ps4/deep_dog_handle_bridge.py` | PC 手柄 → MQTT handle/input 桥接（deep-dog） | `python3 scripts/ps4/deep_dog_handle_bridge.py` |
| `ps4/ds4_bridge_win.py` | DS4(蓝牙) → TCP JSON 桥接（Windows 侧发送端） | Windows 上运行，配合 Linux 端接收 |

## 常用流程速查

```bash
# 开发日常：起仿真栈验证
./scripts/a3_stack.sh start --sim --rviz
./scripts/a3_stack.sh status --sim
./scripts/a3_stack.sh stop --sim

# 真机运维
./scripts/a3_stack.sh start --rviz
./scripts/a3_stack.sh stop        # 臂未失能会拒绝，加 -f 强制

# 跑测试
./scripts/a3_test/a3_test.sh env         # 环境自检
python3 scripts/a3_test/ps4_sim_test.py  # PS4 全功能仿真（需仿真栈在跑）
python3 scripts/a3_test/f181_servo_track.py sweep   # F181 真机三轴往返（默认栈即可）
python3 scripts/a3_test/f116_random_tour_sim_acceptance.py  # 随机巡游验收
python3 scripts/a3_test/f161_tour_smooth_acceptance.py      # 巡游平滑链 A/B 验收（仿真）
python3 scripts/a3_test/f161_real_acceptance.py             # 巡游平滑链 A/B 验收（真机，需 gate open）
```

## 相关文档

- **真机栈运维手册**：`docs/edge/QUICKSTART.md`
- **PS4 操作员手册**：`docs/edge/PS4_OPERATOR_GUIDE.md`
- **需求追踪**：`docs/edge/REQUIREMENTS.md`
- **经验教训**：`docs/lessons_learned/README.md`
