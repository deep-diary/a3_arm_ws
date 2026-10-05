# scripts/ — A3 工作区脚本索引

运维、测试、标定、桥接类脚本。分四组：**栈管理**、**测试验收**、**标定调试**、**桥接/工具**。

## 栈管理

| 脚本 | 功能 | 用法 |
|---|---|---|
| `a3_stack.sh` | **真机/仿真栈一键启停**。真机：can1 检查→电机 7/7 探测→setsid 起栈→健康验证；仿真：`edge_teleop_full_sim`（domain 99 隔离真机，默认带 PS4 手柄 joy_node） | `./scripts/a3_stack.sh {start\|stop\|restart\|status} [-f] [--rviz] [--no-probe] [--keep-rviz] [--sim] [--no-joy]` |
| `a3_shell_env.sh` | ROS 环境 source 脚本（含 PYTHONNOUSERSITE、DISPLAY 处理、micro-ROS prefix 剥离） | `source scripts/a3_shell_env.sh` |
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

## 标定调试

| 脚本 | 功能 | 用法 |
|---|---|---|
| `a3_check_zero_frame.py` | F48 开机零位校验：`/joint_states` 读数 vs URDF 限位（硬）+ 期望位姿（软） | `python3 scripts/a3_check_zero_frame.py` |
| `gravity_calibration.py` | F49 7J 重力标定（复刻官方 dynamics_calibration.py 适配真机栈） | `python3 scripts/gravity_calibration.py` |
| `gravity_scale_calibration.py` | F89 重力模型验证 + 逐关节重力 scale 标定 | `python3 scripts/gravity_scale_calibration.py` |
| `mit_noenable_stream.py` | MIT 电机 CAN 无使能探测 + 零增益流式工具（纯标准库，无 ROS） | `python3 scripts/mit_noenable_stream.py probe --iface can1 --ids 1..7` |
| `traj_smoothness_calibration.py` | F136 轨迹平滑度标定脚手架：标准 quintic 轨迹 vs 录制轨迹 + 指标 J + 参数扫描（离线，无 ROS） | `python3 scripts/traj_smoothness_calibration.py --scan v_scaling` |
| `traj_smooth.py` | F137 录制轨迹几何去噪 + 平滑重规划：逐关节五次 B 样条光顺 + quintic 重定时，三列对比 + ε 扫描 + 报告（离线） | `python3 scripts/traj_smooth.py --dir ~/.a3/trajectories --latest 3 --eps 0.01` |

## 桥接/工具

| 脚本 | 功能 | 用法 |
|---|---|---|
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
python3 scripts/a3_test/f116_random_tour_sim_acceptance.py  # 随机巡游验收
```

## 相关文档

- **真机栈运维手册**：`docs/edge/QUICKSTART.md`
- **PS4 操作员手册**：`docs/edge/PS4_OPERATOR_GUIDE.md`
- **需求追踪**：`docs/edge/REQUIREMENTS.md`
- **经验教训**：`docs/lessons_learned/README.md`
