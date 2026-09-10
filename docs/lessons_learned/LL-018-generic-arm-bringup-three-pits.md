# LL-018 — 通用 N 关节臂接入三坑：无播种无反馈、新 yaml 不重编 build、/joint_states 订阅 QoS

> **日期：** 2026-09-11
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ can1 @ 1 Mbps + 非 A3 的 6 关节 MIT 臂（无夹爪，F38）

## 现象

can1 换接非 A3 机械臂跑 F38 init/示教/回放流程，依次踩到三个问题：

1. can_bridge 启动后（`use_power_sequence:=false`，无任何命令）`/a3/motor/states` 全部 `has_feedback: false, fresh: false`，总线 RX 计数为 0；init 前看起来「电机全不在线」。
2. 新建 `a3_arm_controller/config/arm_controller_6j.yaml` 后 `colcon build --symlink-install` 失败：`error: [Errno 2] No such file or directory: '.../install/a3_arm_controller/share/.../arm_controller_6j.yaml'`。
3. 回放验证脚本订阅 `/joint_states` 收不到任何消息，告警 `offering incompatible QoS. Last incompatible policy: RELIABILITY`。

## 根因

1. EL05 电机在 mode=0（未使能）和 mode=2（使能）下都**不会自发上报反馈**，只在收到 MIT 控制帧后应答；`motor_protocol_node` 的 refresh 流只在 `last_commanded_mit_rad_` 已播种（有限值）时发帧，而它只在轨迹经 `SendMitFrame` 后才有值 → 无命令时桥完全静默，反馈链断。真 A3 流程里 init 的 set_zero/enable 广播命令本身会引发一次应答，故计数能过，但连续遥测仍依赖播种。
2. `a3_arm_controller/setup.py` 用 `glob("config/*.yaml")` 列装文件，glob 针对的是**build 目录里的副本**（`build/a3_arm_controller/config/`），新加的 yaml 不在旧 build 目录中 → 安装阶段找不到文件。`--symlink-install` 不会刷新 build 目录的文件清单。
3. can_bridge 发布 `/joint_states` 是 **BEST_EFFORT**（F32 的 QoS 对齐约定），默认订阅器 RELIABLE 不兼容，直接收不到消息。

## 正确做法 / 规避

1. **init 之后立刻「播种」**：发一条当前位置的单点 `JointTrajectory`（`/joint_group_effort_controller/joint_trajectory`，时长 0.1s），`SendMitFrame` 填上 `last_commanded_mit_rad_`，refresh 流开始 50Hz 持续发帧，反馈/遥测全链恢复。配置了 `enable_startup_smoothing` 时播种前必须关掉它（否则 boot_feedback 是 set_zero 前旧值，播种会触发 2s 伪斜坡 → 真实运动；F38 的 `control_gains_generic.yaml` 已置 false）。
2. 新增/改名任何被 `setup.py` glob 收集的文件后，**删掉 `build/<pkg>` 再重编**（或 `colcon build --cmake-clean-cache` 同类手段），不要只跑 `--symlink-install`。
3. 订阅 `/joint_states` 用 `QoSProfile(depth=…, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)`；测试脚本同款坑（`scripts/a3_test/arm6_playback_verify.py` 已按此修正）。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`（OnTxRefreshTimer 的 NaN 目标跳过 → 无播种静默）
- `src/a3_arm_controller/setup.py`（config glob 对 build 目录生效）
- `src/a3_can_bridge/config/control_gains_generic.yaml`（F38 专用配置，关 startup smoothing）
- `scripts/a3_test/arm6_playback_verify.py`（BEST_EFFORT 订阅示例）
