# LL-015 — F32 回归：单点轨迹「只出 2 帧且无效」——命名轨迹索引回退 + 启动平滑吞帧 + 限速戳共享

> **日期：** 2026-09-06
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ ROS 2 Humble，motor_protocol_node（域 79 无 CAN 验证）

## 现象

F32 改动后真机台架：面板 L7 夹爪指令（gripper 控制器发的**单点轨迹**，t=0.05 s）下发后，`/can_tx_frames` 里 traj 帧只出 2 帧且「无效」（电机不动），而 refresh 帧正常。QoS 被首先怀疑（F32 把 traj 订阅 QoS 注释写错成「reliable 不能匹配 best-effort 发布」），但实测 DDS 上 BE 订阅者两种发布都能匹配、reliable 订阅者只匹配 reliable——轨迹送达本身没问题（traj_cb=1）。

## 根因（三层叠加）

1. **命名轨迹索引回退把目标复制给 L1**：`ApplyPositionTargets` 按关节名匹配失败时回退到 `positions[i]`——对单关节 L7 命名轨迹，L1 也拿到 L7 的目标并被驱动（单电机台架无 L1，帧被当「无效」）。索引回退只应对**无名轨迹**（use_names=false）使用。
2. **启动平滑吞掉短轨迹**：`enable_startup_smoothing=true` 时平滑器从 boot 反馈起步、2 s 爬升、每次调用步长 clamp 到 `command_max_velocity_rad_s × dt`。0.05 s 的单点轨迹只把斜坡推进 ~2.5%，L7 帧携带 ≈0 目标 → 电机不动 → refresh 以 kp=80 保持 ≈0。
3. **traj 与 refresh 共享限速戳**：两者共用 `last_tx_pub_stamp_ns_`，refresh 2 ms 定时器不断刷新戳，traj 帧在 200 Hz 限速器下被大量跳过（单 tick 轨迹几乎必被跳）。需分离 `last_refresh_stamp_ns_`。
4. **master_id(0xFD) 是 RX-only**：只出现在反馈帧里，与 TX 无交互，不是本问题的一部分（排除项）。

## 正确做法 / 规避

- 命名轨迹按名匹配失败 = 该关节不在轨迹内，走 `has_latest_input_` 兜底，**绝不**用 `positions[i]` 喂第一个关节。
- 单点 jog 轨迹：轨迹「结束」后继续按最后一点保持，直到平滑收敛（`|smoothed-limited| ≤ 1e-3`）才清 `has_active_traj_`；保持上限 10 s 防卡死。
- refresh 与 traj 各用各的戳；判断「traj 帧饿死」先看 `skip_max_rate` 窗口日志，别先怀疑 QoS。
- 无 CAN 验证用域隔离 + `--ros-args -r __node:=`（LL-006），15 字节帧解码见 `FrameCodec::Pack`（`[0]=bus [1]=ext [2]=dlc [3..6]=can_id BE [7..14]=data`）——按 12 字节解全是幻觉数据。
- pkill 模式会匹配自己 shell 命令行，用 `pkill -f "repro7[9]"` 字符类技巧（LL-006/LL-014 已记，第三次再犯）。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`（`ApplyPositionTargets`/`OnTrajectoryInterpTimer`/`OnTxRefreshTimer`/`SendMitFrame`）
- `src/a3_gripper_controller/a3_gripper_controller/gripper_controller_node.py`（`_publish_traj`：面板 L7 单点轨迹源头）
- 关联：LL-014（同一事故的另一半——流送器不终止）；F32（本回归的引入侧）

**真机验收（2026-09-06）**：修复后新二进制整组重启 + 24V 断电清 temp_error 锁存后，force_web 阶梯 6 PASS / 0 FAIL——traj_cb=250/tx_traj=250 轨迹帧全部送达（戳分离与保持修复生效），0.3→0.281、0.5→0.508 GRASPED、force 0 3 s 回零、全程无 FAULT、结束 mode=2/temp=44°C/err=0。附带观察：短时断电多圈计数保留（硬止位 0.0056 rad），与「断电必丢多圈」旧认知不符，见 QUICKSTART L7 标定注记。
