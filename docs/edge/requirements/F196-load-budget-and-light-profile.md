# F196 — 运行负载预算与降载档位（/joint_states 降频中转 + 按需附加节点）

- **说明：** 真机栈在 RK3588（8 核：4×A55@1.8G + 4×A76@2.3G）上常驻负载偏高：
  实测 `load1` 7.7–14、ROS 节点合计约 5.6 核，空闲时也会挤压 200 Hz 控制环（LL-180
  feedback stale / 命令台阶）。根因不是单点，而是四类叠加：
  1. `/joint_states` 跟随 `controller_manager.update_rate` 发布 **200 Hz**（Humble 的
     `joint_state_broadcaster` **不支持 `publish_rate`**，YAML 里的 100.0 被静默忽略），
     同时被 6 个 Python 节点各订阅一次，每个订阅者仅执行器唤醒就要 25–35% 单核。
  2. 视觉链常驻：相机 + 显示合成 + 人脸检测/跟踪约 1.0–1.4 核，且人脸检测在
     **未开启跟踪时仍逐帧跑 MediaPipe**。
  3. 非必需节点默认常开：`angular_cruise`（约 40%）、RViz（约 32%）、rosbag（约 13%）、
     topic-rate（约 33%）、watchdog（手动栈约 33%）、self-test（约 13%）。
  4. 热路径重复开销：每帧重建 dict、每 tick 读参数/重建手柄绑定、200 Hz 回调里构造
     ROS 时间对象、`MultiThreadedExecutor` 默认 8 线程 + 单一可重入回调组。

  本需求落地「负载预算 + 降载档位」：控制链保持 200 Hz 不变；新增 C++ 中转
  `/joint_states_lite`（50 Hz）供低频消费者使用；附加节点改为按需启动；修正
  F104 频段误报；并提供 `a3_stack.sh --light/--vision/--full/--demo` 入口。

- **改动：**
  1. `src/a3_trajectory_processing/src/joint_state_relay_node.cpp`：订阅 `/joint_states`
     （RELIABLE depth 1），按 `rate_hz`（默认 50）转发到 `/joint_states_lite`
     （BEST_EFFORT depth 1）；输入停更时不再转发；每 5 s 向 `/diagnostics` 发布原始率
     （默认带宽 180–220 Hz）。
  2. `src/a3_bringup/launch/a3_bringup.launch.py`：新增 `use_joint_state_lite`（默认 true）
     与 `use_demo_nodes`（默认 false）；低频消费者 `ee_pose` / `gravity_torque` /
     `systemd_watchdog_feed` / `a3_self_test` / `topic_rate_monitor` 改订 lite；
     `angular_cruise` 只在 `use_demo_nodes:=true` 时启动；黑匣子去掉冗余诊断话题。
  3. `src/a3_bringup/a3_bringup/systemd_watchdog_feed_node.py`：无 `NOTIFY_SOCKET` 时
     不创建 `/joint_states` 订阅（手动栈不再为空转节点付 200 Hz 开销）。
  4. `src/a3_bringup/a3_bringup/topic_rate_monitor_node.py`：默认频段改为 200 Hz 口径；
     launch 在 lite 模式下监测 50 Hz 的 `/joint_states_lite`（[40,60]）。
  5. `src/a3_bringup/a3_bringup/ps4_mapper.py` + `actions.py` + `mapping.py`：删除
     200 Hz `/joint_states` 订阅（L7 实测角改从 10 Hz `/a3/arm_status` 取）；切模式时
     缓存轴/按键绑定；gate 每 tick 只求值一次；servo/gripper 就绪探测失败后退避 1 s。
  6. `src/a3_bringup/a3_bringup/ee_pose_publisher.py` / `gravity_torque_node.py`：
     缓存 name→index 映射、复用 Pinocchio 缓冲，200 Hz 回调不再查模型/分配缓冲。
  7. `src/a3_bringup/a3_bringup/vision_display_mux.py`：仅在 `/a3/vision/display` 有订阅者时
     才懒建 `/camera/image_raw` 订阅（无观察者时连反序列化都不做）；新增 `max_fps`；
     `usb_camera_node` 无图像订阅者时跳过封装/发布，`camera_info` 降到 1 Hz +
     `CAP_PROP_BUFFERSIZE=1`。
  8. `src/a3_perception/a3_perception/face_detect_node.py`：订阅 `/a3/vision/mode`，
     未进入 `face_track` 时不调用 MediaPipe（`require_tracking_mode` 默认 true）。
  9. `src/a3_arm_controller/a3_arm_controller/arm_controller.py`：执行器 2 线程；高频
     镜像订阅独立互斥组；joint/torque 名称映射缓存；200 Hz 回调不再构造 ROS 时间；
     高频参数缓存（运行时 `ros2 param set` 仍生效）。
  10. `src/a3_description*/config/el_a3_controllers.yaml`：JTC `state_publish_rate`
      200 → 50（实测 `/arm_controller/state` 0 订阅者）。
  11. `scripts/a3_stack.sh`：新增 `--light/--vision/--full/--demo` 与
      `--no-diagnostics/--no-rosbag/--no-self-test`；`NODE_PATTERN` 补 `joint_state_relay`。

- **验收标准：**
  1. 待机/伺服（`--light`，无视觉/无 RViz）：ROS 节点合计 ≤2.5 核，`load1 < 4`。
  2. 常规联调（相机 + 视觉显示，无人脸）：≤3.2 核。
  3. 视觉跟踪全开（相机 + 人脸 + 显示 + RViz）：≤3.8 核，`load1 < 6`。
  4. `/joint_states` 保持 198–200 Hz、max gap <30 ms；`/joint_states_lite` 50 Hz ±5%。
  5. 60 s 伺服跟踪无 `feedback stale`、无 `CAN write failed`；`f181` / `f186` / `f193` 回归通过。
  6. `/diagnostics` 中 `/joint_states_lite` 为 “Desired frequency met”；relay 原始率
     180–220 Hz 为 OK；不再出现 “Frequency too high” 误报。
  7. `use_joint_state_lite:=false` 可一键回退（消费者回到原始 200 Hz）。

- **实测（2026-10-26，6J 真机，`restart --no-rosbag`）：**

  | 进程 | 优化前 %core | 优化后 %core |
  |---|---:|---:|
  | `arm_controller` | 68 | 58–66 |
  | `ps4_mapper` | 56 | 21 |
  | `gravity_torque` | 44 | 11 |
  | `ee_pose` | 31 | 8 |
  | `topic_rate_monitor` | 32 | 7 |
  | `systemd_watchdog_feed` | 32 | 0（手动栈无订阅） |
  | `a3_self_test` | 13 | 2 |
  | `vision_display_mux` | 47 | 6（无 RViz 订阅者时懒订阅） |
  | `usb_camera` | 41 | 11 |
  | `angular_cruise` | 41 | 0（默认关） |
  | `ros2_control` | 22 | 7 |
  | ROS 节点合计 | ≈565% | ≈165% |

  `arm_controller` 剩余约 62% 主要是 200 Hz Python 订阅的固有执行器成本；按本需求
  决策保留 200 Hz（示教录制/限位校验），后续如需再降，可评估「示教时临时订阅原始
  200 Hz、其余走 lite」的懒订阅方案。

- **关联：** F104（话题频率诊断）、F101（watchdog）、F142（关节状态频率口径）、
  F175（ee_pose）、F183/F194（视觉链）、F193（demo 节点）、LL-180（CPU 抢占→stale）、
  LL-196（孤儿节点累积）、LL-197（JSB `publish_rate` 静默无效）
- **状态：** `in progress`（P0/P1/P2 已落地并实测；待 f181/f186/f193 真机回归与
  systemd（`NOTIFY_SOCKET`）路径复测）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
