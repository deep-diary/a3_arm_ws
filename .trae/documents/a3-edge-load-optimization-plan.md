# A3 Edge 运行负载优化方案

> 目标：在不降低控制链路质量（200 Hz 控制环、运动安全门、轨迹平滑）的前提下，把空闲/常规运行的 CPU 负载从 **约 6.4 核（load1 7.7）** 降到 **≤2.5 核（load1 < 4）**，视觉跟踪全开时 **≤3.8 核**。

## 0. 结论摘要（TL;DR）

当前负载不是单点问题，而是 4 类叠加：

1. **`/joint_states` 以 200 Hz 发布，被 6 个 Python 节点各订阅一次**，每个节点仅“收消息 + 唤醒执行器”就要 25–35% 单核；这是最大的系统性税。
2. **视觉链常驻且按需性差**：相机 41% + 显示合成 46% + 人脸检测 31% + 跟踪控制 25% ≈ 1.4 核，其中人脸检测/跟踪在**未开启跟踪时也在跑**。
3. **常驻但不是当前任务必需的节点太多**：RViz 31%、`angular_cruise` 41%、rosbag 12%、topic-rate 33%、watchdog 35%、self-test 12%。
4. **热路径实现有重复开销**：每帧重建 dict、每 tick 读参数、每 tick 重建手柄绑定、每 10 Hz 同步写 YAML、`MultiThreadedExecutor` 默认 8 线程 + 单一 Reentrant 组。

另外发现两个**静默失效配置**（不报错但行为不是配置想要的）：

- Humble 的 `joint_state_broadcaster` **不支持 `publish_rate` 参数**：`el_a3_controllers.yaml` 写了 `publish_rate: 100.0`，实测 `/joint_states` 仍是 **198 Hz**（200 Hz 控制周期原样发布）。
- F104 频率门限仍是 `[36, 66] Hz`，对 187 Hz 实测**永久报 WARN “Frequency too high”**，白烧 33% CPU 且污染诊断。

实施顺序：**P0 运行期降载（当天可做，不改代码）→ P1 低风险代码降本 → P2 结构性降频/节点合并 → P3 调度与核绑定**。

---

## 1. 实测基线（2026-10-26 21:30–21:40，RK3588 8 核）

启动参数（实测命令行）：

```text
ros2 launch a3_bringup a3_bringup.launch.py hardware:=can can_interface:=can1 \
  use_rviz:=true include_gripper:=false use_servo_target:=true use_camera:=true \
  use_mqtt:=false camera_overlay_fps:=false
# 另起：a3_perception face_track.launch.py（人脸检测 + 跟踪控制）
```

系统：`load average 7.76 / 11.23 / 14.19`（8 核），内存 5.1 GiB / 7.7 GiB。
CPU 拓扑：cpu0–3 = A55 @1.8 GHz，cpu4–7 = A76 @2.3 GHz，governor=`performance`。

### 1.1 单进程占用（% 单核，3 s 采样）

| 进程 | %core | 归属 |
|---|---:|---|
| `arm_controller` | 68.0 | 编排层（Python） |
| `ps4_mapper` | 56.0 | 遥操作（Python） |
| `vision_display_mux` | 47.0 | 视觉显示合成 |
| `gravity_torque_node` | 43.7 | 重力前馈 |
| `usb_camera_node` | 41.0 | 相机 |
| `angular_cruise` | 40.6 | DEMO 节点（当前未使用） |
| `topic_rate_monitor` | 32.3 | F104 诊断 |
| `systemd_watchdog_feed` | 32.3 | F101（手动栈下 NOTIFY_SOCKET 未设置） |
| `rviz2` | 31.7 | 可视化 |
| `ee_pose_publisher` | 31.3 | F175 |
| `face_detect_node` | 30.7 | MediaPipe |
| `face_track_controller` | 25.3 | F183 |
| `ros2_control_node` | 22.3 | 控制环 200 Hz |
| `move_group` | 17.0 | MoveIt |
| `servo_target_node` | 13.0 | F181 |
| `ros2 bag record` | 13.0 | F90 黑匣子 |
| `a3_self_test` | 12.7 | F95 |
| `robot_state_publisher` | 11.3 | TF |
| `ds4_feedback_node` | 10.3 | 手柄反馈 |
| `joy_node` / `servo_mode_bridge` / `power_sequence` / `can_bus_monitor` | 7.3 / 6.7 / 6.3 / 5.7 | 辅助 |

ROS 节点合计 **≈565%**（5.65 核）；Trae IDE/桌面另占约 0.5–1 核。

### 1.2 关键实测事实

- `/joint_states`：`average rate ≈ 198 Hz`，max gap 14 ms。
- `/joint_states` 订阅者 11 个，其中 **6 个 Python**：`arm_controller`、`ps4_mapper`、`gravity_torque`、`ee_pose`、`systemd_watchdog_feed`、`topic_rate`。
- `/arm_controller/state`：**订阅者 0** → JTC `state_publish_rate: 200` 是纯浪费。
- `/a3/vision/display`：订阅者 1（RViz）→ 没人看时整条绘制/发布链都可以跳过。
- `/diagnostics` 实测：`a3_topic_rate: /joint_states topic status` → `Frequency too high`，实际 186.7 Hz，允许上限 66 Hz。
- `systemd_watchdog_feed` 进程环境无 `NOTIFY_SOCKET`（非 systemd 启动），但仍订阅 200 Hz。

---

## 2. 根因分析

| 编号 | 根因 | 证据 | 影响 |
|---|---|---|---|
| R1 | 200 Hz 关节状态 + 多 Python 订阅者 | JSB 每个控制周期发布；6 个 Python 订阅 | 约 2.0–2.3 核 |
| R2 | 视觉链常驻、未按需启停 | 人脸检测在跟踪未开启时仍逐帧跑 MediaPipe | 约 1.0–1.4 核 |
| R3 | 非必需节点默认常开 | `angular_cruise` 受 `use_teleop` 门控；rviz/rosbag/diagnostics 默认开 | 约 1.2 核 |
| R4 | 热路径重复计算 | 每帧 dict/参数查询/绑定重建/同步落盘 | 约 0.5–0.8 核 |
| R5 | 执行器结构放大 | `MultiThreadedExecutor()` 默认 8 线程 + 所有高频订阅共用一个 Reentrant 组 | 约 0.3–0.6 核 |
| R6 | 无核绑定/优先级隔离 | 控制环与视觉/UI/IDE 抢同一批核（A55/A76 混跑） | 造成尖峰与 stale，不一定增加总 CPU |
| R7 | 静默失效配置 | JSB `publish_rate` 被忽略；F104 频段未跟随 F142 变更 | 频率失控 + 永久误报 |
| R8 | 任务态未区分 | 伺服验收 / 视觉演示 / 待机 共用同一套常驻节点 | 每种场景都背全部成本 |

---

## 3. 优化方案清单（按收益/风险排序）

| 编号 | 方案 | 预期节省 | 风险 | 阶段 |
|---|---|---:|---|---|
| O1 | 视觉链按需启停（不需要时停 face 两节点 + 相机 + 显示） | −130~−175% | 低 | P0 |
| O2 | 不常开 RViz（需要时再开） | −32% | 低 | P0 |
| O3 | `angular_cruise` 改为显式 `--demo` 才启动 | −41% | 低 | P0/P1 |
| O4 | 手动栈下跳过 watchdog 的 `/joint_states` 订阅 | −33% | 低 | P1 |
| O5 | 修正 F104 频段（或改监测轻量话题） | −25~−32%（P2 后） | 低 | P1/P2 |
| O6 | `ps4_mapper` 去掉 200 Hz 订阅 + 缓存绑定/gate + 服务探测退避 | −35~−40% | 低 | P1 |
| O7 | `ee_pose` / `gravity` 缓存关节索引、复用缓冲 | −40~−45% | 低 | P1 |
| O8 | `arm_controller` 执行器 2 线程 + 订阅分组 + 参数缓存 | −20~−25% | 中 | P1 |
| O9 | `vision_display_mux` 无订阅者直接跳过 + 限制显示帧率 | −30~−45% | 低 | P1 |
| O10 | 相机 15 fps / `camera_info` 1 Hz（轻载档） | −13% | 低 | P1 |
| O11 | 人脸检测按 `/a3/vision/mode` 门控（未跟踪时只收帧不推理） | −30% | 低 | P1 |
| O12 | JTC `state_publish_rate` 200→50（无订阅者） | −4% | 低 | P1 |
| O13 | 黑匣子默认关或裁减录制话题 | −6~−12% | 低 | P1 |
| O14 | 新增 C++ `/joint_states_lite` 50 Hz 中转，迁移低速率消费者 | −60~−90% | 中 | P2 |
| O15 | 诊断节点合并 / `servo_mode_bridge` 并入 servo_target | −10~−15% | 中 | P2 |
| O16 | 统一 CycloneDDS + 缓冲配置 | −5~−15%（待实测） | 中 | P2 |
| O17 | CPU 亲和：控制链绑 A76，视觉/UI 绑 A55；控制线程 RT | 提升确定性 | 中 | P3 |
| O18 | 新增 `a3_stack.sh --light/--demo/--vision` 档位 | 把 P0 固化成入口 | 低 | P0/P1 |

---

## 4. 各方案落地细节

### P0 — 运行期降载（不改代码，先做）

**O1 视觉链按需启停**

```bash
# 不跑视觉时
./scripts/a3_stack.sh restart            # 默认不带 --face-track
# 若已单独起过人脸跟踪
pkill -TERM -f 'a3_perception.face_track.launch|a3_perception.face_detect_node|lib/a3_perception/face_track_controller'
# 完全不需要画面时（伺服/平滑度验收）
./scripts/a3_stack.sh restart --no-camera
```

- 人脸跟踪默认**不使能**，但节点仍在推理：未跟踪时停掉整条链是最直接的降载。
- 伺服/平滑度验收遵循 [LL-180](../../docs/lessons_learned/LL-180-servo-move-jerk-cpu-starvation-feedback-stale.md)：用精简栈（`--no-camera --no-mqtt`，不 `--rviz`）。

**O2 不常开 RViz**：仅在需要看画面时另开；关掉后 `/a3/vision/display` 订阅者变 0，配合 O9 可再省 46%。

**O3 `angular_cruise` 默认不启动**：当前只要 `use_teleop:=true` 就启动，实测空转 41%。

**O13 黑匣子**：只在真机高风险动作（新轨迹/新 IK/标定）时开；日常待机可用 `use_rosbag:=false`。

**O18 固化档位**（在 [a3_stack.sh](../../scripts/a3_stack.sh) 增加参数，透传给 launch）：

| 档位 | 参数 | 用途 |
|---|---|---|
| `--light` | `use_camera:=false use_diagnostics:=false use_self_test:=false use_rosbag:=false use_rviz:=false` | 伺服/平滑度验收、待机 |
| `--vision` | 默认 + 相机 + 视觉显示（不带人脸） | 看画面 |
| `--face-track` | 现有参数 | 人脸跟踪 |
| `--demo` | `use_demo_nodes:=true` | 角度巡航演示 |
| `--full` | 全开（含 rviz/rosbag/diagnostics） | 联调 |

> 预期：P0 全做后，常规待机从 6.4 核降到 **约 3.1 核**（再关 IDE/浏览器可到 2.6 核）。

---

### P1 — 低风险代码降本

#### O4 `systemd_watchdog_feed` 手动栈不订阅
文件：[systemd_watchdog_feed_node.py](../../src/a3_bringup/a3_bringup/systemd_watchdog_feed_node.py#L48-L55)

- 现在无论有没有 `NOTIFY_SOCKET` 都创建 `/joint_states` 订阅（200 Hz）。
- 改法：`if self.sock is not None:` 才 `create_subscription`；无 socket 时只保留 2 s 定时器与日志。
- 效果：手动栈 35% → ~1%；systemd 启动时行为不变。
- P2 再把它改订阅 `/joint_states_lite`。

#### O5 F104 频段修正
文件：[topic_rate_monitor_node.py](../../src/a3_bringup/a3_bringup/topic_rate_monitor_node.py#L27-L31)、[a3_bringup.launch.py](../../src/a3_bringup/launch/a3_bringup.launch.py#L766-L772)

- 现状：默认 `[40, 60]`（容差 10% → 实测 36–66），而 `/joint_states` 是 200 Hz → 永久 WARN。
- 改法（P1）：默认 `min_freq=[180.0]`、`max_freq=[220.0]`，或在 launch 里显式传参。
- 改法（P2）：监测 `/joint_states_lite`，频段 `[40, 60]`；由 C++ 侧（`a3_self_test`）统计原始 200 Hz 速率。

#### O6 `ps4_mapper` 去掉 200 Hz 依赖
文件：[actions.py](../../src/a3_teleop_ps4/a3_teleop_ps4/actions.py#L214-L235)、[ps4_mapper.py](../../src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py#L364-L395)、[mapping.py](../../src/a3_teleop_ps4/a3_teleop_ps4/mapping.py#L345-L379)

1. 删除 `create_subscription(JointState, "/joint_states", ...)`；`_q[6]` 唯一用途是 `_gripper`（[actions.py:942](../../src/a3_teleop_ps4/a3_teleop_ps4/actions.py#L942) 比较 0.9），改为在 `_on_arm_status` 里读 `msg.positions[6]`（`ArmStatus` 明确按 `joint_names` 顺序，10 Hz 足够）。
2. 在 `_commit_mode` 里预先展开并缓存 `self._axis_bindings` / `self._button_bindings`，50 Hz tick 只遍历缓存。
3. 每 tick 先求值一次 gate 键（`l1/r1/share/options`…）缓存成 dict，避免 ~800 次 `button()`/s。
4. `service_is_ready()` / `server_is_ready()` 失败后退避到 1 Hz。
5. `/a3/ds4/imu`、`/a3/ds4/touch` 改 `BEST_EFFORT`；删除 `/a3/teleop/mode` 自回声订阅（改为 service/参数入口）。
- 效果：56% → 15–20%。

#### O7 `ee_pose` / `gravity_torque` 缓存索引与缓冲
文件：[ee_pose_publisher.py](../../src/a3_bringup/a3_bringup/ee_pose_publisher.py#L83-L94)、[gravity_torque_node.py](../../src/a3_bringup/a3_bringup/gravity_torque_node.py#L306-L328)

- 首次收到 `msg.name` 时建立 `name → index` 并缓存；仅当 name 序列变化时重建（JSB 顺序固定）。
- `gravity_torque`：`pin.neutral()` 的结果只算一次，tick 内复用 `q` 缓冲；避免每 tick 分配。
- 效果：各 31/44% → 10/22%。

#### O8 `arm_controller` 执行器与热路径
文件：[arm_controller.py:1105](../../src/a3_arm_controller/a3_arm_controller/arm_controller.py#L1104-L1110)、[`:1486-1526`](../../src/a3_arm_controller/a3_arm_controller/arm_controller.py#L1486-L1526)、[`:2981-3010`](../../src/a3_arm_controller/a3_arm_controller/arm_controller.py#L2981-L3010)、[`:3022-3122`](../../src/a3_arm_controller/a3_arm_controller/arm_controller.py#L3022-L3122)、[`:6905`](../../src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6905)

1. `MultiThreadedExecutor(num_threads=2)`（8 线程抢 GIL 是纯开销）。
2. 高频镜像订阅（`/joint_states`、`motor_states`、`torque_components`）放进独立 `MutuallyExclusiveCallbackGroup`；服务/客户端保持 Reentrant 组（不破坏 `_wait_future` 语义）。
3. `_on_js` 缓存 name→index；`get_clock().now()` 从 200 Hz 回调移到 10 Hz status；`_on_torque_components` 用固定顺序整体切片。
4. 高频回调里的 `get_parameter`（`torque_stats_max_nm`、温度阈值等）改为 `__init__` 缓存。
5. `_publish_status` 复用 `ArmStatus` 实例与数组；`_save_torque_stats` 的 YAML 落盘移出 10 Hz status 回调（独立 1 Hz 定时器 + 快照）。
- 效果：68% → 45% 左右；同时降低回调互相阻塞导致的 status 抖动。

#### O9 `vision_display_mux` 无观察者即短路
文件：[vision_display_mux.py:197-212](../../src/a3_bringup/a3_bringup/vision_display_mux.py#L197-L212)、[`:455-465`](../../src/a3_bringup/a3_bringup/vision_display_mux.py#L455-L465)

- `_on_raw` 开头：`if self.pub.get_subscription_count() == 0: return`（没人看就不解码/绘制/发布）。
- 新增 `max_fps` 参数（轻载 15），低于间隔直接丢帧。
- `_publish` 复用 `Image` 实例与 `array.array` 缓冲，避免每帧新建。
- 跟踪关闭时仅透传时不画网格/文本（已有部分逻辑）。
- 效果：无人看 47% → 接近 0；有人看 47% → 30% 左右。

#### O10 相机轻载
文件：[usb_camera_node.py:39-L46](../../src/a3_bringup/a3_bringup/usb_camera_node.py#L39-L46)、[`:155-172`](../../src/a3_bringup/a3_bringup/usb_camera_node.py#L155-L172)

- 轻载档 `framerate=15`（launch 已可传）。
- `camera_info` 从每帧发布改为 1 Hz（RViz 不需要 25 Hz 内参）。
- 采集线程设置 `CAP_PROP_BUFFERSIZE=1`，减少旧帧堆积。
- 效果：41% → 28%。

#### O11 人脸检测按模式门控
文件：[face_detect_node.py:267-291](../../src/a3_perception/a3_perception/face_detect_node.py#L267-L291)

- 新增订阅 `/a3/vision/mode`（`face_track_controller` 1 Hz 重发，已存在）；非 `face_track` 时 `_worker` 只清空帧、不调用 MediaPipe。
- `face_track.launch.py` 保持两节点常起（服务一调即可用），但未使能时推理成本≈0。
- 效果：未跟踪时 31% → 3%；跟踪时保持不变。

#### O12 JTC 状态发布降频
文件：[el_a3_controllers.yaml:59](../../src/a3_description_6j/config/el_a3_controllers.yaml#L59)

- `/arm_controller/state` 订阅者为 0，`state_publish_rate: 200 → 50`。
- 保留 50 供调试/未来 Web；`action_monitor_rate` 不变。

#### O13 黑匣子裁剪
文件：[a3_bringup.launch.py:777-795](../../src/a3_bringup/launch/a3_bringup.launch.py#L777-L795)

- 去掉 `/diagnostics_agg`、`/diagnostics_toplevel_state`（可由 `/diagnostics` 复算）。
- 轻载档默认 `use_rosbag:=false`；高风险动作再开。

---

### P2 — 结构性降频与节点合并

#### O14 `/joint_states_lite`（核心）
- 在 [a3_trajectory_processing](../../src/a3_trajectory_processing/CMakeLists.txt)（C++，已依赖 `sensor_msgs`）新增 `joint_state_relay_node`：
  - 订阅 `/joint_states`（`RELIABLE` + `depth=1`）；
  - 以 `rate_hz`（默认 50）把最新一条转发到 `/joint_states_lite`（`BEST_EFFORT`，复用消息对象）；
  - 同时统计原始 200 Hz 到达率，按 `/diagnostics` 发布（承接 F104）。
- [a3_bringup.launch.py](../../src/a3_bringup/launch/a3_bringup.launch.py) 在 JSB 之后起该节点（`use_joint_state_lite` 默认 true）。
- 迁移到 lite：`ee_pose`、`gravity_torque`、`systemd_watchdog_feed`、`topic_rate_monitor`、`a3_self_test`、`ps4_mapper`（若 O6 未完全去掉）。
- **保留 200 Hz**：`servo_target_node`、JTC、`arm_controller`（示教录制/限位校验）。
- 效果：上述 Python 节点再降 60–75%；预计再省 **0.6–0.9 核**。
- 文档同步：[TOPIC_CONTRACT.md](../../docs/shared/TOPIC_CONTRACT.md) 新增话题；[F104 需求](../../docs/edge/requirements/F104-topic-rate-health-diagnostics.md) 说明“原始率由 C++ 统计，lite 率由 Python 监测”。

#### O15 节点合并
- `servo_mode_bridge`（20 Hz，6.7%）逻辑并入 `servo_target_node`（C++）。
- `systemd_watchdog_feed` + `topic_rate_monitor` + `a3_self_test` 的 C++ 部分合并为一个 `a3_health` 节点，减少 DDS participant 与重复订阅。
- 收益：再省 0.1–0.2 核，参与者数量下降。

#### O16 DDS 一致性
- systemd 单元已设 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，手工 `a3_stack.sh` 未设（当前跑默认 FastDDS）。
- 在 [a3_shell_env.sh](../../scripts/a3_shell_env.sh) 增加可选开关（默认沿用现状），A/B 实测后再定；注意 LL-104（QoS 严格性）、LL-119（socket buffer）。
- 若采用，统一配置 CycloneDDS 的 shared memory 与 buffer，减少多 participant 拷贝。

#### F142 需求纠偏
[F142](../../docs/edge/requirements/F142-joint-state-100hz.md) 仍写“50 Hz → 100 Hz”，与现状 200 Hz 不符，且 JSB `publish_rate` 无效。需更新为：
- 明确 `/joint_states` 由控制周期决定（200 Hz）；
- 对外低速消费走 `/joint_states_lite`；
- 提供 100/50 Hz 选项的方式是 relay 参数，而不是 JSB 参数。

---

### P3 — 调度与核绑定（可选，提升确定性）

1. **核绑定**：控制关键进程（`ros2_control_node`、`servo_target_node`、`arm_controller`）绑 A76（cpu4–7）；相机/视觉/UI/RViz 绑 A55（cpu0–3）。
   - systemd 启动：`a3-arm.service` 加 `CPUAffinity=4-7`，另起视觉单元绑 `0-3`。
   - 手工栈：launch `prefix='taskset -c 4-7'`（或 `chrt -f 50 taskset ...`）。
2. **RT 优先级**：`ros2_control_node` 已成功启用 FIFO 50（日志已确认）；确认视觉/IDE 不再占用同一核。
3. **CAN IRQ 亲和**：把 `m_can`/`can1` 中断绑到 A76，减少反馈丢帧。
4. **桌面/IDE**：Trae 服务（`harness-server`、`ckg_server`、VSCode node）合计约 0.5–1 核 + 1 GB 内存；正式验收时建议关闭或降低索引强度。
5. 保持 governor=`performance`。

---

## 5. 假设与决策

| 决策 | 选择 | 理由 |
|---|---|---|
| 是否降低控制周期 | **不降**，保持 200 Hz | 控制质量/LL-180 已证明 200 Hz 反馈新鲜度是 freeze-hold 的前提 |
| 是否直接改 JSB 频率 | **不直接改**（Humble 不支持 `publish_rate`） | 已实测该参数被静默忽略；改 update_rate 会同时改变控制环 |
| 低速率消费者方案 | **新增 C++ lite relay** | 一次 C++ 订阅替代多个 Python 200 Hz 订阅，收益最大且不动控制链 |
| 视觉默认策略 | **按需**：默认不启人脸跟踪，RViz 不常驻 | 视觉是最大非必需开销 |
| 诊断完整性 | 保留 F104/F101，但修正频段与采样源 | 不能为了省 CPU 让诊断长期误报或失效 |
| 安全门 | **完全不动**（gate/enable/急停/看门狗判定逻辑） | 降载不能改变安全语义 |
| 优先级 | 安全 > 控制平滑 > 视觉流畅 > 诊断完整性 > 常驻便利 | 与现有 `docs/shared/SAFETY.md` 一致 |

---

## 6. 验收标准与测量方法

### 6.1 目标

| 场景 | 当前 | 目标 |
|---|---:|---:|
| 待机/伺服（无视觉、无 RViz、轻载档） | 5.6 核 / load 7.7 | **≤2.5 核 / load < 4** |
| 常规联调（相机 + 视觉显示，无人脸） | 6.0 核 | **≤3.2 核** |
| 视觉跟踪全开（相机 + 人脸 + 显示 + RViz） | 6.4 核 | **≤3.8 核 / load < 6** |
| `/joint_states` | 198 Hz | **保持 198–200 Hz**，max gap < 30 ms |
| `feedback stale` | 偶发 | 连续 10 min 无 stale |
| `/diagnostics` | `/joint_states` 永久 WARN | 无误报；真实掉频仍能 WARN/ERROR |
| 视觉显示帧率 | 24–25 fps | 有人看 ≥15 fps |

### 6.2 测量命令

```bash
# 1) 总负载
uptime; free -h

# 2) 每进程瞬时 CPU（% 单核）
python3 - <<'EOF'
import os,time
def snap():
    d={}
    for pid in os.listdir('/proc'):
        if not pid.isdigit(): continue
        try:
            st=open(f'/proc/{pid}/stat').read().split()
            c=open(f'/proc/{pid}/comm').read().strip()
        except Exception: continue
        d[pid]=(c,int(st[13])+int(st[14]))
    return d
a=snap(); time.sleep(3); b=snap()
r=sorted(((t-a[p][1])/3.0,b[p][0],p) for p,t in b.items() if p in a and (t-a[p][1])/3.0>2)
for pct,c,p in reversed(r): print(f'{pct:6.1f}%  {c} ({p})')
EOF

# 3) 关节状态频率与最大间隔
source scripts/a3_shell_env.sh
ros2 topic hz /joint_states

# 4) 诊断误报
ros2 topic echo /diagnostics | grep -A6 -i 'joint_states\|topic_rate'

# 5) 回归（真机）
./scripts/a3_test/f181_servo_track.py          # 伺服跟踪/平滑
python3 scripts/a3_test/f186_freeze_hold_acceptance.py
python3 scripts/a3_test/f193_demo_tour_accept.py
```

### 6.3 分阶段验收

- **P0 后**：重启栈（无 face、无 rviz、无 camera）→ load1 在 2 min 内降到 <4；`/joint_states` 仍 198 Hz。
- **P1 后**：全量栈（相机 + RViz，无人脸）→ 各节点达到第 3 节预期区间；`/diagnostics` 无 `/joint_states` 误报。
- **P2 后**：`/joint_states_lite` 50 Hz 稳定；迁移节点 CPU 再降；F104/F101 功能不回退。
- **P3 后**：60 s 伺服跟踪期间 `/joint_states` max gap ≤30 ms、无 `feedback stale`、无 `CAN write failed`。

---

## 7. 实施顺序与回滚

1. **第 1 批（P0，零代码）**：按档位停机/重启，记录新基线。随时可回滚为原启动参数。
2. **第 2 批（P1 独立项）**：O4 → O5 → O6 → O7 → O9 → O10 → O11 → O12 → O3 → O8。
   - 每项单独提交；每项后用 6.2 命令采样并跑一次 `f181` 冒烟。
   - O8（arm_controller 执行器）风险最高，放在最后，保留单独回滚。
3. **第 3 批（P2）**：O14（lite relay）→ 迁移消费者 → O15 → O16 → F142 文档纠偏。
4. **第 4 批（P3）**：核绑定/RT/IRQ，按需启用。

回滚：所有代码改动均通过 launch 参数或默认值开关；`/joint_states_lite` 用 `use_joint_state_lite:=false` 一键回到现状。

---

## 8. 文档与需求同步（实施时必做）

1. `docs/edge/requirements/F196-load-budget-and-light-profile.md`（新增）：
   - **说明**：运行负载预算、轻载档位、`/joint_states_lite` 降频中转、F104 频段修正。
   - **验收标准**：第 6.1 节目标表 + 无 stale + 无误报诊断。
   - 在 [REQUIREMENTS.md](../../docs/edge/REQUIREMENTS.md) 索引表登记 F196。
2. [ARCHITECTURE.md](../../docs/edge/ARCHITECTURE.md)：补充 lite relay 与负载档位。
3. [TOPIC_CONTRACT.md](../../docs/shared/TOPIC_CONTRACT.md)：新增 `/joint_states_lite`。
4. [F104](../../docs/edge/requirements/F104-topic-rate-health-diagnostics.md)、[F101](../../docs/edge/requirements/F101-watchdog-hardening.md)、[F142](../../docs/edge/requirements/F142-joint-state-100hz.md)：更新采样源与频率口径。
5. [QUICKSTART.md](../../docs/edge/QUICKSTART.md) 与 [a3_stack.sh](../../scripts/a3_stack.sh) usage：新增 `--light/--vision/--demo/--full`。
6. 经验教训（确认修复后记录）：
   - `LL-197`：Humble JSB 不支持 `publish_rate`（静默无效）+ F104 频段未跟随 → 永久误报；排查“配置写了但行为不符”时先查 `strings <lib>.so | grep <param>`。
