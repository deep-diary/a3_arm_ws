# 重新规划：统一「轨迹段」执行模型 —— 编排层执行引擎 + 3 类轨迹配置 + melodion 脚本化

## 一、结论（Summary）

采纳你的架构愿景，比上一版（新增「原样执行轨迹」服务）更统一、更本质：

1. **编排层抽出一个「段执行」原语** `execute_segment(target, mode, speed, sleep)`：
   - `target`：目标点（关节 `positions` 或笛卡尔 `pose`）。
   - `mode`：`ptp`（MoveIt 关节规划）或 `lin`（pilz 笛卡尔直线）。
   - `speed`：本段速度缩放（0~1）。
   - `sleep`：到位后停顿秒数。
2. **统一轨迹配置文件（YAML）分 3 类 `kind`**，全部收进本仓库：
   - `continuous`：密集录制点（现有 playback，F68 retime 链路）——**手动录制为主**。
   - `waypoint`：段序列 + 整体平滑（blend，段末不停）——**手动录制为主**（Share 长按 + L2 打点；Circle 长按 = `lin` 段 + `sleep=0`）。
   - `stream`：段序列 + **逐段执行、段末速度 0、每段带 `mode/speed/sleep`**——**软件编排为主**（琴谱等）。
3. **melodion 退化为离线脚本**：从「启动节点直发轨迹」改为「读键位+曲谱 → 生成 `stream` 配置文件」。不启动 ROS 节点、不再自带 Pinocchio IK（规划交给编排层）。

## 二、现状分析（Current State）

### 2.1 已有能力（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)）
| 轨迹类型 | 入口 | 段模型现状 |
|---|---|---|
| continuous | `/a3/arm/playback`（type 空） | 密集点 → F137 去噪 + F68 retime → dispatch（无「段」概念） |
| waypoint | `/a3/arm/playback` type=waypoint | `_play_waypoints`：逐段 `_moveit_move`(ptp)/`_lin_move`(lin)；F134 可 blend 整体。**无 sleep、无每段 speed、strategy 全局统一** |
| 巡游 | `/a3/arm/random_pose_tour` | 命名点位逐腿，无 speed/sleep |

### 2.2 关键 gap（对照 `execute_segment(target, mode, speed, sleep)`）
1. **无 `sleep`**：现有段「到位即停」，但无「到位后停顿 N 秒」参数。
2. **无每段 `speed`**：只有全局 `moveit_velocity_scaling` / `waypoint_lin_velocity_scaling`。
3. **`mode` 全局统一**：waypoint 的 `strategy` 是整个任务 ptp 或 lin，不能逐段混用。
4. **无 `stream` kind**：没有「段末速度 0 + 每段参数」的逐段执行语义。

### 2.3 弹琴现状（要改造）
- [melodion_player_node.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_player_node.py)：常驻节点，订阅 `/joint_states`，`Pinocchio` IK 生成 `JointTrajectory`，**直发** `/joint_group_effort_controller/joint_trajectory`（绕编排层）；用 `edge_web_sim` 旧仿真栈。

## 三、方案设计（Proposed Changes）

### 3.1 统一轨迹配置文件（放仓库）
格式（YAML，3 类 `kind`，段模型统一）：
```yaml
kind: stream                 # continuous | waypoint | stream
name: qinghuaci
joint_names: [L1_joint, L2_joint, L3_joint, L4_joint, L5_joint, L6_joint, L7_joint]
segments:                    # waypoint/stream 用；continuous 用 points
  - target:
      positions: [...]       # 关节目标（ptp 用）
      # 或 pose: {position: {x,y,z}, orientation: {x,y,z,w}}   # 笛卡尔目标（lin 用）
    mode: ptp|lin            # 默认 ptp
    speed: 0.5               # 段速度缩放，可选，默认全局值
    sleep: 0.0               # 到位后停顿秒，可选，默认 0
  - ...
```
- **continuous**：保留现有 `points` 密集录制格式（`~/.a3/trajectories/*.yaml`），`kind: continuous`。
- **waypoint**：现有 `waypoints` 字段升级为 `segments`（含 mode/speed/sleep；手工录制时 sleep=0、mode 由 strategy 决定）。
- **stream**：新增，`segments` 逐段执行、段末速度 0、sleep 表达节奏。

**存放位置**：
- 录制产物（continuous/waypoint 运行时生成）仍写 `~/.a3/trajectories/`（历史行为）。
- 软件编排产物（stream）+ 生成脚本 + 示例：`src/app_melodion/config/streams/`（版本控制），如 `qinghuaci.yaml`、`scale.yaml`。

### 3.2 编排层：段执行引擎
在 [arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) 中：
1. 新增 `_execute_segment(target, mode, speed, sleep)`：
   - `mode=ptp` → `_moveit_move(q, ..., velocity_scaling=speed)`（关节目标；笛卡尔 target 先经 move_group IK）。
   - `mode=lin` → `_lin_move(pose, ..., velocity_scaling=speed)`（笛卡尔直线）。
   - `sleep>0` → 到位后 `time.sleep(sleep)`（或记录为下一段的等待，不占轨迹）。
2. 给 `_moveit_move` / `_lin_move` 增加 `velocity_scaling` 参数（分别映射 MoveIt `max_velocity_scaling_factor` / pilz 速度缩放）。
3. `_playback_cb` 扩展 `type="stream"`：解析 `segments` → 逐段 `_execute_segment`（段末速度 0、每段 speed/sleep），L7 随段末同步，段失败重试复用 `waypoint_segment_retries`。
4. waypoint 段解析升级：`waypoints` → `segments`（向后兼容旧 `waypoints` 字段），支持 per-seg `mode/speed/sleep`（F134 blend 整体时 `sleep` 忽略）。

### 3.3 melodion 脚本化（去掉节点）
- **删除/退役** [melodion_player_node.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_player_node.py)（节点 + 直发话题 + 服务）。
- **新增** `src/app_melodion/melodion_score2stream.py`（纯离线脚本，无 ROS 依赖）：
  - 输入：`melodion_keys.json` + `playing_geometry.yaml` + 曲谱（`score`）。
  - 逻辑：键位 xyz（`key_config.py` 纯几何，无需 Pinocchio）→ 每个音符映射为段序列：`down`（`lin`，speed=快，sleep=音符时长）→ `up`（`lin`，speed=快，sleep=0）；黑键/白键的 up/down 由几何偏移派生。
  - 输出：`stream` 配置文件（`segments`，笛卡尔 `pose` + `mode=lin` + `speed` + `sleep`）到 `src/app_melodion/config/streams/<name>.yaml`。
- 保留 `key_config.py`（几何）、`score.py`（曲谱解析）、`ik.py`/`trajectory.py` 可删或降级为纯几何/无 ROS 依赖。

### 3.4 执行方式（端到端）
```bash
# 1. 生成 stream 配置（离线，一次）
python3 src/app_melodion/melodion_score2stream.py --score qinghuaci

# 2. 标准栈（mock 仿真 / can 真机），先到 READY 再回 home
ros2 launch a3_bringup a3_bringup.launch.py hardware:=mock
ros2 service call /a3/arm/init std_srvs/srv/Trigger "{}"
ros2 service call /a3/arm/enable std_srvs/srv/Trigger "{}"
ros2 service call /a3/arm/goto_named_pose a3_msgs/srv/GotoNamedPose "{pose_name: home}"

# 3. 编排层依次执行 stream 段
ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: qinghuaci, type: stream, strategy: lin}"
```
> 弹琴从「独立节点直发 + edge_web_sim」切到「离线脚本生成配置 + 标准栈编排层执行」，与 waypoint/巡游/真机同一条链。

### 3.5 关于「waypoint 笛卡尔示教 = Circle 长按」的确认
是同一效果：`waypoint` 的 `lin` 策略 = 逐段 `lin`（`sleep=0`、blend 整体），与 Circle 长按 LIN 回放等价。统一段模型后，waypoint 的 PTP/LIN 与 stream 的逐段 lin 共享 `_execute_segment`，只是 blend 与 sleep 语义不同。

### 3.6 文档同步（architecture-sync / docs-update）
- [F162-melodion-playing.md](file:///home/cat/a3_arm_ws/docs/edge/requirements/F162-melodion-playing.md)：执行方式改为「离线脚本 + 编排层 stream 段执行」。
- [docs/shared/TOPIC_CONTRACT.md](file:///home/cat/a3_arm_ws/docs/shared/TOPIC_CONTRACT.md)：`/a3/arm/playback` 增加 `type=stream` 说明；轨迹配置文件 3 类 kind 契约。
- [app_melodion/README.md](file:///home/cat/a3_arm_ws/src/app_melodion/README.md)：用法改为脚本生成 + 标准栈。
- 可能新增 [docs/shared/](file:///home/cat/a3_arm_ws/docs/shared/) 轨迹配置格式说明（3 类 kind）。

## 四、假设与决策（Assumptions & Decisions）
1. **统一段模型**：`segment = (target, mode, speed, sleep)`，3 类 `kind` 共享 `segments` 结构（continuous 例外用 `points`）。
2. **melodion 脚本化**：离线生成 stream 配置，不再启动节点；规划（IK/轨迹）交给编排层/MoveIt/pilz。
3. **弹琴 mode**：默认 `lin`（键在一条直线上，笛卡尔直线下压/抬起最自然）；`ptp` 留给关节目标场景。
4. **speed 映射**：`_moveit_move`/`_lin_move` 增加 `velocity_scaling`，缺省用现有全局参数（`moveit_velocity_scaling`/`waypoint_lin_velocity_scaling`）。
5. **向后兼容**：waypoint 旧 `waypoints` 字段仍可读；`type` 缺省空 = continuous（不变）。
6. **栈**：验收用 `a3_bringup.launch.py hardware:=mock` 标准栈；不再依赖 `edge_web_sim`。

## 五、验证（Verification）
1. **生成**：`melodion_score2stream.py` 输出合法 `stream` YAML（段数 = 音符数×2，sleep=音符时长，mode=lin，pose 齐全）。
2. **mock 栈**：`hardware:=mock` → init/enable → goto home → playback(type=stream)，RViz 观察末端沿键盘线逐键下压/抬起，休止有停顿（sleep 生效）。
3. **节拍**：对照直发版本，《青花瓷》节奏一致、无抹平滑。
4. **waypoint 回归**：Circle 长按 LIN / Square 长按 PTP 行为不变（`_execute_segment` 复用后）。
5. **门禁**：STATE 非 READY / gate 关 / mode=ZERO_TORQUE 时 playback 被拒并返回可读 message。
6. **编译**：`colcon build --packages-select a3_msgs a3_arm_controller app_melodion`。

## 六、实现顺序（建议 todo）
1. 定义统一轨迹配置格式（3 类 kind）+ stream 示例（`src/app_melodion/config/streams/`）。
2. 编排层 `_execute_segment` + `_moveit_move`/`_lin_move` 加 `velocity_scaling`。
3. `_playback_cb` 支持 `type=stream` + waypoint `segments` 解析升级。
4. melodion 脚本化（`melodion_score2stream.py`，退役节点）。
5. 文档同步（F162 / TOPIC_CONTRACT / README）。
6. mock 栈端到端验收。
