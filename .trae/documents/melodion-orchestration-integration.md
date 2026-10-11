# 评估：把弹琴能力合入真机栈（编排层）——「原样执行带时间轴轨迹」服务

## 一、结论（Summary）

**你的判断是对的。** 弹琴本质就是「执行一串带时间轴的关节轨迹点（点位间可停顿/速度 0/延时/控速）」，与 waypoint 示教回放（F131）、巡游（F116/F134/F161）、连续回放（playback）**同质**。当前「独立 app_melodion 节点直发轨迹 + 独立 `edge_web_sim` 旧仿真栈」确实冗余、不优雅——它**绕过了编排层**，享受不到状态机/gate/静态力矩门禁/duty 门禁/FJT 标准后端。

**方案**（已与你确认）：
1. **保留 app_melodion 作为「轨迹生成器」**（键位/曲谱/Pinocchio IK 生成带时间轴的 `JointTrajectory`），不再直发话题。
2. **编排层新增一个「原样执行轨迹」服务**，接受带时间轴的 `JointTrajectory`，走现有 FJT action 后端（复用状态机/gate/门禁），但**不做 F68 retime / F137 去噪 / 平滑**，保留弹琴的精确停顿与节拍。
3. **栈切换**：弹琴验收从 `edge_web_sim`（旧自研仿真栈）切到 **标准栈 `a3_bringup.launch.py hardware:=mock`**（与真机 `hardware:=can` 同拓扑）。

## 二、现状分析（Current State）

### 2.1 弹琴当前实现（独立、绕编排层）
- [melodion_player_node.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_player_node.py)：订阅 `/joint_states` 取当前 `q` → [trajectory.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/trajectory.py) 生成带节拍时间轴的 `JointTrajectory` → **直发** `/joint_group_effort_controller/joint_trajectory`。
- 用 [edge_web_sim.launch.py](file:///home/cat/a3_arm_ws/src/a3_bringup/launch/edge_web_sim.launch.py)（旧自研仿真栈，`sim_motor_node` 订阅该话题）。
- 后果：弹琴期间编排层「不知道」在动；状态机/gate/门禁全部绕过；与遥控器/waypoint/巡游不是一条通道。

### 2.2 编排层已有轨迹执行能力（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)）
| 能力 | 入口 | 语义 |
|---|---|---|
| 连续回放 | `/a3/arm/playback`（PlaybackTrajectory） | 读文件 → 回首点 ramp → **F137 去噪 + F68 retime** → dispatch |
| 路点回放 | `/a3/arm/playback` type=waypoint | PTP（MoveJ）/LIN（笛卡尔），F134 pilz Sequence + blend |
| 巡游 | `/a3/arm/random_pose_tour` | 命名点位序列，F161 过点样条 + retime |
| 单点移动 | `/a3/arm/move_to`（MoveToJointPositions） | 两点轨迹（当前→目标） |
| 执行后端 | `_dispatch_trajectory` | 静态力矩门禁(F107) + duty 门禁 + FJT action / topic |

### 2.3 关键 gap（为什么不能直接复用 playback）
弹琴轨迹的**节奏 = 音乐语义**，靠「相同位置 + 不同 `time_from_start`」表达停顿（如 `down` 点与 `down-hold` 点时间差 = 音符时长）。现有 playback 链路：
- **F137 几何去噪**（B 样条平滑）会平滑掉精确按键点；
- **F68 retime**（Ruckig/TOTG 重定时）会按关节限值重算时间轴，可能压缩/抹掉停顿；
- legacy 链的 `_smooth_points` + `_time_warp_points` 同样会改几何与时间。

因此需要一条**「原样执行」**路径，只复用 `_dispatch_trajectory`，跳过 retime/去噪/平滑。

## 三、方案设计（Proposed Changes）

### 3.1 新增消息：`a3_msgs/srv/PlayTimedTrajectory.srv`
```srv
# 原样执行带时间轴的关节轨迹（不经 retime/去噪/平滑，保留精确停顿与节拍）
trajectory_msgs/JointTrajectory trajectory
---
bool success
string message
```
- [a3_msgs/CMakeLists.txt](file:///home/cat/a3_arm_ws/src/a3_msgs/CMakeLists.txt)：`rosidl_generate_interfaces` 增一行 `"srv/PlayTimedTrajectory.srv"`（`trajectory_msgs` 已在 DEPENDENCIES）。

### 3.2 编排层新增服务处理（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)）
- import `PlayTimedTrajectory`。
- 注册 `/a3/arm/play_timed_trajectory` 服务。
- 处理函数（约 30 行，参照 `_move_to_cb` 与 `_playback_cb`）：
  1. `_can_move()` 检查（state/gate/mode/feedback stale）。
  2. 校验 `trajectory` 非空、`joint_names` 齐备（缺关节补 0，对齐 `L1..L7` 顺序）。
  3. `_publish_mode("TRAJ_RUNNING")` + `_set_state(STATE_TRAJ, ...)`。
  4. 调 `_dispatch_trajectory(traj)`（内部已含静态力矩 F107 + duty 门禁；FJT action 后端）。
  5. 失败回 READY + 报错；成功 `_schedule_back_to_ready(duration + 0.3)`。

> 可选优化：静态力矩门禁对大曲谱（qinghuaci 337 点）逐点查偏慢，可参照 playback 的 `gate_idxs` 抽样（≤96 点）——放在 `_dispatch_trajectory` 之外、服务内先抽样预检，或直接接受逐点（每点 ~ms 级，337 点可接受）。

### 3.3 弹琴节点改走编排层（[melodion_player_node.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_player_node.py)）
- **删除**：直发 `/joint_group_effort_controller/joint_trajectory` 的 publisher（及 `trajectory_topic` 参数）。
- **保留**：订阅 `/joint_states` 取当前 `q`；`/a3/melodion/play`（Trigger）服务；`autoplay` 逻辑。
- **新增**：`PlayTimedTrajectory` 服务 client（`/a3/arm/play_timed_trajectory`）。`_build_and_publish` 改为 `_build_and_execute`：生成轨迹 → 调编排层服务 → 返回 success/message。

### 3.4 栈切换（仿真/真机同拓扑）
- 仿真验收：`ros2 launch a3_bringup a3_bringup.launch.py hardware:=mock use_rviz:=false`（标准 mock 栈，F78 产品形态，含 move_group/retime/servo/编排层）。
- 真机：`hardware:=can`（需先 `can-up.service` + init/enable 到 READY，gate open）。
- 前置流程（弹琴走编排层后需）：`init` → `enable`（READY）→ 可选 `goto_named_pose home` → 触发弹琴。与「遥控器到 home 再弹琴」同一条链。

### 3.5 文档同步（architecture-sync / docs-update）
- [F162-melodion-playing.md](file:///home/cat/a3_arm_ws/docs/edge/requirements/F162-melodion-playing.md)：执行方式由「直发 topic」改为「编排层 `/a3/arm/play_timed_trajectory`」。
- [docs/shared/TOPIC_CONTRACT.md](file:///home/cat/a3_arm_ws/docs/shared/TOPIC_CONTRACT.md)：新增服务契约条目（若该文档登记了 `/a3/arm/*` 服务表）。
- [app_melodion/README.md](file:///home/cat/a3_arm_ws/src/app_melodion/README.md)：用法改标准栈 + 编排层服务。

## 四、假设与决策（Assumptions & Decisions）
1. **执行方式**：新增「原样执行轨迹」服务（用户已确认），保留精确停顿/节拍。
2. **代码组织**：保留 app_melodion 做生成器（用户已确认），不并入编排层。
3. **轨迹起点**：弹琴轨迹第一点 = 当前 `q`（`build_trajectory` 已含），无需回首点 ramp。
4. **门禁**：静态力矩(F107) + duty 由 `_dispatch_trajectory` 内部承担，服务内只做 `_can_move` 与格式校验。
5. **栈**：仿真用 `hardware:=mock` 标准栈，废弃弹琴对 `edge_web_sim` 的依赖。
6. **向后兼容**：不保留弹琴直发话题路径（合入编排层是唯一路径）。

## 五、验证（Verification）
1. **编译**：`colcon build --packages-select a3_msgs a3_arm_controller app_melodion`；`ros2 interface show a3_msgs/srv/PlayTimedTrajectory` 能显示。
2. **mock 栈**：起 `hardware:=mock` → `init` → `enable`（READY）→ `goto_named_pose home` → 触发弹琴，RViz 观察末端按键盘线逐键运动。
3. **节拍保留**：用《青花瓷》验证休止符有停顿（末端在 hover 处保持、无抹平滑）；对照「直发话题」版本行为一致。
4. **门禁生效**：STATE 非 READY / gate 关 / mode=ZERO_TORQUE 时服务被拒并返回可读 message。
5. **单测**：`app_melodion` 现有 `pytest` 仍通过（生成逻辑未变，仅执行方式变）。

## 六、实现顺序（建议 todo）
1. a3_msgs 新增 srv + CMakeLists。
2. 编排层注册 + 处理服务。
3. 弹琴节点改走编排层服务（删直发）。
4. 文档同步（F162 / README / TOPIC_CONTRACT）。
5. mock 栈端到端验收（home → 弹琴 → 节拍）。
