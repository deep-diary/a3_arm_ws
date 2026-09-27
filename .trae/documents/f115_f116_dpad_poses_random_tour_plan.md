# F115/F116：D-pad 重绑 home 偏移点位 + 随机点位 MoveIt 巡游 实施计划

## Repository Research（已完成的调研）

### 需求 1：去掉 D-pad 调速，改默认速度

- F64 调速绑定在 [default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml#L41-L47) 的 `dpad:` 段（`dpad_y` ± → `step_linear_scale`，`dpad_x` ± → `step_angular_scale`，步进 0.15/clamp 0.1..1.0）。
- 调速 fn（[actions.py L624-630](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py#L624-L630)）保留不删（simple.yaml 未用、回归后可能复用），仅解除绑定；`linear_scale/angular_scale` 初值 0.35 不变 → 实际效果=恒速默认档。
- mapper 对 `dpad:` 段的边沿去抖逻辑（[ps4_mapper.py L163-171](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py#L163-L171)）原样复用，新绑定零代码改动。
- 受影响测试：[ps4_sim_test.py](file:///home/cat/a3_arm_ws/scripts/a3_test/ps4_sim_test.py#L502-L570) 场景 5d 整段（8 条调速断言）必须改写。

### 需求 2：home 周边 4 点位（用户已拍板：上/下/前/后；D-pad 上=上/下=下/左=后/右=前）

用仓内现成 Pinocchio（[move_to_pose_ik_node.py](file:///home/cat/a3_arm_ws/src/a3_bringup/a3_bringup/move_to_pose_ik_node.py) 同款）对真实 URDF 做了 FK/IK 探针（只读计算，**未改任何文件**）：

- legacy `home = [0, .785, -.785, 0,0,0,0]`，末端在 base_link 系 **(-0.1766, 0, 0.3263) m**，姿态四元数 (wxyz)=(.5,.5,.5,.5)；+X=前方（cloud_edge S11 契约一致）。
- **保持 home 末端姿态不变**的严格 6D IK 可达包络（200 种子验证 + 独立 FK 复核，位置误差 0~2mm、姿态误差 0°）：

| 点位 | 方向/距离 | 6 关节解（L1..L6, rad） | 备注 |
|---|---|---|---|
| `home_back` | 后 −X **20cm** | `[0, 1.8674, -1.6192, -0.2482, 0, 0]` | 平面构型，腕部不动 |
| `home_front` | 前 +X **15cm** | `[0, 0.1266, -0.8918, 0.7653, 0, 0]` | 前 20cm 达水平臂展边界，15cm 干净可达 |
| `home_up` | 上 +Z **15cm** | `[0, 1.2862, -1.9643, 0.6780, 0, 0]` | 上 20cm 仅腕翻到 ±1.52 限位边缘的稀有分支，15cm 平面干净 |
| `home_down` | 下 −Z **10cm** | `[0, 0.7154, -0.2479, -0.4675, 0, 0]` | 下 15/20 严格保姿态无任何可达分支，10cm 是最大平面距离 |

- **与用户原决策的偏差（审批点）**：用户已同意“前 15 其余 20”；探针进一步发现上/下 20cm 也不可达。默认采用**各方向物理可达最大距离**方案（后 20 / 前 15 / 上 15 / 下 10，上表）。备选：四点统一 10cm 立方体（活动范围偏小）。审批时可否决改统一 10cm。
- 4 点全部：L1=L5=L6=0 的平面构型、在 URDF 硬限位内且与生产 IK 限位内缩（0.05 rad）无关（点位直接存关节角，不再经 IK）。
- 点位写入 [named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml)（L7 补 0.0）；D-pad 复用现有 `goto_named_pose`（F67 MoveIt 路径，sim 默认 `goto_use_moveit=true`），**无需新 action/fn**。
- [el_a3.srdf](file:///home/cat/a3_arm_ws/src/a3_moveit_config/config/el_a3.srdf#L32-L58) 同步加 4 个 `group_state`（RViz 命名目标下拉与 yaml 权威保持一致；现有 idle/ready/zero 有此惯例，legacy home 反而没有）。

### 需求 3：随机点位 MoveIt 巡游（用户已拍板：点位池排除 zero；数量可配，默认 5；可重复，相邻不重；从当前点开始）

- 现有 F67 `_moveit_move(target_joints, ...)`（[arm_controller.py L2205-2312](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L2205-L2312)）已是完整 MoveGroup action 规划+执行（OMPL+TOTG，FJT 下发），内含 F107 静力矩门限与占空比预检。巡游服务**逐腿串行复用**即可，不重写规划。
- 状态门控复用 `_can_move()`（READY 态、gate/mode/feedback 检查）；TRAJ 态由腿间内部调用保持，不经服务门。
- 长阻塞服务有先例：F54 `playback`（[L2735+](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L2735)），ReentrantCallbackGroup 支撑。
- 触发方式：**只提供服务 `/a3/arm/random_pose_tour`**（CLI/仿真调用）。PS4 已无空闲键（L2 刚用于 F114，touchpad 蓝牙无事件 LL-052），不绑手柄。
- 仿真验收栈：`edge_teleop_full_sim.launch.py use_moveit:=true`（[edge_web_sim](file:///home/cat/a3_arm_ws/src/a3_bringup/launch/edge_web_sim.launch.py#L286-L290) `use_moveit` 默认 true，含 move_group+FJT+retime）。

## Files and Modules

- `docs/edge/REQUIREMENTS.md`：新增 F115（D-pad 重绑 4 点位、去 F64 调速）、F116（随机点位巡游服务）条目。
- [src/a3_description/config/named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml)：追加 `home_front/home_back/home_up/home_down` 四点位（含探针来源注释）。
- [src/a3_moveit_config/config/el_a3.srdf](file:///home/cat/a3_arm_ws/src/a3_moveit_config/config/el_a3.srdf)：arm 组追加 4 个 `group_state`。
- [src/a3_teleop_ps4/config/mappings/default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml)：删 `dpad:` 调速条目，改为 4 方向 goto；更新头部速查（调速行删除、位姿行补 D-pad）。
- `src/a3_msgs/srv/RandomPoseTour.srv`（**新建**）+ [src/a3_msgs/CMakeLists.txt](file:///home/cat/a3_arm_ws/src/a3_msgs/CMakeLists.txt#L13-L24) 注册（**需 colcon build**）。
- [src/a3_arm_controller/a3_arm_controller/arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)：导入 RandomPoseTour；新增参数 `random_tour_count`（默认 5）、`random_tour_exclude_poses`（默认 `["zero"]`）；注册服务；新增 `_random_pose_tour_cb`。
- [scripts/a3_test/ps4_sim_test.py](file:///home/cat/a3_arm_ws/scripts/a3_test/ps4_sim_test.py)：场景 5d 从“调速 8 断言”改写为“D-pad 4 键 → 4 个 home_* 点位 goto 到达”断言。
- `scripts/a3_test/f116_random_tour_sim_acceptance.py`（**新建**）：巡游服务仿真数值验收（仿 [f67_f68_sim_acceptance.py](file:///home/cat/a3_arm_ws/scripts/a3_test/f67_f68_sim_acceptance.py) 结构：enable→调服务→/joint_states 顺序到点核对）。
- 文档同步：[PS4_OPERATOR_GUIDE.md](file:///home/cat/a3_arm_ws/docs/edge/PS4_OPERATOR_GUIDE.md)（L32-33/L88 调速段、按键表）、[SAFETY.md](file:///home/cat/a3_arm_ws/docs/shared/SAFETY.md#L72) L72、[TOPIC_CONTRACT.md](file:///home/cat/a3_arm_ws/docs/shared/TOPIC_CONTRACT.md#L121) arm 服务表、`docs/shared/ROBOT_MODEL.md`（命名姿态表）、[QUICKSTART.md](file:///home/cat/a3_arm_ws/docs/edge/QUICKSTART.md#L67) S0“速度档记忆”措辞、a3_teleop_ps4 包 README 映射表（如有）。

## 服务契约（新建 srv 内容）

```text
# F116: 从 named_poses.yaml 随机抽点，逐点 MoveIt 规划执行（从当前位姿起）。
# count=0 → 节点参数 random_tour_count（默认 5）；seed=0 → 真随机。
uint32 count
uint64 seed
---
bool success
string message
string[] sequence       # 实际抽取并执行的点位名（顺序）
float64 total_duration_s
```

## Implementation Steps（依赖序）

1. **需求先行**：REQUIREMENTS.md 追加 F115/F116（说明/验收/关联 F39/F64/F67/F113/F114/状态）。
2. **点位数据**：named_poses.yaml 追加 4 点（上表数值，L7=0.0，注释写明 base_link 偏移与探针日期）；SRDF 加 4 group_state。
3. **新消息+编译**：建 RandomPoseTour.srv、注册 CMakeLists；`colcon build --packages-select a3_msgs`。
4. **编排层服务**：arm_controller 加参数/导入/注册/`_random_pose_tour_cb`：
   - `_can_move()`+`_have_js` 门禁；池=`_poses` 去掉 exclude；池大小 <2 报错；
   - `rng = random.Random(seed if seed else None)`；首项随机，后续从“≠前一项”集合中抽；
   - 逐腿 `_moveit_move(q[:6], JOINTS[:6], label)`；每条腿累计时长、记入 sequence；
   - 任一腿失败：发 IDLE、回 READY、返回 success=false + 已完成序列与失败原因；
   - 全部成功：`_schedule_back_to_ready(total+0.3)`，返回序列与总时长；L7 全程不动（arm 组规划）。
5. **手柄重绑**：default.yaml `dpad:` 改为 `dpad_y: neg=home_up / pos=home_down`、`dpad_x: neg=home_back / pos=home_front`（沿用 goto_named_pose + 边沿去抖）；头部速查同步。
6. **测试改造**：ps4_sim_test.py 场景 5d 重写（4 键→4 点 wait_pose 到达，含回归“无调速副作用”不再需要）；新建 f116 验收脚本（count=5/相邻不重/逐点 /joint_states 到位/seed 可复现/count 覆盖参数/池排除 zero/失败语义按 sim 可达性全部应成功）。
7. **文档**：按清单更新 6 处文档；历史条目（F64 状态行、lessons_learned）不改写，仅在 F115 条目中说明“default 映射退役调速、函数保留”。
8. **仿真回归**：独立 ROS_DOMAIN_ID（真机栈在默认域运行，严禁同域）起 full_sim，依次跑 f116 验收 + ps4_sim_test 全量。

## Dependencies and Considerations

- **必须 colcon build**：a3_msgs（srv 绑定）；SRDF 改动也需重新安装（非 symlink 资源）。Python/config/symlink 资源重启即可。
- 验收仿真栈必须与当前真机全栈**不同 ROS_DOMAIN_ID**（硬约束：真机默认域在跑）。
- 点位 MoveIt 可达 ≠ IK 可达：goto 走关节空间 OMPL，4 点关节角均在限位内；相邻点最大关节差约 2.6 rad（front↔back），OMPL 规划时间允许（`moveit_allowed_planning_time_s`）。
- 巡游服务腿间不重复门禁（TRAJ 态合法），但每条腿保留静力矩/占空比预检 → 真机未来启用时安全门仍在。
- `random_tour_exclude_poses` 默认 `["zero"]`（用户拍板）；exclude 不存在的点不报错（忽略）。
- `sequence` 回传使验收脚本无需猜随机结果即可核对。

## Validation

1. `python3 -m py_compile` 改动的 Python；yaml/srdf 解析检查；srv build 成功。
2. 4 点位数值在 sim 里经 goto_named_pose 全部到达（容差 0.02 rad，f67 同款判据）。
3. `ros2 service call /a3/arm/random_pose_tour` count:=5：success=true、sequence 长度 5、相邻不重、不含 zero、/joint_states 按序到点；seed=123 两次调用序列一致；count:=2 覆盖默认。
4. ps4_sim_test 全量（新 5d + 其余 46 场景零回归：Circle→idle、L2 保存 F114 等不受影响）。
5. 真机回归仅记录为待办（D-pad 4 点 + 巡游），本次不在真机执行。

## Risks

- **上/下点距离与用户预期（20cm）不符**：物理包络限制，计划采用“可达最大”（15/10cm）；审批可否决为统一 10cm。
- **OMPL 个别腿规划超时/失败**：服务 abort 返回失败序列，验收脚本如实记录；必要时单腿规划失败由 MoveGroup 内部 num_attempts 兜底，不新增自研重试。
- **ps4_sim_test 场景 5d 改写影响 46/46 基线计数**：新场景断言数重排，以新全量数为准，QUICKSTART 中的 46 数字同步更新。
- **SRDF 与 yaml 双写漂移**：两处在同一提交内同数值；f116 验收只依赖 yaml（goto 服务权威），SRDF 仅服务 RViz。
