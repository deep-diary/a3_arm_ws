# 路点轨迹 → 多模式 stream 生成器 + 长按 Triangle 触发

## 一、结论（Summary）

把上一版「离线 + 在线基准脚本」改成**纯离线生成器**：读路点录制文件（`waypoints/latest.yaml`，取前 6 点），生成一条 **stream 轨迹文件**（5 种轨迹模式 × 4 档速度的段序列，段间自动回起点）。**脚本不触发运动**，运动由真机栈 `playback type=stream` 触发，并通过 **长按遥控器 Triangle** 一键执行。

| 模式 | mode 命名 | 几何 | 时间/衔接 | 实现 |
|---|---|---|---|---|
| 笛卡尔 pilz（逐段 LIN） | `pilz` | 笛卡尔 pose | pilz LIN，blend=0（逐点精确过点、段末停） | `_sequence_move_group` blend=0 |
| L1 blend | `blend` | 笛卡尔 pose | pilz Sequence + blend（转角不停车） | `_goto_via_lin_blend` |
| L1 保形 | `l1p5` | 关节 positions | 线性插值 + quintic 加→巡→减 | `_goto_via_smooth` profile=l1p5 |
| L2 梯形 | `l2` | 关节 positions | 线性插值 + 梯形速度 | `_goto_via_smooth` profile=l2 |
| L3 钟形 | `l3` | 关节 positions | cubic 样条 + 钟形（纯 quintic） | `_goto_via_smooth` profile=l3 |

速度降为 **0.1 / 0.2 / 0.4 / 0.8**（4 档）。

---

## 二、现状分析（已读代码确认）

1. **路点录制文件**：`_dump_waypoints()`（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) L5280）→ `waypoints/latest.yaml`，格式 `{kind: waypoint, joint_names, waypoints:[{positions[7], pose{position,orientation}}]}`；目录 `self._wp_dir = <traj_dir>/waypoints`（L1186）。
2. **stream 段模型**：`_execute_segment()`（L4163）mode 仅 ptp/lin；via 段走 `_goto_via_smooth`（ptp，关节过点样条）或 `_goto_via_lin_blend`（lin，pilz blend）。
3. **时间剖面函数已就绪**：`_jerk_limited_timed_traj`（L225 加-巡-减）、`_trapezoidal_timed_traj`（L427 梯形）、`_bell_timed_traj`（L497 钟形）、`_interpolate_via_spline`（L179 cubic 样条）。
4. **按钮触发机制**：[actions.py](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py) 的 `fn`（如 `playback_waypoints_ptp` L492 调 `/a3/arm/playback`）；[action_registry.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/action_registry.yaml) 定义 fn；[default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml) 的 buttons 映射 fn→edge/hold_s（Triangle 当前短按=home，L63）。

---

## 三、方案设计

### 3.1 编排层：`_goto_via_smooth` 加时间剖面参数

`_goto_via_smooth(..., profile="l1p5")`：`profile ∈ {l1p5, l2, l3}` 选择重定时函数
- `l1p5` → `_jerk_limited_timed_traj`（现有默认，线性插值 dense）
- `l2` → `_trapezoidal_timed_traj`（线性插值 dense）
- `l3` → `_interpolate_via_spline` + `_bell_timed_traj`（cubic 样条 dense）

### 3.2 编排层：`_execute_segment` 扩展 mode

via 段 mode 分支扩展为：
- `lin`/`blend` → `_goto_via_lin_blend`（pilz blend，不停车）
- `pilz` → `_goto_via_lin_blend` 变体 blend=0（逐点精确过点、段末停）
- `l1p5`/`l2`/`l3` → `_goto_via_smooth(..., profile=mode)`
- 其余回落 `ptp`（现行为）

单段（无 via）：`pilz`/`lin` 走 `_lin_move`（笛卡尔单段 LIN），`l1p5/l2/l3/ptp` 走 `_moveit_move`。

### 3.3 生成器脚本（新 `scripts/gen_stream_modes.py`）

- 输入：路点文件（默认 `<ws>/trajectories/waypoints/latest.yaml`，`--waypoints` 可覆盖）＋模式列表＋速度列表。
- 取前 6 点（`waypoints[:6]`）；不足 2 点报错。
- 生成 stream：对每个 `(mode, speed)` 段：
  - **去程**：`via = [wp1..wpN-2]`、`target = wpN-1`（N-1 个 via 表达完整 N 点路径）；
    `pilz`/`blend` 用 `pose`（笛卡尔），`l1p5`/`l2`/`l3` 用 `positions`（关节）。
  - **回程**：`{mode: ptp, speed: 0.5, target: wp0.positions}`（回到起点，供下一档对比同一路径）。
- 输出：`<ws>/trajectories/streams/modes_sweep.yaml`（git 忽略；`_stream_path` 首个搜索目录即 `traj_dir/streams/`，playback 直接命中）。

> 段结构示例（N=4 点，模式 pilz + speed 0.1）：
> `{mode: pilz, speed: 0.1, via: [wp1.pose, wp2.pose], target: wp3.pose}` 后跟回程 `{mode: ptp, target: wp0.positions}`。

### 3.4 长按 Triangle 触发

- [actions.py](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py)：新增 `playback_stream(name="modes_sweep")` → 调 `/a3/arm/playback {type:"stream", name:"modes_sweep"}`。
- [action_registry.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/action_registry.yaml)：新增 `playback_stream`（kind: discrete）。
- [default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml)：Triangle 改双键义——短按=home（保留）、长按 1.5s=`playback_stream`。

---

## 四、假设与决策

1. **路点取前 6**：`waypoints[:6]`，≥2 点即可生成；不足 2 点脚本报错退出。
2. **段间回程**：每个「模式×速度」去程段后跟一段 `ptp` 回起点，保证各档对比同一路径、同一初始位形。
3. **笛卡尔 vs 关节**：`pilz`/`blend` 用路点 `pose` 字段（FK 笛卡尔）；`l1p5`/`l2`/`l3` 用 `positions` 字段（关节）。
4. **mode 命名**：`pilz`/`blend`/`l1p5`/`l2`/`l3`（与现有 `ptp`/`lin` 兼容并存）。
5. **脚本只生成不触发**：产物 stream 落 `<ws>/trajectories/streams/`（git 忽略），playback 由长按 Triangle 或手动 `ros2 service call` 触发。
6. **速度 4 档**：0.1/0.2/0.4/0.8（替代上一版 10 档）。

---

## 五、验证

1. **生成器**：读一份 3~6 点路点文件，`gen_stream_modes.py` 生成 `modes_sweep.yaml`，含 5 模式 × 4 速度 × 2（去/回）段；via/target 的 pose/positions 取对（笛卡尔模式用 pose、关节模式用 positions）。
2. **编排层**：仿真栈（domain 99）`playback {name: modes_sweep, type: stream}` 逐段跑通，日志区分 `pilz`/`blend`/`l1p5`/`l2`/`l3` 分支，无 rejected/failed。
3. **按钮**：长按 Triangle 1.5s 触发 playback stream（日志 `/a3/arm/playback {type:stream, name:modes_sweep}`）；短按仍 goto home。
4. **编译/单测**：`py_compile` + `colcon build a3_arm_controller a3_teleop_ps4` + `GetDiagnostics` 无报错。

---

## 六、实现顺序

1. 编排层 `_goto_via_smooth` 加 `profile` 参数（l1p5/l2/l3 时间剖面）。
2. 编排层 `_execute_segment` mode 扩展（pilz/blend/l1p5/l2/l3）。
3. `scripts/gen_stream_modes.py` 生成器（读路点 → 生成 stream）。
4. 按钮：actions.py `playback_stream` + action_registry.yaml + default.yaml Triangle 双键义。
5. 文档同步（scripts/README、PS4_OPERATOR_GUIDE、TOPIC_CONTRACT 的 mode 扩展）。
6. 仿真验收（生成器产物 + playback 逐段 + 长按 Triangle）。
