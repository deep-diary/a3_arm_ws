# 路点模式闭环：双槽位录制 + R2 切换规划方式 + HUD 显示

## 1. 结论（先回答“这样合理不”）

**合理，而且比现状更自洽**：把“录路点、播路点、选规划方式”全部收口到 WAYPOINT 模式，
与 DANCE 的双槽位结构对称，NORMAL 只保留日常位姿/连续回放，职责不再交叉。
“Share+□ 回 NORMAL”确实多余——`Options` 短按已经统一退出，删除即可。

但落地前有 5 个必须处理的现实约束（已按你的选择定案）：

1. **R2 与夹爪冲突**：R2 现在是夹爪力控的模拟扳机（`resolve_mode` 里明确 `r2` 永不屏蔽）。
   已确认走「WAYPOINT 内 R2 切规划方式」，因此必须在 WAYPOINT 模式屏蔽 R2 轴；
   代价是该模式下不能用手柄手调夹爪（夹爪角度仍随路点录制/回放）。
   好消息：`ds4_linux.yaml` 里 `buttons.r2: 7` 已存在（hid-sony 的 R2 数字键），
   不需要新增虚拟键；若真机 `joy_dump` 发现按键 7 不触发，再补一个基于 `r2` 轴的虚拟键。
2. **现有路点回放不走 9 模式注册表**：`_play_waypoints` 只有 `strategy: ptp|lin` 两条 legacy 链
   （`strategy=ptp` 在 `waypoint_use_sequence=true` 时实际是 pilz PTP Sequence+blend，
   并不等于规范名 `ptp` 的“逐腿停”）。要让三角/圆真的按 8 种方式规划，必须把路点任务
   **转成段模型**再走 `_execute_segment`，不能直接给 `strategy` 塞模式名。
3. **circ 必须剔除**：pilz CIRC 需要辅助点（当前实现强制“当前位 + 恰好 1 个 via + target”），
   路点序列没有“辅助点”语义，放进循环只会误用。剔出 R2 循环列表，服务端显式拒绝并提示。
4. **循环必须可打断**：路点回放是阻塞式服务回调，循环不能只写 `while True`；
   需要取消事件，让 `□ / Options / 切模式` 的 `/a3/arm/stop` 能从另一个执行器线程终止。
   路点循环天然每轮从“末点→wp1”走一条正常规划腿，**不需要**连续轨迹 F197 的接缝回归段。
5. **HUD 不能塞进 `/a3/teleop/mode`**：该话题同时驱动 mode echo、灯条颜色、模式切换判断，
   拼成 `waypoint:ptp_blend` 会破坏语义。新增独立话题 `/a3/teleop/plan_mode`，
   由 mux 在右上角第二行组合成 `waypoint · PTP`。

## 2. 现状分析（关键代码位置）

| 关注点 | 位置 | 现状 |
|---|---|---|
| 操作模式选择 | `src/a3_teleop_ps4/config/mappings/default.yaml:63-77` | Share+方向/面键点名；`square: normal`；Options 退出 |
| NORMAL 绑定 | 同上 `79-103` | △短=ready / 长=连续 latest；□短=存点 / **长=路点 PTP**；○短=停止 / **长=路点 LIN**；L2=存点 |
| WAYPOINT 绑定 | 同上 `117-126` | 进入即自动开录；△=开始/打点；L2=打点 |
| 轨迹模式注册表 | `src/a3_arm_controller/a3_arm_controller/arm_controller.py:413-457` | 9 种：ptp / ptp_blend / lin_exact / lin_blend / jnt_g64_quintic / jnt_g64_trapezoid / jnt_spline_bell / circ / jnt_spline_approx |
| 模式分派 | 同文件 `5074-5112`、`5123-5159` | `_plan_mode` / `_execute_segment`（stream/cp 使用） |
| 路点回放 | 同文件 `4418-4574` | 仅 `strategy=ptp|lin`；序列 blend 或逐段兜底；不走模式注册表 |
| 路点录制 | 同文件 `6326-6351`、`6442-6488` | `slot=None` → `waypoints/latest.yaml`；`slot` 非空**一律当成口风琴槽**写 `melodion/waypoints/` |
| 回放前导/门禁 | 同文件 `6874-7001` | `is_wp and strategy not in ("ptp","lin")` 直接拒 |
| 循环机制 | 同文件 `3028-3123`、`3354-3359`、`7225-7270` | 只有 DANCE 连续轨迹循环（非阻塞 timer 重下发） |
| 停止服务 | 同文件 `6213-6246` | 清舞蹈循环 → 刹停 → 退出示教/伺服 |
| 手柄动作 | `src/a3_teleop_ps4/a3_teleop_ps4/actions.py:938-991` | `playback_waypoints_ptp/lin` 固定 strategy，回放 latest |
| 模式解析 | `src/a3_teleop_ps4/a3_teleop_ps4/mapping.py:311-335` | `suppress_common_axes` 显式跳过 `r2` |
| HUD | `src/a3_bringup/a3_bringup/vision_display_mux.py:453-488` | 右上角第一行 fps、第二行 `/a3/teleop/mode` |
| 服务契约 | `src/a3_msgs/srv/PlaybackTrajectory.srv` | `name / type / strategy / loop` |

## 3. 目标行为（定案）

### 3.1 模式选择

- 删除 `select.bindings.square: normal`（Options 短按仍是唯一统一退出）。
- Share 选模式其余不变：↑TEACH / ↓WAYPOINT / ←TRACK / →PIANO / △DEMO / ○DANCE。

### 3.2 WAYPOINT 模式键位（完全模仿 DANCE）

| 按键 | 动作 |
|---|---|
| `↓` | 开始录 wp1（`waypoints/wp1.yaml`） |
| `↑` | 结束并保存 wp1（录制中=保存；播放中=停止） |
| `←` | 开始录 wp2（`waypoints/wp2.yaml`） |
| `→` | 结束并保存 wp2 |
| `L2` | 打点（捕获当前 7 关节 + FK 位姿） |
| `△` 短按 / 长按 | 播放 wp1（单次 / 循环） |
| `○` 短按 / 长按 | 播放 wp2（单次 / 循环） |
| `□` | 停止（`edge: rising`，按下即停，回 READY） |
| `R2` | 循环切换规划方式（`edge: rising`，8 种，见 3.3） |
| `Options` 短按 | 退出回 NORMAL（录制中先保存） |

- 进入 WAYPOINT **不再自动开录**（槽位由 ↓/← 明确选择）；`enter` 钩子改为发布/复位规划方式。
- `exit` 保留 `teach_leave`，切走时录制中会自动保存。

### 3.3 规划方式循环（剔除 circ）

顺序与 HUD 简称：

| # | 规范名 | HUD | 说明 |
|---|---|---|---|
| 1 | `ptp` | PTP | 逐腿 OMPL+TOTG，每点停（默认） |
| 2 | `ptp_blend` | PTP+B | pilz PTP Sequence + 自适应 blend |
| 3 | `lin_exact` | LIN | pilz LIN，逐点精确停 |
| 4 | `lin_blend` | LIN+B | pilz LIN Sequence + blend |
| 5 | `jnt_g64_trapezoid` | G64-T | 自研 G64 圆角 + 梯形 |
| 6 | `jnt_g64_quintic` | G64-Q | 自研 G64 圆角 + quintic |
| 7 | `jnt_spline_bell` | SPLINE | 三次插值样条 + 钟形（精确过点） |
| 8 | `jnt_spline_approx` | FIT | 最小二乘 B 样条（仅端点精确） |

- 默认 `ptp`；`R2` 每按一次前进一格并回绕。
- `circ` 不在列表；服务端收到 `mode=circ` 返回明确错误：路点模式不支持 CIRC（需辅助点）。
- 模式只影响“下一次播放用哪种规划”；**播放中按 R2**：先 `/a3/arm/stop` 终止当前循环/运动，
  再切换显示，不会自动重播（避免误触导致突然换轨迹重跑）。

### 3.4 路点文件与槽位

```
trajectories/waypoints/
├── latest.yaml                 # 兼容旧行为（无槽位录制/空名回放）
├── wp1.yaml                    # ↓/↑ 录制的槽 1
├── wp2.yaml                    # ←/→ 录制的槽 2
└── teach_wp_<slot>_YYYYMMDD_HHMMSS.yaml   # 槽位历史备份
```

- 槽位重新录制成功后才覆盖，并先写备份。
- 文件结构不变：`kind: waypoint` + `waypoints[{positions(7), pose}]`。
- `melodion/waypoints/{press,extremes}.yaml` 保持原路径，不受影响（用 slot 命名空间区分）。

### 3.5 HUD

- 前端新增 `/a3/teleop/plan_mode`（`std_msgs/String`，值为规范名，1 Hz 重发 + 变化立即发）。
- `vision_display_mux` 第二行：`teleop_mode == "waypoint"` 时显示 `waypoint · <HUD 简称>`；
  其它模式仍只显示模式名。CPU/load 行不变。

## 4. 实施步骤（按顺序执行）

### 步骤 1：需求先行

新建 `docs/edge/requirements/F198-waypoint-dual-slots-plan-modes.md`（仿 `F197-dance-slots.md` 结构）：
背景、槽位目录、服务契约、PS4 键位表、8 模式表、循环语义、验收标准、关联文件。
在 `docs/edge/REQUIREMENTS.md` 表尾（F197 之后）新增 F198 行，状态 `implemented-pending-hw`。

### 步骤 2：扩展回放契约

`src/a3_msgs/srv/PlaybackTrajectory.srv` 在 `strategy` 后新增：

```text
# F198: 路点规划方式（type="waypoint" 时生效；留空=沿用旧 strategy ptp/lin）
# 允许: ptp|ptp_blend|lin_exact|lin_blend|jnt_g64_quintic|jnt_g64_trapezoid|jnt_spline_bell|jnt_spline_approx
string mode
```

保留 `strategy` 与旧路径，旧脚本/旧绑定行为不变（避免 `strategy=ptp` 被误解释为规范名 `ptp`）。

### 步骤 3：`arm_controller.py` —— 路点回放支持 8 模式

1. 模式白名单常量（放在 `TRAJ_MODES` 定义之后）：
   `WAYPOINT_PLAN_MODES = ("ptp","ptp_blend","lin_exact","lin_blend","jnt_g64_quintic","jnt_g64_trapezoid","jnt_spline_bell","jnt_spline_approx")`。
2. `_prepare_playback`（`6874`）：
   - `is_wp` 且 `req.mode` 非空时校验白名单；`circ` 返回
     `"waypoint mode 'circ' unsupported (needs auxiliary point)"`；
   - `req.mode` 为空时维持现有 `strategy in ("ptp","lin")` 校验。
3. `_playback_cb`（`7003`）：
   - **进入时先取消上一轮路点循环**：`self._wp_loop_stop.set()`，轮询等待
     `self._wp_loop_active == False`，超时 `waypoint_loop_cancel_timeout_s`（默认 30 s）
     则直接返回失败 `"previous waypoint loop still stopping; retry"`，避免两路运动并发；
   - 为本轮创建新的 `threading.Event()` 并赋给 `self._wp_loop_stop`；
   - `kind == "waypoint" and req.mode` → `_play_waypoints_mode(data, label, req.mode, req.loop, event, resp)`；
     否则走原 `_play_waypoints`（legacy）。
4. 新增 `_waypoint_segments(parsed, mode) -> (segments|None, err)`：
   - `ptp`：`n-1` 个无 via 段，`{"target": {"positions": wp.positions}, "mode": "ptp"}`（逐腿停）；
   - `lin_exact`：`n-1` 个无 via 段 `{"target": {"pose": wp.pose}, "mode": "lin_exact"}`；
     缺 `pose.position/orientation` 直接报错；
   - 其余 6 种：一条段，`via = wp[1..n-2]`、`target = wp[n-1]`；
     关节模式用 `{"positions": ...}`，`lin_blend` 用 `{"pose": ...}`。
5. 新增 `_play_waypoints_mode(...)`：
   - 复用 `_play_waypoints` 的解析/关节序归一/L7 统一/min_count 检查；
   - `lin_exact` 逐腿直接调 `_lin_move(pose, label, gate_q=wp.positions)`，保留 F107 静态重力矩门禁；
   - 其余段调 `_execute_segment`，失败重试 `waypoint_segment_retries` 次，
     段间 `_wait_move_group_current`（LL-103）；
   - L7：逐腿模式每段末 `_dispatch_l7_linear(wp[6])`；多 via 模式在整段末同步最后一个路点的 L7；
   - 循环：`loop=False` 执行一轮；`loop=True` 时每轮结束 `time.sleep(waypoint_loop_gap_s)`，
     且每轮/每段前检查 `stop_event.is_set()`，命中则返回
     `success=True, message="... loop stopped by operator"`；
   - 状态：首轮前已由 `_prepare_playback` 置 `STATE_TRAJ`/`TRAJ_RUNNING`；轮间不落 READY，
     结束（或取消）后 `_schedule_back_to_ready(0.3)`；
   - `finally` 中置 `self._wp_loop_active = False`。
   - 配置点不足时按 `docs/edge/TRAJECTORY_MODE_SELECTION.md:44-60` 打 WARN（模式退化），不硬失败。
6. `__init__`：新增 `self._wp_loop_stop = threading.Event()`、`self._wp_loop_active = False`。
7. `_stop_cb`（`6213`）：在 `_clear_dance_loop()` 旁新增 `self._wp_loop_stop.set()`（不等待，
   保证停止服务快速返回；刹停轨迹由现有 `_publish_hold_trajectory` 完成）。

### 步骤 4：`arm_controller.py` —— 路点双槽位录制

1. 新增服务（`1600` 附近，`dance` 服务之后）：
   `/a3/arm/waypoint/record1`、`/a3/arm/waypoint/record2`（`std_srvs/Trigger`），
   分别转调 `_begin_waypoint_teach("wp1")` / `("wp2")`。
2. `_begin_waypoint_teach`（`6326`）：`where` 提示按 slot 区分
   （`wp1/wp2` → `waypoints/<slot>.yaml`；`press/extremes` → `melodion/waypoints/<slot>.yaml`；空 → latest）。
3. `_stop_waypoint_teach`（`6442`）：目录路由改为
   - `slot in ("wp1","wp2")` → `self._wp_dir`，主文件 `<slot>.yaml`，备份 `teach_wp_<slot>_<ts>.yaml`；
   - `slot in ("press","extremes")` → 现有 `_melodion_wp_dir` 逻辑（含生成 stream）不变；
   - `slot is None` → 现有 `latest.yaml` 逻辑不变。
4. `arm_controller.yaml` 新增参数（路点段）：
   `waypoint_loop_gap_s: 0.2`、`waypoint_loop_cancel_timeout_s: 30.0`。

### 步骤 5：`a3_teleop_ps4` —— 键位、动作、规划方式状态

1. `mapping.py:325-327`：去掉 `if str(name) != "r2":` 的硬跳过，改为允许模式显式
   `suppress_common_axes: [r2]`；更新 docstring（仅 WAYPOINT 使用，其他模式行为不变）。
2. `actions.py`：
   - 常量 `WAYPOINT_PLAN_MODES`（与控制器一致）+ `WAYPOINT_PLAN_LABELS`（HUD 简称）；
   - `__init__`：`self._plan_mode = "ptp"`、发布器 `/a3/teleop/plan_mode`、
     `self._wp_slot = None`、`self._wp_slot_at = 0.0`；
   - `publish_plan_mode()` / `cycle_plan_mode()`（前进+回绕，发布并 `notify_operator_event("plan_mode:<name>")` 供灯条反馈）；
     `poll()` 里 1 Hz 重发；
   - `waypoint_enter()`：`_wp_slot=None`、发布当前 `_plan_mode`（作为 WAYPOINT 的 `enter` 钩子）；
   - `waypoint_record(slot)` / `waypoint_save(slot)`：完全仿 `dance_record/dance_save`
     （含未 READY 自动上电+挂起补发、槽位不符拒绝、`teach_stop` 保存）；
   - `playback_waypoints(slot, loop=False)`：`type="waypoint"`、`name=slot`、`mode=self._plan_mode`、`loop=loop`；
   - 删除 `playback_waypoints_ptp` / `playback_waypoints_lin` / `waypoint_toggle` / `teach_start_waypoint`
     （无绑定后不再使用）；
   - `_advance_pending_teach`：新增 `wp_` 分支（`wp1/wp2` → `/a3/arm/waypoint/record1|2`），
     并清理 `_wp_slot` 镜像（同 `_dance_slot` 宽限 1 s）。
3. `mappings/default.yaml`：
   - `select.bindings` 删除 `square: normal`；
   - `normal.buttons`：删除 `square` 的长按 `playback_waypoints_ptp`、`circle` 的长按
     `playback_waypoints_lin`（`square` 只留短按存点；`circle` 只留短按停止）；
   - `waypoint` 模式改写为：
     ```yaml
     enter: [{fn: waypoint_enter}]
     exit: [{fn: teach_leave}]
     suppress_common_axes: [r2]      # R2 用于切规划方式
     dpad:
       - axis: dpad_y
         pos: {fn: waypoint_record, kwargs: {slot: wp1}}
         neg: {fn: waypoint_save, kwargs: {slot: wp1}}
       - axis: dpad_x
         neg: {fn: waypoint_record, kwargs: {slot: wp2}}
         pos: {fn: waypoint_save, kwargs: {slot: wp2}}
     buttons:
       l2: {fn: save_pose_or_capture, edge: shortpress, hold_s: 1.5}
       triangle:
         - {fn: playback_waypoints, kwargs: {slot: wp1, loop: false}, edge: shortpress, hold_s: 1.5}
         - {fn: playback_waypoints, kwargs: {slot: wp1, loop: true},  edge: longpress,  hold_s: 1.5}
       circle:
         - {fn: playback_waypoints, kwargs: {slot: wp2, loop: false}, edge: shortpress, hold_s: 1.5}
         - {fn: playback_waypoints, kwargs: {slot: wp2, loop: true},  edge: longpress,  hold_s: 1.5}
       square: {fn: arm_stop, edge: rising}
       r2: {fn: cycle_plan_mode, edge: rising}
     ```
4. `action_registry.yaml`：新增 `waypoint_enter / waypoint_record / waypoint_save /
   playback_waypoints / waypoint_cycle_mode`（绑定 fn 名用 `cycle_plan_mode`，需与 registry 一致）；
   删除 `playback_waypoints_ptp` / `playback_waypoints_lin` / `teach_start_waypoint` 条目；
   修正过期的 description（`teach_start_waypoint` 的“Share 长按”叙述）。

### 步骤 6：HUD

`vision_display_mux.py`：
1. 参数 `plan_mode_topic`（默认 `/a3/teleop/plan_mode`），订阅更新 `self._plan_mode`。
2. `_draw_fps`（`453`）：`self._teleop_mode == "waypoint"` 且 `_plan_mode` 非空时，
   第二行显示 `f"waypoint · {短名}"`；短名映射表放本文件（8 项，见 3.3）。
3. 未进入 WAYPOINT 时不显示该后缀。

### 步骤 7：文档同步（workspace 规则要求）

- `docs/edge/PS4_OPERATOR_GUIDE.md`：删除 `Share + □` 行；改写 NORMAL（□/○ 长按已移除）；
  重写路点示教为双槽位；新增 R2 规划方式表 + HUD 说明；DANCE 的“路点槽不支持循环”改为
  “路点请到 WAYPOINT 模式播放/循环”。
- `src/a3_teleop_ps4/README.md`：键位表同步。
- `docs/shared/TOPIC_CONTRACT.md`：新增 `/a3/teleop/plan_mode` 与 `PlaybackTrajectory.mode`。
- `docs/shared/TRAJECTORY_KINDS.md`：waypoint 回放增加 8 模式（circ 不支持）、循环语义。
- `docs/edge/TRAJECTORY_MODE_SELECTION.md`：补充“waypoint 任务按模式转段执行”的说明。
- `docs/edge/requirements/F198-...md` + `docs/edge/REQUIREMENTS.md`（步骤 1 已建）。

## 5. 决策与假设

- 已确认：WAYPOINT 内 R2 切模式（屏蔽 R2 夹爪）；键位完全模仿 DANCE；删除 NORMAL 旧路点回放绑定。
- 默认规划方式 `ptp`（逐腿停），HUD 可见；不沿用旧 `strategy=ptp` 的 blend 语义。
- `strategy` 旧链路保留（旧脚本/外部调用），新链路只认 `mode`。
- `circ` 仅从路点模式剔除，`TRAJ_MODES` 注册表本身保留（stream/cp 仍可用）。
- 路点循环不做接缝插值：每轮重新规划“末点→wp1”，速度/加速度由各模式自身保证。
- R2 使用 hid-sony 的数字键 `buttons.r2:7`；若真机不触发，退化方案是给 `r2` 轴加虚拟键
  （`op: lt`，阈值按 `trigger_rest/pressed` 标定），不改上层动作。

## 6. 验证步骤

1. **构建**：
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select a3_msgs a3_arm_controller a3_teleop_ps4 a3_bringup --symlink-install
   source install/setup.bash
   ```
2. **映射校验**：启动 `ps4_mapper`（或直接调用 `validate_mapping`）零报错；
   `default.yaml` 中 `r2` 绑定能通过 registry 校验。
3. **服务级（仿真，带 pilz）**：`ros2 launch a3_bringup edge_full_mock.launch.py`，确认
   - `ros2 service call /a3/arm/waypoint/record1 std_srvs/srv/Trigger` 进入 `WAYPOINT_TEACH`；
   - `/a3/arm/stop` 后生成 `trajectories/waypoints/wp1.yaml` + 备份；
   - 对 8 个模式逐个 `ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory
     "{type: waypoint, name: wp1, mode: <mode>}"`，均能执行；
   - `mode: circ` 被拒且信息明确；
   - `loop: true` 时循环，`/a3/arm/stop` 能在一次腿内终止并回 READY。
4. **手柄级**：`joy_dump` 确认 R2 数字键触发；进入 WAYPOINT 后
   `ros2 topic echo /a3/teleop/plan_mode` 随 R2 变化；`/a3/teleop/mode` 仍只发 `waypoint`。
5. **HUD**：RViz Image（`/a3/vision/display`）右上角显示 `waypoint · PTP` 等，切 R2 立即变化。
6. **回归**：NORMAL 的 △/□/○/D-pad、DANCE 双槽位与循环、PIANO 录制/播放、TEACH 进出均不受影响；
   `□` 长按不再触发路点回放。

## 7. 风险与规避

| 风险 | 规避 |
|---|---|
| 路点循环与 `/a3/arm/stop` 并发导致继续运动 | 取消事件 + 每段/每轮检查；新回放请求先等旧循环落 `active=False`，超时直接拒绝 |
| 两路回放并发（旧循环未退、新请求已起） | `_wp_loop_active` + 取消等待，超时失败返回；不引入阻塞锁（避免 2 线程死锁） |
| WAYPOINT 内无法手调夹爪 | 文档明确；夹爪角度随路点录制/回放；需要手调时切回 NORMAL（R2 恢复力控） |
| LIN 模式缺 FK 位姿 | 录制时 FK 失败已有 WARN；播放前显式拒绝并提示重录 |
| 模式退化（点数不足） | 按 `TRAJECTORY_MODE_SELECTION.md` WARN，不静默改变已选模式名 |
