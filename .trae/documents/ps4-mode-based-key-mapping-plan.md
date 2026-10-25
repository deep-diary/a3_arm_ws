# PS4 模式化按键映射（Share 组合键直接选模式）

## 一、Summary

把 `a3_teleop_ps4` 从**单层映射**升级为**「固定层 + 模式层」双层结构**，并用**修饰键直接点名模式**（不循环）：

- **固定层**：急停 / 上电 / 失能 / init / 夹爪 / 摇杆伺服，任何模式都不变。
- **模式选择**：`按住 Share + 方向键/面键` 直接进入目标模式，**不经过中间模式**：
  - `Share + D-pad ↑` = 连续示教 TEACH
  - `Share + D-pad ↓` = 路点示教 WAYPOINT
  - `Share + D-pad ←` = 跟踪 TRACK
  - `Share + D-pad →` = 弹琴 PIANO
  - `Share + Triangle` = 演示 DEMO
  - `Share + Square` = 正常 NORMAL
- **统一退出**：`Options` 短按 = 结束/退出当前模式 → NORMAL（示教类会先保存）。
- **进入策略分级**：示教/路点只切语义（待命），模式内按 `Triangle` 才开始录制；跟踪进入即 enable；弹琴进入即定位。
- **反馈**：灯带颜色 + 呼吸/常亮区分模式与录制态，切换时短震 N 次，并发布 `/a3/teleop/mode`。

关键点：`Share` 由原来的"开始示教"独立动作键**改为纯模式修饰键**（单独按住不产生任何动作），这样 5 个非正常模式各有唯一入口，互不误触。

## 二、Current State Analysis

### 2.1 现有实现

| 文件 | 现状 |
|---|---|
| [default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml) | 单层映射：`speed_normal + axes + dpad + buttons`。`Share` 短/长 = 开始连续/路点示教；`Options` 短/长 = 停止/巡游；`Triangle/Circle/Square` 各带短+长双义；`D-pad` = 口风琴四点 |
| [mapping.py](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/mapping.py) | `Ds4Layout`、`ButtonEdgeTracker`（rising/longpress/shortpress）、`iter_axis_bindings` / `iter_button_bindings`、`validate_mapping`（只校验顶层三层） |
| [ps4_mapper.py](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py#L125-L194) | 50 Hz 定时器：`pose_blocking` → 逐轴 gates → 逐键边沿 → D-pad 边沿。不知道编排层状态，无模式概念 |
| [actions.py](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py) | 动作实现；`_call_trigger` 不消费响应；未订阅 `arm_status`，无法判断"当前是否在录制" |
| [action_registry.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/action_registry.yaml) | `validate_mapping` 强校验绑定 `fn` 必须在此登记 |
| [ds4_linux.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/ds4_linux.yaml#L46-L68) | 已有 `l1_r1` 组合虚拟键、`dpad_up/down/left/right` 轴阈值虚拟键——**模式选择可直接复用 `dpad_*`，无需新增虚拟键** |
| [ds4_feedback_node.py](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/ds4_feedback_node.py#L283-L337) | `_derive()` 按 `arm_status / power / monitor` 派生灯色；无模式输入 |

### 2.2 关键约束

- `ButtonEdgeTracker.shortpress` 是**松手时判定**（[mapping.py#L163-L184](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/mapping.py#L163-L184)）；模式选择要用 **rising（按下即触发）**，避免松手延迟。
- `Ds4Layout.button()` 已支持 `dpad_up/down/left/right` 虚拟键（带迟滞），可直接当"Share + 方向"的方向源。
- 编排层 `SERVO` 是死枚举；PS4 伺服体现在 `/a3/control_mode = SERVO`。**不新增编排层状态**。
- `/a3/face_track/enable` 是 `std_srvs/SetBool`（[face_track_controller.py#L248](file:///home/cat/a3_arm_ws/src/a3_perception/a3_perception/face_track_controller.py#L248)），mapper 目前无该 client。
- `save_named_pose` 只有在 `WAYPOINT_TEACH` 状态才会被服务端重路由为"打点"（[arm_controller.py:5725](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L5725)）。
- 点位：`home / idle / ready` 与 `melodion_white1/white19/black1/black13`（[named_poses.yaml#L102-L109](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml#L102-L109)）。
- `PS` 是手柄开机/唤醒键，F135 已解绑；**本次保持解绑**，不引入 PS 快捷键。

## 三、目标按键方案（决策定稿）

### 3.1 固定层（所有模式恒定，模式不可覆盖）

| 按键 | 手势 | 动作 | 底层 |
|---|---|---|---|
| Cross | 长按 1.0s | 硬急停（断电关闸） | `/power_sequence/command shutdown` |
| L3 | 短按 | 上电 + 使能 | `power start` → `/a3/arm/enable` |
| R3 | 短按 | 软失能（先退模式再 safe-park） | `/a3/arm/disable` |
| L1+R1 | 长按 2.0s | init 设零 | `/a3/arm/init` |
| L1 + 摇杆 | 按住 | 末端平移 X/Y/Z | `/servo_node/delta_twist_cmds` |
| R1 + 摇杆 | 按住 | 末端旋转 X/Y/Z | 同上 |
| R2 | 模拟量 | 夹爪力控 | `/gripper_controller/gripper_cmd` |
| **Share（按住）** | 修饰键 | 进入模式选择窗口，**屏蔽其它所有按键与摇杆** | mapper 内建 |
| **Options** | 短按 | 结束/退出当前模式 → NORMAL | mapper 内建 + 模式 exit hook |

### 3.2 模式选择（按住 Share 时生效，方向键/面键按下即触发）

| 组合 | 目标模式 | 灯色（已使能时） |
|---|---|---|
| Share + D-pad ↑ | **TEACH** 连续示教 | 蓝：待命常亮 / 录制中呼吸 |
| Share + D-pad ↓ | **WAYPOINT** 路点示教 | 青：待命常亮 / 录制中呼吸 |
| Share + D-pad ← | **TRACK** 人脸跟踪 | 紫呼吸 |
| Share + D-pad → | **PIANO** 弹琴 | 黄呼吸 |
| Share + Triangle | **DEMO** 演示 | 白呼吸 |
| Share + Square | **NORMAL** 正常 | 绿常亮 |

规则：
1. 按住 Share 期间，本 tick 只处理模式选择，**不执行**任何普通按键、D-pad 动作或摇杆伺服。
2. 方向键/面键用 **rising 边沿**触发；同一方向回中后才能再次触发。
3. 已处于目标模式时再选一次 = 幂等（不重复执行 enter/exit hook，仅重发模式话题）。
4. 松开 Share 不产生任何动作。

### 3.3 各模式按键

#### NORMAL（手动 / 通用回放）— 绿常亮
| 按键 | 手势 | 动作 |
|---|---|---|
| Triangle | 短按 1.5s | goto `home` |
| Triangle | 长按 1.5s | goto `ready`（示教/弹琴前预备位） |
| Circle | 短按 1.5s | goto `idle` |
| Circle | 长按 1.5s | 路点 LIN 回放 |
| Square | 短按 1.5s | 回放 latest 连续轨迹 |
| Square | 长按 1.5s | 路点 PTP 回放 |
| L2 | 短按 | 保存当前位姿 `snap_*` |
| D-pad ↑ / ↓ | 点按 | 平移速度档 +0.15 / −0.15 |
| D-pad ← / → | 点按 | 旋转速度档 −0.15 / +0.15 |
| **Options** | 短按 | 停止当前运动（`stop_motion`：零 twist + pause servo） |

#### TEACH（连续示教）
- 进入：**只切语义**，不启动录制（灯蓝常亮 = 待命）
- 退出：`Options` → 若在录制中先 `stop_teach`（自动保存）再回 NORMAL

| 按键 | 手势 | 动作 |
|---|---|---|
| Triangle | 短按 | **toggle**：未录制 → `start_teach`；录制中 → `stop_teach`（保存，仍留在 TEACH 待命） |
| L2 | 短按 | 保存当前点位 `snap_*` |

#### WAYPOINT（路点示教）
- 进入：只切语义（灯青常亮 = 待命）
- 退出：`Options` → 若在录制中先 `stop_teach`（自动保存）再回 NORMAL

| 按键 | 手势 | 动作 |
|---|---|---|
| Triangle | 短按 | 未录制 → `start_waypoint_teach`；录制中 → **打点**（`save_named_pose`，服务端重路由） |
| L2 | 短按 | 录制中 → 打点；未录制 → 保存 `snap_*`（兼容旧习惯） |

#### TRACK（人脸跟踪）
- 进入：`/a3/face_track/enable {data: true}`（**required**，失败回退 NORMAL）
- 退出：`/a3/face_track/enable {data: false}`
- `common_axes: false`：**禁用 L1/R1 摇杆**，避免与视觉伺服抢 twist 话题

| 按键 | 手势 | 动作 |
|---|---|---|
| Triangle | 短按 | toggle 跟踪：enable ⇄ disable（暂停/恢复） |
| Circle | 短按 | `/a3/face_track/reload_jacobian` 重标定 |
| L2 | 短按 | 保存当前点位 |
| Options | 短按 | disable 跟踪 → NORMAL |

#### PIANO（弹琴）
- 进入：goto `melodion_white1`（**optional**，被拒仅 WARN 不回退）
- 退出：无动作

| 按键 | 手势 | 动作 |
|---|---|---|
| D-pad ↑ / ↓ / ← / → | 点按 | goto `melodion_black13 / black1 / white1 / white19` |
| Triangle | 短按 1.5s | 演奏 `stream latest` |
| Circle | 短按 | goto `melodion_white1`（回起始键） |
| Square | 短按 | goto `melodion_white19`（到最右键） |
| L2 | 短按 | 保存当前对位点 |

#### DEMO（演示）
- 进入：goto `ready`（**optional**，失败不回退）
- 退出：无动作

| 按键 | 手势 | 动作 |
|---|---|---|
| Triangle | 短按 | 随机命名点位巡游 `random_pose_tour` |
| Square | 短按 | 回放 latest 连续轨迹 |
| Circle | 短按 | goto `ready` 复位 |
| L2 | 短按 | 保存当前点位 |

### 3.4 与旧映射的差异（需要写进文档）

| 旧 | 新 |
|---|---|
| Share 短按 = 开始连续示教 | Share 改为纯修饰键；`Share+↑` 进入 TEACH，再按 Triangle 开始 |
| Share 长按 = 开始路点示教 | `Share+↓` 进入 WAYPOINT，再按 Triangle 开始 |
| Options 短按 = 停止示教 | Options 统一 = 结束/退出当前模式 |
| Options 长按 = 随机巡游 | 迁到 DEMO 模式 `Triangle` |
| Triangle 长按 = 弹琴 stream | 迁到 PIANO 模式 `Triangle` |
| D-pad = 口风琴四点 | 迁到 PIANO 模式；NORMAL 下改为速度档 |
| L2 = 保存点位 | 保留（WAYPOINT 录制中 = 打点） |

## 四、Proposed Changes

### 4.1 `src/a3_teleop_ps4/config/mappings/default.yaml`（重构）

```yaml
speed_normal: 0.35
joy_alive_ticks: 25

common:                       # 固定层：所有模式生效，模式不可覆盖
  axes:                       # 与现状一致
    r2: {fn: gripper_force, kind: analog_01, source: trigger}
    left_x: [{fn: servo_lin_y, kind: analog_n11, gates: [l1]},
             {fn: servo_ang_x, kind: analog_n11, gates: [r1]}]
    left_y:  {fn: servo_lin_z, kind: analog_n11, gates: [l1]}
    right_y: [{fn: servo_lin_x, kind: analog_n11, gates: [l1]},
              {fn: servo_ang_y, kind: analog_n11, gates: [r1]}]
    right_x: {fn: servo_ang_z, kind: analog_n11, gates: [r1]}
  buttons:
    cross: {fn: power_shutdown, edge: longpress, hold_s: 1.0}
    l1_r1: {fn: arm_init, edge: longpress, hold_s: 2.0}
    l3:    {fn: arm_power_enable, edge: shortpress, hold_s: 3.0}
    r3:    {fn: arm_disable, edge: shortpress, hold_s: 3.0}
  dpad: []

modes:
  select:                     # 按住 modifier 才生效；方向/面键 rising 触发
    modifier: share
    bindings:
      dpad_up: teach
      dpad_down: waypoint
      dpad_left: track
      dpad_right: piano
      triangle: demo
      square: normal
  exit:                       # 统一退出键
    button: options
    edge: shortpress
    hold_s: 3.0

  normal:
    label: NORMAL
    color: green
    pattern: solid
    rumble_index: 1
    common_axes: true
    enter: []
    exit: []
    axes: {}
    buttons:
      triangle:
        - {fn: goto_named_pose, kwargs: {name: home},  edge: shortpress, hold_s: 1.5}
        - {fn: goto_named_pose, kwargs: {name: ready}, edge: longpress,  hold_s: 1.5}
      circle:
        - {fn: goto_named_pose, kwargs: {name: idle}, edge: shortpress, hold_s: 1.5}
        - {fn: playback_waypoints_lin, edge: longpress, hold_s: 1.5}
      square:
        - {fn: playback_latest, edge: shortpress, hold_s: 1.5}
        - {fn: playback_waypoints_ptp, edge: longpress, hold_s: 1.5}
      l2: {fn: save_named_pose, edge: shortpress, hold_s: 3.0}
    dpad:
      - axis: dpad_y
        neg: {fn: step_linear_scale,  kwargs: {delta: 0.15}}
        pos: {fn: step_linear_scale,  kwargs: {delta: -0.15}}
      - axis: dpad_x
        neg: {fn: step_angular_scale, kwargs: {delta: -0.15}}
        pos: {fn: step_angular_scale, kwargs: {delta: 0.15}}

  teach:
    label: TEACH
    color: blue
    pattern: breathe
    rumble_index: 2
    common_axes: true
    enter: []
    exit: [{fn: teach_stop_if_active}]
    axes: {}
    dpad: []
    buttons:
      triangle: {fn: teach_toggle, edge: shortpress, hold_s: 3.0}
      l2:       {fn: save_named_pose, edge: shortpress, hold_s: 3.0}

  waypoint:
    label: WAYPOINT
    color: cyan
    pattern: breathe
    rumble_index: 3
    common_axes: true
    enter: []
    exit: [{fn: teach_stop_if_active}]
    axes: {}
    dpad: []
    buttons:
      triangle: {fn: waypoint_capture_toggle, edge: shortpress, hold_s: 3.0}
      l2:       {fn: save_named_pose_or_capture, edge: shortpress, hold_s: 3.0}

  track:
    label: TRACK
    color: purple
    pattern: breathe
    rumble_index: 4
    common_axes: false        # 禁用 L1/R1 摇杆
    enter: [{fn: face_track_enable, required: true}]
    exit:  [{fn: face_track_disable}]
    axes: {}
    dpad: []
    buttons:
      triangle: {fn: face_track_toggle, edge: shortpress, hold_s: 3.0}
      circle:   {fn: face_track_reload_jacobian, edge: shortpress, hold_s: 3.0}
      l2:       {fn: save_named_pose, edge: shortpress, hold_s: 3.0}

  piano:
    label: PIANO
    color: yellow
    pattern: breathe
    rumble_index: 5
    common_axes: true
    enter: [{fn: goto_named_pose, kwargs: {name: melodion_white1}, required: false}]
    exit: []
    axes: {}
    dpad:
      - axis: dpad_y
        neg: {fn: goto_named_pose, kwargs: {name: melodion_black13}}
        pos: {fn: goto_named_pose, kwargs: {name: melodion_black1}}
      - axis: dpad_x
        neg: {fn: goto_named_pose, kwargs: {name: melodion_white1}}
        pos: {fn: goto_named_pose, kwargs: {name: melodion_white19}}
    buttons:
      triangle: {fn: playback_stream, kwargs: {name: latest}, edge: shortpress, hold_s: 1.5}
      circle:   {fn: goto_named_pose, kwargs: {name: melodion_white1},  edge: shortpress, hold_s: 3.0}
      square:   {fn: goto_named_pose, kwargs: {name: melodion_white19}, edge: shortpress, hold_s: 3.0}
      l2:       {fn: save_named_pose, edge: shortpress, hold_s: 3.0}

  demo:
    label: DEMO
    color: white
    pattern: breathe
    rumble_index: 6
    common_axes: true
    enter: [{fn: goto_named_pose, kwargs: {name: ready}, required: false}]
    exit: []
    axes: {}
    dpad: []
    buttons:
      triangle: {fn: random_pose_tour, edge: shortpress, hold_s: 3.0}
      square:   {fn: playback_latest, edge: shortpress, hold_s: 3.0}
      circle:   {fn: goto_named_pose, kwargs: {name: ready}, edge: shortpress, hold_s: 3.0}
      l2:       {fn: save_named_pose, edge: shortpress, hold_s: 3.0}
```

> 兼容：`simple.yaml` 无 `modes:` 段 → mapper 退化为单模式（顶层 `axes/buttons/dpad` 直接生效），不需要改 `simple.yaml`。

### 4.2 `src/a3_teleop_ps4/a3_teleop_ps4/mapping.py`

新增纯函数（不改现有公开 API）：

1. `has_modes(mapping) -> bool`
2. `load_modes(mapping) -> dict`：返回 `{select, exit, order, modes}`；`order` 由 `modes` 中非 `select/exit` 键按 YAML 出现顺序生成。
3. `resolve_mode(mapping, mode_name) -> dict`：返回 `{axes, buttons, dpad}`：
   - 有 `modes`：以 `common` 为底，`modes.<name>` 同名键覆盖；`common_axes: false` → `axes = {}`。
   - 无 `modes`：直接返回顶层三层（旧行为）。
4. `validate_mapping` 扩展：
   - 遍历 `common` + 每个模式的三层 + `enter/exit` 的 `fn`，按 registry 校验；
   - 校验 `select.bindings` 的目标模式名存在于 `modes`；`select.modifier` / `exit.button` 非空；
   - 校验每个模式有 `label/color`；
   - `iter_button_bindings` / `iter_axis_bindings` 签名不变，入参改为 `resolve_mode()` 的产物。

### 4.3 `src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py`

新增状态：
- `self._modes = load_modes(mapping)`；`self._mode = "normal"`（无 `modes` 时为 `None`）
- `self._select_edges = ButtonEdgeTracker()`（模式选择专用边沿，独立于普通按键）
- `self._mode_guard_until = 0.0`（切换后 300 ms 屏蔽）
- 发布者 `/a3/teleop/mode`（`std_msgs/String`，纯模式名，如 `teach`）

`_on_timer` 重排（其余保持不变）：

```
1. joy 超时 / 未 ready → return
2. if _handle_mode_select(joy, now): return      # Share 按住：只做模式选择
3. if _handle_mode_exit(joy, now): return        # Options 短按：退出当前模式
4. if now < _mode_guard_until: return
5. resolve_mode(self._mode) → 执行轴 / 按键 / D-pad（现有逻辑）
```

`_handle_mode_select(joy, now) -> bool`
- `modifier` 未按住：`self._select_edges.reset()`，返回 `False`
- 按住：遍历 `select.bindings`，对每个 `source`（`dpad_up` 等虚拟键 / `triangle` 等物理键）取 `held`，`rising` 命中 → `_switch_mode(target, now)` 并返回 `True`
- 未命中任何方向也返回 `True`（吞掉本 tick，屏蔽摇杆与普通按键）

`_handle_mode_exit(joy, now) -> bool`
- `exit.button` 用 `shortpress(hold_s)` 判定；命中 → `NORMAL` 下执行 `stop_motion`，其它模式执行 `_switch_mode("normal", now)`

`_switch_mode(target, now, *, run_exit=True)`
1. `target == self._mode` → 直接 return（幂等）
2. `run_exit` 时执行旧模式 `exit` hooks（失败只 WARN）
3. 设 `self._mode = target`，重置 `ButtonEdgeTracker` 与 `_dpad_active`
4. 执行新模式 `enter` hooks；`required: true` 失败 → 回滚到 `normal` + WARN + `/a3/arm/operator_event` 发 `mode_rejected:<target>`
5. `_mode_guard_until = now + 0.3`
6. 发布 `/a3/teleop/mode`；`/a3/arm/operator_event` 发 `mode_changed:<target>:<rumble_index>`

`enter/exit` hook 执行：新增 `self._exec.run_hook(spec, on_done)`，异步等待服务响应，用 `required` 决定是否回滚。

### 4.4 `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`

新增：

1. **订阅 `/a3/arm_status`**（`a3_msgs/ArmStatus`），保存 `self.arm_state`；供录制态判断使用。
2. `_call_trigger_cb(client, label, on_result)`：带响应回调的 Trigger 调用。
3. `teach_toggle()`：`arm_state == "TEACH"` → `teach_stop()`，否则 `teach_start()`。
4. `waypoint_capture_toggle()`：`arm_state == "WAYPOINT_TEACH"` → `save_named_pose()`（打点），否则 `teach_start_waypoint()`。
5. `save_named_pose_or_capture()`：`arm_state == "WAYPOINT_TEACH"` → 打点，否则 `save_named_pose()`。
6. `teach_stop_if_active()`：`arm_state in ("TEACH","WAYPOINT_TEACH")` 时才调 `/a3/arm/stop_teach`，否则静默跳过（避免模式 exit 时产生拒绝日志）。
7. `face_track_enable()` / `face_track_disable()`：`std_srvs/SetBool` client（服务名参数 `face_track_enable_service`，默认 `/a3/face_track/enable`）。
8. `face_track_toggle()`：按本地 `self._face_track_on` 标志 toggle（进入模式时置 `True`）。
9. `face_track_reload_jacobian()`：`Trigger` client `/a3/face_track/reload_jacobian`。
10. `run_hook(spec, on_done)`：统一执行 `{fn, kwargs, required}`，回调 `(ok, required)`。

### 4.5 `src/a3_teleop_ps4/config/action_registry.yaml`

新增登记（`kind: discrete`）：
`teach_toggle`、`waypoint_capture_toggle`、`save_named_pose_or_capture`、`teach_stop_if_active`、`face_track_enable`、`face_track_disable`、`face_track_toggle`、`face_track_reload_jacobian`。

`playback_latest / playback_stream / playback_waypoints_* / random_pose_tour / step_linear_scale / step_angular_scale / stop_motion` 已存在。

### 4.6 `src/a3_teleop_ps4/a3_teleop_ps4/ds4_feedback_node.py`

- 订阅 `/a3/teleop/mode`（`String`），保存 `self._mode`、`self._mode_rumble_index`
- `COLORS` 新增 `yellow: (255, 200, 0)`
- `_derive()` 优先级（安全色永远最高）：
  1. `FAULT/COOLING/temp_warn/monitor TRIGGERED` → 红双闪
  2. 失电 / `INIT` / `POWERING` → 红闪 / 橙
  3. `arm_state == TEACH` → 蓝呼吸；`WAYPOINT_TEACH` → 青呼吸（录制中）
  4. `TRAJ / SAFE_PARK` → 紫/青（不变）
  5. **模式非 `normal` → 模式色 + 模式 pattern**（TEACH/WAYPOINT 待命时即为蓝/青常亮）
  6. `READY/SERVO` → 绿（NORMAL）
  7. `DISABLED/IDLE` → 橙（安全优先，不显示模式色）
- `/a3/ds4/feedback` JSON 增加 `"mode": self._mode` 字段（便于调试/Web）
- 模式切换震动：检测 `self._mode` 变化，生成 `rumble_index` 段 `_RumblePhase(0.18*i, 0.08, 0.7, 0.0)`；NORMAL 震 1 次
- 新增参数 `mode_topic`（默认 `/a3/teleop/mode`）、`mode_rumble_step_s`（默认 0.18）

### 4.7 文档同步（仓库规则要求）

| 文件 | 改动 |
|---|---|
| `docs/edge/requirements/F188-mode-based-ps4-mapping.md` | **新建**：6 模式、Share 组合键表、分级进入语义、Options 退出、验收标准 |
| `docs/edge/REQUIREMENTS.md` | 索引加 F188 一行，状态 `implemented-pending-hw` |
| `src/a3_teleop_ps4/README.md` | 改为「固定层 + 模式选择 + 6 模式」四张表，说明 Share 已改修饰键 |
| `docs/edge/PS4_OPERATOR_GUIDE.md` | 操作流程加"选模式"；灯色表补 TRACK 紫 / PIANO 黄 / DEMO 白 / 示教待命常亮 |
| `docs/shared/TOPIC_CONTRACT.md` | 新增 `/a3/teleop/mode`（String，HMI 层模式名） |
| `docs/edge/STATE_MACHINE.md` | 新增"HMI 模式 vs 编排状态"小节：模式属 HMI 层，不新增编排态；TEACH/WAYPOINT 模式与状态解耦（待命/录制由 `arm_status` 区分） |
| `docs/edge/requirements/F131-waypoint-teach.md` | 标注 L2 打点保留、新增 Triangle 打点入口 |

### 4.8 测试脚本

`scripts/a3_test/ps4_sim_test.py` 新增场景（合成 `/joy`，无硬件）：

- **S11 直接选模式**：按住 `share` + `dpad_up` → `/a3/teleop/mode == "teach"`；松手后 `share`+`dpad_left` → `"track"`（**不经过 teach→waypoint**）。
- **S12 Share 屏蔽**：按住 `share` 时按 `triangle`（无方向）不得触发 NORMAL 的 `goto home`。
- **S13 示教分级**：进入 `teach` 后 `arm_status` 仍非 `TEACH`；按 `triangle` 才出现 `start_teach` 调用；再按 `triangle` 出现 `stop_teach`。
- **S14 路点打点**：进入 `waypoint` → `triangle` 开始录制 → 再 `triangle` 触发 `save_named_pose`（打点）。
- **S15 Options 退出**：`teach` 录制中按 `options` → 先 `stop_teach` 再 `/a3/teleop/mode == "normal"`；`normal` 下按 `options` → `stop_motion`。
- **S16 TRACK 轴封锁**：`track` 模式下 `L1+left_x` 不得产生 `/servo_node/delta_twist_cmds`。
- **S17 进入失败回退**：`DISABLED` 下选 `track`（`face_track` 服务不可用时）→ 模式回 `normal` + `mode_rejected` 事件。

## 五、Assumptions & Decisions

| 编号 | 决策 | 理由 |
|---|---|---|
| D1 | 模式在 **HMI 层**实现，不改编排层 12 态 | 模式是按键复用问题；新增 PIANO/TRACK/DEMO 编排态会污染安全状态机 |
| D2 | `Share` 改为纯修饰键（单独按住无动作） | 用直接组合键点名模式，不会像循环那样"路过示教"；单键循环被否决 |
| D3 | 模式选择用 **rising 边沿**（按下即触发），`Options` 退出用 `shortpress`（松手判定） | 选模式要即时；退出键沿用现有短按语义，防止误触 |
| D4 | 固定层（Cross/L3/R3/L1+R1/L1/R1/R2）完全不动 | 用户选定"固定层保留，复用层全面重排" |
| D5 | 进入策略分级：示教/路点待命，跟踪自动 enable，弹琴自动定位 | 用户选定；避免"选个模式就进零力矩" |
| D6 | `Options` 统一退出；NORMAL 下 = `stop_motion` | 用户选定；`stop_motion` 已注册未绑定，正好补空 |
| D7 | TRACK 模式 `common_axes: false` | 视觉伺服与手柄共用 `/servo_node/delta_twist_cmds`，必须互斥 |
| D8 | NORMAL 的 D-pad 改为速度档调节；口风琴四点迁入 PIANO | 弹琴用点不再占用 NORMAL；速度档是唯一无入口的手感参数 |
| D9 | 保持 `PS` 解绑 | PS 是开机/唤醒键，短按有断连风险（F135 已解绑） |
| D10 | 模式名发布为纯字符串；灯色映射留在 feedback 节点 | 避免 HMI 与反馈耦合，Web 侧可直接用 |
| D11 | 不改 `simple.yaml` | 无 `modes:` 段时 mapper 自动退化为单模式 |
| D12 | DEMO 模式不接入 F185 三轴巡航脚本 | 该能力只有 CLI 入口、无 ROS 服务，作为已知缺口记录在 F188 |

## 六、Verification

1. **构建**：`colcon build --packages-select a3_teleop_ps4 --symlink-install`
2. **YAML 自检**：启动 `ps4_mapper` 必须零 ERROR（`validate_mapping` 会校验 6 模式全部 `fn` 与 `select.bindings` 目标）
3. **无手柄逻辑验证**：
   - `ros2 run a3_teleop_ps4 ps4_mapper`
   - 另起终端合成 `/joy`（`share=1` + `dpad_up=1` 数帧）→ `ros2 topic echo /a3/teleop/mode` 应为 `teach`，且**不出现** `waypoint`
4. **灯带逻辑验证**：`ros2 topic echo /a3/ds4/feedback`，切模式时 `color/pattern` 依次为 `blue/cyan/purple/yellow/white` 呼吸，`mode` 字段同步；震动脉冲数 = `rumble_index`
5. **分级进入验证**：切到 `teach` 后 `ros2 topic echo /a3/arm_status` 仍非 `TEACH`；按 Triangle 后变 `TEACH`
6. **退出验证**：录制中按 Options → 日志出现 `stop_teach`，`/a3/teleop/mode` 变 `normal`
7. **固定层回归**：Cross 长按断电、L3 上电使能、R3 失能、L1+R1 init、R2 夹爪、L1/R1 摇杆伺服逐项复测
8. **真机验收**（待硬件）：按 `PS4_OPERATOR_GUIDE.md` 新流程走一遍六个模式
