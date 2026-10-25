# PS4 模式化按键映射 v3（7 模式 + 切模式先停运动）

## 一、Summary

把 `a3_teleop_ps4` 从单层映射升级为**「固定层 + 模式层」**，用 **`按住 Share + 方向/面键`直接点名模式**（不循环）：

| 组合 | 模式 |
|---|---|
| Share + D-pad ↑ | TEACH 连续示教 |
| Share + D-pad ↓ | WAYPOINT 路点示教 |
| Share + D-pad ← | TRACK 人脸跟踪 |
| Share + D-pad → | PIANO 弹琴 |
| Share + Triangle | DEMO 演示 |
| Share + Circle | **DANCE 舞蹈（新增）** |
| Share + Square | NORMAL 正常 |

- **统一退出**：`Options` 短按 = 结束/退出 → NORMAL（示教类先保存）。
- **切模式前必须先停运动**：新增编排层 `/a3/arm/stop`，切换流程先停当前轨迹/示教/servo，再执行新旧模式的 enter/exit hook，杜绝串扰。
- **进入策略分级**：示教/路点只切语义（待命），模式内 `Triangle` 才开录；TRACK 进入即 enable；PIANO 进入即定位。
- 为支撑新按键，本次同步补 4 项底层能力：编排层停止服务、TRACK 标定补全、口风琴 stream 生成服务、角度巡游节点 + 舞蹈槽位。

## 二、Current State Analysis（本轮新增确认）

| 结论 | 证据 |
|---|---|
| 编排层**没有**任何 stop/pause/cancel 服务；MoveGroup/FJT goal_handle 都是局部变量，从未保存 | [arm_controller.py#L1532-L1579](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L1532-L1579)、`#L3956-L3970`、`#L2553-L2572` |
| 唯一"轻停"在执行层 `/a3/motor/stop`，但它**不回收编排状态**（仍停在 TRAJ 直到 `_traj_done_at`） | `motor_protocol_node.cpp:2258-2340`、[arm_controller.py#L2922-L2935](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L2922-L2935) |
| `playback` 期间所有编排服务都被 `_can_move` 拒绝（`STATE_TRAJ` 在拒绝表） | [arm_controller.py#L2350-L2365](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L2350-L2365) |
| TRACK 只有 2 个服务：`enable(SetBool)`、`reload_jacobian(Trigger)`；**目标框中心硬编码为画面中心**，bbox 只发不用 | [face_track_controller.py#L248-L250](file:///home/cat/a3_arm_ws/src/a3_perception/a3_perception/face_track_controller.py#L248-L250)、检测节点 `309/321-324` |
| 检测节点**没有任何 service**，`depth_units_mm` 是运行时参数 | `face_detect_node.py:206-212` |
| `melodion_pts2stream.py` 是纯离线 CLI，无 ROS 包装；stream 支持 `mode/via/sleep/hold_orientation`，但 `playback` 服务只能传 `name/type/strategy` | [melodion_pts2stream.py](file:///home/cat/a3_arm_ws/scripts/melodion_pts2stream.py)、`PlaybackTrajectory.srv:1-9` |
| `_play_stream` **优先读 `<name>.yaml.baked.yaml`**，生成新 stream 后必须清掉，否则跑旧轨迹 | `arm_controller.py:4956-4960` |
| `random_pose_tour` 支持 `fixed_sequence`（命名点）/ `xyz`（base_link 绝对坐标）/ `orientation_mode`；**不支持角度连续摆动** | `RandomPoseTour.srv:1-23`、`arm_controller.py:5562-5656` |
| 角度巡航 F185 只存在于 CLI 脚本，直接发 twist，无服务 | `scripts/a3_test/f185_angular_cruise_demo.py:180-226` |
| `playback` 只认 `""/waypoint/stream/cp`，无 dance 槽位 | `arm_controller.py:6312-6314` |
| 没有"把 latest 复制到指定槽位"的服务/脚本 | 全仓无 `promote/archive/slot` |
| `melodion` 生成逻辑可安全搬进包（纯 Python + PyYAML，毫秒级） | `app_melodion/` 已 `exec_depend` 于编排层 |

## 三、最终按键方案

### 3.1 固定层（所有模式恒定）

| 按键 | 手势 | 动作 |
|---|---|---|
| Cross | 长按 1.0s | 硬急停（`power_sequence shutdown`） |
| L3 | 短按 | 上电 + 使能 |
| R3 | 短按 | 软失能（safe-park） |
| L1+R1 | 长按 2.0s | init 设零 |
| L1 + 摇杆 | 按住 | 末端平移 X/Y/Z |
| R1 + 摇杆 | 按住 | 末端旋转 X/Y/Z |
| R2 | 模拟量 | 夹爪力控 |
| **Share（按住）** | 修饰键 | 模式选择窗口，屏蔽摇杆/其它按键 |
| **Options** | 短按 | 退出当前模式 → NORMAL；NORMAL 下 = 停止运动 |

### 3.2 NORMAL（手动 / 通用回放）— 绿常亮

| 按键 | 手势 | 动作 |
|---|---|---|
| **D-pad ↑** | 点按 | goto `home` |
| **D-pad ↓** | 点按 | goto `idle` |
| **D-pad ←** | 点按 | goto `normal_slot_left`（**预留位**，改 named_poses 即换位置） |
| **D-pad →** | 点按 | goto `normal_slot_right`（预留位） |
| Triangle | 长按 1.5s | 回放连续轨迹 `latest` |
| Square | 长按 1.5s | 路点 PTP 回放 |
| Circle | 长按 1.5s | 路点 LIN 回放 |
| Triangle | 短按 1.5s | goto `ready`（预备位） |
| Square | 短按 1.5s | 保存当前位姿 `snap_*` |
| Circle | 短按 1.5s | **停止当前运动**（`/a3/arm/stop`） |
| L2 | 短按 | 保存当前位姿（保留旧习惯） |
| Options | 短按 | 停止当前运动 |

> 速度档保持固定（不再用 D-pad 调），`speed_normal: 0.35` 不动。

### 3.3 TEACH（连续示教）— 蓝：待命常亮 / 录制呼吸

| 按键 | 动作 |
|---|---|
| Triangle 短按 | toggle：未录制 → `start_teach`；录制中 → `stop_teach`（保存后留在 TEACH 待命） |
| L2 短按 | 保存当前点位 `snap_*` |
| Options 短按 | 若在录制先 `stop_teach`，再回 NORMAL |

### 3.4 WAYPOINT（路点示教）— 青：待命常亮 / 录制呼吸

| 按键 | 动作 |
|---|---|
| Triangle 短按 | 未录制 → `start_waypoint_teach`；录制中 → **打点** |
| L2 短按 | 录制中打点；未录制存 `snap_*` |
| Options 短按 | 若在录制先 `stop_teach`，再回 NORMAL |

### 3.5 TRACK（人脸跟踪 + 标定补全）— 紫呼吸

进入：`face_track enable(true)`（required，失败回退）；`suppress_common_axes: [left_x, left_y, right_x, right_y]`（禁用摇杆、**保留 R2 夹爪**）

| 按键 | 动作 |
|---|---|
| Triangle 短按 | toggle 跟踪（enable ⇄ disable） |
| **Circle 短按** | **记录目标**：`/a3/face_track/set_target {record_now: true}`（当前画面中心偏差 + 距离作为新基准） |
| **Circle 长按 1.5s** | `reload_jacobian`（雅可比热重载） |
| **Square 短按** | `status` 查询（日志 + 灯带反馈） |
| **Square 长按 1.5s** | **深度比例标定**：`/a3/face_track/calibrate_depth`（脸放 200 mm 处按一下） |
| **D-pad ↑ / ↓** | 跟踪距离 `z_ref_m` +0.05 / −0.05 |
| **D-pad ← / →** | 目标中心 `target_err_x` −0.02 / +0.02 |
| L2 短按 | 保存当前点位 |
| Options 短按 | disable 跟踪 → NORMAL |

> 标定补全说明（回答"要不要补全"）：现有只有雅可比重载，缺"目标框中心 / 跟踪距离 / 深度比例"三类。本次全部补上，见 4.3。

### 3.6 PIANO（弹琴）— 黄呼吸

进入：goto `melodion_white1`（optional）；`suppress_common_axes: [left_x, left_y, right_x, right_y]`

| 按键 | 动作 |
|---|---|
| D-pad ↑ / ↓ / ← / → | goto `melodion_black13 / black1 / white1 / white19` |
| Triangle 短按 1.5s | 播放 `stream latest` |
| **Triangle 长按 2.0s** | **生成 stream**：读 `waypoints/latest.yaml` → PTP+3 过点 → 写 `streams/latest.yaml` → 自动播放 |
| Circle 短按 | 回起始键 `melodion_white1` |
| Square 短按 | 到最右键 `melodion_white19` |
| L2 短按 | 保存当前对位点 |
| Options 短按 | 回 NORMAL |

### 3.7 DEMO（演示）— 白呼吸

进入：goto `ready`（optional）

| 按键 | 动作 |
|---|---|
| **Triangle 短按** | **home 六向平移巡游**：`random_pose_tour {fixed_sequence: [home, home_up, home_down, home_left, home_right, home_front, home_back]}` |
| **Circle 短按** | **角度巡游启停**：`/a3/demo/angular_cruise {action: toggle, axis: all}` |
| Square 短按 | 随机命名点巡游 `random_pose_tour` |
| **D-pad ↑ / ↓ / ← / →** | 单轴角度巡游 X / Y / Z / 综合（各按一次启动，再按停止） |
| L2 短按 | 保存当前点位 |
| Options 短按 | 停止巡游 → NORMAL |

### 3.8 DANCE（舞蹈，新增）— 品红呼吸

进入：goto `ready`（optional）

| 按键 | 动作 |
|---|---|
| **Triangle 短按** | 播放 `dance/dance1` |
| **Square 短按** | 播放 `dance/dance2` |
| Circle 短按 | 停止（`/a3/arm/stop`） |
| **L2 短按** | 把 `latest` 路点/连续轨迹**存为 dance1** |
| **L2 长按 1.5s** | 把 `latest` 存为 **dance2** |
| Options 短按 | 停止 → NORMAL |

> "预留 2 个按键的舞蹈" = Triangle/Square 对应 `dance1`/`dance2`；`dance3/4` 可在 DANCE 模式的 `dpad` 段扩展。

## 四、新增底层能力

### 4.1 `/a3/arm/stop`（Trigger）— 切模式先停运动

**文件**：[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) 注册在 `#L1579` 之后

行为（顺序固定）：
1. 若 `state in (TEACH, WAYPOINT_TEACH)` → 调 `_stop_teach_cb` 逻辑（保存）
2. 调 `/a3/motor/stop`（新增 `MotorStop` client，`a3_can_bridge.srv.MotorStop`）
3. `_stop_l7_sequence()`
4. 若 `control_mode == SERVO` → 复用 `_servo_stop_cli`
5. 若存在已保存的 MoveGroup / FJT goal_handle → `cancel_goal_async()`（**本次新增保存 handle**）；取消失败只 WARN
6. 清 FSM：`_traj_done_at = 0.0`、`_jogging = False`、`_publish_mode("IDLE")`、`_set_state(STATE_READY, "stop")`
7. 响应 `success=true, message="stopped from <old_state>"`

门禁：任何状态都放行（安全路径），`DISABLED/FAULT` 下只做幂等返回。

### 4.2 模式切换流程（`ps4_mapper`）

`_switch_mode(target)` 改为**两阶段异步**，避免阻塞 50 Hz 定时器：

1. `target == self._mode` → 幂等返回
2. `self._pending_mode = target`；`self._pending_since = now`；调 `/a3/arm/stop`
3. 每 tick 在 `_on_timer` 顶部检查：
   - stop 响应成功 → `_commit_mode(target)`；`_pending_mode = None`
   - stop 服务不可用（未起编排层）→ 直接 `_commit_mode(target)`
   - 超时 1.0 s → WARN `stop timeout`，仍执行 `_commit_mode(target)`
   - pending 期间每 tick 重置 `_mode_guard_until = now + 0.1`（持续屏蔽按键/摇杆）
4. `_commit_mode(target)`：
   - 执行旧模式 `exit` hooks
   - 设新 `self._mode`、重置 `ButtonEdgeTracker` / `_dpad_active` / `_select_edges`
   - 执行新模式 `enter` hooks；`required: true` 失败 → 回滚 `normal` + `mode_rejected` 事件
   - `_mode_guard_until = now + 0.3`（切换后防抖）
   - 发布 `/a3/teleop/mode`；发 `mode_changed:<target>:<rumble_index>`

### 4.3 TRACK 标定补全

#### 4.3.1 新增 `a3_msgs/srv/SetFaceTarget.srv`

```
bool   record_now      # true=用当前中位帧记录；false=用下面的值精确设置
float32 target_err_x   # 归一化 [-1,1]，控制器从 image_error 中减去
float32 target_err_y
float32 z_ref_m        # <=0 表示跟随标定距离 Z0
bool   persist         # 写 ~/.a3/camera/face_target.yaml
---
bool success
string message
float32 target_err_x
float32 target_err_y
float32 z_ref_m
```

控制器改动（`face_track_controller.py`）：
- 新增参数 `target_err_x=0.0`、`target_err_y=0.0`、`calibration_file=~/.a3/camera/face_target.yaml`
- `_on_params` **显式加分支**（否则 `ros2 param set` 会"成功但无效"——现有回调恒返回 successful）
- 控制律里 `ex -= target_err_x; ey -= target_err_y`（在 I/D 整形之前）
- `record_now` 取 `_hist` 最近 `median_n` 帧的 `image_error` / `xyz_mm.z` 中位数
- 记录/设置后清 `_i_term/_de_lp/_prog_*`（复用 `_reset_err_state`），并补发 0.2 s 零速

#### 4.3.2 `/a3/face_track/status`（Trigger）
响应 `message` 返回：`enabled=%d err=(%.3f,%.3f) z=%.3fm z_ref=%.3f target=(%.3f,%.3f) jac=%d joint_scale=%.2f`

#### 4.3.3 `/a3/face_track/calibrate_depth`（Trigger）
加在 `face_detect_node`（现无服务）；参数 `calib_known_distance_mm=200.0`：
- 取最近一帧 `xyz_mm.z` 中位数 → `depth_units_mm *= known / z_meas` → 原地生效 + 写 `~/.a3/camera/face_calib.yaml`
- 前置：`detected && valid`，否则 `success=false`

#### 4.3.4 D-pad 调参
`actions.py` 新增 `face_target_step(dx, dz)`：读本地缓存的目标值（进入 TRACK 时通过 `status` 拉一次），增量后调 `set_target`。

### 4.4 `/a3/arm/generate_stream`（GenerateStream.srv）

文件：`a3_msgs/srv/GenerateStream.srv`

```
string source            # 空 = <traj_dir>/waypoints/latest.yaml
string out_name          # 默认 "latest"
string point_names       # 默认 "white1,white19,black13,black1"
string mode              # 默认 "ptp_blend"
float64 speed            # 0 = 用 0.2
float64 press_depth_mm   # 0 = 读 playing_geometry（内部仍按 6mm 下限）
float64 lift_src         # 40
float64 lift_apex        # 80
float64 lift_dst         # 40
float64 press_sleep      # 0.2
bool    exact            # false
bool    hold_orientation # true
bool    auto_play        # true
---
bool success
string message
string out_path
int32  segments
```

实现：
1. 把 `scripts/melodion_pts2stream.py` 的纯逻辑迁到 `src/app_melodion/app_melodion/pts2stream.py`（`load_yaml/_up_poses/_down_pose/_at/_transition_vias/_segment/build_stream/header_lines`），原脚本退化为薄 CLI（参数与默认值保持不变）
2. `arm_controller.py` 注册服务，回调门禁 `state == READY`（TEACH/WAYPOINT 拒绝）
3. 校验 `kind == waypoint`、`len(waypoints) >= 2`
4. 输出 `<traj_dir>/streams/<out_name>.yaml`，**临时文件 + `os.replace` 原子写**
5. **删除 `<out_path>.baked.yaml`**（否则 `_play_stream` 读旧缓存）
6. `auto_play` 用 one-shot timer 在回调返回后再调 playback（避免同回调组重入）

### 4.5 角度巡游节点 `/a3/demo/angular_cruise`

新文件：`src/a3_bringup/a3_bringup/angular_cruise_node.py`（由 `a3_bringup.launch.py` 常驻拉起）

`a3_msgs/srv/AngularCruise.srv`：

```
uint8 action        # 0=stop 1=start 2=toggle
string axis         # "x" | "y" | "z" | "all"
float64 amp_deg     # 0 = 用节点默认（x 80/80、y 25/45、z 45/45）
float64 peak_dps    # 0 = 用节点默认 30
uint32 cycles       # 0 = 1 次
---
bool success
string message
```

实现：把 F185 的正弦包络（`v(t)=v_peak·sin(πt/T)`、`0→+A→−B→0`、三轴接力）搬进节点，直接发 `TwistStamped` 到 `/servo_node/delta_twist_cmds`。门禁：`arm_status == READY`、`face_track` 未 enable、无其它 twist 发布者；`stop` 立即发零速并停发。

### 4.6 舞蹈槽位（`playback type=dance` + `/a3/arm/save_slot`）

1. `playback` 新增 `type: dance` → 路径 `<traj_dir>/dance/<name>.yaml`
2. kind 门禁：`dance` 接受 `continuous` 或 `waypoint`，读文件头后分派到 `_play_continuous` / `_play_waypoints`
3. 新文件 `a3_msgs/srv/SaveSlot.srv`：

```
string slot          # "dance1" | "dance2" | ...
string source_type   # "" = continuous latest | "waypoint" = waypoints/latest
---
bool success
string message
string out_path
string kind
```

4. 回调：校验源文件存在 → 复制到 `<traj_dir>/dance/<slot>.yaml`（保留原 `kind`，纯复制，不做结构转换）→ 原子写
5. 新增点位数：`normal_slot_left` / `normal_slot_right` / `home_left` / `home_right`
   - 写 `a3_description/config/named_poses.yaml` **和** `a3_description_6j/config/named_poses.yaml`
   - `normal_slot_left/right` 默认复制 `home_back/home_front` 的 positions（用户自行改）
   - `home_left/home_right`：以 home 末端 `(-0.177, 0, 0.326)` 为基准，±Y 15 cm，严格保持 home 姿态，用 Pinocchio 6D IK 生成并校验可达（沿用 F115 的方法）

## 五、Proposed Changes（逐文件）

### 5.1 配置与映射

| 文件 | 改动 |
|---|---|
| `src/a3_teleop_ps4/config/mappings/default.yaml` | 重写为 `common` + `modes`（select/exit + 7 模式），结构见下 |
| `src/a3_teleop_ps4/config/action_registry.yaml` | 新增 `teach_toggle`、`waypoint_capture_toggle`、`save_named_pose_or_capture`、`teach_stop_if_active`、`face_track_enable/disable/toggle`、`face_track_record_target`、`face_track_reload_jacobian`、`face_track_status`、`face_track_calibrate_depth`、`face_target_step`、`generate_stream`、`save_dance_slot`、`angular_cruise` |
| `src/a3_description/config/named_poses.yaml` | 新增 `normal_slot_left/right`、`home_left/right` |
| `src/a3_description_6j/config/named_poses.yaml` | 同上（同步） |
| `src/a3_teleop_ps4/config/ds4_linux.yaml` | **不改**（`dpad_*` 虚拟键已够用） |

`default.yaml` 骨架：

```yaml
speed_normal: 0.35
joy_alive_ticks: 25

common:
  axes:   # R2 + L1/R1 伺服，与现状一致
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
  select:
    modifier: share
    bindings:
      dpad_up: teach
      dpad_down: waypoint
      dpad_left: track
      dpad_right: piano
      triangle: demo
      circle: dance
      square: normal
  exit: {button: options, edge: shortpress, hold_s: 3.0}

  normal:
    label: NORMAL
    color: green
    pattern: solid
    rumble_index: 1
    suppress_common_axes: []
    enter: []
    exit: []
    axes: {}
    dpad:
      - axis: dpad_y
        neg: {fn: goto_named_pose, kwargs: {name: home}}
        pos: {fn: goto_named_pose, kwargs: {name: idle}}
      - axis: dpad_x
        neg: {fn: goto_named_pose, kwargs: {name: normal_slot_left}}
        pos: {fn: goto_named_pose, kwargs: {name: normal_slot_right}}
    buttons:
      triangle:
        - {fn: goto_named_pose, kwargs: {name: ready}, edge: shortpress, hold_s: 1.5}
        - {fn: playback_latest, edge: longpress, hold_s: 1.5}
      square:
        - {fn: save_named_pose, edge: shortpress, hold_s: 1.5}
        - {fn: playback_waypoints_ptp, edge: longpress, hold_s: 1.5}
      circle:
        - {fn: arm_stop, edge: shortpress, hold_s: 1.5}
        - {fn: playback_waypoints_lin, edge: longpress, hold_s: 1.5}
      l2: {fn: save_named_pose, edge: shortpress, hold_s: 3.0}

  teach:
    label: TEACH
    color: blue
    pattern: breathe
    rumble_index: 2
    suppress_common_axes: []
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
    suppress_common_axes: []
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
    suppress_common_axes: [left_x, left_y, right_x, right_y]
    enter: [{fn: face_track_enable, required: true}]
    exit:  [{fn: face_track_disable}]
    axes: {}
    dpad:
      - axis: dpad_y
        neg: {fn: face_target_step, kwargs: {dz: 0.05}}
        pos: {fn: face_target_step, kwargs: {dz: -0.05}}
      - axis: dpad_x
        neg: {fn: face_target_step, kwargs: {dx: -0.02}}
        pos: {fn: face_target_step, kwargs: {dx: 0.02}}
    buttons:
      triangle:
        - {fn: face_track_toggle, edge: shortpress, hold_s: 3.0}
      circle:
        - {fn: face_track_record_target, edge: shortpress, hold_s: 1.5}
        - {fn: face_track_reload_jacobian, edge: longpress, hold_s: 1.5}
      square:
        - {fn: face_track_status, edge: shortpress, hold_s: 1.5}
        - {fn: face_track_calibrate_depth, edge: longpress, hold_s: 1.5}
      l2: {fn: save_named_pose, edge: shortpress, hold_s: 3.0}

  piano:
    label: PIANO
    color: yellow
    pattern: breathe
    rumble_index: 5
    suppress_common_axes: [left_x, left_y, right_x, right_y]
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
      triangle:
        - {fn: playback_stream, kwargs: {name: latest}, edge: shortpress, hold_s: 1.5}
        - {fn: generate_stream, kwargs: {out_name: latest, auto_play: true}, edge: longpress, hold_s: 2.0}
      circle: {fn: goto_named_pose, kwargs: {name: melodion_white1},  edge: shortpress, hold_s: 3.0}
      square: {fn: goto_named_pose, kwargs: {name: melodion_white19}, edge: shortpress, hold_s: 3.0}
      l2:     {fn: save_named_pose, edge: shortpress, hold_s: 3.0}

  demo:
    label: DEMO
    color: white
    pattern: breathe
    rumble_index: 6
    suppress_common_axes: []
    enter: [{fn: goto_named_pose, kwargs: {name: ready}, required: false}]
    exit: [{fn: angular_cruise_stop}, {fn: arm_stop}]
    axes: {}
    dpad:
      - axis: dpad_y
        neg: {fn: angular_cruise, kwargs: {axis: x}}
        pos: {fn: angular_cruise, kwargs: {axis: y}}
      - axis: dpad_x
        neg: {fn: angular_cruise, kwargs: {axis: z}}
        pos: {fn: angular_cruise, kwargs: {axis: all}}
    buttons:
      triangle: {fn: demo_tour_home, edge: shortpress, hold_s: 3.0}
      circle:   {fn: angular_cruise, kwargs: {axis: all}, edge: shortpress, hold_s: 3.0}
      square:   {fn: random_pose_tour, edge: shortpress, hold_s: 3.0}
      l2:       {fn: save_named_pose, edge: shortpress, hold_s: 3.0}

  dance:
    label: DANCE
    color: magenta
    pattern: breathe
    rumble_index: 7
    suppress_common_axes: []
    enter: [{fn: goto_named_pose, kwargs: {name: ready}, required: false}]
    exit: [{fn: arm_stop}]
    axes: {}
    dpad: []
    buttons:
      triangle: {fn: playback_dance, kwargs: {name: dance1}, edge: shortpress, hold_s: 3.0}
      square:   {fn: playback_dance, kwargs: {name: dance2}, edge: shortpress, hold_s: 3.0}
      circle:   {fn: arm_stop, edge: shortpress, hold_s: 3.0}
      l2:
        - {fn: save_dance_slot, kwargs: {slot: dance1, source_type: auto}, edge: shortpress, hold_s: 1.5}
        - {fn: save_dance_slot, kwargs: {slot: dance2, source_type: auto}, edge: longpress,  hold_s: 1.5}
```

### 5.2 `a3_teleop_ps4` 代码

| 文件 | 改动 |
|---|---|
| `mapping.py` | 新增 `has_modes` / `load_modes` / `resolve_mode`；`resolve_mode` 以 `common` 为底、模式同名键覆盖、按 `suppress_common_axes` 剔除轴（`r2` 始终保留），无 `modes` 段时退回旧三层；`validate_mapping` 扩展到 `common` + 7 模式 + `select.bindings` 目标 + `enter/exit` 的 fn |
| `ps4_mapper.py` | 模式选择窗口（Share 按住 + rising）、Options 退出、`_switch_mode` 两阶段（先 `/a3/arm/stop`）、`_commit_mode`、`/a3/teleop/mode` 发布、`/a3/arm_status` 订阅 |
| `actions.py` | 新增 `arm_stop`（Trigger `/a3/arm/stop`）、`teach_toggle`、`waypoint_capture_toggle`、`save_named_pose_or_capture`、`teach_stop_if_active`、`playback_dance`、`save_dance_slot`、`generate_stream`、`face_track_*`（enable/disable/toggle/record_target/reload/status/calibrate_depth）、`face_target_step`、`angular_cruise`、`angular_cruise_stop`、`demo_tour_home`（`random_pose_tour` + fixed_sequence）；订阅 `/a3/arm_status` 保存 `arm_state` |
| `ds4_feedback_node.py` | 订阅 `/a3/teleop/mode`；新增颜色 `yellow(255,200,0)`、`magenta(255,0,160)`；`_derive()` 在安全色之后插入模式色；JSON 增 `mode`；模式切换震 `rumble_index` 次 |

### 5.3 `a3_arm_controller`

| 改动 | 位置 |
|---|---|
| 注册 `/a3/arm/stop` | `arm_controller.py` 服务表 `#L1579` 后 |
| 保存 MoveGroup / FJT / Sequence goal_handle 成员，供 stop 取消 | `#L3956`、`#L2553`、`#L3831` |
| 注册 `/a3/arm/generate_stream` | 服务表后 |
| `playback` 支持 `type=dance`（路径 + kind 分派） | `#L6303-L6409`、新增 `_dance_path` |
| 注册 `/a3/arm/save_slot` | 服务表后 |
| 新增参数：`melodion_*` 组、`dance_dir`、`stop_timeout_s` | `config/arm_controller.yaml` |

### 5.4 `a3_perception`

| 改动 | 文件 |
|---|---|
| 新增 `SetFaceTarget` 服务、`status`、`target_err_x/y` 参数、控制律减偏移 | `face_track_controller.py` |
| 新增 `calibrate_depth` 服务（改 `depth_units_mm` 并持久化） | `face_detect_node.py` |
| 参数文档更新 | `config/face_track.yaml`、`README.md` |

### 5.5 `a3_bringup` / `a3_msgs` / `app_melodion`

| 改动 | 文件 |
|---|---|
| 新节点 `angular_cruise_node` + launch 常驻 | `a3_bringup/a3_bringup/angular_cruise_node.py`、`a3_bringup.launch.py` |
| 新 srv：`SetFaceTarget`、`GenerateStream`、`AngularCruise`、`SaveSlot` | `a3_msgs/srv/*.srv` + `CMakeLists.txt` |
| 生成逻辑迁入包 | `app_melodion/app_melodion/pts2stream.py`（新），`scripts/melodion_pts2stream.py` 改薄 CLI |

### 5.6 文档（仓库规则要求）

| 文件 | 改动 |
|---|---|
| `docs/edge/requirements/F189-ps4-mode-based-mapping.md` | 新建：7 模式、Share 组合键、Options 退出、切模式先停、验收 |
| `docs/edge/requirements/F190-arm-stop-service.md` | 新建：`/a3/arm/stop` 语义与门禁 |
| `docs/edge/requirements/F191-face-track-calibration.md` | 新建：`set_target/status/calibrate_depth` 三类标定 |
| `docs/edge/requirements/F192-melodion-generate-stream-service.md` | 新建：生成服务与 baked 缓存清理 |
| `docs/edge/requirements/F193-demo-dance-modes.md` | 新建：角度巡游节点、舞蹈槽位、`playback type=dance` |
| `docs/edge/REQUIREMENTS.md` | 索引加 F189–F193 |
| `src/a3_teleop_ps4/README.md` | 改为「固定层 + 模式选择 + 7 模式」五张表 |
| `docs/edge/PS4_OPERATOR_GUIDE.md` | 操作流程加"选模式"；灯色表补 TRACK/PIANO/DEMO/DANCE |
| `docs/shared/TOPIC_CONTRACT.md` | 新增 `/a3/teleop/mode`、`/a3/arm/stop`、`generate_stream`、`save_slot`、`/a3/demo/angular_cruise`、`set_target`、`status`、`calibrate_depth` |
| `docs/shared/TRAJECTORY_KINDS.md` | 新增 `dance` 槽位与 kind 分派规则 |
| `docs/edge/STATE_MACHINE.md` | 新增"HMI 模式 vs 编排状态"、"stop 的语义"（含 stop 后回 READY） |
| `docs/edge/requirements/F131-waypoint-teach.md` | 标注 Triangle 打点新增、L2 打点保留 |
| `src/a3_perception/README.md` | 标定三件套 + 按键流程 |
| `src/app_melodion/README.md` | 新增生成服务说明 |

### 5.7 测试

`scripts/a3_test/ps4_sim_test.py` 新增：

- **S11 直接选模式**：`share+dpad_up` → `teach`；`share+dpad_left` → `track`（不经过中间模式）
- **S12 Share 屏蔽**：按住 share 按 triangle 不得触发 NORMAL 的 goto
- **S13 切模式先停**：`piano` 播放中按 `share+square` 切 NORMAL → 先出现 `/a3/arm/stop` 调用，模式才变更
- **S14 示教分级**：进 `teach` 后 arm_status 非 TEACH；按 triangle 才 `start_teach`
- **S15 路点打点**：进 `waypoint` → triangle 开始 → triangle 打点
- **S16 Options 退出**：`teach` 录制中 options → `stop_teach` + 模式回 normal
- **S17 TRACK 轴封锁**：`track` 下 L1+left_x 不产生 twist
- **S18 进入失败回退**：DISABLED 下选 track 且服务不可用 → 回 normal + `mode_rejected`
- **S19 PIANO 生成**：triangle 长按 → `/a3/arm/generate_stream` 调用，且 `<out>.baked.yaml` 被删除
- **S20 DANCE 槽位**：L2 短 → `save_slot dance1`；triangle 短 → `playback {type: dance, name: dance1}`

## 六、实现顺序（里程碑，可独立验证）

| 里程碑 | 内容 | 验证 |
|---|---|---|
| **M1** | 模式框架（mapping/ps4_mapper）+ `/a3/arm/stop` + 固定层回归 | 能选 7 模式、切模式先停、固定层不变 |
| **M2** | NORMAL / TEACH / WAYPOINT 重排 + 新命名点 | D-pad 四点、三键长按回放、示教两级 |
| **M3** | TRACK + 标定补全（4.3） | `set_target` 记录、D-pad 微调、深度标定 |
| **M4** | PIANO + `generate_stream`（4.4） | 录路点 → 按键生成 → 自动播放 |
| **M5** | DEMO + 角度巡游节点（4.5） | home 六向巡游、四轴角度巡航 |
| **M6** | DANCE + `playback type=dance` + `save_slot`（4.6） | latest 存槽、两段舞蹈播放 |
| **M7** | 文档 + 仿真测试 + 真机验收清单 | S11–S20 全过 |

## 七、Assumptions & Decisions

| 编号 | 决策 | 理由 |
|---|---|---|
| D1 | 模式在 HMI 层实现，不改编排层 12 态 | 模式是按键复用；新增 PIANO/TRACK/DEMO/DANCE 编排态会污染安全状态机 |
| D2 | Share 作纯修饰键，直接点名模式；Options 统一退出 | 用户要求；避免循环"路过示教" |
| D3 | 切模式先调 `/a3/arm/stop` 并等待（超时 1 s 兜底） | 用户明确要求"不要形成模式之间串扰" |
| D4 | `/a3/arm/stop` 走"执行层 stop + 编排层状态回收"，不改 `_can_move` | 改动最小且能让新命令立即可用 |
| D5 | 示教/路点进入只切语义；TRACK 进入即 enable；PIANO 进入即定位 | 用户选定"按模式分级" |
| D6 | NORMAL 的 D-pad 四向 = home/idle/两个预留位；速度档固定 | 用户要求；预留位改名即可换点 |
| D7 | NORMAL 三键短按 = ready / 存快照 / 停止 | 用户让"帮考虑场景"；分别覆盖准备、记录、急停 |
| D8 | TRACK 标定补 3 类（目标中心/距离、深度比例、雅可比重载）+ 状态查询 | 回答用户"要不要补全"：现有只有雅可比重载，确实不完整 |
| D9 | PIANO 生成走 `/a3/arm/generate_stream`，逻辑迁入 `app_melodion` 模块 | 不在实时节点里起 subprocess，路径可靠 |
| D10 | DEMO 的 home 平移巡游复用 `random_pose_tour.fixed_sequence`；角度巡游单独节点 | srv 已支持固定序列；角度往复无法用点位表达 |
| D11 | 新增 `home_left/right`、`normal_slot_left/right` 四个命名点（7J/6J 同步） | home 六向巡游与 NORMAL 预留位需要 |
| D12 | DANCE 槽位用 `playback type=dance` + `<traj_dir>/dance/*.yaml`，kind 由文件头决定 | 用户要求"单独舞蹈配置文件"；纯复制 latest，零结构转换 |
| D13 | PIANO/TRACK 用 `suppress_common_axes` 只禁用 4 根摇杆轴，**保留 R2 夹爪** | 弹琴/跟踪需要独占 twist，但夹爪在弹琴时应可用 |
| D14 | 保持 PS 解绑 | PS 是开机/唤醒键，有断连风险 |
| D15 | TRACK 的雅可比探针标定（主动运动）**不在本次**，仅保留 `reload_jacobian` | 探针需主动小幅度运动，风险与工期都高；先用离线脚本 `f183_calib_jacobian.py` |

## 八、Verification

1. **构建**：`colcon build --packages-select a3_msgs a3_teleop_ps4 a3_arm_controller a3_perception a3_bringup app_melodion --symlink-install`
2. **YAML 自检**：`ps4_mapper` 启动零 ERROR（7 模式全部 fn + select 目标校验）
3. **模式切换**：合成 `/joy` 依次 `share+↑/↓/←/→/triangle/circle/square`，`ros2 topic echo /a3/teleop/mode` 应为 `teach/waypoint/track/piano/demo/dance/normal`，且每次先出现 `/a3/arm/stop` 调用
4. **停止服务**：`ros2 service call /a3/arm/stop std_srvs/srv/Trigger` → 回 READY，`TRAJ` 立即结束（`ros2 topic echo /a3/arm_status`）
5. **TRACK 标定**：`set_target record_now=true` → 响应回读 `target_err/z_ref`；`calibrate_depth` 后 `depth_units_mm` 变化并写盘
6. **PIANO 生成**：录 4 点路点 → Triangle 长按 → `streams/latest.yaml` 更新、`.baked.yaml` 被删除、自动播放
7. **DEMO**：`fixed_sequence` 巡游 7 点；`angular_cruise axis=all` 三轴接力，`stop` 立即归零
8. **DANCE**：L2 短 → `dance/dance1.yaml` 生成；Triangle 短 → 播放
9. **固定层回归**：Cross 断电、L3 上电使能、R3 失能、L1+R1 init、R2 夹爪、L1/R1 摇杆
10. **真机验收**（待硬件）：按 `PS4_OPERATOR_GUIDE.md` 走完 7 个模式
