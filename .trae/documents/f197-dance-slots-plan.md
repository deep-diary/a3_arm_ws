# F197 舞蹈双槽位（DANCE 模式录制 + 单次/循环播放）实施计划

## 一、Summary

在**不改变 7 模式结构**（`normal/teach/waypoint/track/piano/demo/dance`）的前提下，把 DANCE 模式从
"回放 latest / 路点 PTP" 升级为**双舞蹈槽位**：

- D-pad 四向：`↓` 开始录连续舞1、`↑` 结束保存舞1；`←` 开始录连续舞2、`→` 结束保存舞2。
- `L2` 短/长：把 `latest`（连续或路点，取较新者）**整文件提升**为舞1/舞2（第二录制链路）。
- `△`/`○` 短/长：播放舞1/舞2，**短按单次、长按循环直到 `□`**。
- `□` 短按：停止；`Options`：停止并退出（沿用现有 `arm_stop` hook）。

产物集中到 `trajectories/dance/{dance1,dance2}.yaml`，覆盖前自动备份到 `trajectories/dance/backup/`。

本次**不做**：模式合并（保留 Share+↑ TEACH、Share+↓ WAYPOINT、Share+○ DANCE）、D-pad 长按机制、
绕过占空比/温度门禁。

## 二、Current State Analysis（基于现有代码）

| 结论 | 证据 |
|---|---|
| DANCE 模式目前只有 4 个面键，D-pad 未绑定 | [default.yaml#L197-L208](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml#L197-L208) |
| D-pad 仅支持 `0→±1` 边沿、回中重武装，无短/长仲裁 | [ps4_mapper.py#L435-L443](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py#L435-L443)；按钮短/长在 [ButtonEdgeTracker](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/mapping.py#L123-L184) |
| 连续示教入口只录通用 `latest.yaml`，无槽位概念 | [`_start_teach_cb`](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6099-L6123)、[`_stop_teach_cb` 保存分支](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6194-L6213) |
| 路点示教已有槽位机制（`_waypoint_slot`），但只服务 melodion | [初始化 L1479-L1480](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L1473-L1483)、[`_stop_waypoint_teach`](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6246-L6292) |
| `playback` 目前只认 `""/waypoint/stream/cp`，无 dance 槽位 | [`_prepare_playback`](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6580-L6686) |
| `PlaybackTrajectory.srv` 只有 `name/type/strategy`，无循环字段 | [PlaybackTrajectory.srv](file:///home/cat/a3_arm_ws/src/a3_msgs/srv/PlaybackTrajectory.srv) |
| 连续回放已保录制时长（TOTG 目标时长），节奏可复现 | [arm_controller.yaml#L99-L103](file:///home/cat/a3_arm_ws/src/a3_arm_controller/config/arm_controller.yaml#L99-L103)、[`_play_continuous`](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6702-L6922) |
| TRAJ 结束回 READY 由 `_publish_status` 的时间戳判定，可在此插入下一轮 | [arm_controller.py#L3206-L3213](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L3206-L3213) |
| 下发前有静态力矩门禁 + 占空比门禁（600 s 窗口 × 0.8） | [`_dispatch_trajectory`](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L2466-L2510)、[duty 参数](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L1357-L1359) |
| teleop 已有"未 READY 自动上电+补发"挂起机制，可复用 | [`_start_melodion_teach`](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py#L548-L577)、[`_advance_pending_teach`](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py#L648-L669) |
| 动作分发按 `kwargs` 直接调用同名方法 | [`apply_discrete`](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py#L307-L313) |

## 三、决策记录（Assumptions & Decisions）

| 编号 | 决策 | 理由 |
|---|---|---|
| D1 | 保留 7 模式，不合并 TEACH/WAYPOINT/DANCE | 用户确认；避免 D-pad 长按仲裁、通用 `latest` 链路丢失、手势密度翻倍 |
| D2 | 录制分两条链路：D-pad 直接连续录 + L2 提升 latest | 用户要求"两种都保留"；路点复用 WAYPOINT 模式，零新规划逻辑 |
| D3 | 提升源按 mtime 取较新者（`trajectories/latest.yaml` vs `waypoints/latest.yaml`） | 不需要额外按键；日志打印实际来源，可预测 |
| D4 | 播放 `type=dance`，按文件头 `kind` 分派 `_play_continuous` / `_play_waypoints` | 与现有 kind 双向门禁一致；路点槽零转换 |
| D5 | 循环复用内存中已 retime 的轨迹重新下发，不重读盘 | 接缝停顿最小；避免每轮 1–3 s 的加载/重定时 |
| D6 | 循环首轮若需从当前位姿回首点，必须走 MoveIt；失败则降级单次 | 几何 ramp 会污染循环体，导致每轮跳回旧起点 |
| D7 | 首末关节差 ≤ `dance_loop_closed_tol_rad`(0.05) 视为闭环无缝；否则追加 `dance_loop_seam_s`(0.8 s) 线性回归段 | 允许非闭环舞蹈循环，避免位置跳变 |
| D8 | 路点槽循环**不支持**，长按只播一次 + 日志说明 | MoveIt 重规划无法做无缝循环 |
| D9 | 不绕过静态力矩/占空比/温度门禁 | 安全契约；循环累计到上限自动停属预期 |
| D10 | 槽位录制**不覆盖**通用 `latest.yaml` | 避免录舞把日常回放素材冲掉 |
| D11 | DANCE 模式的 `L2` 从"存命名点"改为"提升 latest 到槽位" | 槽位操作优先；存点在其他 6 个模式仍可用 |

## 四、Proposed Changes

### 4.1 `src/a3_msgs/srv/PlaybackTrajectory.srv`

在 `strategy` 之后、`---` 之前追加：

```
# F197: 循环播放（仅 type=dance 的连续槽生效；waypoint 槽忽略并单次播放）
bool loop
```

> 追加字段对现有调用方向后兼容（默认 false），但消息哈希变化，必须重编 `a3_msgs` 及其依赖。

### 4.2 `src/a3_arm_controller/a3_arm_controller/arm_controller.py`

#### (a) 目录与状态（初始化段 L1473-L1481 之后）

```python
self._dance_dir = os.path.join(self._traj_dir, "dance")
self._dance_bak_dir = os.path.join(self._dance_dir, "backup")
for _d in (self._dance_dir, self._dance_bak_dir):
    os.makedirs(_d, exist_ok=True)
self._teach_slot: Optional[str] = None   # None=通用 latest；"dance1"/"dance2"
```

循环上下文（与 `_l7_seq` 同区声明）：

```python
self._dance_loop: Optional[dict] = None  # {traj, l7_traj, label, gap_s, cycles}
```

#### (b) 录制槽位

1. 把 `_start_teach_cb` 主体抽成 `_begin_teach(slot: Optional[str], resp)`：
   - `_teach_guard()` 不变（要求 READY、非 BLOCKED_MODES）。
   - 若 `self._state == STATE_TEACH`：
     - `slot == self._teach_slot` → `success=True, message="already recording <slot>"`（**幂等**，防误触重置）；
     - 否则 → `success=False, message="正在录制 <当前槽>，先按对应方向结束"`。
   - `_enter_freedrive()` → 清 `_record` / `_record_start` / `_recording=True` / `_teach_slot=slot`。
   - `_publish_mode("ZERO_TORQUE")`（controller_switch 时）→ `_set_state(STATE_TEACH, ...)`。
   - 响应里写明目标路径：`dance/dance1.yaml` 或 `latest.yaml`。
2. `_start_teach_cb` 改为 `return self._begin_teach(None, resp)`。
3. 新增服务（注册在 melodion 服务之后，L1611 附近）：

```python
self.create_service(Trigger, "/a3/arm/dance/record1", self._dance_record1_cb, ...)
self.create_service(Trigger, "/a3/arm/dance/record2", self._dance_record2_cb, ...)
```

回调分别转 `_begin_teach("dance1"/"dance2", resp)`。

4. `_stop_teach_cb` 连续分支（L6179-L6213）改为槽位感知：

```python
slot, self._teach_slot = self._teach_slot, None
...
if n >= min_samples:
    if slot:
        main = os.path.join(self._dance_dir, f"{slot}.yaml")
        bak = os.path.join(self._dance_bak_dir,
                           f"{slot}_{time.strftime('%Y%m%d_%H%M%S')}.yaml")
        label = f"dance/{slot}.yaml"
    else:
        main = _traj_path_for(self._traj_dir, "")   # latest.yaml
        bak = os.path.join(self._traj_dir, f"teach_{ts}.yaml")
        label = "latest.yaml"
    if self._save_recording_to(main, meta=meta):
        ...
```

- `meta` 仅槽位录制注入：`{"kind": "continuous", "name": slot, "recorded_at": "YYYY-MM-DD HH:MM:SS"}`。
- `_save_recording_to(path, meta: Optional[dict] = None)`：先 `data = self._dump_recording()`，再 `data.update(meta or {})`，其余不变。
- 槽位分支**不写** `latest.yaml`。

#### (c) 提升（promote）服务

注册：

```python
self.create_service(Trigger, "/a3/arm/dance/promote1", self._dance_promote1_cb, ...)
self.create_service(Trigger, "/a3/arm/dance/promote2", self._dance_promote2_cb, ...)
```

`_promote_dance(slot)` 逻辑：

1. `state` 必须是 `READY`，否则拒绝（`f"先停止当前运动/示教（state={state}）"`）。
2. 源候选：`[traj_dir/latest.yaml, traj_dir/waypoints/latest.yaml]`，取存在者中 `os.path.getmtime` 最大；
   都不存在 → 失败，消息提示"先用 TEACH 连续录 / WAYPOINT 路点录"。
3. 用 `_YamlSafeLoader` 读源，校验：
   - `kind` ∈ {`""`, `continuous`} → 必须有非空 `points`；
   - `kind == "waypoint"` → 必须有非空 `waypoints`；
   - 其他（`stream`/`cp`）→ 拒绝。
4. 目标存在则先复制到 `dance/backup/{slot}_{ts}.yaml`。
5. 读源字节 → 写 `dance/{slot}.yaml.tmp` → `os.replace`（原子，零结构转换）。
6. 返回 `message`：`f"promoted {src} ({kind}) -> dance/{slot}.yaml ({n} points)"`。

#### (d) 回放：`type=dance`

`_prepare_playback` 增加：

```python
is_dance = (req.type or "").strip().lower() == "dance"
```

- `is_latest` 判定对 dance 无效：dance 必须显式 `name`，缺失 → `"dance requires explicit name"`。
- 路径：`path = os.path.join(self._dance_dir, _sanitize_name(req.name) + ".yaml")`。
- `label` 前缀 `dn_`。
- 不存在 → `f"dance 槽未录制：{path}（DANCE 模式 ↓/← 录制，或 L2 把 latest 提升到槽位）"`。
- kind 门禁（在现有门禁后追加）：
  - dance 允许 `continuous`（默认）与 `waypoint`；
  - `stream`/`cp` → 拒绝并提示用对应 `type` 回放；
  - 非 dance 的通用连续回放遇到 dance 目录文件不受影响（路径不同）。

`_playback_cb`：

```python
is_dance = (req.type or "").strip().lower() == "dance"
if kind == "waypoint":
    resp = self._play_waypoints(data, label, strategy, resp)
    if is_dance and req.loop:
        resp.message += "；路点槽不支持循环，已单次播放"
    return resp
if kind == "stream":
    return self._play_stream(data, label, path, resp)
if kind == "cp":
    return self._play_cp(data, label, resp)
return self._play_continuous(data, label, path, resp, loop=bool(req.loop and is_dance))
```

#### (e) 循环实现

1. `_play_continuous(self, data, label, path, resp, loop: bool = False)`：
   - 计算 `use_ramp` 后，若 `loop=True`：
     - 强制 `use_moveit = True`（参数 `playback_return_use_moveit` 为 false 时也临时按 true 处理）；
     - `_moveit_move` 成功 → `moved=True`，`ramp_in_geo=False`，循环体 = `recorded_std`；
     - 失败 → `loop=False`，在 `resp.message` 追加 `"；循环降级为单次（回首点规划失败）"`。
   - 末尾（L6912-L6915）：
     ```python
     if loop:
         self._dance_loop = self._make_dance_loop(traj, l7_traj, label)
         self._traj_done_at = time.monotonic() + duration + self._dance_loop["gap_s"]
     else:
         self._schedule_back_to_ready(duration + 0.3)
     ```
     注意：`_dispatch_trajectory` 已记录一次 duty（L2509），保持现状不改。

2. `_make_dance_loop(traj, l7_traj, label) -> dict`：
   - `closed = max(abs(a-b) for a, b in zip(traj.points[-1].positions, traj.points[0].positions)) <= dance_loop_closed_tol_rad`（只比 L1–L6）。
   - `loop_traj = traj` if `closed` else `_append_seam(traj, seam_s)`。
   - `seam_s=0` 且非闭环 → 仍按闭环处理并 WARN。
   - L7：若 `l7_traj` 首末差 > `dance_loop_l7_tol_rad` → `WARN`（循环时夹爪会跳回首值，不阻断）。
   - 返回 `{traj: loop_traj, l7_traj, label, gap_s, cycles: 0, closed}`。

3. `_append_seam(traj, seam_s)`：深拷贝轨迹，从最后一点到首点线性插值 `max(2, int(seam_s/0.05)+1)` 个点，
   `time_from_start` 依次 +`seam_s`；`velocities/accelerations` 留空（JTC splines 按位置插值）。

4. `_publish_status` 的 TRAJ 结束分支改为：

   ```python
   if self._dance_loop is not None:
       if not self._dance_loop_redispatch():
           self._clear_dance_loop()
       else:
           return
   self._back_to_ready()
   ```

   `_dance_loop_redispatch()`：
   - `ok, msg = self._dispatch_trajectory(ctx["traj"], l7_traj=ctx["l7_traj"], skip_static_gate=True)`
     （同一轨迹首轮已完整过静态门禁，循环复用不重复逐点校验）；
   - `ok` → `cycles += 1`；`self._traj_done_at = time.monotonic() + duration + gap_s`；保持 `STATE_TRAJ`；返回 True；
   - `not ok`（通常是 duty gate）→ `WARN`，返回 False（回 READY，并清 loop）。
   - duty gate 仍生效：默认 600 s × 0.8，累计到限自动停。

5. `_dispatch_trajectory(..., skip_static_gate: bool = False)`：仅跳过静态力矩逐点校验，占空比门禁保留。

6. `_clear_dance_loop()` 调用点（必须全覆盖，避免故障后仍在循环）：
   - `_stop_cb` 开头；
   - `_back_to_ready` 开头；
   - `_handle_motor_fault`、`_trigger_temp_protect`、`_handle_oob_disable` 开头；
   - `_prepare_playback` 通过门禁后（开始新回放前）。

#### (f) 参数

`src/a3_arm_controller/config/arm_controller.yaml` 第 6 节后追加：

```yaml
# F197 舞蹈循环
dance_loop_gap_s: 0.2           # 每轮之间的静止间隔
dance_loop_seam_s: 0.8          # 非闭环时尾→首的线性回归段（0=不用，按闭环处理并 WARN）
dance_loop_closed_tol_rad: 0.05 # 首末关节差 ≤ 此值视为闭环，无缝衔接
dance_loop_l7_tol_rad: 0.05     # 夹爪首末差超过则 WARN（循环时会跳回首值）
```

### 4.3 `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`

1. 客户端（`__init__` 内，紧跟 melodion 客户端）：

```python
self._dance_record1 = node.create_client(Trigger, "/a3/arm/dance/record1")
self._dance_record2 = node.create_client(Trigger, "/a3/arm/dance/record2")
self._dance_promote1 = node.create_client(Trigger, "/a3/arm/dance/promote1")
self._dance_promote2 = node.create_client(Trigger, "/a3/arm/dance/promote2")
self._dance_slot: Optional[str] = None   # 当前正在录制的舞蹈槽（客户端侧镜像）
```

2. 新动作：

```python
def dance_record(self, slot: str = "dance1") -> None:
    """DANCE ↓/←：开始连续录舞1/舞2；未 READY 先上电+使能，READY 后补发。"""
    client = self._dance_record1 if slot == "dance1" else self._dance_record2
    if self.arm_state == "READY":
        self._dance_slot = slot
        self._call_trigger_report(client, f"arm/dance/record_{slot}")
        return
    if self.arm_state in ("TEACH", "WAYPOINT_TEACH"):
        self._n.get_logger().warn(f"已在录制（{self.arm_state}）：先结束再切换槽位")
        return
    now = self._n.get_clock().now().nanoseconds * 1e-9
    self._pending_teach = f"dance_{slot}"
    self._pending_teach_since = now
    self.arm_power_enable()

def dance_save(self, slot: str = "dance1") -> None:
    """DANCE ↑/→：结束并保存；播放中则停止。"""
    if self.arm_state == "TRAJ":
        self.arm_stop()
        return
    if self.arm_state == "TEACH":
        if self._dance_slot not in (None, slot):
            self._n.get_logger().warn(
                f"正在录制 {self._dance_slot}：请按对应方向或 □ 保存")
            return
        self.teach_stop()
        return
    self._n.get_logger().warn(f"当前没有可保存的舞蹈录制（state={self.arm_state or '?'}）")

def dance_promote(self, slot: str = "dance1") -> None:
    """DANCE L2 短/长：把较新的 latest（连续或路点）整文件提升为 dance1/dance2。"""
    client = self._dance_promote1 if slot == "dance1" else self._dance_promote2
    self._call_trigger_report(client, f"arm/dance/promote_{slot}")

def playback_dance(self, name: str = "dance1", loop: bool = False) -> None:
    if not self._playback.service_is_ready():
        self._n.get_logger().warn("arm/playback not available")
        return
    req = PlaybackTrajectory.Request()
    req.name = name
    req.type = "dance"
    req.loop = bool(loop)
    self._playback.call_async(req)
    self._n.get_logger().info(
        f"playback_dance -> {{type:'dance', name:'{name}', loop:{bool(loop)}}}")
```

3. `poll()` / `_advance_pending_teach` 扩展：

```python
if kind.startswith("dance_"):
    slot = kind.split("_", 1)[1]
    client = self._dance_record1 if slot == "dance1" else self._dance_record2
    self._dance_slot = slot
    self._call_trigger_report(client, f"arm/dance/record_{slot}")
    return
```

并在 `poll()` 末尾（或 `_advance_pending_teach` 开头）清理客户端镜像：

```python
if self.arm_state not in ("TEACH",):
    self._dance_slot = None
```

### 4.4 `src/a3_teleop_ps4/config/action_registry.yaml`

新增 4 条（`kind: discrete`）：

```yaml
  dance_record:
    kind: discrete
    description: 'F197 DANCE ↓/←：开始连续录制 dance1/dance2（未 READY 自动上电+补发）'
  dance_save:
    kind: discrete
    description: 'F197 DANCE ↑/→：结束并保存当前舞蹈槽；播放中则停止'
  dance_promote:
    kind: discrete
    description: 'F197 DANCE L2 短/长：把较新的 latest（连续/路点）提升为 dance1/dance2'
  playback_dance:
    kind: discrete
    description: 'F197 DANCE △/○：回放 dance 槽 {name, loop}（loop 仅连续槽生效）'
```

### 4.5 `src/a3_teleop_ps4/config/mappings/default.yaml`

DANCE 段整体替换为（其余模式不动）：

```yaml
  dance:
    label: DANCE
    color: magenta
    pattern: breathe
    rumble_index: 7
    enter: [{fn: goto_named_pose, kwargs: {name: ready}, required: false}]
    exit: [{fn: arm_stop}]
    # F197：↓=录舞1 开始、↑=保存舞1；←=录舞2 开始、→=保存舞2
    dpad:
      - axis: dpad_y
        pos: {fn: dance_record, kwargs: {slot: dance1}}
        neg: {fn: dance_save, kwargs: {slot: dance1}}
      - axis: dpad_x
        neg: {fn: dance_record, kwargs: {slot: dance2}}
        pos: {fn: dance_save, kwargs: {slot: dance2}}
    buttons:
      triangle:
        - {fn: playback_dance, kwargs: {name: dance1, loop: false}, edge: shortpress, hold_s: 1.5}
        - {fn: playback_dance, kwargs: {name: dance1, loop: true},  edge: longpress,  hold_s: 1.5}
      circle:
        - {fn: playback_dance, kwargs: {name: dance2, loop: false}, edge: shortpress, hold_s: 1.5}
        - {fn: playback_dance, kwargs: {name: dance2, loop: true},  edge: longpress,  hold_s: 1.5}
      square: {fn: arm_stop, edge: shortpress, hold_s: 1.5}
      # L2 短/长：把 latest 提升为舞1/舞2（DANCE 内不再占用 L2 存命名点）
      l2:
        - {fn: dance_promote, kwargs: {slot: dance1}, edge: shortpress, hold_s: 1.5}
        - {fn: dance_promote, kwargs: {slot: dance2}, edge: longpress,  hold_s: 1.5}
```

> 方向对照：`dpad_y.neg=↑`、`pos=↓`；`dpad_x.neg=←`、`pos=→`（NORMAL 段同样约定，见 L86-L92）。

### 4.6 文档

| 文件 | 改动 |
|---|---|
| `docs/edge/REQUIREMENTS.md` | 表尾新增 `F197` 行（状态 `implemented-pending-hw`） |
| `docs/edge/requirements/F197-dance-slots.md` | 新建：需求、验收标准、目录/服务契约 |
| `docs/edge/PS4_OPERATOR_GUIDE.md` | 模式表 DANCE 说明；新增/替换 DANCE 操作段（录制/提升/单次/循环/停止） |
| `src/a3_teleop_ps4/README.md` | 模式表 L31、按键表 L44 改为 F197 语义 |
| `trajectories/README.md` | 表格新增 `dance/` 行（`dance1/2.yaml` + `backup/`） |
| `docs/shared/TOPIC_CONTRACT.md` | 服务表新增 4 个 Trigger；`playback` 增加 `type=dance`、`loop` 字段说明 |
| `docs/shared/TRAJECTORY_KINDS.md` | 文件位置表补 `dance/`；顺手修正 continuous/waypoint 实际路径（根目录 + `waypoints/`） |
| `.trae/documents/ps4-mode-based-key-mapping-plan.md` | "F193 舞蹈槽位 ❌ 未重建" → 被 F197 取代（“直接录槽 + latest 提升”） |

## 五、风险与边界

1. **循环累计时长**：占空比门禁默认 600 s 窗口内最多 480 s 运动，连跳约 8 分钟后自动停并提示。
   另注：`_play_continuous` 在 `_dispatch_trajectory` 内已记一次 duty（L2509），函数末尾 L6914 又记一次，
   属既有重复计数；本次**不改**，实际预算可能约为名义值一半。
2. **非闭环接缝**：追加 0.8 s 线性回归段只保证位置连续，速度在接缝处仍有突变；建议录制时首末姿态接近。
3. **L7 夹爪**：循环每轮会跳回首值；首末差 >0.05 rad 时 WARN。需要夹爪参与循环的舞蹈应录成闭环。
4. **静态力矩门禁**：舞蹈低姿可能被拒（与回放 latest 同机制），需调 `static_torque_margin_ratio` 而非绕过。
5. **温度**：高位保持 + 反复循环可能触发 95 °C 保护（LL-146），循环会被温度路径终止。
6. **提升源选择**：mtime 取较新者，若同一分钟内两者都更新过，可能提升到非预期源；日志会打印实际来源。
7. **消息兼容**：`PlaybackTrajectory.srv` 加字段后必须重编 `a3_msgs`，旧二进制不要与新消息混跑。
8. **DANCE 的 L2**：不再保存命名点；需要存点请回 NORMAL/TEACH 等其他模式。

## 六、实施顺序

1. `docs/edge/REQUIREMENTS.md` + `docs/edge/requirements/F197-dance-slots.md`（需求先行）。
2. `a3_msgs`：`PlaybackTrajectory.srv` 加 `loop`，重编。
3. `arm_controller.py`：目录/状态 → 录制槽位 → promote → `type=dance` → 循环 → 参数。
4. `actions.py` → `action_registry.yaml` → `default.yaml`。
5. 其余文档同步。
6. 构建与验证（见下）。

## 七、Verification

### 构建与静态检查

```bash
cd /home/cat/a3_arm_ws
colcon build --packages-select a3_msgs a3_arm_controller a3_teleop_ps4 --symlink-install
python3 -m py_compile src/a3_arm_controller/a3_arm_controller/arm_controller.py \
  src/a3_teleop_ps4/a3_teleop_ps4/actions.py
# 映射合法性由节点启动时的 validate_mapping 校验
```

### 仿真/真机验收

1. **连续录舞1**：Share+○ 进 DANCE → `↓` → 拖动 → `↑`；
   日志出现 `dance/dance1.yaml`，文件含 `kind: continuous`、`name: dance1`，`dance/backup/` 生成备份。
2. **单次播放**：`△` 短按 → `playback {type:dance,name:dance1,loop:false}`，
   总时长≈录制时长，播完回 READY。
3. **循环播放**：`△` 长按 → 连续跳，`□` 短按停止回 READY；
   日志无 duty 拒绝；若累计到上限，出现 duty gate 提示并自动停。
4. **舞2**：`←` 录、`→` 存、`○` 短/长播放。
5. **幂等**：录制 dance1 时再按 `↓` → 返回 `already recording dance1`，已录样本不丢。
6. **方向不匹配**：录制 dance1 时按 `→` → 客户端提示“请按对应方向或 □ 保存”，服务端不被调用。
7. **路点链路**：Share+↓ 进 WAYPOINT 录 3 点 → Options 保存 → Share+○ 进 DANCE → `L2` 短按
   → `dance/dance1.yaml` 为 `kind: waypoint`，`△` 短按可回放（`_play_waypoints`）。
8. **路点循环**：`△` 长按 → 只播一次，响应/日志含“路点槽不支持循环”。
9. **提升覆盖备份**：重复 L2 提升同一槽 → `dance/backup/dance1_*.yaml` 出现。
10. **退出保存**：录制中 Options → 自动保存并回 NORMAL；`/a3/arm/stop` 清理循环上下文。
11. **故障**：循环中触发 R3 软失能/超温保护 → 循环停止，不残留 `_dance_loop`。

### 安全检查清单

- 循环中按 `□` 只停止运动，不失能、不掉臂（沿用 `_publish_hold_trajectory`）。
- 循环中 `X` 长按硬急停仍立即断电。
- 循环不绕过静态力矩门禁（首轮全量校验）与占空比门禁（每轮校验）。
