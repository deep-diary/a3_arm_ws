# 规划：封装 `gotoby([middle_pt], target, spd, sleep)` 单条过点平滑轨迹（弹琴跨键过渡）

## 一、结论（Summary）

把「当前位 → 若干中间过点 → 目标点」规划成**一条** C2 平滑轨迹（复用本仓 F161 已落地的
「过点样条 + jerk-limited 时间重参数化 + JTC 下发」链路），而不是像现在这样把中间点当独立段
逐段执行、段末停一下。落地方式（已与你确认）：

1. **不加新服务**，扩展 `stream` 段模型：段内新增可选 `via: [中间点...]` 数组。
2. 弹琴键间过渡的「中间抬高点」默认取 **AB 中垂线上方**（方式一，弧顶对称、路径最短、方向
   变化最缓），做成可配置。

收益：弹琴从「点 A → 点 B 由 2 条轨迹（下压 + 抬起）组成」变成 **1 条过点弧线**，减少一次
中间停顿、更连贯高效；同一原语也天然适用于 waypoint 多过点场景（F161 巡游平滑链同源）。

---

## 二、现状分析（Current State，已读代码确认）

### 2.1 弹琴现有链路（F162）
- 生成器 [melodion_score2stream.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_score2stream.py) 每个音符产出 **down + up 两段**（`mode=ptp`，`sleep=音符时长`）。
  - 两键之间实际是：`down_A →(停)→ up_A →(停)→ down_B`，中间抬高点（up_A）**必然停一次**。
- 编排层逐段执行 [arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)：
  - `_execute_segment`（L3680）逐段 `_moveit_move`（ptp）或 `_lin_move`（lin），段末速度 0。
  - `_play_stream`（L3726）逐段调用 `_execute_segment`。
- 段模型目前只有 `{target, mode, speed, sleep}`，**没有 `via`**。

### 2.2 已有可复用的「过点平滑」能力（F161，本次直接复用，不重写）
在 [arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) 已存在模块级函数（巡游平滑链 `_tour_smooth_chain` L3888 正在用）：
- `_interpolate_via_spline(via_points, resample_n)`（L178）：稀疏 via 点 → 过点样条稠密 C2 路径（`make_interp_spline`，严格过点；n=3 退二次弧、n≥4 三次、n≥6 五次）。
- `_jerk_limited_timed_traj(dense, vmax, amax)`（L224）：对 C2 路径施加 quintic 时间剖面（加加速度有界）重定时，返回 `(times, positions)`。
- `_dispatch_trajectory`（L1855）：JTC 下发（L1–L6；L7 不参与保持 0），**下发前逐点静态力矩门禁 + 占空比门禁**（L1876/L1890），所以过点样条路径的安全门禁可交给它，无需重复预检。
- `_solve_ik_position(pose)`（L3016）：笛卡尔 xyz → L1–L6 关节（Pinocchio position-only IK，姿态自由），弹琴目标姿态自由时用。

### 2.3 关键 gap
1. `_execute_segment` 只能单目标（ptp/lin），**不能「过 via 点 → 目标」一条轨迹**。
2. 生成器按「down + up」两段建模，中间抬高点必停。
3. 平滑链参数 `random_tour_smooth_max_vel_rad_s / _acc / _resample_n` 是巡游专用，且**无 per-call 速度**（`spd`）映射，也**不接收笛卡尔 via 点**。

---

## 三、方案设计（Proposed Changes）

### 3.1 编排层：新增 `_goto_via_smooth` 原语（`arm_controller.py`）

新增方法（放在 `_execute_segment` 附近，L3680 前后），签名：

```python
def _goto_via_smooth(self, via: List[dict], target: dict, speed: float,
                     label: str) -> Tuple[bool, str]:
    """当前位 → via 过点数组 → target 规划为一条 C2 平滑轨迹（关节空间过点样条 + quintic retime + JTC）。
    每个 via / target 为 {"positions":[...]}（L1..L6 关节）或 {"pose":{position,orientation}}（笛卡尔，走 _solve_ik_position）。
    speed(0~1) 缩放 vmax/amax。失败返回 (False, msg)，调用方回落单段 _moveit_move。"""
```

内部步骤（全部复用现有函数，不新增算法）：
1. 取当前关节 `q_cur`（L1–L6）。
2. 解析每个 via 点与 target 为 L1–L6 关节向量（`positions` 直接取前 6；`pose` 走 `_solve_ik_position`；任一 IK 失败 → 返回 `(False, "via IK failed")`）。
3. 构造 `[q_cur] + via_q + [target_q]`，调用 `_interpolate_via_spline(...)`（`resample_n = goto_via_resample_n`）。返回 None（点数 <3，即无 via）→ 返回 `(False, ...)` 让调用方回落单段。
4. 由 `speed` 映射峰值上限：`vmax = speed * goto_via_max_vel_rad_s`、`amax = speed * goto_via_max_acc_rad_s2`，调用 `_jerk_limited_timed_traj(dense, vmax, amax)`。
5. 组装 `JointTrajectory`（`joint_names = JOINTS[:6]`），`_publish_mode("TRAJ_RUNNING")` → `_dispatch_trajectory(traj)`（静态力矩/占空比门禁在内部生效）。
6. 成功后 `_set_state(STATE_TRAJ, label)`、`_schedule_back_to_ready(duration + 0.3)`；返回 `(True, msg)`（含点数/时长/平滑度 `a_rms/j_rms` 可选）。

新增参数（`declare_parameter` + `config/arm_controller.yaml` 同步，见 3.4）：
- `goto_via_max_vel_rad_s`（默认 0.6）
- `goto_via_max_acc_rad_s2`（默认 2.0）
- `goto_via_resample_n`（默认 200）

> 说明：**不复构 `_tour_smooth_chain`**，避免回归巡游；两者复用同一批模块级函数。waypoint 多过点以后若要统一，可再让 `_play_waypoints` 走本原语（本次不扩scope）。

### 3.2 扩展 `_execute_segment` 支持 `via`

在 [arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) `_execute_segment`（L3680）开头加：

```python
via = seg.get("via")
if via:                       # 过点段：单条平滑弧线
    ok, msg = self._goto_via_smooth(list(via), target, speed or 1.0, label)
    if ok and sleep_s > 0.0:
        time.sleep(sleep_s)
    return ok, msg
# 否则走原有 ptp/lin 单段逻辑（不变）
```

- `via` 仅对 `mode=ptp`（或缺省）生效；`mode=lin` + `via` 不启用（本次不支持，生成器也不会产出）。
- `speed` 为 None 时取全局默认（`goto_via_*` 上限 × 1.0，或复用 `moveit_velocity_scaling` 语义，见 3.5 决策）。

### 3.3 生成器：弹琴跨键过渡改为单条过点弧

**`app_melodion/key_config.py`** 新增：
```python
def transition_via_pose(self, key_from: Key, key_to: Key) -> Dict[str, object]:
    """返回 key_from 按压点与 key_to 按压点的中垂线上方 hover 位姿（via 过点）。
    策略取 playing_geometry.yaml 的 via_apex.strategy：
      midpoint       —— 两按压点水平中点、z = base.z + approach_height（默认，方式一）
      target_above   —— 目标键正上方（方式二）
      source_above   —— 当前键正上方
    """
```
（中垂线弧顶 = 两点按压坐标 `(x/y)` 取中点，`z = base.z + approach_height_mm/1000`，姿态沿用 `_orientation()`。）

**`config/playing_geometry.yaml`** 新增：
```yaml
# 键间过渡「中间抬高点」策略（via 过点）
via_apex:
  strategy: midpoint        # midpoint | target_above | source_above
  height_mm: null           # null=用 approach_height_mm；否则显式指定
```

**`app_melodion/melodion_score2stream.py`** `build_stream` 改为：
- 首音符：`{target: {pose: down_first}, mode: ptp, speed, sleep: dur_first}`（接近 + 下压 + 按住）。
- 后续音符 i：`{via: [{pose: transition_via_pose(prev_key, cur_key)}], target: {pose: down_cur}, mode: ptp, speed, sleep: dur_cur}`（抬升过弧 → 下压 → 按住）。
- 休止符 REST：累加到上一段 `sleep`（不变）。
- 末尾追加一段 `{target: {pose: up_last}, mode: ptp, speed, sleep: 0.0}`，把手臂抬离末键（避免停在按压位）。

结果：`scale` 32 音符 → **33 段**（32 音符 + 1 末段抬离）；原 64 段（down+up）减半，且每处跨键从「停一次」变「一条弧」。

### 3.4 参数/契约同步
- [arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)：新增 `goto_via_*` 三个 `declare_parameter`。
- [arm_controller.yaml](file:///home/cat/a3_arm_ws/src/a3_arm_controller/config/arm_controller.yaml)：在 goto 段（L24 附近）补 `goto_via_max_vel_rad_s / goto_via_max_acc_rad_s2 / goto_via_resample_n`。

### 3.5 需求文档（requirements-first）
- 新增 `docs/edge/requirements/F163-via-point-smooth-transition.md`：ID/说明/验收标准/关联文档/状态。
- `docs/edge/REQUIREMENTS.md` 功能需求表加 `F163` 一行。
- 更新 [F162-melodion-playing.md](file:///home/cat/a3_arm_ws/docs/edge/requirements/F162-melodion-playing.md) 的段模型说明（down+up 两段 → 过 via 一条弧 + 末段抬离）。
- [TOPIC_CONTRACT.md](file:///home/cat/a3_arm_ws/docs/shared/TOPIC_CONTRACT.md)：`/a3/arm/playback type=stream` 的段模型补 `via` 字段说明。

### 3.6 文档与测试
- [app_melodion/README.md](file:///home/cat/a3_arm_ws/src/app_melodion/README.md)：段模型补 `via`、键间过渡策略说明。
- 单测 [test_score_stream.py](file:///home/cat/a3_arm_ws/src/app_melodion/test/test_score_stream.py)：段数断言由 `64`（down+up）改为 `33`（32 音符 + 末段）；新增断言「跨键段含 `via` 且 via 为两键中垂线上方」。

---

## 四、假设与决策（Assumptions & Decisions）

1. **触发方式**：扩展 `stream` 段模型加 `via` 数组，**不加新服务**（已确认）。
2. **默认 PTP**：`via` 段固定走关节空间过点样条（`mode=ptp`），不回退 LIN。理由见 5.1。
3. **中间抬高点默认方式一（中垂线弧顶）**，`via_apex.strategy` 可切 `target_above`/`source_above`（已确认）。
4. **速度映射**：`spd`（段级，0~1）线性缩放 `goto_via_max_vel_rad_s / goto_via_max_acc_rad_s2`；`spd=None` 时取全局上限（即 1.0×上限）。简单、可预测，不新增复杂曲线。
5. **安全门禁**：过点样条路径的静态力矩 + 占空比门禁由 `_dispatch_trajectory` 内部完成（已存在），`_goto_via_smooth` 不再重复预检；失败返回 False 由 `_play_stream` 走现有段重试/中止逻辑。
6. **L7 夹爪**：弹奏保持关闭（0），via 轨迹只含 L1–L6，L7 不参与（与 F161 巡游一致）。
7. **笛卡尔 overshoot 风险**：过点样条是**关节空间**插值，跨键弧的笛卡尔抬升量由 via 点高度（approach_height，20mm）提供裕量；若真机观察到弧底擦键，调大 `via_apex.height_mm` 或降低 `goto_via_max_vel_rad_s`（见验收 5.4）。

---

## 五、对你两个评估问题的回答（结论已并入方案）

### 5.1 LIN 中间点是否必停？（不必停，但弹琴仍推荐 PTP）
- pilz Sequence（F134 已落地）支持 LIN 段间 **blend 过中间点不停车**，`_adaptive_blend_radii`（L3064）会给相邻段算 blend 半径。
- 但有三点限制：① blend **不精确过中间点**（F134 已注明「精确过点 + 速度非零物理上不可能」）；② blend 半径被压到 `≤0.45×min(相邻段距)`，段距不足降到 0（该点又变停车）；③ blend 只是 L1 转角平滑（速度连续、**加速度不连续**，见 F161 表）。
- 对弹琴：键距 18.9mm、抬升 20mm，相邻段距约 22mm → blend 上限约 10mm，**小于抬升高度**，会削减安全间隙；且速度连续性不如关节空间过点样条（quintic 时间剖面 = 加加速度有界）。
- **结论**：LIN 中间点可 blend 不停车，但对弹琴收益低、间隙裕量差，故 **默认 PTP（关节过点样条）**，与你的判断一致。

### 5.2 方式一（中垂线弧顶）是否更短、更平滑？（是，你的判断正确）
设键距 `d`、抬升 `h`：
- 方式一（中垂线弧顶）：路径 ≈ `2·√((d/2)²+h²) = √(d²+4h²)`。
- 方式二（目标正上方）：路径 ≈ `√(d²+h²) + h`。

代入 `d=18.9mm`、`h=20mm`：方式一 ≈ 44.2mm，方式二 ≈ 47.5mm → **方式一短约 7%**。
平滑度：方式一弧顶对称，方向仅在竖直分量反向、水平分量连续；方式二在「目标正上方」处有近 90° 直角折返。**方式一更平滑**。
> 注：过点样条实际走的是「A→弧顶→B」的连续曲线（非折线三角），比上面直线近似更顺滑；弧顶高度 `h` 同时充当跨键安全间隙。

---

## 六、验证（Verification）

1. **单测**：`cd src/app_melodion && python3 -m pytest test/ -q`——`scale` 33 段、跨键段含 `via`、via 为中垂线上方、REST 累加 sleep 不变。
2. **生成**：`python3 -m app_melodion.melodion_score2stream --score scale && --score qinghuaci` 输出合法 stream（via 段齐全）。
3. **mock 栈端到端**：
   - `a3_stack.sh` 起 mock 栈 → init/enable → goto home → `playback type=stream name=scale`。
   - RViz/`/joint_states`：跨键处末端走「抬升弧 → 下压」单条连续轨迹，**无中间停车**（对比现状 down+up 两段）。
4. **平滑度 A/B**（复用 F161 `_finite_diff_metrics` 口径）：同一条跨键过渡，`via` 段 vs 现状两段，`j_rms/a_rms` 明显下降、跨键无「速度归零再跳起」。
5. **真机回归（待上电）**：跨键弧底不擦键（间隙足够）、按压力点准确（`via` 段末 `_solve_ik_position` 到 down 点误差 ≤ 阈值）、L3 长时间持有不触发温度保护（沿用手势/门禁）。
6. **回退**：`_solve_ik_position` 失败 / `_interpolate_via_spline` 返回 None / `_dispatch_trajectory` 被门禁拒绝时，`_play_stream` 走现有段重试并返回可读 message，状态机不卡死。

---

## 七、实现顺序（建议 todo）

1. `F163` 需求文档 + `REQUIREMENTS.md` 索引。
2. `arm_controller.py`：`goto_via_*` 参数 + `_goto_via_smooth` + `_execute_segment` 支持 `via`；`arm_controller.yaml` 补参数。
3. `app_melodion`：`key_config.transition_via_pose` + `playing_geometry.yaml` 加 `via_apex` + `melodion_score2stream.py` 改为过 via 段。
4. 单测更新 `test_score_stream.py`。
5. 文档同步（F162 / TOPIC_CONTRACT / app_melodion README）。
6. mock 栈端到端 + 平滑度 A/B 验收。
