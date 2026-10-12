# 轨迹系统三增强：相对位置、预烘焙缓存、模式×速度扫描测试报告

## 一、结论（Summary）

| 需求 | 结论 | 落地方式 |
|---|---|---|
| 1. 相对位置 | 评估后**实现**：stream 顶层加 `frame: absolute/relative` + `anchor` 锚点，相对模式下只改锚点即可换口风琴位置 | 编排层解析 + key_config 生成 |
| 2. 预烘焙缓存 | 整条 stream 烘焙成 `<name>.baked.yaml` 关节轨迹文件；playback 检测到存在则直接下发（跳过 IK+重定时），不存在则实时算；删掉即重新烘焙 | `_play_stream` 透明缓存 + 离线烘焙脚本 |
| 3. 扫描测试报告 | 生成 F3↔C6 模式(ptp/lin)×速度(0.1~1.0) 的来回扫描轨迹，仿真运行采集指标，出 MD+图表报告 | 生成脚本 + 运行采集 + 报告脚本 |

---

## 二、现状分析（已读代码确认）

### 2.1 轨迹点是绝对位置
- [key_config.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/key_config.py) `key_pose()` 用 `base_pose`（绝对 xyz）生成每键 up/down 笛卡尔位姿；`transition_via_pose()` 同。生成的 stream（[scale.yaml](file:///home/cat/a3_arm_ws/src/app_melodion/config/streams/scale.yaml)/[inspection.yaml](file:///home/cat/a3_arm_ws/src/app_melodion/config/streams/inspection.yaml)）target/via 都是 `pose.position`（base_link 绝对 xyz）。**无 relative 概念。**

### 2.2 计算耗时在 via 段实时 IK+重定时
- [arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) `_goto_via_smooth()`：每段 via 都 `_solve_ik_position`（姿态保持 6D）+ `_interpolate_via_spline` + `_jerk_limited_timed_traj`，逐段实时算。实测 inspection playback 壁钟 ~113s（名义轨迹 32.4s），大头是这 31 段实时计算。
- F133 有 `retimed` 写回机制（continuous 的 `already_retimed`），可借鉴：**已算好的轨迹存文件，下次直接读**。

### 2.3 评价指标与报告已有基础
- [traj_smoothness_calibration.py](file:///home/cat/a3_arm_ws/scripts/traj_smoothness_calibration.py) `compute_metrics`：命令轨迹 v_rms/a_rms/j_rms/a_peak/j_peak（有限差分）。
- [f160_real_ab_acceptance.py](file:///home/cat/a3_arm_ws/scripts/a3_test/f160_real_ab_acceptance.py)：订阅 `/arm_controller/state`（跟踪误差）+ `/a3/arm_status`（力矩分量），`compute_metrics` 给 peak_err/rms_err/steady_lag/pd_peak/eff_jitter/vel_jitter。可复用「跟踪误差采集」。
- [make_report.py](file:///home/cat/a3_arm_ws/scripts/make_report.py)/[traj_smooth.py](file:///home/cat/a3_arm_ws/scripts/traj_smooth.py)：手写 SVG 叠加图 + MD 报告。

---

## 三、方案设计（Proposed Changes）

### 3.1 需求 1：相对位置（frame 字段 + 编排层解析）

**格式**（stream 顶层新增，向后兼容）：
```yaml
kind: stream
frame: relative        # absolute（默认，缺省=absolute）| relative
anchor:               # relative 模式的锚点（第一个按压点的绝对位置）
  position: {x: -0.20, y: -0.17, z: 0.255}
segments:
  - target: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, ...}}   # 相对偏移（dx,dy,dz）
```

- `frame=relative` 时，每个 target/via 的 `pose.position` 存**相对锚点的偏移**；`orientation` 仍绝对（口风琴恒 [0,0,0,1]，不随锚点变）。
- 换位置：只改 `anchor`，其余不动。

**编排层**（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) `_play_stream` / `_play_cp`）：
- 加载 stream 后，若 `frame == "relative"`：读取 `anchor.position`，把每个 segment 的 `target`/`via` 的 `position` 加上 anchor（展开成绝对），再走既有 `_execute_segment`/`_goto_via_smooth`。展开是纯 dict 加法，改动小。
- 若缺 `anchor` 或 `frame` 值非法 → 返回可读错误（不静默）。

**生成端**（[melodion_score2stream.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_score2stream.py)）：
- `build_stream`/`build_inspection_stream` 加 `relative: bool = False` 参数：relative=True 时，以第一个键 down 位置为锚点，所有 target/via 的 position 减去锚点，顶层写 `frame: relative` + `anchor`。
- CLI 加 `--relative` 标志。

> 评估结论（写进文档）：绝对=简单直白但换位要整体重生成；相对=换位只改锚点、便于现场对位，代价是编排层多一步展开。二者并存，默认绝对（向后兼容）。

### 3.2 需求 2：预烘焙缓存（`<name>.baked.yaml`）

**烘焙文件格式**（关节轨迹，复用 continuous 的 points 结构 + `retimed: true` 免重烘焙）：
```yaml
kind: continuous
retimed: true          # F133 标志：跳过 F68 重烘焙
joint_names: [L1_joint, ..., L7_joint]
points:
  - positions: [7 关节]
    time_from_start_sec: 0.0
  - ...
```

**生成**（新脚本 `scripts/melodion_bake.py`，或 [melodion_score2stream.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_score2stream.py) 加 `--bake`）：
- 复用 `_interpolate_via_spline` + `_jerk_limited_timed_traj` + 姿态保持 IK，把 stream 的逐段 via 轨迹**拼接成一条连续关节轨迹**（段间 sleep 用时间轴间隔表达），输出 `<name>.baked.yaml`。
- 姿态保持 IK 与重定时用与编排层**同源**的函数（离线脚本 import `arm_controller` 的模块级函数），保证烘焙结果与实时算一致。

**加载**（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) `_play_stream` 开头）：
- 检测 `<name>.baked.yaml` 存在（`_stream_path` 同目录）→ 加载 points，`_dispatch_trajectory` 一条下发（含 sleep 时间轴），`_schedule_back_to_ready`，直接返回（跳过逐段 IK+重定时）。
- 不存在 → 走既有逐段 via 实时算（原行为）。

> 重新烘焙 = 删除 `<name>.baked.yaml`（baked 文件 git 忽略，不入库）。

### 3.3 需求 3：模式×速度扫描测试轨迹 + 报告

**生成**（[melodion_score2stream.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_score2stream.py) 加 `build_sweep_stream()` + `--sweep`）：
- 最左 F3 ↔ 最右 C6，来回扫描。段序列：`mode ∈ {ptp, lin}` × `speed ∈ {0.1, 0.2, …, 1.0}`，每档 `F3 down → C6 down → F3 down`（一去一回），段末 sleep 短（0.1s）。输出 `streams/sweep.yaml`（含 `frame: relative` 便于复用锚点语义）。

**运行采集**（新脚本 `scripts/a3_test/melodion_sweep_run.py`）：
- 复用 [f160_real_ab_acceptance.py](file:///home/cat/a3_arm_ws/scripts/a3_test/f160_real_ab_acceptance.py) 的采集思路：订阅 `/arm_controller/state`（跟踪误差）与 `/a3/arm_status`（state/mode/力矩），在 `TRAJ_RUNNING` 段采集。
- 逐段回放 sweep（或一次回放整条 sweep，用 mode 切换切段），按段记录指标。
- 每段指标：`时长`（命令轨迹 T）、`a_rms/j_rms`（compute_metrics）、`peak_err/rms_err`（跟踪误差）、`成功/失败`。

**报告**（新脚本 `scripts/a3_test/melodion_sweep_report.py`）：
- 汇总成 Markdown 报告 + **matplotlib PNG 图表**（3 张）：速度 vs 时长、速度 vs j_rms、速度 vs 跟踪误差（ptp/lin 两线对比）。
- 稳定性判定：无失败段、无 FAULT、`peak_err` 不超阈值（默认 0.05 rad）；超阈值的速度档标「不建议」。

---

## 四、假设与决策

1. **相对位置默认 absolute**（向后兼容），`frame: relative` + `anchor` 可选。
2. **锚点 = 第一个键 down 位置**；orientation 不随锚点变（口风琴恒朝下）。
3. **烘焙文件透明缓存**：不新增 kind，复用 `continuous` + `retimed: true`；`_play_stream` 自动检测 `<name>.baked.yaml`，存在则用，不存在则实时算；baked 文件 git 忽略。
4. **烘焙与实时算同源**：离线烘焙脚本 import `arm_controller` 的 `_interpolate_via_spline`/`_jerk_limited_timed_traj`/姿态保持 IK，保证一致。
5. **扫描报告图表用 matplotlib PNG**（速度 vs 指标的对比散点/折线），MD 引用 PNG；稳定性判定阈值 `peak_err=0.05 rad` 可调。
6. **本轮仿真验收**：相对/烘焙/扫描都在仿真栈（domain 99）验证；真机跟踪误差指标留真机（脚本已按真机/仿真通用设计）。

---

## 五、验证

1. **相对位置**：生成 `--relative` 的 scale，确认 target/via position 为相对偏移、顶层含 `frame: relative`+`anchor`；仿真 playback 后末端落点与绝对模式一致（对位不漂移）。
2. **预烘焙**：`melodion_bake --score scale` 生成 `scale.baked.yaml`；playback `type=stream name=scale` 时走 baked（日志「baked」），壁钟从 ~113s 降到 ~秒级（跳过 31 段 IK+重定时）；删掉 baked 后回退实时算。
3. **扫描报告**：`--sweep` 生成 sweep.yaml；仿真运行，报告输出 MD + 3 张 PNG，含每档时长/a_rms/j_rms/跟踪误差 + 稳定性判定；无失败段/FAULT。
4. **单测/编译**：app_melodion `pytest` 仍通过；`colcon build` 通过。

---

## 六、实现顺序

1. 需求 1：编排层 `_play_stream`/`_play_cp` 相对展开 + key_config `--relative`。
2. 需求 2：`melodion_bake.py` 烘焙脚本 + `_play_stream` 检测 baked 缓存 + `.gitignore` 加 `*.baked.yaml`。
3. 需求 3：`build_sweep_stream` + `melodion_sweep_run.py` + `melodion_sweep_report.py`。
4. 文档同步（TOPIC_CONTRACT 的 playback 描述 + app_melodion/README）。
5. 仿真验收：相对 playback、baked 提速、sweep 报告。
