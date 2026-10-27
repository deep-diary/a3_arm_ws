# 轨迹种类（Trajectory Kinds）清单

编排层 `/a3/arm/playback` 的轨迹配置文件按 `kind` 区分执行语义。本文是权威清单，
与 [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md) 的 `playback` 服务、[CONTROL_ROADMAP.md](CONTROL_ROADMAP.md) 的能力矩阵对齐。

## 四类 playback kind

| kind | 语义 | 执行链 | 状态 |
|---|---|---|---|
| `continuous` | 密集录制点（手动拖动录制） | F137 几何去噪 + F68 retime + JTC 下发 | 已有 |
| `waypoint` | 离散点序列（手动打点；可按 8 种方式规划，支持循环） | `mode` 为空走旧 `strategy`（pilz Sequence / 逐段）；F198 给 `mode` 时转段复用 `_execute_segment`（`ptp`/`lin_exact` 逐腿停，其余单段 via+target） | 已有 |
| `stream` | 段序列（软件编排；每段 `mode/speed/sleep/via`） | `_execute_segment`（ptp/lin/via）逐段 | 已有 |
| `cp` | 连续笛卡尔路径（软件编排；via 点列 + 目标一条轨迹） | `_play_cp` → `_goto_via_smooth`/`_goto_via_lin_blend` 一条 C2 轨迹 | **F163/F164 新增** |

> F197：`/a3/arm/playback` 另有 `type=dance` 的**槽位**（不是新 kind）——路径固定
> `trajectories/dance/<name>.yaml`，按文件头 `kind`（`continuous`/`waypoint`）分派；
> 请求里的 `loop` 只对连续槽生效（复用已 retime 轨迹循环下发）。
>
> F198：`type=waypoint` 新增 `mode` 字段（8 种规划方式，`circ` 因需辅助点被剔除）；
> `loop=true` 时整轮循环（轮间 `waypoint_loop_gap_s`，每轮由正常规划腿从末点回 wp1），
> `/a3/arm/stop` 置取消事件后在下一段前退出。旧 `strategy=ptp|lin` 链路语义不变。

## 非 playback kind（不在 `/a3/arm/playback` 内）

| 场景 | 运行时载体 | 说明 |
|---|---|---|
| `servo` | `/a3/servo/joint_trajectory`（L6/C4） | 实时笛卡尔速度，独立运行时模式 |
| 力控/导纳 | L8/C8 | 依赖末端六维 F/T 硬件，非轨迹 kind |
| 防晃运输（slosh-free） | 未实现 | 对标缺口，见 [CONTROL_ROADMAP.md](CONTROL_ROADMAP.md) |

## 段模型（stream/cp 共用）

- `stream` 段：`{target: {positions|pose, exact?}, mode, speed, sleep, via:[{positions|pose, exact?}], hold_orientation?}`。
  - `hold_orientation`（F176，可选，默认 false）：true 时关节空间段（ptp_legs/soft_smooth/无 via 单腿）
    用该点 `pose.orientation` 做 **6D IK**（并以链上前一点解作种子），姿态不可达时回落 position-only；
    笛卡尔段（lin_*/circ）本就按 6D 位姿走，无需该开关。
  - `mode`：**9 种规范名**（`arm_controller.TRAJ_MODES`）——`ptp`/`ptp_blend`/`lin_exact`/`lin_blend`/
    `jnt_g64_quintic`/`jnt_g64_trapezoid`/`jnt_spline_bell`/`circ`/`jnt_spline_approx`；
    旧名 `pilz/blend/l1p5/l2/l3/lin/ptp` 为**别名**（F166，`resolve_traj_mode`）。
  - `mode: auto`：由离线 `scripts/traj_plan_auto.py` 按逐点 `exact` + 时长/平滑择优解析为具体模式；
    运行时若仍是 auto → 保守兜底 + WARN。
  - `exact`（F166，逐点）：该点是否**必须精确过点**（`true` → blend/圆角半径置 0）；via 默认 false、target 默认 true。
- `cp`：`{kind: cp, joint_names, speed, mode, via:[{pose|positions},...], target}`，一条过点样条
  （圆弧用 `mode=circ` 的 stream 段，pilz CIRC，辅助点放 `path_constraints(name=interim)`）。


## 文件位置

| kind | 位置 |
|---|---|
| `continuous` | `<ws>/trajectories/*.yaml`（根目录，`latest.yaml` 为滚动槽；运行时录制，untracked） |
| `waypoint` | `<ws>/trajectories/waypoints/*.yaml`（`latest.yaml` 为滚动槽；F198 起 `wp1.yaml`/`wp2.yaml` 为 WAYPOINT 双槽位，备份 `teach_wp_<slot>_<ts>.yaml`） |
| `stream` / `cp` | `app_melodion/config/streams/`（版本控制，脚本生成源）+ `<ws>/trajectories/streams/`（实验槽） |
| `continuous` / `waypoint`（舞蹈槽） | `<ws>/trajectories/dance/{dance1,dance2}.yaml`（F197；`type=dance` 回放，文件头 `kind` 决定执行链） |

搜索顺序见 `arm_controller._stream_path` / `_cp_path`（仓库 trajectories → app_melodion config → 旧 `~/.a3`）。

## 相关文档

- [control/03-trajectory.md](../control/03-trajectory.md) — 轨迹规划与时间律「从简到难」进阶清单（模块化学习手册）
- [CONTROL_ROADMAP.md](CONTROL_ROADMAP.md) — 控制功能分层路线（L2/L4/L5）
