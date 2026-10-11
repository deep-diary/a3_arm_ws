# 轨迹种类（Trajectory Kinds）清单

编排层 `/a3/arm/playback` 的轨迹配置文件按 `kind` 区分执行语义。本文是权威清单，
与 [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md) 的 `playback` 服务、[CONTROL_ROADMAP.md](CONTROL_ROADMAP.md) 的能力矩阵对齐。

## 四类 playback kind

| kind | 语义 | 执行链 | 状态 |
|---|---|---|---|
| `continuous` | 密集录制点（手动拖动录制） | F137 几何去噪 + F68 retime + JTC 下发 | 已有 |
| `waypoint` | 离散点序列（手动打点；blend/逐段） | pilz Sequence（blend）/ `_moveit_move`+`_lin_move` 逐段 | 已有 |
| `stream` | 段序列（软件编排；每段 `mode/speed/sleep/via`） | `_execute_segment`（ptp/lin/via）逐段 | 已有 |
| `cp` | 连续笛卡尔路径（软件编排；lin 折线 + circ 圆弧） | `_play_cp` → `_goto_via_smooth`/`_goto_via_lin_blend` 一条 C2 轨迹 | **F163/F164 新增** |

## 非 playback kind（不在 `/a3/arm/playback` 内）

| 场景 | 运行时载体 | 说明 |
|---|---|---|
| `servo` | `/a3/servo/joint_trajectory`（L6/C4） | 实时笛卡尔速度，独立运行时模式 |
| 力控/导纳 | L8/C8 | 依赖末端六维 F/T 硬件，非轨迹 kind |
| 防晃运输（slosh-free） | 未实现 | 对标缺口，见 [CONTROL_ROADMAP.md](CONTROL_ROADMAP.md) |

## 段模型（stream/cp 共用）

- `stream` 段：`{target: {positions|pose}, mode: ptp|lin, speed, sleep, via:[...]}`。
  - `via` 非空时：`mode=ptp` 走关节过点样条（`_goto_via_smooth`，C2 精确过点）；
    `mode=lin` 走 pilz Sequence blend（`_goto_via_lin_blend`，不停车但不精确过点）。
- `cp`：`{kind: cp, joint_names, speed, mode, via:[{pose|positions},...], target}`，一条过点样条。

## 文件位置

| kind | 位置 |
|---|---|
| `continuous` / `waypoint` | `<ws>/trajectories/{continuous,waypoint}/`（运行时录制，untracked） |
| `stream` / `cp` | `app_melodion/config/streams/`（版本控制，脚本生成源）+ `<ws>/trajectories/streams/`（实验槽） |

搜索顺序见 `arm_controller._stream_path` / `_cp_path`（仓库 trajectories → app_melodion config → 旧 `~/.a3`）。
