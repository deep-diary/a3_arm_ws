# F198 路点双槽位 + 规划方式切换（WAYPOINT 闭环）

## 背景

WAYPOINT 模式此前只负责“录路点”（`waypoints/latest.yaml`），回放却散落在 NORMAL
（`□` 长按 = PTP、`○` 长按 = LIN），且不能循环、不能选规划方式。路点文件本身不带
`mode` 字段，回放也就无法复用 F166 的 9 模式注册表。结果是：

- 录点和播点分处两个模式，操作路径不连贯；
- “方形盘回到 NORMAL”与 `Options` 短按重复；
- 9 种规划方式只有 stream/cp 能用，路点只能 ptp/lin 二选一。

本需求把“录 wp1/wp2、选规划方式、单次/循环播放、停止”全部收口到 WAYPOINT 模式，
结构对齐 F197 DANCE 双槽位。

## 目录与槽位

```
trajectories/waypoints/
├── latest.yaml                          # 兼容旧行为（无槽位录制 / 空名回放）
├── wp1.yaml                             # ↓/↑ 录制的槽 1
├── wp2.yaml                             # ←/→ 录制的槽 2
└── teach_wp_<slot>_YYYYMMDD_HHMMSS.yaml # 槽位历史备份
```

- 文件结构不变：`kind: waypoint` + `waypoints[{positions(L1..L6/L7), pose}]`。
- 槽位录制不覆盖通用 `latest.yaml`；`melodion/waypoints/{press,extremes}.yaml` 不受影响。

## 服务契约

| 服务 | 类型 | 说明 |
|---|---|---|
| `/a3/arm/waypoint/record1` | `std_srvs/Trigger` | 开始路点示教，保存到 `waypoints/wp1.yaml` |
| `/a3/arm/waypoint/record2` | `std_srvs/Trigger` | 开始路点示教，保存到 `waypoints/wp2.yaml` |
| `/a3/arm/playback` | `a3_msgs/PlaybackTrajectory` | 新增 `mode` 字段（路点规划方式）；`loop` 对 `type=waypoint` 生效 |

`PlaybackTrajectory.mode` 允许值（`circ` 不支持）：

```
ptp | ptp_blend | lin_exact | lin_blend
jnt_g64_quintic | jnt_g64_trapezoid | jnt_spline_bell | jnt_spline_approx
```

- `mode` 留空 = 沿用旧 `strategy: ptp|lin` 链路（旧脚本/旧调用不受影响）。
- `mode=circ` 明确拒绝：路点没有“辅助点”语义。
- `loop=true` 时整条路点序列循环，直到 `/a3/arm/stop`（`□` / `Options` / 切模式）。

## PS4 映射（WAYPOINT 模式，Share + □）

| 操作 | 动作 |
|---|---|
| `↓` | 开始录 wp1 |
| `↑` | 结束并保存 wp1；播放中则停止 |
| `←` | 开始录 wp2 |
| `→` | 结束并保存 wp2 |
| `L2` | 打点（当前 7 关节 + FK 位姿） |
| `△` 短按 / 长按 | 播放 wp1（单次 / 循环） |
| `○` 短按 / 长按 | 播放 wp2（单次 / 循环） |
| `□` 按下 | 停止回 READY（`edge: rising`） |
| `R2` 按下 | 循环切换规划方式（8 种，默认 `ptp_blend`） |
| `Options` 短按 | 退出回 NORMAL（录制中先保存） |

- 进入 WAYPOINT **不再自动开录**，由 `↓`/`←` 明确选槽。
- `R2` 在 WAYPOINT 内改为离散键，因此该模式**屏蔽 R2 夹爪力控**
  （夹爪角度仍随路点录制/回放；需要手调时切回 NORMAL）。
- `Share + □` 从原来的“回 NORMAL”改绑为进入 WAYPOINT（`Options` 仍是统一退出）；
  删除 NORMAL 的
  `□` 长按路点 PTP、`○` 长按路点 LIN，避免两个入口。

## 规划方式与 HUD

| 规范名 | HUD | 语义 |
|---|---|---|
| `ptp` | PTP | 关节逐腿 OMPL+TOTG，每点停 |
| `ptp_blend` | PTP+B | pilz PTP Sequence + 自适应 blend（默认） |
| `lin_exact` | LIN | pilz LIN，逐点精确停 |
| `lin_blend` | LIN+B | pilz LIN Sequence + blend |
| `jnt_g64_trapezoid` | G64-T | 自研 G64 圆角 + 梯形 |
| `jnt_g64_quintic` | G64-Q | 自研 G64 圆角 + quintic |
| `jnt_spline_bell` | SPLINE | 插值样条 + 钟形（过点、可能过冲） |
| `jnt_spline_approx` | FIT | 最小二乘 B 样条（仅端点精确） |

- `ps4_mapper` 侧发布 `/a3/teleop/plan_mode`（`std_msgs/String`，规范名，1 Hz 重发），
  `vision_display_mux` 在右上角第二行显示 `waypoint · <HUD 简称>`。
- `/a3/teleop/mode` 仍只发模式名，不拼接规划方式（避免破坏 mode echo / 灯色语义）。
- 播放中按 `R2`：先 `/a3/arm/stop` 终止当前运动/循环，再切换显示，不自动重播。

## 循环语义

- 一轮 = 从当前位姿依次经过 wp1..wpN；`loop=true` 时轮间间隔 `waypoint_loop_gap_s`（0.2 s）。
- 路点循环不做连续轨迹的接缝回归段：每轮由正常规划腿从末点回到 wp1。
- 取消：`/a3/arm/stop` 置取消事件，循环在**下一段开始前**退出并回 READY；
  新回放请求会先等旧循环退出（超时 `waypoint_loop_cancel_timeout_s`，默认 30 s），
  超时则拒绝，避免两路运动并发。
- 静态力矩 / 占空比 / 温度门禁沿用各段原有实现，不因循环绕过。

## 验收标准

- [ ] `↓` 拖动、`L2` 打点、`↑` 保存后生成 `trajectories/waypoints/wp1.yaml` + 备份；`←/→` 同理生成 wp2；
- [ ] 录制中按 `→`（保存 wp2）被拒绝并提示按对应方向或 `□`；
- [ ] 进入 WAYPOINT 不再自动开录；
- [ ] `R2` 依次切换 8 种规划方式，`/a3/teleop/plan_mode` 与 HUD 同步变化；
- [ ] `△`/`○` 短按单次、长按循环，`□` 立即停止并回 READY；
- [ ] `mode: circ` 被拒且信息明确；
- [ ] 循环中 `/a3/arm/stop`、R3 软失能、切模式均能终止循环；
- [ ] 旧 `strategy: ptp|lin` 调用行为不变。

## 关联文件

- `src/a3_msgs/srv/PlaybackTrajectory.srv`（`string mode`）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`
- `src/a3_arm_controller/config/arm_controller.yaml`
- `src/a3_teleop_ps4/a3_teleop_ps4/{actions.py,mapping.py}`
- `src/a3_teleop_ps4/config/{action_registry.yaml,mappings/default.yaml}`
- `src/a3_bringup/a3_bringup/vision_display_mux.py`
- `docs/shared/{TOPIC_CONTRACT.md,TRAJECTORY_KINDS.md}`、`docs/edge/TRAJECTORY_MODE_SELECTION.md`
