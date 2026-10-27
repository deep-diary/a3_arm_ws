# F197 舞蹈双槽位（DANCE 录制 + 单次/循环播放）

## 背景

DANCE 模式（Share + ○）此前只是"回放 latest / 路点 PTP"的临时组合：
没有独立的舞蹈文件、无法同时保留两支舞、也无法循环播放。舞蹈是连续动作，
讲究节奏与时长，与口风琴的离散按键（F195）语义不同，需要独立槽位。

## 目录与槽位

```
trajectories/dance/
├── dance1.yaml              # kind: continuous（↓/↑ 直录）或 waypoint（L2 提升）
├── dance2.yaml
└── backup/dance1_YYYYMMDD_HHMMSS.yaml
```

- 连续录制额外写 `kind: continuous`、`name: dance1|dance2`、`recorded_at`。
- 路点提升是**整文件复制**（保留 `kind: waypoint`），不做结构转换。
- 槽位录制/提升**不覆盖**通用 `trajectories/latest.yaml` 与 `waypoints/latest.yaml`。

## 服务契约

| 服务 | 类型 | 说明 |
|---|---|---|
| `/a3/arm/dance/record1` | `std_srvs/Trigger` | 开始连续录制舞1（幂等：同槽重复调用返回 already recording） |
| `/a3/arm/dance/record2` | `std_srvs/Trigger` | 开始连续录制舞2 |
| `/a3/arm/dance/promote1` | `std_srvs/Trigger` | 把 `trajectories/latest.yaml` 或 `waypoints/latest.yaml` 中**较新**者提升为舞1 |
| `/a3/arm/dance/promote2` | `std_srvs/Trigger` | 同上，提升为舞2 |

回放：`/a3/arm/playback {type: "dance", name: "dance1|dance2", loop: bool}`。
`kind` 由文件头决定（`continuous` / `waypoint`）；`loop` 仅连续槽生效。

## PS4 映射（DANCE 模式，Share + ○）

| 操作 | 动作 |
|---|---|
| `↓` | 开始连续录舞1 |
| `↑` | 结束并保存舞1；播放中则停止 |
| `←` | 开始连续录舞2 |
| `→` | 结束并保存舞2；播放中则停止 |
| `△` 短按 / 长按 | 播放舞1（单次 / 循环） |
| `○` 短按 / 长按 | 播放舞2（单次 / 循环） |
| `□` 短按 | 停止回 READY |
| `L2` 短按 / 长按 | 把 latest 提升为舞1 / 舞2 |
| `Options` 短按 | 停止并退出（沿用 `/a3/arm/stop`） |

路点链路的用法：Share + ↓ 进 WAYPOINT 打点 → Options 保存 → 回 DANCE → `L2` 提升。

## 循环语义

- 复用内存中已 retime 的轨迹重新下发，轮间间隔 `dance_loop_gap_s`（0.2 s）。
- 首末关节差 ≤ `dance_loop_closed_tol_rad`（0.05 rad）视为闭环，无缝衔接；
  否则追加 `dance_loop_seam_s`（0.8 s）线性回归段。
- 循环首轮若需从当前位姿回首点，必须走 MoveIt；失败则降级为单次。
- 路点槽不支持循环（长按只播一次并提示）。
- 静态力矩门禁首轮全量校验；占空比/温度门禁每轮生效，累计到限自动停。

## 验收标准

- [ ] `↓` 拖动 `↑` 后生成 `trajectories/dance/dance1.yaml`（`kind: continuous`）+ `backup/` 备份；
      `←/→` 同理生成 `dance2.yaml`；
- [ ] 录制中重复按同向开始键返回 `already recording`，已录样本不丢；
- [ ] 录制舞1时按 `→` 被客户端拒绝并提示按对应方向或 `□`；
- [ ] `△` 短按单次播放、长按循环，`□` 停止回 READY；`○` 对舞2 同理；
- [ ] 循环中 R3 软失能/超温保护/`/a3/arm/stop` 均能终止循环，不残留循环上下文；
- [ ] WAYPOINT 录制后 `L2` 短/长可提升为舞1/舞2，`△/○` 能回放路点槽（单次）；
- [ ] 提升覆盖已有槽位前生成 `dance/backup/<slot>_<ts>.yaml`；
- [ ] 未录制时播放返回明确的"dance 槽未录制"提示。

## 关联文件

- `src/a3_msgs/srv/PlaybackTrajectory.srv`（`bool loop`）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`
- `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`
- `src/a3_teleop_ps4/config/{action_registry.yaml,mappings/default.yaml}`
- `src/a3_arm_controller/config/arm_controller.yaml`（`dance_loop_*` 参数）
- `docs/shared/TOPIC_CONTRACT.md`、`docs/shared/TRAJECTORY_KINDS.md`
