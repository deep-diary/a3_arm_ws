# F160 真机自动 A/B 复验计划

## 1. 摘要

把上一轮「手动遥控器跑 idle→home→弹琴」的 A/B 对比，改成**脚本自动执行**：自动设
`feedforward_mode`（gravity / full）、自动 enable → goto ready → playback 弹琴轨迹 →
disable，全程用 `mode==TRAJ_RUNNING` 自动界定运动段采集，消除手动启停时间不一致引入
的偏差。跑完 gravity 与 full 各一遍，自动出对比表，确认「full 显著改善」结论可复现，从而
验收 F160。

## 2. 现状分析（已探索）

| 项 | 结论 |
|---|---|
| 机械臂状态 | 当前 idle 失能（用户已 R3 / 断电后未使能） |
| 弹琴轨迹 | `~/.a3/trajectories/latest.yaml`（7 关节，含 L7，F137+F68 已处理），首点 `[0.10, 0.64, -0.63, 0.82, 0.11, -1.61, 1.75]` |
| 命名点 | `ready`/`idle`/`home`/`zero`（[named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml)）；`ready` 为 F109 稳定位 |
| 服务接口 | `/a3/arm/enable`、`/a3/arm/disable`（`std_srvs/Trigger`）；`/a3/arm/goto_named_pose`（`a3_msgs/GotoNamedPose`，字段 `pose_name`）；`/a3/arm/playback`（`a3_msgs/PlaybackTrajectory`，`name=""` 即回放 latest） |
| 状态机 | `state` ∈ IDLE/INIT/READY/TRAJ/SERVO/TEACH/AI/SAFE_PARK/DISABLED/COOLING/FAULT；`mode` ∈ IDLE/TRAJ_RUNNING/GRAVITY_COMP/ZERO_TORQUE/SERVO |
| 模式切换 | `/a3_hardware_health/set_parameters` 设 `feedforward_mode`（gravity/full） |
| 采集复用 | 现有 [f160_real_ab_acceptance.py](file:///home/cat/a3_arm_ws/scripts/a3_test/f160_real_ab_acceptance.py) 已有 `Collector`（订阅 `/arm_controller/state` + `/a3/arm_status`，仅 `mode==TRAJ_RUNNING` 时采样）、`compute_metrics`、`compare_cmd` |
| goto 实现 | `goto_use_moveit` 默认 `true`（走 move_group，需 move_group 就绪 ~30s）；`playback_return_use_moveit` 默认 `true` |

## 3. 变更方案

**只改一个文件**：`scripts/a3_test/f160_real_ab_acceptance.py`，新增 `auto` 子命令 +
`AutoRunner` 节点类，复用已有 `Collector`/`compute_metrics`/`compare` 逻辑。不改产品代码。

### 3.1 `AutoRunner` 节点（组合采集 + 服务 + 状态监控）

在现有 `Collector` 基础上扩展（或继承），新增：
- 服务 client：`enable`/`disable`（`Trigger`）、`goto`（`GotoNamedPose`）、`playback`
  （`PlaybackTrajectory`）。
- 状态缓存：订阅 `/a3/arm_status`，维护 `self._state` / `self._mode`。
- 等待辅助：`wait_state(target, timeout)`（轮询 `state`）、`wait_motion_done(timeout)`
  （轮询 `mode != TRAJ_RUNNING`）。
- 采集开关：`start_capture()`（清空样本）/ `stop_capture()`（返回 metrics）。

### 3.2 自动流程（每个 mode 各一遍）

```
run_mode(mode):
  1. set feedforward_mode = mode          # /a3_hardware_health/set_parameters
  2. call /a3/arm/enable; wait_state("READY", 20s)
  3. call /a3/arm/goto_named_pose(pose_name="home"); wait_motion_done(60s)
  4. start_capture()                       # 只采集 playback 弹琴段（goto 不混入）
  5. call /a3/arm/playback(name="", type=""); wait_motion_done(120s)
  6. stop_capture() -> metrics -> 写 JSON（mode 标注）
  7. call /a3/arm/disable; wait_state ∈ {DISABLED, IDLE}, 30s
```

- 采集窗口 = **纯 playback 弹琴段**（`start_capture` 在 goto 完成后、playback 前；更聚焦
  弹琴，且两遍起始位一致，比上一轮手动「goto+弹琴」混合更干净）。
- 两遍都从「idle 失能 → enable → goto ready → playback → disable → idle 失能」完全对称，
  消除启停时间不一致。

### 3.3 `auto` 子命令

- 参数：`--out-dir`（默认 `/tmp`，输出 `/tmp/f160_auto_gravity.json` 与
  `/tmp/f160_auto_full.json`）、`--repeat`（默认 1）。
- 顺序：先 `gravity` 一遍 → `full` 一遍 → 自动调用已有 `compare` 逻辑输出对比表。
- 安全护栏：任一等待超时 / `state` 进入 `FAULT` / 服务返回 `success=false` → 立即
  `disable` 并报错退出；Ctrl+C 时先 `disable` 再退出。

## 4. 假设与决策

- **采集窗口**：只采 playback（弹琴）段，goto ready 仅作两遍一致的定位。
- **定位点**：`home`（弹琴轨迹首点 L2/L3≈0.64/-0.63 更接近 home(0.785/-0.785)，ramp 更小、
  更贴合第一次手动 idle→home→弹琴），playback 内部 `playback_return_use_moveit=true`
  会自动从 home ramp 到轨迹首点，该 ramp 属 playback 一部分、两遍一致，纳入采集。
- **每模式重复 1 遍**（`--repeat` 可加）；数据若波动大再加平均。
- **判读沿用**：以 `compare` 输出为主——期望峰值误差 / RMS / 稳态滞后 / PD 负担均显著
  下降、纹波不劣化（+25% 或 +0.02 Nm 预算），与上一轮一致。
- **不引入新需求**：这是 F160 的复验，不新增功能 ID。

## 5. 验证步骤

1. `python3 -m py_compile scripts/a3_test/f160_real_ab_acceptance.py`（语法）。
2. 确认栈已启动且 move_group 就绪（起栈 ≥30s，`/a3/arm_status` 可读、`/a3/arm/goto_named_pose` 服务在）。
3. 执行：
   ```bash
   python3 scripts/a3_test/f160_real_ab_acceptance.py auto
   ```
4. 观察输出：自动跑完 gravity + full，打印对比表。判定「类似第一次」= full 相对 gravity
   的峰值误差/RMS/稳态滞后/PD 负担显著下降、纹波不劣化。
5. 若通过 → 验收 F160（状态已是 `implemented`，无需再改）；若 FAIL → 根据哪项异常定位
   （如纹波/某关节），再决定是否加重复或回看 AFF 数值质量。
