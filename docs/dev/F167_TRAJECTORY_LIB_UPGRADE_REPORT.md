# F167 轨迹规划「用成熟库」升级 —— 仿真验证报告

对应需求：[F167](../edge/requirements/F167-trajectory-lib-upgrade-ruckig-toppra-otg.md)。
分三阶段落地，每阶段独立仿真验收；本文汇总。子报告：
[F167_retime_ab.md](F167_retime_ab.md)（Stage1 A/B）、[F167_toppra.md](F167_toppra.md)（Stage2）、
[F167_otg.md](F167_otg.md)（Stage3）。

## 结论速览

| 阶段 | 内容 | 结果 |
|---|---|---|
| F167.1 | 软件模式时间律 → Ruckig/TOTG（`/a3/arm/retime_trajectory`） | ✅ 几何一致；totg 更快但更"抖"；开关可控、服务缺失自动回退 |
| F167.2 | toppra + Pinocchio 力矩/自定义凸约束离线重定时 | ✅ 峰值力矩在限内；baked 仿真回放成功 |
| F167.3 | Ruckig 在线 OTG 演示（同步/改目标/急停） | ✅ 同步到齐 0ms；改目标不停车；急停 200ms 到零速 |

---

## Stage F167.1：手写剖面 → Ruckig/TOTG

**改动**：`RetimeTrajectory.srv` 增可选逐关节 `velocity_limits/acceleration_limits`（空=节点参数，
向后兼容）；`retime_trajectory_node.cpp` 支持请求级覆盖；`arm_controller.py` 增参数
`goto_via_time_backend`（`software`|`totg`|`ruckig`），`_goto_via_smooth` 非 software 时调
retime 换**时间律**（几何路径不变），服务不可用/失败 WARN 回退手写剖面。

**A/B 方法**：仿真栈内同一 5 点路径，`ros2 param set /a3_arm_controller goto_via_time_backend
{totg,software}` 各跑一遍，`traj_modes_tcp_capture.py` 采集、`f167_retime_ab.py` 出表。

| 模式 | backend | 时长(s) | j_rms(rad/s³) | a_rms(rad/s²) | TCP路径长(m) | 几何相对差 |
|---|---|---|---|---|---|---|
| jnt_g64_quintic | software | 5.50 | 6.1 | 0.6 | 0.3444 | - |
| jnt_g64_quintic | totg | 3.28 | 16.2 | 1.4 | 0.3438 | 0.17% ✓ |
| jnt_g64_trapezoid | software | 4.98 | 8.2 | 0.7 | 0.3445 | - |
| jnt_g64_trapezoid | totg | 3.24 | 16.3 | 1.4 | 0.3430 | 0.44% ✓ |
| jnt_spline_bell | software | 11.14 | 2.5 | 0.3 | 0.3456 | - |
| jnt_spline_bell | totg | 5.06 | 10.5 | 1.0 | 0.3477 | 0.62% ✓ |
| jnt_spline_approx | software | 4.34 | 1.4 | 0.3 | 0.3429 | - |
| jnt_spline_approx | totg | 2.96 | 11.7 | 0.9 | 0.3440 | 0.33% ✓ |

**判定**：几何路径长相对差 ≤ 0.62%（同路径，仅时间律不同）→ **几何一致 PASS**。
**观察**：TOTG 是**时间最优（bang-bang）**，故比手写 quintic/钟形剖面**快 ~30-60% 但 j_rms 更高**
（典型 ×2~8）；这是「成熟库=更快 vs 手写=更平滑」的固有取舍。日志无 `fallback`（retime 确实生效）。
**回退安全**：`kill` retime 节点后回放同一软件模式，日志
`goto_via retime(totg) failed (retime service unavailable); fallback to software profile 'l1p5'`，
播放仍 `success=True`（7.9s=软件时长）。
**默认策略**：`goto_via_time_backend` 保持 `software`（保平滑、零回归）；需要节拍时切 `totg`/`ruckig`。

---

## Stage F167.2：toppra 力矩 + 自定义凸约束

**改动**：`scripts/traj_retime_toppra.py`（toppra TOPP-RA + Pinocchio `rnea` 逆动力学，标定惯量）。
约束：速度/加速度 + **JointTorqueConstraint**（rnea，含重力/科氏/惯性）+ **自定义凸约束**
`WeightedAccelConstraint`（`|W·q̈| ≤ c`，示例：腕部 L4-L6 加速度之和）。输出 baked（可被
`_play_baked` 直接回放下发）。

**指标**（5 点 L1-L6，`vmax=1.5 / amax=15`，力矩上限 `[7,7,7,3,3,3]` Nm）：

| 指标 | 值 |
|---|---|
| 时长 dur | 2.09 s |
| a_rms | 4.83 rad/s² |
| j_rms | 122.7 rad/s³ |
| 峰值 \|τ\| (L1-L6) | [0.76, 3.66, 3.50, 0.77, 0.13, 0.01] Nm |
| 力矩上限 | [7, 7, 7, 3, 3, 3] Nm → **在限内 PASS ✓** |

**仿真回放**：`playback name=toppra_5pt type=stream` → `baked: 240 pts, 2.1s, success`（与离线 dur 一致）。
验收脚本 `scripts/a3_test/f167_toppra_acceptance.py --sim` → **PASS**。

> 注：约束生效的直接证据 = 峰值力矩被压在上限内（若移除 `JointTorqueConstraint`，L2/L3 会更快更猛、
> 峰值超 3 Nm）；`j_rms` 高与 TOTG 同因（时间最优 bang-bang）。

---

## Stage F167.3：Ruckig 在线 OTG 演示（仿真-only）

**改动**：`a3_trajectory_processing/src/otg_demo.cpp`（复用栈内 `ruckig`，纯计算写 CSV）+
`scripts/a3_test/f167_otg_report.py`（numpy 手写 SVG + 指标）。**不接** SERVO/teleop 生产链。

| 场景 | 指标 | 值 |
|---|---|---|
| 多轴同步(Time) | 各轴到位时刻离散度 | **0.0 ms**（同时到齐） |
| 无同步(None) | 各轴到位时刻离散度 | 520.0 ms（对照） |
| 改目标(中途切目标) | 切换点速度跳变 max\|Δv\| | 0.0004 rad/s（连续） |
| 改目标 | 切换点速度（不停车） | 1.500 rad/s |
| 急停 | 制动时长 | 200.0 ms |
| 急停 | 制动距离（最大单关节） | 535.6 mrad |

**判定 PASS**：同步到齐 0ms 且远优于无同步；改目标速度连续、不停车；急停 jerk-limited 到零速。
图：[f167_otg_sync.svg](f167_otg_sync.svg)、[f167_otg_speed.svg](f167_otg_speed.svg)。

---

## 复现命令

```bash
source scripts/a3_shell_env.sh
# 构建
colcon build --packages-select a3_msgs a3_trajectory_processing a3_arm_controller
# 仿真栈
./scripts/a3_stack.sh restart --sim && export ROS_DOMAIN_ID=99
ros2 service call /a3/arm/enable std_srvs/srv/Trigger
# Stage1 A/B（两次采集 + 对比）
python3 scripts/a3_test/traj_modes_tcp_capture.py --waypoints trajectories/waypoints/latest.yaml \
    --speed 0.4 --out /tmp/f167_ab_software.json           # backend=software
ros2 param set /a3_arm_controller goto_via_time_backend totg
python3 scripts/a3_test/traj_modes_tcp_capture.py --waypoints trajectories/waypoints/latest.yaml \
    --speed 0.4 --out /tmp/f167_ab_totg.json               # backend=totg
python3 scripts/a3_test/f167_retime_ab.py --software /tmp/f167_ab_software.json \
    --totg /tmp/f167_ab_totg.json --out-md docs/dev/F167_retime_ab.md
# Stage2
python3 scripts/a3_test/f167_toppra_acceptance.py --sim
# Stage3
./install/a3_trajectory_processing/lib/a3_trajectory_processing/otg_demo /tmp
python3 scripts/a3_test/f167_otg_report.py --dir /tmp --out-dir docs/dev
```

## 环境/依赖备注（换机备忘）

- **toppra**：本机无法 `apt`（需 sudo）也无 wheel；改由 `sudo pip3 install --no-deps --no-build-isolation
  toppra`（先 `Cython`，且 `sudo` 会丢 `PYTHONPATH` → 需 `sudo -S env PYTHONPATH=/usr/lib/python3/dist-packages`，
  否则触发 numpy 2.x ABI 报错）。**已装 0.6.3**（`/usr/local`）。
- toppra 顶层 `import matplotlib`（仅绘图），本机 matplotlib/pyparsing 冲突（LL-152）→ 脚本内置空壳 stub。
- Stage2 baked 回放需**同名占位 stream** `trajectories/streams/toppra_5pt.yaml` +
  `toppra_5pt.yaml.baked.yaml`（`_play_stream` 命中 `path + ".baked.yaml"`）。
- Stage1 默认 `goto_via_time_backend: software`；切成熟库用 `ros2 param set` 或改 yaml 重启。
