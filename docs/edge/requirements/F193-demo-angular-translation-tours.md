# F193 DEMO 巡游：角度巡航 + 平移随机巡游

## 背景

演示模式需要两类"自动表演"：

1. **角度巡航**：末端点位置不动，只做姿态摆动（展示三自由度）。
   F185 的 CLI 脚本已验证运动可行，但只有命令行、不能上手柄。
2. **平移随机巡游**：保持姿态，在当前位置附近随机取点走过去（展示可达空间）。

本次把两者接进 DEMO 模式，并新增常驻节点 `a3_bringup/angular_cruise`。

## 角度巡航实现

服务：`/a3/demo/angular_cruise`（`a3_msgs/srv/AngularCruise`）

```
uint8 action     # 0=stop 1=start 2=toggle
string mode      # "xyz" | "spin" | "random" | "translate"
float64 amp_deg  # 0=默认；random 时作为幅度上限
float64 peak_dps # 0=默认 30
uint32 cycles    # 0=循环到 stop（spin/random）；xyz 固定一轮
# translate 专用（0=节点默认）
float64 duration_s     # 默认 30 s
float64 vmax_mps       # 默认 0.04 m/s
float64 bounds_m       # 水平半径默认 0.06 m（z 用 0.8×）
float64 rot_bound_deg  # 姿态漂移上限默认 15°
```

运动形状沿用 F185：每段 `0→+A→−B→0`，速度正弦包络
`v(t)=v_peak·sin(πt/T)`，`T=π|Δ|/(2·v_peak)`；绕末端点旋转由
`a3_servo_target` 的 `rotation_about_tip` 语义保证（只发 angular，linear=0）。

| 模式 | 行为 | 默认幅度 / 速度 |
|---|---|---|
| `xyz` | X→Y→Z 依次各摆一轮，自动结束（约 34 s） | X ±80°、Y +25/−45°、Z ±45°，峰值 30°/s |
| `spin` | 偏航(Z) 往复连续，再按一次停止 | ±45°，峰值 30°/s |
| `random` | 每轮随机挑一根基座轴 + 随机幅度（0.5~1.0×安全幅度）、随机峰值（0.7~1.0×30°/s），循环到停止 | 上限同上 |

幅度取 F185 `--analyze` 的 URDF 限位扫描结果（留 3–10° 余量），**不生成任意斜轴**，
避免超出已验证包络。

安全门禁（不满足 → 拒绝启动；运行中失效 → 立即停发零速）：

- `/a3/arm_status` 必须是 `READY` / `SERVO`
- `/a3/control_mode` 不能是 `ZERO_TORQUE`
- `/a3/vision/mode` 不能是 `face_track`（否则两路 twist 抢同一话题）
- 停止时补发 0.3 s 零速，避免下游保持最后一拍

## 平移随机巡游（`translate`，连续不点停）

对齐 `scripts/a3_test/f181_servo_track.py random --continuous` 的 OU 速度过程：

- 速度 `v` 每 20 ms 更新：`v += -(v/τ)·dt + σ·√dt·N(0,1)`，`σ = 0.8·vmax`，`τ=0.8 s`
- 加速度限幅 `0.8 m/s²`，模长限幅 `vmax`（默认 0.04 m/s）
- **软边界回中**：水平半径 > 70% 边界（默认 4.2 cm）后叠加反向速度；z 同理（0.8×）
- **自动停止条件**（任一命中，发零速并停发）：
  1. 达到时长（默认 **30 s**）
  2. 水平半径 ≥ 6 cm 或 |Δz| ≥ 4.8 cm
  3. 姿态相对起始漂移 ≥ 15°
  4. `/a3/ee_pose` 超过 0.5 s 没更新 / 门禁失效（READY 掉出、开人脸跟踪等）
- 起点取按下瞬间的 `/a3/ee_pose` 位置与四元数，**锁姿态**只在位置空间巡游

`a3_bringup/angular_cruise` 每拍只发 linear（angular=0），姿态由
`a3_servo_target` 的目标姿态保持。

## DEMO 按键（Options 已负责停止，不再占 ○）

| 键 | 短按 | 长按 1.5 s |
|---|---|---|
| △ | `demo_angular_xyz`（固定三轴角度巡航） | `demo_random_translation`（连续随机平移，30 s/越界自停） |
| ○ | `demo_angular_spin`（旋转巡游，再按停止） | `demo_angular_random`（随机角度巡游，再按停止） |
| □ | `random_pose_tour`（随机命名点巡游） | — |
| L2 | 保存当前点位 | — |
| Options | 停止并退出（`demo_angular_stop` + `/a3/arm/stop`） | — |

进入 DEMO 先 `goto home`；D-pad 不绑定。

## 验收标准

1. READY 下 `ros2 service call /a3/demo/angular_cruise a3_msgs/srv/AngularCruise
   "{action: 1, mode: 'xyz'}"` → 臂原地做 X/Y/Z 三轴摆动，位置漂移 < 10 mm
2. 同一调用在 `DISABLED` 下返回 `cannot start: arm_state=DISABLED（需 READY）`，
   且 `/servo_node/delta_twist_cmds` 无消息
3. `mode: 'spin'` → 偏航往复；`action: 2` 再调一次 → 停
4. `mode: 'random'` → 每轮轴/幅度不同，仍在 X/Y/Z 包络内
5. 人脸跟踪开启时启动 → 被拒绝（"人脸跟踪开着"）
6. DEMO 里 `△` 长按 → 连续随机平移（不是逐点）：30 s 到点自动停；中途把臂推出 6 cm
   或姿态漂移 15° 也会立即停，日志给出原因（`触平移边界` / `姿态漂移` / `达到时长`）

## 关联文档

- [F185](F185-angular-cruise-demo.md)（CLI 来源与限位扫描）
- [F189](F189-ps4-mode-based-mapping.md)
- `src/a3_bringup/a3_bringup/angular_cruise_node.py`
- `src/a3_msgs/srv/AngularCruise.srv`
