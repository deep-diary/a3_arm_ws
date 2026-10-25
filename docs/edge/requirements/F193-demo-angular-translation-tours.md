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
string mode      # "translate_xyz" | "translate" | "xyz" | "random" | "spin"
float64 amp_deg  # 0=默认；random 时作为幅度上限
float64 peak_dps # 0=默认 30
uint32 cycles    # 0=循环到 stop（spin/random）；xyz 固定一轮
# translate/translate_xyz 专用（0=节点默认）
float64 duration_s     # 默认 30 s（仅 translate）
float64 vmax_mps       # 默认 0.04 m/s
float64 bounds_m       # 水平幅度/半径默认 0.06 m（z 用 0.75×）
float64 rot_bound_deg  # 姿态漂移上限默认 15°
```

| 模式 | 行为 | 几何 |
|---|---|---|
| `translate_xyz` | X→Y→Z 依次 `0→+A→−A→0`，只发 linear | 位置 ±6cm（z ±4.5cm），姿态不变 |
| `translate` | OU 随机速度连续平移，30 s 或越界/姿态漂移自停 | 位置留在 6cm 半径内，姿态不变 |
| `xyz` | 绕基座 X→Y→Z 各摆一轮，只发 angular | 末端位置保持，姿态 ±80/25-45/45° |
| `random` | 每轮随机轴 + 随机幅度/速度，只发 angular | 末端位置保持 |
| `spin`（备用） | 偏航往复连续 | 末端位置保持 |

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

## 固定三轴平移（`translate_xyz`）

末端点依次沿基座 X → Y → Z 做 `0→+A→−A→0`，**只发 linear（angular=0）**，姿态保持：

- 幅度：水平 ±6 cm、z ±4.5 cm（`bounds_m` 可调，z 取 0.75×）
- 速度：正弦包络，峰值 0.04 m/s；每段 `T=π|Δ|/(2·v_peak)`，一轮约 25 s
- 守卫：|Δx|/|Δy| > 1.2×amp、|Δz| > 0.95×amp 或姿态漂移 > 15° → 立即停
- 走完自动停（`_stop("完成")`）

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
| △ | `demo_translate_xyz`（**只平移**：X→Y→Z 各摆一轮） | `demo_random_translation`（**只平移**：连续随机方向，30 s/越界自停） |
| ○ | `demo_angular_xyz`（**只旋转**：X→Y→Z 各摆一轮） | `demo_angular_random`（**只旋转**：随机轴/幅度，再按停止） |
| □ | `random_pose_tour`（随机命名点巡游） | — |
| L2 | 保存当前点位 | — |
| Options | 停止并退出（`demo_angular_stop` + `/a3/arm/stop`） | — |

进入 DEMO 先 `goto home`；D-pad 不绑定。

## 验收标准

1. `mode:'translate_xyz'`：末端沿 X/Y/Z 依次平移到 ±6cm（z ±4.5cm），**姿态变化 ≈0°**，
   一轮后自动停
2. `mode:'translate'`：连续随机平移，位置留在 6cm 半径内，姿态 ≈0°，30 s 或越界自停
3. `mode:'xyz'`：只旋转（X/Y/Z 各一轮），末端位移 < 10 mm，姿态最大变化 > 40°
4. `mode:'random'`：只旋转，末端位移 < 10 mm，姿态有变化，再 `action:2`/`0` 停止
5. `DISABLED` 下启动 → `cannot start: arm_state=DISABLED（需 READY）`，
   `/servo_node/delta_twist_cmds` 无消息
6. 人脸跟踪开启时启动 → 被拒绝（"人脸跟踪开着"）

### 已完成的 mock 硬件验收（2026-10-25，domain 77）

环境：`hardware:=mock use_servo_target:=true`（与真机同一条 `a3_servo_target` 链），
脚本 `scripts/a3_test/f193_demo_tour_accept.py`：

| 模式 | 实测 |
|---|---|
| `translate_xyz` | 位置 ext x −60..+61 mm、y −60..+61 mm、z −46..+46 mm；姿态 0.0° → **PASS** |
| `translate`（12 s） | 位置留在 6 cm 内；姿态 0.0° → **PASS** |
| `xyz`（30°/s） | 姿态 80.5°；末端漂移 **1.9 mm** → **PASS** |
| `random`（12 s） | 姿态 76.2°；末端漂移 **0.0 mm** → **PASS** |

> 不能用 `a3_stack.sh start --sim` 的 moveit_servo 验收几何：该链在当前姿态大量报
> `Could not find IK solution (error -31)`；mock + `a3_servo_target` 才与真机一致。

## 关联文档

- [F185](F185-angular-cruise-demo.md)（CLI 来源与限位扫描）
- [F189](F189-ps4-mode-based-mapping.md)
- `src/a3_bringup/a3_bringup/angular_cruise_node.py`
- `src/a3_msgs/srv/AngularCruise.srv`
