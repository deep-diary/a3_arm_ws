# 角速度通路修复 + R1 三轴旋转 + 角速度标定/协同（A→R1→B→C）

> 目标：让「按住 R1 + 摇杆」真正实现绕 X/Y/Z 三轴旋转（末端位置不变），并打通人脸跟踪在
> 「高度受限」时用俯仰继续跟人的能力。分四步，按依赖排序执行。

## 1. Summary

| 步骤 | 内容 | 是否依赖真机 |
|---|---|---|
| **A** | 修 `a3_servo_target` 的角速度语义：改成**绕末端点旋转**（位置不变），并加 IK 失败时目标回退，防止目标跑飞 | 代码 + 新脚本可在真机验证；无真机则只做编译/静态检查 |
| **R1** | PS4：R1 按住 = 偏航(右摇杆左右) + 俯仰(右摇杆上下) + 滚转(左摇杆左右)；扩展 mapping 支持一轴多绑定 | 仿真可验证（仿真走 moveit_servo，本身支持绕末端旋转） |
| **B** | 用已写好的角速度探针标定角雅可比 `Ja`（`--ang-probe 0.1`） | **必须真机 + 人脸/平板在视野内**，需用户配合 |
| **C** | 控制器协同：竖向误差优先用平移，平移饱和/顶到高度上限时改由俯仰承担 | 依赖 B；先实现，真机验收 |

**执行顺序：A → R1 → B → C。**
理由：真机 R1 旋转走 `a3_servo_target`，当前它的角速度语义是「绕基座原点转」，会让末端位置一起甩出去；
不先修 A，R1 接上也只是“看起来坏掉”。B 依赖 A（探针要能真的转起来），C 依赖 B（需要实测 `Ja`）。

**硬件门禁**：任一步骤出现 `feedback stale` 连续 ≥3 次 / `FAULT` / `CAN write failed`，立即停止真机动作，
只保留软件与仿真改动，真机验收留给用户在场时进行。

---

## 2. Current State Analysis（基于实际代码）

### 2.1 `a3_servo_target` 的角速度路径

`src/a3_trajectory_processing/src/servo_target_node.cpp`

| 位置 | 现状 |
|---|---|
| `355-383` | 6 维 twist 限幅/加速度限幅/一阶低通，`twist_max_angular=1.2`、`twist_max_ang_accel=3.0` |
| `385-409` | **问题点**：`d = Isometry(R(w·dt), v·dt)`，`cand = d * target_` → 纯角速度时 `p_new = R·p`，**位置绕基座原点画弧**，末端点被甩动 |
| `398-401` | 位置按 `workspace_min/max` 夹紧 |
| `402-407` | `hold_z_`（launch 里为 `True`）：`|v_z|<1e-4` 时把目标 z 钉回 `z_lock_`。绕 Y 转时弧线本来会改变 z，被钉住后目标位姿自相矛盾 |
| `417-445` | IK 失败/换支被拒时**只保留上一拍解**，`target_` 继续积分 → 残差可以无限涨（实测 700 mm、`ik_fail` 100%） |
| 参数 | `workspace_min=[-0.55,-0.45,0.15]`、`max_step_trans=0.004`、`max_ik_jump=0.05` 均在 `a3_bringup.launch.py:483-498` |

### 2.2 PS4 映射（`a3_teleop_ps4`）

- 映射驱动：`ps4_mapper.py:142-164` 遍历 `axes`，按 `gates` 判定是否采样；`actions.py:701-717` 写入 `_lin/_ang`；
  `actions.py:267-304` 按 `max_linear(0.35)/max_angular(1.0) × speed_normal(0.35)` 组装并发布 `TwistStamped`（frame=`base_link`）。
- `config/mappings/default.yaml` 现状：
  - `left_x → servo_lin_y (gate l1)`、`left_y → servo_lin_z (gate l1)`、`right_y → servo_lin_x (gate l1)`
  - `right_x → servo_ang_z (gate r1)` ← **R1 目前只有偏航**
  - 一个轴名只能绑定一个 `fn`（`iter_axis_bindings` 单条），所以「同一根摇杆在 L1/R1 下做不同事」目前做不到。
- `servo_ang_x()` / `servo_ang_y()` 已存在但无绑定；`action_registry.yaml:136-146` 已登记。
- 仿真栈：`edge_teleop_full_sim.launch.py` 用 **moveit_servo**（无 `a3_servo_target`），天然支持 6 维 twist；
  `ps4_sim_test.py:445-463` 已有「R1+right_x 必须产生 yaw」的验收场景，可扩展。

### 2.3 人脸跟踪控制器（已实现但未启用角速度）

`src/a3_perception/a3_perception/face_track_controller.py`
- `519-535`：`enable_angular` 时 `w_cal = -ka·Ja⁻¹·e`（有 `angular_jacobian` 时）否则用解析近似；再经 `R_now·R_calᵀ` 转到 base。
- `wrap`：`w_max=0.6 rad/s`、`w_ramp=2.0`、开转动时平移乘 `lin_frac=0.5`。
- 现状 `enable_angular=false`，`Ja` 从未标定（`~/.a3/camera/face_jacobian.yaml` 无 `angular_jacobian`）。

---

## 3. Proposed Changes

### Step A — `a3_servo_target`：绕末端点旋转 + 目标不跑飞

文件：`src/a3_trajectory_processing/src/servo_target_node.cpp`

1. **改旋转语义（核心）**：把 `d * target_` 拆开——
   ```cpp
   Eigen::Isometry3d cand = target_;
   const Eigen::Vector3d w = v.tail<3>();
   const double th = w.norm();
   if (th > 1e-9) {
     const Eigen::Matrix3d Rd = Eigen::AngleAxisd(th, w / th).toRotationMatrix();
     cand.linear() = Rd * target_.linear();          // 只转姿态，末端点不动
   }
   Eigen::Vector3d dp = v.head<3>() * dt_step;       // 线速度仍按 base 平移
   if (dp.norm() > max_step_trans_) dp *= max_step_trans_ / dp.norm();
   cand.translation() = target_.translation() + dp;
   // 其后保留 workspace 夹紧 + hold_z_ 逻辑（此时 z 不再被旋转改变，二者不再打架）
   ```
2. **新增参数 `rotation_about_tip`（默认 `true`）**：`false` 时保持旧行为，便于回退对照。
3. **IK 失败回退**：缓存最近一次成功的目标 `target_ok_`（初始化与 `reanchor` 时同步）；
   `ik_ok == false` 或 `dmax >= max_ik_jump_` 时执行 `target_ = target_ok_`，
   避免「目标继续积分、臂不动」造成 700 mm 残差。日志里把 `ik_fail_streak_` 打出来（已有计数）。
4. `a3_bringup.launch.py` 的 `a3_servo_target` 参数块显式加 `{"rotation_about_tip": True}`，便于日后切换。

### Step R1 — PS4：R1 三轴旋转

1. **mapping 支持一轴多绑定**：`src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py`
   - `iter_axis_bindings()` 允许 `axes.<name>` 是 **dict 或 list[dict]**；list 时逐条按各自 `gates` 判定，命中即执行。
   - 保持旧格式（单 dict）100% 兼容。
2. **`config/mappings/default.yaml` 绑定**（在 R1 下复用空闲轴，不加新按键）：
   ```yaml
   left_x:
     - {fn: servo_lin_y, kind: analog_n11, gates: [l1]}   # 原有：平移 Y
     - {fn: servo_ang_x, kind: analog_n11, gates: [r1]}   # 新增：绕 X 滚转
   right_y:
     - {fn: servo_lin_x, kind: analog_n11, gates: [l1]}   # 原有：平移 X
     - {fn: servo_ang_y, kind: analog_n11, gates: [r1]}   # 新增：绕 Y 俯仰
   right_x:
     - {fn: servo_ang_z, kind: analog_n11, gates: [r1]}   # 原有：绕 Z 偏航
   ```
   即：**R1 + 右摇杆左右=偏航、右摇杆上下=俯仰、左摇杆左右=滚转**；L1 行为完全不变。
3. 更新 `default.yaml` 顶部速查注释、`src/a3_teleop_ps4/README.md` 与 `docs/edge/PS4_OPERATOR_GUIDE.md` 的操作表。

### Step B — 角速度探针标定（真机，需人脸/平板）

- 脚本已具备：`scripts/a3_test/f183_calib_jacobian.py --ang-probe 0.1`，会写
  `angular_jacobian`、`angular_probe_rad`、`angular_columns` 到 `~/.a3/camera/face_jacobian.yaml`。
- A 修好后预期 `Ja` 收敛（`det` 非零、`cond < 25`）；标定后
  `ros2 service call /a3/face_track/reload_jacobian std_srvs/srv/Trigger` 热加载。
- 若 `Ja` 仍退化/探针仍不转：回到 A 继续查（先看 `ik_fail_streak` 与 `ik_jump_rej`）。

### Step C — 协同：高度受限时用俯仰继续跟

文件：`face_track_controller.py`
- 新增参数：`pitch_assist`(bool, 默认 false)、`pitch_gain`（默认 0.5，1/s）、`pitch_max`（默认 0.35 rad/s）。
- 触发条件（同时满足）：
  1. `enable_angular == false`（避免与整体转动环叠加）；
  2. 竖向误差 `|e_v| > deadband_exit`；
  3. **竖向平移被钉住**：本拍虚拟墙返回了 `z+`/`z-`（即 `clamped` 含 z），或上一拍 `_stall_until` 被触发。
- 输出：`w_cal_y = -pitch_gain · e_v`（符号由 B 的 `Ja` 或一次实测确定），限幅 `pitch_max`，经 `R_now·R_calᵀ` 转 base；
  只在上述条件成立时叠加，其余情况保持 `w=0`（不影响已验证的纯平移档）。
- 验收：抬高到高度上限后，`err_y` 应继续收敛、`ee_z` 不再上升也不卡死；`x/y` 侧不受影响。

### 新增测试脚本（Step A 的验收工具）

`scripts/a3_test/f183_servo_rotation_check.py`（新文件，只读状态 + 发 twist）
1. 前置：`arm_status=READY`、0.5 s 内无其它 twist、`|u|,|v|` 不参与（纯运动学测试）。
2. **定向测试**：对 X/Y/Z 各轴、正负号：发 `w = ±0.15 rad/s` 约 0.7 s（≈0.1 rad），停 0.5 s；
   记录 `ΔR`（相对角，轴+角度）与 `Δp`；再反向转回。
   判定：`|Δp| < 5 mm`、实测角与指令角偏差 < 25%、回位后 `|Δp| < 5 mm` 且残角 < 0.01 rad。
3. **随机测试**：每 1.5 s 随机选轴与符号，累计指令角；**总时长 60 s 或累计 |角| > 60° 即停**；
   结束后 `goto_named_pose home` 回位并复核姿态误差。
4. 输出逐项表格 + 汇总；有任一失败返回非零退出码。

---

## 4. Assumptions & Decisions

1. **旋转参考系 = 基座 XYZ**（与 `TwistStamped.frame_id=base_link`、以及用户要求的 “绕 x/y/z 正负旋转” 一致）；
   语义为「绕**末端点**的基座轴旋转」——姿态变、位置不变。
2. **滚转绑定到左摇杆左右**（R1 下左摇杆纵向仍由 L1 门控，不会误触发），不新增按键；
   右摇杆保持「左右=偏航、上下=俯仰」。
3. `hold_z_` 保持 `True`：改成绕末端点后 z 不再被旋转牵动，z 锁只在线速度无 z 分量时生效，不再与旋转冲突。
4. IK 失败时**冻结目标**（回退到上次成功位姿），而不是继续积分；这同时修复了之前
   “目标跑飞 → 残差 700 mm” 的根因。
5. Step B/C 均为真机项：B 必须有人脸在画面中，C 依赖 B。用户不在场时：A/R1 完成软件+仿真验证，
   B/C 代码就绪并写好验收步骤，等用户回来执行。
6. 不改 `moveit_servo` 路径（仿真与旧链路），只改真机的 `a3_servo_target`。

---

## 5. Verification

### A（真机，需 READY）
```bash
colcon build --packages-select a3_trajectory_processing && \
./scripts/a3_stack.sh restart --face-track --rviz
python3 scripts/a3_test/f183_servo_rotation_check.py --ang 0.15 --hold-axis-s 0.7
```
通过标准：三轴正负共 6 次定向旋转全部满足「位置漂移 < 5 mm、角度误差 < 25%、回位误差 < 5 mm」；
随机相位 60 s 无 IK 失败堆积（日志 `ik_fail` 增量 < 1%）。

### R1（仿真优先）
```bash
./scripts/a3_stack.sh start --sim --no-joy --rviz
# 用 ps4_sim_test.py 现有场景注入 /joy：r1+right_x→ang.z、r1+right_y→ang.x?（按实际映射）
# 校验 /servo_node/delta_twist_cmds 三个 angular 分量分别随对应轴变化
```
真机 R1 验收留给用户：按住 R1 → 右摇杆左右偏航、上下俯仰、左摇杆左右滚转，末端位置目视不动。

### B/C（真机 + 人脸，用户在场）
```bash
python3 scripts/a3_test/f183_calib_jacobian.py --probe 0.03 --ang-probe 0.10
ros2 service call /a3/face_track/reload_jacobian std_srvs/srv/Trigger
# 然后 enable 跟踪，抬高到限位：err_y 应继续收敛
```

### 硬件故障分支
出现 `feedback stale`（L1/L3/L5/L6 轮换）/`FAULT`/`CAN write failed`：停止所有真机动作，
记录日志，仅保留代码与仿真验证，等用户排查（参考 LL-131、LL-180）。

---

## 6. 风险与回退

| 风险 | 回退 |
|---|---|
| 改旋转语义影响既有 servo 行为 | `rotation_about_tip:=false` 恢复旧行为 |
| 一轴多绑定改动影响其它 mapping | 保留单 dict 兼容；只改 `default.yaml`，`simple.yaml` 不动 |
| C 的俯仰与平移互相打架 | `pitch_assist` 默认 false；开启后仍受 `w_max/w_ramp` 与 stall 保护 |
| 真机 CAN 再次丢帧 | 立即停真机测试，转仿真 |
