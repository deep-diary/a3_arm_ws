# servo 绝对目标：launch 开关 + 速度突变平滑 + 验收脚本

## 1. 概述

三件事：

1. 把已验证的 `a3_servo_target`（F180）接进真机统一 launch，新增 `use_servo_target` 开关（默认 `false`，零回归）；
   `use_servo_target:=true` 时不启 `moveit_servo/servo_node_main`，只启本节点，PS4/mapper/`servo_mode_bridge` 链路不变。
2. 给节点加**速度突变平滑**：6D twist 先做加速度限幅，再做一阶低通（τ≈60ms），
   解决正→0、正→负时目标速度瞬时跳变导致的过冲/抖动。
3. 新增一个验收脚本 `scripts/a3_test/f181_servo_track.py`，两个子命令：
   - `sweep`：回 home → 0.03 m/s 三轴各 +10 → −20 → +10 cm → 自动出指标表；
   - `random`：回 home → 随机轴/随机正负/随机速度的方形跟踪，**60s 或触 home±10cm 边界先到先停**（用户已确认），最后回 home 出报告。

## 2. 现状分析

### 2.1 节点已可用，但只能手工起

- 实现在 [servo_target_node.cpp](file:///home/cat/a3_arm_ws/src/a3_trajectory_processing/src/servo_target_node.cpp)：
  绝对末端目标 + CLIK + 有状态速度积分（`pos_gain=4`、`acc_gain=20`、`max_acc=5`、`max_jerk=50`）→
  `/arm_controller/joint_trajectory` 单点流。真机三轴 sweep 已达标（命令 CV 0.008–0.062、回 home 2.2mm）。
- 真机栈统一入口是 [a3_bringup.launch.py](file:///home/cat/a3_arm_ws/src/a3_bringup/launch/a3_bringup.launch.py)：
  `servo_node = Node(moveit_servo/servo_node_main, ...)`（L431–455）**无条件启动**，
  没有 `use_servo_target` 之类的开关；`servo_anchor`（F118）由 `use_servo_anchor` 条件控制（L491–505）。
- [a3_stack.sh](file:///home/cat/a3_arm_ws/scripts/a3_stack.sh) 起栈命令固定为
  `ros2 launch a3_bringup a3_bringup.launch.py hardware:=can can_interface:=can1 use_rviz:=... include_gripper:=...`（L584），
  孤儿兜底名单 `NODE_PATTERN`（L39）里没有 `servo_target_node`。
- 上游输入不变：PS4 mapper 以 **RELIABLE depth 10** 发布 `/servo_node/delta_twist_cmds`
  （[actions.py](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/a3_teleop_ps4/actions.py) L109–116，`max_linear_mps=0.35`、`max_angular_rps=1.0`），
  本节点用 RELIABLE 订阅，匹配。mapper 会尝试调 `/servo_node/start_servo`，服务不存在时只 `return False`，
  仍会继续发 twist（L290–295），所以不启 moveit_servo 不影响摇杆。
- 现有可用于验收的公共设施：`/a3/ee_pose`（F175，20Hz PoseStamped）、
  `/a3/arm/goto_named_pose`（`a3_msgs/srv/GotoNamedPose{pose_name}`）、`/a3/arm_status`。

### 2.2 速度突变现在怎么处理（回答用户问题）

当前**没有任何输入滤波**：

- twist 变成 0 → `target_` 立即冻结，绝对目标停在“松手瞬间再往前 1 个 tick”的位置；
  `q_cmd` 通过 `v_des = pos_gain·(q_goal−q_cmd)` 指数收敛（τ=1/4=0.25s），再经 `max_acc/max_jerk` 限幅 →
  是平滑减速，但因为巡航时目标领先实测约 4mm，臂会继续走约 4mm 才停（“过冲”感来源）。
- twist 正→负 → **目标速度瞬时反向**，只有下游关节级 `max_acc/max_jerk` 兜底；
  目标是阶跃，位置误差瞬间变大，前馈也阶跃，这是随机跟踪里抖动/过冲的主因。

不需要卡尔曼滤波：这里输入是**指令速度**（不是带噪测量），Kalman 只会额外增加滞后。
`control_toolbox` / `filters` 虽已安装，但一阶 IIR + 限幅只有几行，引入库不划算。
采用用户已确认的方案：**加速度限幅 + 一阶低通**。

## 3. 变更方案

### 3.1 先写需求（仓库规则 requirements-first）

新增 `docs/edge/requirements/F181-servo-target-launch-twist-smoothing.md`：

- ID `F181`，说明：`use_servo_target` launch 开关（默认 false）+ twist 加速度限幅/一阶低通 + 两套真机验收脚本。
- 验收标准：
  1. `a3_stack.sh restart`（不带新参数）→ 启 `servo_node_main`，**不**启 `a3_servo_target`；
  2. `a3_stack.sh restart --servo-target` → 只启 `a3_servo_target`，不启 `servo_node_main`；
  3. 0.03 m/s 三轴 sweep：命令速度 CV < 0.15、0.3–3Hz 纹波 < 10%、回 home < 5mm、命令流空洞 >20ms = 0；
  4. 正→负反向后无速度台阶（`|Δv|` 每 10ms ≤ `twist_max_accel·dt` + LPF 贡献），末端停稳后 0.5s 漂移 < 3mm；
  5. 随机模式 60s 内不越过 home ±10cm，跟踪误差 RMS < 10mm。
- 在 `docs/edge/REQUIREMENTS.md` 表里新增 F181 行；同时把 F180 行状态 `proposed` 改为
  `implemented-pending-hw`（已真机验证 sweep，launch 接线随 F181 落地）。

### 3.2 节点：twist 平滑层（servo_target_node.cpp）

新增参数（`declare_parameter`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `twist_max_linear` | 0.40 | [m/s] 线速度模长上限（略高于 mapper 0.35，作安全网） |
| `twist_max_angular` | 1.2 | [rad/s] 角速度模长上限（mapper 1.0） |
| `twist_max_accel` | 1.0 | [m/s²] 线速度每轴加速度限幅 |
| `twist_max_ang_accel` | 3.0 | [rad/s²] 角速度每轴加速度限幅 |
| `twist_lpf_tau` | 0.06 | [s] 一阶低通；≤1e-3 关闭 |

实现（`step()` 的“1) 指令有效性”之后、目标积分之前）：

```cpp
// 1) 原始 twist（超时 = 0）
Eigen::Matrix<double,6,1> v_raw = Eigen::Matrix<double,6,1>::Zero();
if (have_twist_ && fresh) v_raw << ...;
// 模长上限（只缩不放大）
cap_norm(v_raw.head<3>(), twist_max_linear_);
cap_norm(v_raw.tail<3>(), twist_max_angular_);
// 2) 加速度限幅（逐轴）
for i in 0..2: v_lim_[i] += clamp(v_raw[i]-v_lim_[i], ±twist_max_accel_*dt_step);
for i in 3..5: v_lim_[i] += clamp(v_raw[i]-v_lim_[i], ±twist_max_ang_accel_*dt_step);
// 3) 一阶低通
const double a = (twist_lpf_tau_>1e-3) ? std::min(1.0, dt_step/twist_lpf_tau_) : 1.0;
v_lp_ += a * (v_lim_ - v_lp_);
const Eigen::Matrix<double,6,1> v = v_lp_;
const bool moving = v.head<3>().norm() > 1e-4 || v.tail<3>().norm() > 1e-3;
```

- `!active` 分支（模式互锁/超时）里把 `v_lim_`、`v_lp_` 清零，避免恢复时带旧速度。
- 目标积分、`hold_z` 判断、`vj` 前馈全部改用滤波后的 `v`。
- 顺带删除不再使用的 `otg_`/`Otg*`/`lookahead_` 遗留（已在 F180 v6 列为待办）。
- 头注释补一段“为什么先限加速度再低通”。

### 3.3 launch：`use_servo_target` 开关（a3_bringup.launch.py）

1. import 补 `UnlessCondition`、`AndSubstitution`（L47 `from launch.conditions import IfCondition` 处）。
2. `use_servo_target = LaunchConfiguration("use_servo_target")`，与其它 `use_*` 放一起（L143–156 附近）。
3. `servo_node`（moveit_servo）加 `condition=UnlessCondition(use_servo_target)`。
4. 新增节点（放在 `servo_node` 定义之后）：

```python
servo_target_node = Node(
    package="a3_trajectory_processing",
    executable="servo_target_node",
    name="a3_servo_target",
    output="screen",
    parameters=[
        {"robot_description": robot_description},
        {"robot_description_semantic": robot_description_semantic},
        {"robot_description_kinematics": kinematics_yaml},
        {"group_name": "arm", "base_frame": "base_link", "tip_frame": "end_effector"},
        {"twist_topic": "/servo_node/delta_twist_cmds"},
        {"out_topic": "/arm_controller/joint_trajectory"},
        {"publish_period": 0.01, "hold_z": True, "max_step_trans": 0.004,
         "twist_timeout": 0.30, "max_ik_jump": 0.05,
         "pos_gain": 4.0, "acc_gain": 20.0,
         "twist_max_linear": 0.40, "twist_max_angular": 1.2,
         "twist_max_accel": 1.0, "twist_max_ang_accel": 3.0,
         "twist_lpf_tau": 0.06},
    ],
    condition=IfCondition(use_servo_target),
)
```

5. `servo_anchor_node` 的条件改为
   `AndSubstitution(use_servo_anchor, UnlessCondition(use_servo_target))`
   （本节点已直接输出到 JTC 原生话题，不能再叠锚定器）。
6. 启动顺序列表（L740–755）里在 `servo_node` 旁加入 `servo_target_node`（二者互斥）。
7. 声明 launch 参数：

```python
DeclareLaunchArgument(
    "use_servo_target", default_value="false",
    description="F181: 用 a3_trajectory_processing/servo_target_node 替代 moveit_servo"
                "（绝对目标 + CLIK + twist 平滑）；默认 false"),
```

8. 文件头拓扑说明补一行“`use_servo_target:=true` 时 servo 链换为 a3_servo_target”。
9. `servo_mode_bridge` 保持无条件启动（它只负责把 `/a3/control_mode` 置 SERVO，链路与执行器无关）。

### 3.4 栈脚本：`--servo-target`（scripts/a3_stack.sh）

1. 参数解析（L892–914 的 while 循环）加 `--servo-target) servo_target=1 ;;`。
2. `do_start()` 组装 `servo_arg`，传给 setsid 里的 launch（L584）：

```bash
local servo_arg="use_servo_target:=false"
[ "$servo_target" = 1 ] && servo_arg="use_servo_target:=true"
...
ros2 launch a3_bringup a3_bringup.launch.py hardware:=can can_interface:=can1 \
  use_rviz:=$rviz_arg include_gripper:=$grip_arg $servo_arg
```

3. `NODE_PATTERN`（L39）尾部追加 `|servo_target_node`，保证停栈能清孤儿。
4. `usage()` 与文件头用法补 `--servo-target`，并注明“默认 false；开启后 PS4 L1+摇杆走绝对目标链路”。
5. `do_status` 核心子节点列表加 `servo_target_node` 一行（与 `servo_node` 并列）。

### 3.5 验收脚本：`scripts/a3_test/f181_servo_track.py`

单文件、`argparse` 子命令，LF、可执行，风格对齐 `sweep_test.py` 与 `a3_test/*` 现有脚本。

**公共部分**

- 订阅：`/joint_states`（BEST_EFFORT）、`/a3/ee_pose`（PoseStamped）、
  `/arm_controller/joint_trajectory`（RELIABLE）、`/a3/arm_status`（取 state/mode）。
- 发布：`/servo_node/delta_twist_cmds`（TwistStamped，RELIABLE depth 10，`frame_id=base_link`），50Hz 泵。
- `goto_home()`：服务 `/a3/arm/goto_named_pose {pose_name: home}`；等 EE 速度 <0.5mm/s 连续 0.5s，超时 25s 报错。
- 前置校验：`/a3/arm_status.state == READY`，否则打印“先 L3 使能”并退出码 2；
  读不到 `/a3/ee_pose` 时提示“需 use_servo_target/统一栈”并退出。
- 采样与输出：50Hz 记录 `cmd/关节/EE`；结果写 `~/.a3/servo_track/<YYYYmmdd-HHMMSS>/<mode>.json`，
  终端打印表格 + PASS/FAIL（不写仓库目录）。

**`sweep` 子命令**（默认 `--v 0.03`）

- 流程与已验证版本一致：x/y/z 各 +0.10 → −0.20 → +0.10 m，段间 0.4s 停、轴间 1.0s 停。
- 指标（每段去掉首尾 0.8s，按 CSV/采样 50Hz，命令速度用关节命令 + Pinocchio FK 的 6j 模型）：
  行程、命令速度 CV、0.3–3Hz 纹波、跟踪误差均值/最大、实测末端去趋势残差、回 home 误差、>20ms 空洞数。
- 判定：命令 CV < 0.15、纹波 < 10%、回 home < 5mm、空洞 0 → `PASS`。

**`random` 子命令**（默认 `--vmax 0.05 --duration 60 --bound 0.10`）

- 目标盒：目标轴坐标相对 home 的软边界 `0.9×bound`；实测硬边界 `bound`（用户确认触边界即停）。
- 每步：随机轴（x/y/z）、随机方向、随机速度 `U(0.01, vmax)`、持续 `U(0.6, 1.6)s`；
  若这一步会越过软边界，则改选反方向/其它轴；每步结束停 0.15–0.35s（制造正→0），
  并保证相邻两步至少有 70% 概率方向相反（制造正→负），逼出阶跃场景。
- 安全停止（先到先停）：
  1. 累计运行 ≥ `duration`（默认 60s）；
  2. 任一轴 `|EE − home| ≥ bound`；
  3. `state != READY`、或跟踪误差 > 50mm 持续 0.5s；
  4. Ctrl-C。
- 停止后：发零 twist、等 0.6s 记录停稳漂移，`goto_home()`，输出：
  运行时长、方向反转次数、各轴最大偏离、跟踪误差 RMS/最大、
  反转时命令速度台阶（应 ≤ `twist_max_accel·dt`）、停止后 0.5s 末端漂移、>20ms 空洞、触边界次数。
- 判定：未越界、跟踪 RMS < 10mm、停止漂移 < 3mm、无 >20ms 空洞 → `PASS`。

### 3.6 文档收尾

- `docs/edge/QUICKSTART.md` servo 段（L125–135、L177–185 附近）补：
  `./scripts/a3_stack.sh restart --servo-target`、停栈回默认、以及两个子命令示例。
- `docs/edge/requirements/F180-servo-absolute-target-integrator.md` 追加 v7：
  launch 接线方式、twist 平滑参数与实测前后对比（含停止/反转行为）。
- `scripts/README.md`、`scripts/a3_test/README.md` 加脚本索引与前置条件。
- 本次没有新增/改动话题或服务（`/servo_node/delta_twist_cmds`、`/arm_controller/joint_trajectory` 均沿用），
  `docs/shared/TOPIC_CONTRACT.md`、`ARCHITECTURE.md` 不改；只在 F181 里注明“拓扑等价替换 moveit_servo”。

## 4. 决策与假设

| 决策 | 理由 |
|---|---|
| 开关默认 `false` | 零回归；真机出问题可一键回 moveit_servo |
| 保留 `servo_mode_bridge` | SERVO 模式声明与其它消费方不变；本节点不依赖它 |
| 不启 moveit_servo 时不提供 `start/pause/unpause` 服务 | mapper 对服务缺失已容错（继续发 twist）；避免再造一套状态机 |
| 平滑放在节点内（twist 层），不加卡尔曼 | 输入是干净指令速度；限幅+低通足够，滞后可控（~60ms） |
| 平滑后目标“松手多走”约 `v·(v/a + τ)` ≈ 3–4mm | 这是平滑减速的必然结果，比原来的 4mm 追赶更缓 |
| 脚本只做 Cartesian 相对 home 的测试 | 与 F180 验收口径一致，便于前后对比 |
| 仿真栈（edge_teleop_full_sim）本次不接 | 用户只要求真机；节点本身已支持 `/a3/servo/joint_trajectory` 出口，后续可加同样开关 |

## 5. 验证步骤

1. **静态检查**
   - `python3 -m py_compile scripts/a3_test/f181_servo_track.py`、`bash -n scripts/a3_stack.sh`
   - `python3 -c "import launch"` 侧无法直接验 launch；用 `ros2 launch a3_bringup a3_bringup.launch.py --show-args | grep use_servo_target`
2. **构建**：`colcon build --packages-select a3_trajectory_processing a3_bringup`
3. **默认模式零回归**：臂 R3 失能 → `./scripts/a3_stack.sh restart` →
   确认 `servo_node_main` 在、`servo_target_node` 不在；L3 使能后短推摇杆有响应（用户确认）。
4. **新模式**：`./scripts/a3_stack.sh restart --servo-target` →
   `pgrep -af servo_target_node` 恰 1 个、无 `servo_node_main`；L3 使能。
5. **sweep**：`python3 scripts/a3_test/f181_servo_track.py sweep`（自动回 home）→ 表格 PASS。
6. **random**：`python3 scripts/a3_test/f181_servo_track.py random --vmax 0.05 --duration 60` →
   观察正→负换向是否平滑、边界是否守住、报告 PASS；最后臂回 home。
7. **PS4 手感**：L1+摇杆推拉/急停/反转，确认松手缓慢停下、无过冲抖动；
   Triangle/Circle 回 home/idle 仍正常（模式互锁）。
8. 结果与曲线结论写回 F181/F180 文档；若发现新坑，按规则补 `docs/lessons_learned/LL-NNN`。

## 6. 风险与回滚

- 风险 1：launch 里参数名/嵌套写错 → 节点起不来。验证：起栈后 `ros2 param get /a3_servo_target acc_gain`。
- 风险 2：`use_servo_target:=true` 时 `servo_anchor` 仍叠加 → 双重积分。已用 `AndSubstitution` 强制互斥。
- 风险 3：twist 低通让摇杆主观变“钝”。回滚：`-p twist_lpf_tau:=0.0`，或把 `twist_max_accel` 调大。
- 风险 4：随机脚本越界。脚本内硬边界 + 50Hz 监控 + 状态异常即停；首跑建议 `--vmax 0.03 --duration 20 --bound 0.05` 预演。
- 回滚：`./scripts/a3_stack.sh restart`（不带 `--servo-target`）即回 moveit_servo；代码层面新节点由开关隔离，不影响默认路径。
