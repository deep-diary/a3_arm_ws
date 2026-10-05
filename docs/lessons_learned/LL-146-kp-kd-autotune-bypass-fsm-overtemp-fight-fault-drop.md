# LL-146 — kp/kd 自动整定绕过 FSM：过温保护被 FJT 抢占 + FAULT 失能直掉臂

> **产品线：** Edge
> **日期：** 2026-10-05
> **关联需求：** F138（MIT 位置环 kp/kd 伺服层自动整定）

## 现象

真机在 home 半抬位跑 kp/kd 逐关节整定时，L3（前臂俯仰）长时间保位承受 ~3.2 Nm 重力负载，几分钟内升至 108°C 触发 F44 过温保护（protect=95°C）。随后暴露两个安全问题：

1. **过温保护与整定互相打架**：温度 >95°C 时 FSM 触发 `safe park -> idle`，但整定脚本直接消费 `/arm_controller/follow_joint_trajectory`（JTC action）绕过编排层状态机，下一拍又把臂拉回 home 继续扫——「回 idle → 又回 home」反复，两者逻辑未联动。
2. **FAULT 下 disable 直接断电掉臂**：safe-park 被整定抢占后超时 → FAULT「still enabled」；此时调 `/a3/arm/disable` 走 `_disable_cb` 的「IDLE/FAULT 直达 reset」分支，不先 safe-park，臂在 home（半抬）直接失能垂落，需人托住。

## 根因

1. **整定脚本绕过编排层状态机**：为简单直接消费 JTC action（`/arm_controller/follow_joint_trajectory`），不读 `/a3/arm_status`、不读温度，FSM 进入 SAFE_PARK/FAULT/COOLING 后脚本仍继续发轨迹，与过温保护抢占同一条 JTC 轨迹流（JTC 轨迹天然可被后发替换，谁后发谁赢）。
2. **`_disable_cb` 的 FAULT 分支设计为「直达 reset」**（`arm_controller.py` L2563-2571）：假定 FAULT 时臂已安全；但本次 FAULT 恰是 safe-park 超时（臂仍使能、仍在 home 半抬），直达 reset 即掉臂。应区分「已失能/已在 idle」与「仍使能且偏离 idle」。
3. （附带物理根因）**L3 在伸展位姿保位力矩 ~3.2 Nm**：home/ready 下前臂重力负载大，整定 duty cycle 高时必过热；F44 保护正常触发，但整定流程未控 duty cycle。

## 正确做法

1. **整定脚本必须闭环 FSM 状态与温度**：
   - 订阅 `/a3/arm_status`，state 离开 READY（SAFE_PARK/FAULT/COOLING/DISABLED）立即中止并回名义增益；
   - 每轮评估前查温度（`/a3/motor/states`），任一关节 ≥ warn(90°C) 暂停/中止；
   - 关节间加冷却间隔（duty cycle），L3 等重力负载关节单独控温；伸展位姿整定优先在 home 而非折叠 idle（idle 腕部连杆干涉，见同轮）。
2. **`_disable_cb` FAULT 分支修正**：若臂仍使能且偏离 idle（有新鲜反馈、不在 home 容差内），应先 `safe_park` 再 reset，而非直达 reset；仅「已失能/已在 idle」才直达 reset。
3. 伸展位姿（home/ready）长时保位/整定要盯 L3 温度，必要时回 idle 冷却再继续。

## 相关路径

- `scripts/kp_kd_autotune.py`（整定脚本，需加 FSM/温度监护）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`：`_disable_cb`(L2463)、`_trigger_temp_protect`(L2059)、`_safe_park_then_disable`(L1899)
- `src/a3_arm_controller/config/arm_controller.yaml`：`temp_warn_c`/`temp_protect_c`/`temp_hysteresis_c`、`disable_park_*`
- 关联：LL-023（L3 过温首触发，阈值 65→90/95）、LL-044/LL-045/LL-142（safe-park 系列）

## 追加（2026-10-05 续跑复现）：过温保护重入致 switch_controller rejected

**现象：** 续跑整定 L5 时 L3 又到 100°C，第一次过温保护成功（safe-park→DISABLED→COOLING），但随后报
`overtemp: , protect failed: switch_controller rejected (STRICT) (parked at idle; stop power sequence first)`，
日志根因行：`Controller with name 'arm_controller' can not be deactivated since it is not active.` → `Aborting, no controller is switched! (::STRICT switch)`。

**根因：** `ReentrantCallbackGroup` 下 `_trigger_temp_protect` 阻塞期间可被重入：
safe-park 里 `_moveit_move` 报 `error -4`（PATH_TOLERANCE_VIOLATED，被整定脚本的 return FJT 抢占）会短暂把
state 置回 READY，绕过 `SAFE_PARK` 守卫；此刻 L3 仍 ≥100°C 使 `_temp_protect_pending` 再次置位，重入的
`_trigger_temp_protect` 见 READY → 再次 `_safe_park_then_disable` → 对**已失能**的 `arm_controller` 再
`deactivate` → STRICT 整体拒绝 → FAULT。

**修：** `_trigger_temp_protect` 加 `_temp_protect_busy` 忙锁（进入置位、`finally` 清除）防重入；把过温保护
的 deactivate 语义变幂等（臂已失能则不再重复 deactivate）。顺带修整定脚本两处：TempMonitor 订阅改
`BEST_EFFORT`（否则 max_temp=0 读不到温度）；恢复流程遇 FAULT 先调 `/a3/arm/disable` 清理到 DISABLED 再续跑。

## 追加（2026-10-05 第三轮）：F51 带外失能不做 ros2_control teardown → enable 永久 STRICT 死锁

**现象：** 整定后真机实测 idle→home，臂在 READY 时电机「带外失能」（F51 本地兜底：7 个 fresh 电机均报
关闭持续 1.0 s，或看门狗 `/a3/monitor/status` 发 TRIGGERED/UNEXPECTED_DISABLE）。FSM 正确进入 DISABLED，
但之后 `/a3/arm/enable` 永远失败：
`switch_controller rejected (STRICT)`，日志：`Controller with name 'arm_controller' is not inactive so ...
cannot be activated.` → `Aborting, no controller is switched!` → 硬件被回退 INACTIVE。只能手动
`switch_controller deactivate` 或重启栈恢复。

**根因：** F51 的两处带外失能收敛路径（`_publish_status` 消费 pending、`_safe_park_then_disable` 循环内
早退）只做了 `_set_state(STATE_DISABLED)` + `mode=IDLE`，**没有 deactivate 控制器、没有把硬件退回
INACTIVE**。状态簿（FSM=DISABLED）与 controller_manager（arm_controller/gripper_controller 仍 active、
硬件仍 ACTIVE 但电机已离线）不一致；下次 enable 的 activate 对已 active 控制器发 activate，STRICT 整体拒绝。

**修：** 新增 `_handle_oob_disable(why)` 统一收敛：置 DISABLED 后立即 `_motor_command(reset, 2)`
（ros2_control 栈即 `_cm_switch(2)`：BEST_EFFORT deactivate → 硬件 INACTIVE），best-effort 失败只告警
不改 DISABLED 事实。两条 F51 路径都改调它。验证：idle 位发一帧假 TRIGGERED → DISABLED 且两控制器
inactive；随后 enable 一次成功回 READY。

## 追加（2026-10-05 第三轮）：整定 kd 偏小——小幅度单关节考核与真实大行程多关节运动不等价

**现象：** L2 整定结果 kp/kd=150/0.5（原名义 80/2，kd/kp 从 0.025 掉到 0.0033），真机 idle→home
（L2 行程 ~0.75 rad）到位后来回晃动，到位后峰值速度 0.25 rad/s、最大残摆 0.31°。

**物理：** MIT 律 τ=kp·Δq+kd·Δq̇+τ_ff，关节闭环近似二阶，ζ=kd/(2√(kp·J))。惯量不变时保持阻尼比，
**kd 应随 √kp 增长**：kp 80→150 ⇒ kd 应 ×1.37 ≈ 2.7，而非降到 0.5。

**整定为什么选错：** ① 激励只有 0.2 rad/0.8 s 慢斜坡，真实行程大、速度高，低频余振激不起来；
② 单关节考核无多轴联动/重力前馈模型误差；③ 落定窗仅 0.6 s，采不到周期 0.5–1 s 的低频晃动；
④ J 中 torque_rms 对 kd 产生的阻尼力矩扣分，小幅下「振动分」没拉开 → 系统性偏向小 kd。

**做法：** 不盲目重标。运行时只改 kd 复测真实 idle→home（到位后 2.5 s 窗指标）：kd 0.5→2.5 残余 RMS
降 3.8×、残速 0.25→0.11；2.5→4.0 仅再 2× 且费力矩，取 **kd=2.5**（与 √kp 预测一致）。
后续改进整定器：激励改双向大幅度 + 更接近真实轨迹的速度曲线，落定窗 ≥1.5 s，或增加一条「完整
idle→home 多轴轨迹」考核项。
