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
