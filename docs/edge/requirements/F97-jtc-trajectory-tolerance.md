# F97 — JTC 轨迹容差工业级配置（goal_time 超时必 abort + 逐关节 trajectory 跟踪容差；堵住卡死轨迹永久挂起）


- **说明：** `arm_controller`（joint_trajectory_controller 2.53.x）现有 `constraints` 只有逐关节 `goal: 0.03` 与 `stopped_velocity_tolerance: 0.1`，存在两个工业现场不可接受的缺口：
  1. **`goal_time: 0.0` 禁用目标时间约束**：JTC 语义中 goal_time ≤ 0 时检查直接跳过——轨迹段结束后若实际状态始终进不了 goal 容差，FollowJointTrajectory 目标**永远不结束**。F81 已实证此行为：单电机反馈失联触发插件 freeze-hold（写指令冻结在最后位置），运动中的轨迹 JTC 目标保持 pending 永不返回。工业控制器必须在轨迹结束后给定有限收敛窗口，超时即 abort
  2. **无逐关节 `trajectory:` 跟踪容差**：运动过程中实际位置与指令的偏差不做任何检查，电机堵转/被拽偏/严重滞后时轨迹照常「成功」走完时间轴
  - FSM 执行后端（arm_controller.py）下发的 FJT 目标不携带逐目标 path/goal_time 容差，控制器默认约束即对所有产品路径（F88 两点轨迹、goto、playback、jog）生效，无需改应用代码
- **改动（仅标准 JTC 参数，零自研代码）：**
  1. `src/a3_description/config/el_a3_controllers.yaml` `arm_controller.constraints`：
     - `goal_time: 1.0`（轨迹最后一点之后 1.0 s 内必须进入 goal 容差，否则 abort → GOAL_TOLERANCE_VIOLATED）
     - L1–L6 逐关节增加 `trajectory: 0.05`（运动中位置偏差 > 0.05 rad 即 abort → PATH_TOLERANCE_VIOLATED）；保留 `goal: 0.03`（后由 F112 放宽至 0.15，见下）
  2. 新增 `scripts/a3_test/f97_jtc_tolerance_acceptance.py`（vcan 注入，不触真机）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f97_jtc_tolerance_acceptance.py`）：**
  1. **回归**：vcan 栈（vcan_motor_sim 一阶跟随）使能后下发正常两点轨迹，结果 `SUCCESSFUL`(0)
  2. **跟踪/超时 abort**：运动前用 `/tmp/f97_silence.json` 冻结电机 4 反馈（插件 freeze-hold），下发 2 s 轨迹：目标必须在 `轨迹时长 + goal_time + 1.0 s` 内返回（不再永久 pending），error_code 为 `PATH_TOLERANCE_VIOLATED`(-4) 或 `GOAL_TOLERANCE_VIOLATED`(-5)
  3. **恢复**：清除 silence 后再发正常轨迹，恢复 `SUCCESSFUL`（容差 latch 不残留）
- **关联：** F81（freeze-hold 是本需求要兜底的故障形态）、F88/F94（产品轨迹均经同一 JTC）、F74（FSM 执行后端）
- **状态：** `completed`（2026-09-23，vcan 验收 7/7：`scripts/a3_test/f97_jtc_tolerance_acceptance.py`。A 正常两点轨迹 SUCCESSFUL（2.98 s）；B 冻结电机 4 反馈后轨迹在运动 0.90 s 处即被逐关节跟踪容差中止，error_code=PATH_TOLERANCE_VIOLATED(-4)（远早于 2+1+1 s 上限，不再永久 pending）；C 清除 silence 后恢复 SUCCESSFUL（3.01 s）。验收脚本 in-process 驱动节点的 domain/RMW 环境坑见 LL-109）。真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
