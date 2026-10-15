# F173 — 笛卡尔 xyz 直接巡游（请求给点，姿态默认不限、可选锁定）

- **说明：** 现有巡游（F116 随机命名点 / F161 固定序列）点位池一律来自 `named_poses.yaml`——每个点是**录制/标定得到的 7 关节角**，要巡游新位置必须先示教或保存命名点。本需求给 `/a3/arm/random_pose_tour` 增加**笛卡尔直给入口**：请求里直接给一串末端 x/y/z（`base_link` 系、`end_effector` 目标点，单位 m），编排层逐点解 IK 转成关节目标，再走既有巡游链（F161 平滑链 → F134 pilz Sequence → F116 逐腿链），无需先落盘点位。**姿态默认不限**（position-only IK，只求末端到位，对应「弹琴只要求末端到键位」语义），另可选锁定姿态。

  ### 与既有入口的关系

  ```mermaid
  flowchart LR
      A[random_pose_tour 请求] --> B{xyz 非空?}
      B -->|是 F173| C[逐点 position-only / 6D IK]
      B -->|否 fixed_sequence| D[命名点序列]
      B -->|否| E[F116 随机抽点]
      C --> F[F161 平滑链]
      D --> F
      E --> F
      F -->|失败| G[F134 pilz Sequence]
      G -->|失败| H[F116 逐腿链]
  ```

- **实现方式：**
  1. `a3_msgs/srv/RandomPoseTour.srv` 追加三个可选字段（原有 `count/seed/fixed_sequence` 语义不变）：
     - `float64[] xyz`：扁平三元组 `x1,y1,z1,x2,y2,z2,...`，非空时**优先于** `fixed_sequence` 与随机抽取；
     - `uint8 orientation_mode`：`0=free`（默认，姿态不限）/ `1=hold_start`（锁巡游起始末端姿态）/ `2=lock_quat`（锁 `ref_quat`）；
     - `float64[] ref_quat`：`orientation_mode=2` 时的参考姿态四元数 `(x,y,z,w)`。
  2. 编排层新增 `_cartesian_tour_targets()`：逐点调 `_solve_ik_position()`（Pinocchio，L1–L6，限位内缩 0.05 rad）；`_solve_ik_position` 增加可选 `seed`，**本点用上一点的解作种子**（保证多解时腕部连续，压 L4/L5 大摆）；IK 成功后再 `_pad_pose7` 补齐 L7（=0）。
  3. **姿态锁定是软约束**（`1/2` 模式）：先解位置+姿态（`log3` 旋转误差软约束）；该点锁姿态无解时**退化为姿态自由**并 WARN，不因一个点的姿态约束否掉整条巡游（`hold_start` 语义 = 尽量锁住起始腕姿）。
  4. **显式失败不静默跳点**：xyz 长度非 3 的倍数、`orientation_mode` 非法、`ref_quat` 长度非 4、任一点**位置**无解（不可达/限位）→ `success=false` 并回报出错点序号与坐标（沿用仓内「显式拒绝不回落」语义）。
  5. 复用现有三链与门禁（`_static_segment_violation` / `_duty_check`），不新增规划路径；单点 xyz 退化为 goto 语义（与 F161 单点一致）。

- **验收标准：**
  1. 仿真给出 3 个可达 xyz（如 `0,0.25,0` / `0.15,0,0` / `0,0.25,-0.074`）→ `success=true`，`sequence` 长度 3 且按请求顺序执行，逐点 `/joint_states` FK 末端与目标误差 ≤ 10 mm（含 F134 blend 转角偏置与跟踪残差）。
  2. `orientation_mode=0`（默认）与 `=1` 都能跑通；`=1` 时巡游全程末端姿态相对起始姿态偏差 ≤ 0.10 rad（对照 free 模式 3.14 rad），腕部连续。
  3. 位置不可达点（如 `0.30,0,0`，工作空间外）→ `success=false`、`message` 指出第几个点与坐标，状态机回 READY 不卡死。
  4. `xyz` 为空时行为与 F116/F161 完全一致（回归：随机巡游仍正常）。
  5. 长度非 3 倍数 / `orientation_mode=2` 但 `ref_quat` 非 4 个 / `orientation_mode` 越界 → 立即拒绝且 message 可读。

- **验收脚本：** `scripts/a3_test/f173_xyz_tour_sim_acceptance.py`（仿真，domain 99）：

  ```bash
  export ROS_DOMAIN_ID=99 && source install/setup.bash
  python3 scripts/a3_test/f173_xyz_tour_sim_acceptance.py   # 退出码 0 = ALL PASS
  ```

- **关联：** F116（随机命名点巡游，本需求不改变其语义）、F161（平滑链 / 固定序列）、F134（pilz Sequence + blend）、F39（`/a3/arm/move_to` 单一关节目标）、F115/F131（现有"先示教或先存点"路线）；`_solve_ik_position` 复用见 `arm_controller.py`（弹琴 position-only / 6D IK）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（arm 服务表）
- **状态：** `implemented`（2026-10-15 仿真验收 ALL PASS：3 点 free 链逐点到位 ≤9.1 mm、hold_start 姿态偏差 0.018 rad 对照 free 3.140 rad、位置不可达点显式拒绝、非法参数立即拒绝、F116 回归通过）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
