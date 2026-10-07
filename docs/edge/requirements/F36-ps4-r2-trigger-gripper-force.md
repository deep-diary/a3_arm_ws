# F36 — PS4 R2 扳机力控夹爪


- **说明：** 改造 PS4 映射（`simple` 与 `default` 两份）：R2 从「夹爪位置模拟量」升级为「扳机力控」——新 action `gripper_force`（`a3_teleop_ps4/actions.py`，经 `/a3/gripper/command` 服务，`a3_msgs` 依赖新增）。语义：松开（v<0.15，迟滞下沿）→ 下降沿发一次 `release`（全开）；按过 0.22（迟滞上沿）→ `force`，扳机 0.2..1 线性映射目标力矩 **0.1..1.0 Nm**（下限 0.1 因目标 ≤0 触发全开硬逻辑且接触判定下限 0.1 Nm），按得越深抓得越紧；持按期间目标变化 ≥0.1 Nm 才重发（避免 50 Hz 重发把积分清零）；服务未就绪/互锁拒绝（臂运动中等）→ 0.5 s 间隔重试，松手即停；`timeout_s=15` 与节点默认一致，GRASPED 后持续持握。L2→L6 不动，急停键（Triangle/L1/R1/Share 等）不动；顺带把 `set_gripper`/`set_joint_L7`/`gripper_toggle` 的旧标定常量对齐 2026-09-06 标定（open=0/close=1.79，修复 Square/Circle 开合反向）。
- **验收标准：**
  1. `simple.yaml`/`default.yaml` 经 `validate_mapping` 校验通过，mapper 正常启动
  2. sim 注入假 `/joy`（axes[5] 扫描 0→1→0）：收到 force 且目标力矩随扳机单调递增，下沿只发一次 release
  3. 真机（泡棉）：按住 R2 → GRASPED 且实际力矩 ≈ 映射值；松手 → 3 s 内回 0 位全开；扳机深浅变化抓力跟随
  4. 手臂运动中按 R2：命令被拒（互锁）且 0.5 s 重试、松手停止；急停按键行为不变
- **关联：** F24（力矩上限 1.0 Nm，映射上限对齐）、F26（GripperCommand 服务）、F33（force 0 全开/互锁）；[shared/SAFETY.md](../shared/SAFETY.md)（R2 力控安全）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（扳机契约）
- **状态：** `implemented`（2026-09-07 真机验收：`simple`/`default` 双映射 validate_mapping 通过；泡棉手柄实测 R2 深浅 → 目标力矩 0.3→1.0 Nm 跟随（含持按期间深浅双向调制）、GRASPED 实际力矩 ±2% 内（1.00→1.01、0.96→0.97、0.78→0.79）、每次松手下降沿单次 release 全开、全程 error_code=0；验收项 4（臂运动互锁重试）在单电机台架（仅 ID7）无法触发臂运动，代码路径由 sim 互锁检查覆盖）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
