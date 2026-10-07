# F120 — Triangle 改绑 home（不绑 ready）


- **说明：** ready（`[0,1.05,-1.575,0,0,0]`，F109 折叠竖直构型）距 idle 较远，作长按前就位不合用。把 **Triangle 短按从 goto ready 改为 goto home**（`[0.0, 0.785, -0.785, 0.0, 0.0, 0.0, 0.0]`，即 F115 D-pad 偏移基准点 / F40 safe-park 目标 / 重力标定锚点），工作位语义与 ready 一样确定离 idle、非奇异。纯 `default.yaml` 映射改动，零代码；ready 点位本身保留（service `/a3/arm/goto_named_pose {pose_name: ready}`、F68 回放/测试仍用）。
- **验收标准：**
  1. `default.yaml` `buttons.triangle` → `goto_named_pose {name: home}`（edge: rising），Circle → `{name: idle}` 保持不变
  2. 仿真 ps4_sim_test 场景 2 回归：Triangle 短按 → 状态 TRAJ(紫 reason "goto home") → 收敛 home 位姿（tol 0.08 rad）→ READY(绿)
  3. 场景 5/5b/8 预备（servo 扫轴 / 门控互斥 / safe-park 触发）改用 home 作为离 idle 标定位，全部 PASS
- **关联：** F109（ready 保留）、F115（home 为 D-pad 偏移基准）、F40（safe-park 目标）、F89（重力标定锚点）、F60/F64（PS4 全映射）；[PS4_OPERATOR_GUIDE.md](PS4_OPERATOR_GUIDE.md)、[README.md](../../src/a3_teleop_ps4/README.md)
- **状态：** `implemented`（2026-09-26 配置落地；仿真回归见 ps4_sim_test，真机复验待上电）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
