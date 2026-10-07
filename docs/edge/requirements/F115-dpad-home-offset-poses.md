# F115 — D-pad 退役 F64 调速，改绑 home 周边 4 个笛卡尔偏移命名点位


- **说明：** 2026-09-26 用户确认：default 映射下 D-pad 不再分通道调速（速度恒为默认档 0.35，`step_linear_scale/step_angular_scale` 函数保留但不绑定），改为一键 goto 4 个以 legacy `home`（半抬位 `[0,.785,-.785,0,0,0,0]`，末端 base_link 系 (-0.177, 0, 0.326) m）为基准、末端保持 home 姿态不变的偏移点。点位值由 Pinocchio 严格 6D IK 探针（el_a3.urdf，200 种子+独立 FK 复核）按物理可达包络确定：
  | 点位 | base_link 偏移 | L1..L6 (rad) | 绑定 |
  |---|---|---|---|
  | `home_back` | 后 −X 20cm | `[0, 1.8674, -1.6192, -0.2482, 0, 0]` | D-pad 左 |
  | `home_front` | 前 +X 15cm（20cm 越水平臂展） | `[0, 0.1266, -0.8918, 0.7653, 0, 0]` | D-pad 右 |
  | `home_up` | 上 +Z 15cm（20cm 仅腕翻转限位边缘分支） | `[0, 1.2862, -1.9643, 0.6780, 0, 0]` | D-pad 上 |
  | `home_down` | 下 −Z 10cm（15/20cm 严格保姿态不可达） | `[0, 0.7154, -0.2479, -0.4675, 0, 0]` | D-pad 下 |
  4 点写入包内 `named_poses.yaml`（L7=0）并同步 `el_a3.srdf` group_state；goto 复用 F67 MoveIt 路径，不新增 action。
- **验收标准：**
  1. 仿真 READY 下合成 /joy 依次触发 D-pad 上/下/左/右 → `/a3/arm/goto_named_pose` 收到 `home_up/home_down/home_back/home_front`，7 关节到点容差 0.02 rad
  2. D-pad 操作后 `linear_scale/angular_scale` 恒为 0.35（无调速副作用）；L1/R1 摇杆 jog、R2 夹爪等其余映射零回归
  3. RViz MotionPlanning  Goal State 下拉可见 4 个新 group_state
- **关联：** F64（退役其 default 调速绑定，函数保留）、F67（goto MoveIt 路径）、F113（点位包内统一）；[shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)（命名姿态表）
- **状态：** `implemented`（2026-09-26 落地；真机回归待上电）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
