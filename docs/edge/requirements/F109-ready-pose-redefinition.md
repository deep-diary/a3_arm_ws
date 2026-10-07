# F109 — ready 点位重定义（折叠竖直、臂重心投影过底座中心；对标官方 zero/home/ready 点位语义）


- **说明：** 用户真机-仿真手柄验收（2026-09-24）反馈：Triangle 到达的 F69 ready（[0, 1.6, -0.7]）只是 L2 向上抬一个角度，整臂重心落在底座前方，静态受力大，希望 ready 时重心位于底座中心正上方、静态力矩最小。pinocchio 复核 F69：活动臂（L1–L6 连杆，2.51 kg）CoM 在底座系前移 106 mm，静态重力矩绝对值之和 4.83 N·m。官方 EDULITE_A3 另有两个点位可对标：SRDF home [0, .785, -.785]（CoM 前移 8 mm、2.64 N·m）与 test_moveit_waypoints ready [0, .785, -1.57, 0, .785, 0]（前移 43 mm、2.86 N·m，6 轴 Jacobian 条件数 47），均不满足「重心过中心 + 静态力矩最小」。本需求在 |CoM_x| ≤ 10 mm 约束下重新网格搜索选点，并把结论同步到全部三处点位来源
- **设计（数据驱动选点，零运动代码改动）：**
  1. 网格 L2 ∈ [0.4, 2.05]、L3 ∈ [-2.0, 0]，步长 0.05，L1=L4=L5=L6=0；逐点用 RNEA 求重力矩、遍历连杆惯量求整体 CoM、computeJointJacobian(L6) 求条件数。约束 |CoM_x| ≤ 0.010 m，目标最小化 sum|τ|，并要求 6 轴条件数尽量远离 F69 伺服奇异起始阈值 25
  2. 选中 **ready = [0, 1.05, -1.575, 0, 0, 0]**：折叠竖直构型，CoM 前移 8 mm，sum|τ| = 1.90 N·m（较 F69 降 61%，也优于官方 home 的 2.64）；cond3 = 5.0、cond6 = 29.0。L7 保持当前值（用户覆盖文件 -0.0002）
  3. 已知遗留：cond6 = 29 高于 F69 伺服奇异缩放起始 25，ready 点启动 Servo 可能立即进入奇异缩放（但 cond3=5.0 平移方向健康）；验收必须实测摇杆手感，若明显变软改用备选点 [0, 0.85, -0.675]（CoM_x=-20 mm 略破约束、τ=3.01、cond6=19.7）
- **改动（仅点位数据 + 文档）：**
  1. `src/a3_description/config/named_poses.yaml`：ready → [0, 1.05, -1.575, 0, 0, 0, 0] + F109 注释（包级默认；goto 与 MoveIt 共用）
  2. `src/a3_moveit_config/config/el_a3.srdf`：ready group_state 同步（L2=1.05、L3=-1.575、L5=0）
  3. `~/.a3/poses.yaml`（仓外用户覆盖，不进 git）：ready 同步 [0,1.05,-1.575,0,0,0,-0.0002]
  4. 新增 `scripts/a3_test/f109_ready_pose_acceptance.py`（mock 栈：goto ready 规划/到位 + 独立 RNEA/CoM 复核 + gripper action 开合，顺带验收 R2 修复）；`docs/edge/QUICKSTART.md` 同步点位表
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f109_ready_pose_acceptance.py`）：**
  1. mock 全栈 L3 使能后 `/a3/arm/goto_named_pose`（pose_name=ready）MoveIt 规划 + 执行 SUCCESSFUL，到位关节与 [0,1.05,-1.575,0,0,0] 偏差 ≤ 0.03 rad
  2. 到位后独立 RNEA 复核：|CoM_x| ≤ 0.010 m，sum|τ| ≤ 2.2 N·m（允许模型/舍入裕量）
  3. R2 链路：`/gripper_controller/gripper_cmd` action 发送开合目标，L7 在 3 s 内到位（0↔1.79），position 误差 ≤ 0.05 rad
  4. home/zero goto 回归不受影响；结束无残留进程，退出码 0
- **关联：** F69（前版 ready + 伺服奇异阈值）、F74（goto FJT 后端）、F87（GripperActionController 是 R2 修复路径）、F108（位置帧重力前馈在新低力矩点位收益更小但仍必需）、EDULITE SRDF / test_moveit_waypoints（点位对标来源）
- **状态：** `completed`（2026-09-25，mock 验收 14/14：`scripts/a3_test/f109_ready_pose_acceptance.py`。P1 零位使能一次成功；P2 goto ready MoveIt 9 点/0.7 s 执行 SUCCESSFUL，到位 [0.002,1.056,-1.580,0.005,-0.005,0.009]；P3 独立 RNEA 复核 CoM_x=-8.0 mm、sum|τ|=1.90 N·m；P4 GripperCommand 闭合 1.790/打开 0.000 均 reached_goal（R2 链路）；P5 goto home MoveIt 12 点/1.0 s、到位与用户覆盖值一致。排查中两处均为测试脚本问题而非产品缺陷：TRAJ→READY 回切有 ~1 s 状态延迟（脚本加 wait_state）；home 期望须按 FSM 优先级读 ~/.a3/poses.yaml 覆盖）。用户手柄 R2/Triangle 实测与 ready 点 Servo 手感（cond6=29 vs 阈值 25）待复验；真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
