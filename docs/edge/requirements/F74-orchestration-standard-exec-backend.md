# F74 — 编排层标准执行后端（control_msgs/FollowJointTrajectory action → JTC，参数门控）


- **说明：** F70–F73 建成 ros2_control 标准栈后，编排层兜底轨迹（goto/move_to 线性兜底、set_joint_positions jog、playback、safe-park）仍只往旧栈话题 `/joint_group_effort_controller/joint_trajectory` 直发，在标准栈上没有接收者。本需求统一执行入口，不重写任何轨迹生成逻辑：
  1. 新增参数 `control_backend`（默认 `"topic"`，旧行为零变化；`"fjt_action"` = 标准栈）、`arm_fjt_action`（默认 `/arm_controller/follow_joint_trajectory`）、`gripper_fjt_action`（默认 `/gripper_controller/follow_joint_trajectory`）。
  2. 新增 `_dispatch_trajectory(traj)`：topic 后端保持话题直发；fjt_action 后端把 7 关节轨迹按 JTC claim 集拆分投影（L1–L6 → arm FJT goal，L7 → gripper FJT goal；velocities/accelerations/effort 字段随点投影，时间戳不变），经两个标准 `control_msgs/FollowJointTrajectory` action 客户端异步发送；action 不可用/goal 拒绝/非 SUCCESSFUL 结束码均有 ERROR/WARN 日志。新 goal 抢占同 server 旧 goal，与旧栈话题替换语义一致；服务回调不阻塞，完成判定仍走既有时长调度与状态轮询。
  3. 五处 `self._traj_pub.publish(traj)` 全部改走 `_dispatch_trajectory`；move_group 成功路径（goto/move_to/safe-park 主路径）不经过本入口，行为不变。**增量并存：不删旧栈、默认参数不变。**
- **验收标准：**
  1. 默认（topic）后端：旧仿真栈行为零回归
  2. fjt_action 后端 + mock 标准栈（无 CAN/无电机）：jog（set_joint_positions）连续多次下发均到位（≤0.02），L7 同步运动；goto 线性兜底（关闭 moveit）到位；playback（含 retime）到位；safe-park 轮询收敛正常
  3. 全程轨迹只经标准 FJT action（无话题直发），无节点崩溃，抢占语义正确（连续 jog 不排队、不报错）
- **关联：** F70（JTC/标准栈）、F72/F73（真机/力矩栈）、F67/F68（上层规划/重定时，本需求是其兜底路径的执行落地）；审计任务 #10；LL-076（组外 L7 补发、单向关节夹紧）
- **状态：** `completed`（2026-09-22 仿真验收，F70 mock 双 JTC 栈，ROS_DOMAIN_ID=60，`scripts/a3_test/f74_fjt_backend_mock_acceptance.py` 12/12：显式 enable→READY；jog ×3（含 L7）落点误差 ≤0.0192；goto 线性兜底 ready/home 到位；playback 61 点正弦经 ruckig retime → 拆 arm/gripper FJT 执行，落 home err=0.0079；连续 jog 抢占语义正确（P1 dist=0.502 未跟踪）；旧话题全程零消息；disable safe-park move_group 后补发 L7 gripper 轨迹→全 7 关节收敛 home、reset 调用 1 次→DISABLED。L2/L3 单向限位目标必须在界内（JTC 越限静默夹紧）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
