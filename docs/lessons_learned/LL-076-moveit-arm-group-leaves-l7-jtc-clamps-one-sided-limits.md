# LL-076 — F74：move_group 只规划 arm 组，safe-park 漏掉 L7（夹爪保持上一 goal）/ 单向关节目标越限时 JTC 静默夹紧并「成功」

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，MoveIt 2.5.9 / ros2_control 2.54.0 / JTC 2.53.3，mock_components/GenericSystem
> **关联：** [[LL-072-ros2-control-mock-jsb-order-jtc-single-point]]、[[LL-074-ros2-control-system-interface-vcan-xacro-humble-pitfalls]]、F74

## 现象

F74 把编排层执行后端从自研话题切到标准 `control_msgs/FollowJointTrajectory` action（arm/gripper 双 JTC，参数 `control_backend` 门控），mock 验收时两个「栈是对的、结果不对」的坑：

1. disable 失能保护（safe-park）走 move_group 回 home：move_group 与 JTC 全部报告 **Goal reached, success**，但最终关节位 `[0.009, 0.0098, -0.0089, 0.327, 0.019, 0.0047, 0.10]`——L7（夹爪）停在 0.10，没回 home 的 −0.0002。
2. 首组 jog 目标给了 L2=−0.30 / L3=+0.25：JTC 同样报告成功，但 L2/L3 纹丝不动（误差 0.30/0.25），一度误判为「激活后首个 goal」的栈级 bug。

## 根因

### 1. move_group 的规划组是 arm（L1–L6），组外关节保持各自控制器上一个活跃 goal

MoveGroup 请求只约束规划组内关节；`moveit_simple_controller_manager` 只把轨迹发给 `arm_controller`。gripper 控制器上一个 goal（来自前序 jog）仍在保位，于是 L7 留在 0.10。编排层随后的 `_at_home` 按 7 关节 tol=0.15 检查居然通过（0.10 < 0.15），reset 被调用、状态进 DISABLED——「流程全绿，夹爪没归位」。

### 2. L2/L3 是单向物理关节，越限目标被 JTC 按 command interface min/max 静默夹紧

URDF/控制器限位：**L2 ∈ [0, 3.665]、L3 ∈ [−4.014, 0]**。发送 L2=−0.30 时，JTC 在轨迹采样处把命令夹到 min=0，轨迹合法执行完毕并返回成功——失败的是「目标本身不可达」，不是执行。

### 3. 验收时序：落点已到 ≠ 编排层已回 READY

jog 落点达成时 FSM 仍在 STATE_TRAJ（`_schedule_back_to_ready(duration+0.5)`），此时发 goto/playback 必然被拒（`state=TRAJ`）。外部编排/测试必须订阅 `/a3/arm_status` 等显式状态，不能只看关节落点。

## 正确做法 / 规避

- **凡走 move_group（只规划 arm 组）的回零/停机路径，必须显式补发 L7 轨迹**：safe-park 在 `_moveit_move` 成功后，对夹爪单独构造多点轨迹经 `_dispatch_trajectory` 发 gripper FJT action；收敛检查天然覆盖 7 关节，等夹爪也到位再 reset。
- **所有目标在发送前过 URDF 限位**（编排层 `_joint_limits` 已做 clamp）；单向关节（L2 只能正、L3 只能负）的测试目标尤其要检查。JTC 对越限的静默夹紧 + 成功返回意味着「action 成功 ≠ 到达你以为的目标」，落点必须逐关节复核。
- **测试/外部节点按 FSM 显式状态推进**：jog/goto/playback 后等 `/a3/arm_status.state == READY` 再发下一条，不要按「关节不动了」推断状态。
- 多层控制器栈（arm/gripper 两个 JTC）验收时，最终位姿一律按**全部关节**比对，规划组外的关节最容易被漏看。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_safe_park_then_disable` L7 补发、`_dispatch_trajectory` arm/gripper 拆分）
- `src/a3_description/urdf/el_a3_ros2_control.xacro`（L2/L3 单向限位）
- `scripts/a3_test/f74_fjt_backend_mock_acceptance.py`（状态门控 + 全 12 项验收）
