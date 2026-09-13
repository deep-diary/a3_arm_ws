# LL-030 仿真/真机 QoS 分叉：/joint_states 真机是 BEST_EFFORT，默认 RELIABLE 订阅静默断流

- **日期**：2026-09-13
- **产品线**：Edge（真机 7J；仿真闭环同源）
- **严重度**：高（真机上 FJT 动作收敛判定恒失败、IK 种子恒零、js 镜像断流——仿真全部正常，极具迷惑性）

## 现象

阶段 2 真机验证：FJT 动作 `/arm_controller/follow_joint_trajectory` 执行 4s 轨迹，臂肉眼可见到达目标且电机跟踪 abs_err ≤0.046，动作却报 timeout（code=-5）；IK 节点 `_q` 种子恒为零、求解发散；trajectory_bridge 的 `/rebotarm/joint_states` 镜像无数据。同一批节点在仿真闭环下全部正常。

## 根因

`motor_protocol_node`（真机执行层）发布 `/joint_states` 用 **BEST_EFFORT** 可靠性（SensorDataQoS 传统）；仿真三节点（sim_motor_node 等）用默认 **RELIABLE**。ROS 2 的 QoS 兼容规则：RELIABLE 订阅者与 BEST_EFFORT 发布者**不兼容**（要求相反不满足）→ 静默零投递，无任何错误日志。受影响的三处默认 RELIABLE 订阅：

- `follow_joint_trajectory_action.py`：`_joint_pos` 恒空 → 收敛判定（|err|≤0.05 保持 0.2s）永远不满足 → 动作超时；
- `move_to_pose_ik_node.py`：`_q` 恒零 → IK 种子/轨迹起点全错；
- `trajectory_bridge.py`：js 镜像断流。

## 修复

三处订阅统一显式声明 BEST_EFFORT（均已在源码注释 LL-030）：

```python
rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
```

## 教训

- 「仿真通过、真机静默失败」排查第一条：**对比两端发布者 QoS**（`ros2 topic info -v` 看 Publisher/Subscription 的 Reliability）。本仓真机 js 是 BEST_EFFORT，凡默认构造的订阅都要怀疑。
- 动作节点收敛判定若依赖 js，超时也要打出「收到 0 帧」类诊断，否则现象与「跟踪慢」无法区分（本次误判为收敛容差过紧，绕了远路）。
- 修完必须重启动节点（Python 包 symlink-install 源码即生效，但**运行中的进程不会热加载**）。
