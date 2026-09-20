# LL-059 同一 QoS 坑第四次复现：teleop 默认 RELIABLE 订 /joint_states，仿真画像（RELIABLE 发布）再次掩盖真机静默断流

- **日期**：2026-09-20
- **产品线**：Edge（真机 7J；F60 键位重设计真机联调暴露）
- **严重度**：高（真机命名位姿键完全失效，仿真逐键全绿；与 LL-025/LL-030 同族，第四处）

## 现象

F55 真机遥控联调：Triangle/Circle 等命名位姿键按下后日志连续
`no /joint_states yet; skip named pose`，`ActionExecutor._have_js` 恒为 False；
R2 夹爪力控（不依赖 js）与直发轨迹的 Square 开机正常，造成「只有两个键是好的」
的错觉。同一提交在仿真闭环（edge_web_sim）下命名位姿全部正常。

## 根因

`a3_teleop_ps4/actions.py:145` 与 legacy `ps4_arm_teleop.py:64` 均用默认 QoS（depth=10，
RELIABLE）订阅 `/joint_states`。真机发布端 `motor_protocol_node` 是
SensorDataQoS/**BEST_EFFORT**（LL-005/LL-018/LL-025/LL-030 已记录），
RELIABLE 订阅与 BEST_EFFORT 发布不兼容 → 静默零投递。仿真 `sim_motor_node` 用默认
**RELIABLE** 发布，于是仿真永远测不出这类订阅——这是 LL-030 已总结的
「仿真过、真机静默挂」画像，第三次（LL-025 gravity、LL-030 FJT/IK/bridge、本次 teleop）
因完全相同的仿真画像漏到真机。

## 修复

两处订阅显式 BEST_EFFORT（BEST_EFFORT 订阅同时兼容 RELIABLE 与 BEST_EFFORT 发布，
仿真行为不变）：

```python
rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
```

## 教训

- **每新增一个 `/joint_states` 订阅，QoS 必须抄 gravity_torque_node.py 的 BEST_EFFORT
  写法，没有例外**；指望仿真回归拦住是不可能的——仿真发布端不改，这条漏检路径永久
  存在。评审清单加一条：grep `create_subscription(JointState` 逐个核对。
- 要从根上止血，得让仿真发布端也用 SensorDataQoS（BEST_EFFORT），或在 CI 做
  `ros2 topic info -v` QoS 一致性检查；本次只修订阅，仿真画像问题留作后续。
- 现象与「键位/边沿引擎失效」难以区分（日志是 skip named pose，不是无按键事件）：
  排查顺序应先确认 `_have_js`/订阅计数，再怀疑映射。

## 相关路径

- `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`（ActionExecutor `/joint_states` 订阅）
- `src/a3_teleop_ps4/a3_teleop_ps4/ps4_arm_teleop.py`（legacy 同坑）
- `src/a3_bringup/a3_bringup/gravity_torque_node.py:109`（标准 BEST_EFFORT 写法参照）
- 关联：LL-005、LL-025、LL-030（同族 QoS 静默不兼容）
