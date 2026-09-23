# LL-104 — QoS 不匹配只有换严格 RMW 才现形：servo 订阅是 RELIABLE；门禁固定 Cyclone；source 与 set -u 互斥

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，Fast-DDS（默认）/ Cyclone DDS 1.3.5

## 现象

F92 门禁固定到 Cyclone DDS 后，F77 joint jog 验收稳定失败：jog 指令照常 30 Hz 发布，`dL1=0.0000、max_vel=0.000`，servo_node 完全没反应。切回默认 Fast-DDS 则 8/8 通过。起栈时 Cyclone 日志直接给出：

```
incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY
```

## 根因

1. **QoS 可靠性是兼容条件，不是协商条件**：DDS 规范要求 Requested/Offered Reliability 兼容——BEST_EFFORT 发布者对 RELIABLE 订阅者是**不兼容**组合。Fast-DDS 对此宽松放行（实际会投递），Cyclone 严格按规范判不兼容并丢全部消息。
2. 旧代码注释声称 moveit_servo 输入是 SensorDataQoS/BEST_EFFORT，据此把 JointJog 发布端配成 BEST_EFFORT。用活栈实测 `ros2 topic info /servo_node/delta_joint_cmds -v`：Humble moveit_servo 的订阅是 **RELIABLE KEEP_LAST(1)**（SystemDefaultsQoS），注释是错的。
3. 门禁脚本用 `set -euo pipefail`，source ROS setup 脚本立即报 `AMENT_TRACE_SETUP_FILES: 未绑定的变量`——setup 脚本内部引用未预先定义的变量，与 `set -u` 天然冲突。
4. pep257（ament 约定）对多行 docstring 的硬要求：摘要必须在**第二行**（首行只有 `"""`，D213），且摘要以 ASCII 句点 `.` 结束（D400/D415）——中文句号 `。` 不算。

## 正确做法 / 规避

1. 以 `ros2 topic info <topic> -v` 的实测 QoS 为准，别信代码注释/记忆：publisher QoS 必须与订阅端兼容。servo 双输入发布端（`a3_teleop_ps4/actions.py` + F77 harness）统一改为 `RELIABLE`（depth=10），与上游 servo 示例一致。
2. 门禁/验收固定 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（`apt install ros-humble-rmw-cyclonedds-cpp`）：严格 QoS 把真问题逼出来；起栈突发下不丢 service 响应（Fast-DDS `rmw_response.cpp` 超时问题见 LL-103 第 7 条）；ARM 上 CPU 占用更低。
3. source ROS setup 期间临时放开 nounset：
   ```bash
   set +u
   source /opt/ros/humble/setup.bash
   source "$WS/install/local_setup.bash"
   set -u
   ```
4. 多行 docstring 用既定过检风格：`"""` 独占首行，第二行写摘要并以 ASCII `.` 结尾。

## 相关路径

- `scripts/a3_test/a3_ci_gate.sh`（固定 RMW + set +u 包 source）
- `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`（servo 发布端 RELIABLE）
- `scripts/a3_test/f77_joint_jog_acceptance.py`（harness jog_pub RELIABLE）
- 关联：[[LL-103-lint-gate-curated-subset-and-honest-wiring]]（起栈 service 响应丢失、spawner 串链）
