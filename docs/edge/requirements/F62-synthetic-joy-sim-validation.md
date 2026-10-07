# F62 — 合成 /joy 全功能仿真验证（无手柄自动化）


- **说明：** 真机联调前先用脚本模拟手柄输出逐键验证，避免「人感觉按键失效」的不可重复排查。新增：
  1. `a3_bringup/launch/edge_teleop_full_sim.launch.py`：include `edge_web_sim`（use_rviz:=false, use_target_ghost:=true, use_gripper:=true）+ 内联 MoveIt Servo 块（复用 a3_bringup.launch.py 的 servo_mode_bridge + servo_node_main 写法，禁止叠加会双 RSP/双 /joint_states 的 servo.launch）+ ps4_teleop（use_joy_node:=false, mapping:=default, enable_feedback:=true）+ 双模型 RViz（软件渲染，LL-027）。
  2. `scripts/a3_test/ps4_sim_test.py`：50 Hz 合成 `sensor_msgs/Joy`（axes[8]/buttons[14]，ds4_linux 索引；开场 ≥25 拍全零基线满足 mapper alive 计数，每步回零）；BEST_EFFORT 订阅 /joint_states；纯 rclpy 计时（LL-050，不用 ros2 CLI 做时序断言）；Reporter 风格逐步打印 PASS/FAIL + 关节位移证据。
  3. 隔离域 ROS_DOMAIN_ID=45。
  12 场景：基线橙 → PS init（**F135 起改为 L1+R1 长按 init，前置 PS 解绑回归断言**）→ Triangle ready → L3 幂等 → R2 脱离 L1 开合 L7 → 摇杆 deadman 逐轴方向表 + R1 速度档 → Circle home → 示教三件套（断言 latest.yaml mtime）→ R3 失能 → L3 恢复 → X 长按硬急停+恢复 → Options 长按 set_zero。
  仿真与真机已知差异**只记录不断言**：sim_power shutdown 无 SoftProne 动画/无 F51 联动；sim 不校验 set_zero 状态前提；sim /joint_states 为 RELIABLE（F60 的 QoS 修复在仿真不可复现，仅代码审查）；sim 不强制门禁。
- **验收标准：**
  1. `ros2 launch a3_bringup edge_teleop_full_sim.launch.py`（DOMAIN 45）起栈无致命错误，双模型 RViz 可见
  2. `python3 scripts/a3_test/ps4_sim_test.py` 12 场景全绿，输出每步关节/状态/灯效证据；摇杆方向表用于标定各轴 invert
  3. 全绿后用户实操（真手柄）复测一轮作为最终签收
- **关联：** F60（被测键位）、F61（被测灯效诊断话题）、LL-027（软件渲染）、LL-050（纯 rclpy 时序）、LL-059（仿真 QoS 画像掩盖真机不兼容）
- **状态：** `implemented-pending-sim`（2026-09-20 launch + 脚本；验收执行随本次任务）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
