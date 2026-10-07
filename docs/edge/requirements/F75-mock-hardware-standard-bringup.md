# F75 — 全产品 mock-hardware 标准栈 bringup（零自研 sim 节点：控制器 inactive 启动 + 编排层 switch_controller 使能）


- **说明：** F70–F74 建成标准执行/规划栈后，「整体启动」仍走 `edge_web_sim.launch.py` 的三个自研模拟节点（sim_motor_node / sim_power_sequence_node / gravity_torque_node），且真机栈的使能语义（`/a3/motor/enable|reset|set_zero` 服务）在标准栈上没有对应物。本需求统一产品级 bringup 拓扑，与真机 F72 栈同构：
  1. 编排层新增参数 `motor_service_backend`（默认 `"can_service"`，旧行为零变化；`"controller_switch"` = 标准栈）：enable → `/controller_manager/switch_controller` STRICT 激活 `arm_controller` + `gripper_controller`（硬件插件 `on_activate` 内 reset→enable，见 F72）；reset → 同服务去激活两控制器（`on_deactivate` 零增益刷新 + MIT stop）；set_zero → 标准栈绝对编码帧无此操作，返回成功跳过。switch 是幂等操作，`_enable_cb` 在本后端不再依赖 `/a3/motor/states` 的使能镜像；`_init_cb` 在本后端跳过 set_zero 与零位确认，直接激活控制器（在当前反馈位重锚）。
  2. 新增 `edge_full_mock.launch.py`：xacro `use_mock_hardware:=true` → `mock_components/GenericSystem`；两个 JTC 经 spawner `--inactive` 启动（上电不使能，等待显式 enable），JSB 保活提供 `/joint_states`；move_group + retime + 编排层（`control_backend=fjt_action`、`motor_service_backend=controller_switch`、`require_gate=false`）+ 夹爪产品节点（`traj_topic:=/gripper_controller/joint_trajectory`——JTC 原生话题入口，位置类命令直入标准控制器）+ MQTT 桥 + arm_monitor（可选）。**全程无 sim_motor_node / sim_power_sequence_node / gravity_torque_node。**
- **验收标准：**
  1. 启动后节点清单无任何自研 sim 节点；boot 态两个 JTC `inactive`、FSM `IDLE`
  2. enable → 两 JTC `active`、FSM `READY`；jog / goto(move_group) / playback(retime) 落点 ≤0.02，`/joint_states` 速度字段有效（LL-072 顺序不回退）
  3. 夹爪 `/a3/gripper/command` 位置命令 → L7 经标准 JTC 话题实际运动
  4. disable safe-park → 两 JTC `inactive`、FSM `DISABLED`；MQTT 桥节点存活无崩溃
- **关联：** F70（标准栈）、F72（硬件 on_activate/deactivate 使能语义）、F74（FJT action 后端）；LL-072（JTC/JSB 启动顺序与单点语义）、LL-076、LL-077（goto L7 补发 + 位置/速度双落定）；审计任务 #10
- **状态：** `completed`（2026-09-22 仿真验收，ROS_DOMAIN_ID=61，`scripts/a3_test/f75_full_mock_acceptance.py` 15/15：boot 两 JTC inactive、FSM IDLE、零自研 sim 节点、产品节点齐；enable→controllers activated→READY、两 JTC active；jog ×3（含 L7）落点 err ≤0.0199、速度字段有效 max_vel=0.429；goto move_group ready/home err ≤0.0094（L7 同步补发）；playback 61 点正弦经 retime+双 JTC 落 home err=0.0118；夹爪位置命令经标准 JTC 话题驱动 L7（0→1.780→0）；disable safe-park 位置/速度双落定后两 JTC inactive、FSM DISABLED；MQTT 桥全程存活）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
