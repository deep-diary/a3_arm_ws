# F70 — ros2_control 标准栈仿真激活（JTC/JSB/controller_manager 取代手搓 FJT/插值）


- **说明：** 审计结论（用户：「自己折腾总会出错，尽量复用现有工具」）：现行执行链是手搓三件套——`a3_fjt_action`（自研 FJT action 桥）+ `motor_protocol_node` 内 200 Hz 插值 + 自研 /joint_states 反馈，MoveIt Execute 实际落到自研 action 再转 effort 话题。工业标准路径是 **ros2_control 标准栈**：`controller_manager`（200 Hz update loop）+ `joint_state_broadcaster`（标准 /joint_states）+ `joint_trajectory_controller`（官方 JTC，样条插值、容差监控、FJT action 原生暴露）；MoveIt `moveit_simple_controller_manager` 的 `arm_controller`/`gripper_controller` 命名空间与 JTC 默认 action（`<ns>/follow_joint_trajectory`）天然一致——move_group 可**直连**官方 JTC action，自研 `a3_fjt_action` 整条删除。仿真用 `mock_components/GenericSystem`（xacro 已具备 `use_mock_hardware:=true` 分支，`mock_sensor_commands=false`）替代三个自研 sim 节点。**本需求为增量式**：新增 `edge_ros2_control_sim.launch.py`（与旧栈并存，不删旧节点/旧 launch），真机迁移（SocketCAN/MIT 协议的 `hardware_interface::SystemInterface` 插件）另立需求。控制器配置 `el_a3_controllers.yaml` 已就绪但从未激活；**不加载其中 zero_torque_controller**（a3_can_bridge 自研插件，属旧栈）。
- **验收标准：**
  1. 新 launch 启动无致命错误：controller_manager 加载 joint_state_broadcaster + arm_controller（JTC，L1–L6）+ gripper_controller（JTC，L7）三个控制器且状态 active
  2. joint_state_broadcaster 以稳定频率发布 /joint_states（7 关节 name/position/velocity 齐全）
  3. 直接向 `/arm_controller/follow_joint_trajectory` 发 action 目标（home→ready→home）：样条轨迹平滑、到位（per-joint goal 容差内），action 成功返回；gripper_controller 同理可动 L7
  4. move_group plan+execute 端到端：经官方 JTC 完成 goto（全程无 a3_fjt_action 进程）
  5. 数值验收脚本：起止速度≈0、v/a 不超限、目标误差 ≤0.02 rad；结论 ALL PASS
- **关联：** F67/F68（同 MoveIt 体系，本次替换执行底座）、审计任务（自研→标准工具）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；真机 SystemInterface 插件（后续需求）
- **状态：** `completed（仿真）`（2026-09-22 仿真验收 11/11 ALL PASS，ROS_DOMAIN_ID=58：JTC 直连 home→ready→home（31 点五次 S 曲线），vmax ≤0.52、amax ≤2.77、目标误差 0.0000；夹爪开合 vmax ≤0.78、amax ≤2.57；move_group plan+execute 经官方 JTC（9 点 TOTG 轨迹），vmax ≤2.09、amax ≤7.31、目标误差 ≤0.0093；栈内无自研 FJT 节点。踩坑见 [LL-072](../lessons_learned/LL-072-ros2-control-mock-jsb-order-jtc-single-point.md)。真机迁移 = SocketCAN/MIT SystemInterface 插件，另立需求）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
