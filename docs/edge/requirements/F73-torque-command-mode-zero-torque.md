# F73 — 力矩指令模式 + 标准重力补偿自由拖动控制器（对标官方 ZeroTorqueController，switch_controllers 切换示教）


- **说明：** 对标官方参考仓 `el_a3_hardware/ZeroTorqueController` 的工业级示教路径，取代手搓的 zero-torque 示教：
  1. `A3MITHardwareInterface` 增加力矩指令模式：覆写 `prepare_command_mode_switch` / `perform_command_mode_switch`，按接口名 `effort` 跟踪 `effort_mode_`；`write()` 在力矩模式下发 MIT 控制帧 **kp=0、kd=effort_kd（硬件参数，默认 2.0）、torque_ff=cmd_eff×direction**（关节空间力矩→电机轴：motor τ = joint τ / direction，direction=±1 等价乘；与官方一致）、位置域填当前实测电机角（保持）、速度域 0；力矩超量程由 codec 按 ±torque_max 钳位（RS00 ±14、EL05 ±6）。位置模式行为与 F72 完全不变。
  2. 同包新增 `controller_interface::ControllerInterface` 插件 `a3_hardware_interface/GravityCompensationController`：claim 各关节 `/effort` 命令接口 + position/velocity 状态接口；`on_configure` 优先从 controller_manager 节点 `robot_description` 参数取 URDF（兜底参数 `urdf_path`），`pinocchio::buildModelFromXML` 建模；`update()` 以实测 q 跑 RNEA `rnea(q,0,0)` 得重力矩写入 effort 命令，每 10 拍发布 `~/gravity_torque`（sensor_msgs/JointState）供观测。插件名/参数按官方 ZeroTorqueController 语义（joints 数组），不做自研惯性标定（URDF 惯性参数为准，后续可加标定文件）。
  3. `el_a3_controllers.yaml` 增加 `zero_torque_controller` 条目（L1–L6）；vcan launch 以 `--inactive` 预生成；示教进入/退出全部走标准 `ros2 control switch_controllers`（与 arm_controller 互斥），不再有自定义模式门。**增量并存：不改位置模式任何既有行为、不删旧示教代码路径。**
- **验收标准：**
  1. vcan0 栈启动后 `zero_torque_controller` 存在且 inactive；`switch_controllers --deactivate arm_controller --activate zero_torque_controller` 成功，控制器 active
  2. CAN 抓包：切换后每帧 kp≈0、kd≈effort_kd；torque_ff 与验收脚本用独立 Pinocchio（python bindings）RNEA 计算的重力矩（×direction）一致（误差 ≤0.02 Nm，仅量化误差）；多姿态（home/ready/中间位）逐一核对
  3. 切回 arm_controller 后 CAN 帧恢复 kp=80/kd=2，JTC home→ready→home 运动验收仍 ALL PASS（到位 ≤0.02）
  4. 全过程无节点崩溃、无自定义 FJT/插值节点；力矩帧值不超出 ±torque_max
- **关联：** F72（真机插件，本需求补全 effort 命令通路）、F70（标准控制器体系）；官方 `el_a3_hardware/ZeroTorqueController`（直接蓝本）；[shared/SAFETY.md](../shared/SAFETY.md)（示教安全）
- **状态：** `completed`（2026-09-22 vcan0 仿真验收，机械臂保持断电：`scripts/a3_test/f73_gravity_comp_vcan_acceptance.py` 41 项 ALL PASS——home/ready/mid 三姿态标准 switch_controllers 互斥切换全部 ok；CAN 帧 kp=0、kd=2.0、vel≈0、位置字段=实测位；torque_ff 对独立 Python-RNEA×direction 静态姿态最大偏差 0.0003 Nm；外力注入（motor3 +0.6 Nm ×1.5 s）L3 同号位移 −0.437 rad、10/10 步单调、全程 kp=0、运动中 torque_ff 偏差 ≤0.0013 Nm；撤力后稳定窗口间漂移 0.0008 rad；切回后 kp=80/kd=2/t_ff=0，JTC home→ready→home 落位 0.0003。坑见 LL-075）；真机验收待上电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
