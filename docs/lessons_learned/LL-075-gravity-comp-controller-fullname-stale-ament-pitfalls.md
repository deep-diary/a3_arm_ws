# LL-075 — F73 重力补偿控制器六连坑：Humble get_name() 全名语义 / 旧 shell AMENT_PREFIX_PATH 致 pluginlib 只认 mock / SyncParametersClient 不能借 CM 的 executor / pinocchio idx_q() 是方法 / 命令句柄顺序不定 / 中位数窗口运动期滞后

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，ros2_control / controller_interface Humble 分支，pinocchio 4.0.0，vcan0 + Python 电机模拟器
> **关联：** [[LL-074-ros2-control-system-interface-vcan-xacro-humble-pitfalls]]、[[LL-072-ros2-control-mock-jsb-order-jtc-single-point]]、F72/F73

## 现象

F73 在 F72 真机 SystemInterface 上补 effort 命令通路，并新增标准 `controller_interface` 插件 `GravityCompensationController`（对标官方 ZeroTorqueController：RNEA 重力矩写 `/effort`，`ros2 control switch_controllers` 互斥切换）。调试期两类故障：

1. 控制器激活后 CAN 帧 torque_ff 与独立 RNEA 计算对不上（q 全 0 的重力矩），但代码看着在正常读状态。
2. ros2_control_node 启动直接崩：`pluginlib LibraryLoadException: class a3_hardware_interface/A3MITHardwareInterface does not exist. Declared types are fake_components/GenericSystem mock_components/GenericSystem`——明明包已构建安装、xml 注册无误。

## 根因 / 规避

### 1. Humble 的 LoanedStateInterface::get_name() 返回全名 "joint/interface"

Humble controller API 里 `get_name()` 是 **`L1_joint/position` 全名**（不是关节前缀名，`get_prefix_name()` 才是 `L1_joint`）。按裸关节名比较永远匹配不上 → q 保持初值 0，RNEA 出的是 q=0 的重力矩。正确做法用全名拼 `joint + "/" + iface` 比较；命令句柄则用 `get_prefix_name() == joint`。升级到 Iron+ 前所有 Humble 控制器都按此写。

### 2. setsid 继承旧 shell 的 AMENT_PREFIX_PATH → pluginlib 找不到新插件

长会话里先开了一个 shell、之后才 `colcon build` 安装新包；从这个旧 shell `setsid ros2 launch`，子进程的 `AMENT_PREFIX_PATH` 不含 install 的新条目（或指向安装前状态），pluginlib 枚举时只有 ros2_control 自带的 mock 类型。**source 必须与 setsid 在同一个 `bash -c` 内**：`bash -c 'source /opt/ros/humble/setup.bash && source install/local_setup.bash && ... && setsid ros2 launch ...'`。排查时直接看 `cat /proc/<ros2_control_node-pid>/environ | tr '\0' '\n' | grep AMENT_PREFIX`，不要只看当前 shell。

### 3. SyncParametersClient 内部 executor 不能 spin 已被 CM 托管的节点

控制器节点本身已经 add 到 controller_manager 的 executor；在控制器里用 `SyncParametersClient(get_node(), ...)` 拉 `robot_description` 会因它内部的 SingleThreadedExecutor 重复 spin 同一节点而拿不到结果。用一个**独立临时 `rclcpp::Node`** 构造 client，拉完即销毁。另外根命名空间拼接要防双斜杠：`ns=="/" ? "/" + cm_name : ns + "/" + cm_name`（`"" + "//controller_manager"` 服务名非法）。

### 4. pinocchio 4.0：idx_q() / idx_v() 是方法

`model.joints[jid].idx_q` 拿到的是成员函数指针不是索引；必须 `model.joints[jid].idx_q()`、`.idx_v()`。配套：`model.existJointName(name)` + `model.getJointId(name)`，C++ 侧建模用 `pinocchio::urdf::buildModelFromXML(urdf, model)`，Python bindings 用 `pinocchio.buildModelFromXML`。

### 5. command_interfaces_ 顺序不保证与 joints 参数一致

ResourceManager 按自己的 claim 顺序返回命令句柄，不能用 `command_interfaces_[i]` 对应 `joint_names_[i]`（错配=力矩写到错关节）。按关节名建 map / 每次 name-keyed 查找。

### 6. 验收脚本：中位数"当前位"在快速运动期滞后

复用 F72 Recorder：`current()` = 最近 200 个 /joint_states 样本的逐关节中位数（~2 s 窗口）。匀速运动中中位数比真实位置滞后约 1 s，导致：外力注入阶段步进符号偶发翻转、RNEA 期望值与帧对不上（时间错位 0.4 Nm）、撤力后"漂移"虚高（窗口里还是运动样本）。正确判据：

- 实时跟随/单调判定用**最新原始样本** `rec.samples[-1]`，不用中位数；
- 实时 torque_ff 核对以**该帧内嵌的电机位置**（frame pos / direction）反推 q 跑 RNEA，不与 rec 时刻混；
- 撤力后先 settle ≥2.5 s（超过 2 s 窗口）再读中位数，且"零力矩保持"用两个已稳定窗口之差，不依赖推末尾的样本；
- 新起的验收节点要等 /joint_states 发现完成（仅 spin 1 s 偶发拿到全零假失败，重跑或加长等待）。

## 正确做法 / 结论

- 力矩模式硬件侧：`prepare/perform_command_mode_switch` 按接口名跟踪 effort；`write()` effort 分支 kp=0、kd=effort_kd(2.0)、位置域填实测电机角、`motor τ = clamp(joint τ) × direction`，位置模式行为零改动。
- 控制器侧：claim `/effort` 命令 + pos/vel 状态，on_configure 三级取 URDF（参数 robot_description → urdf_path → 临时节点拉 controller_manager 参数），update() 实测 q → RNEA → 按名写力矩；on_deactivate 命令清零。
- vcan 验收 `f73_gravity_comp_vcan_acceptance.py`：home/ready/mid 三姿态互斥切换 + 外力注入跟随/保持 + 切回 JTC 回归，41 项 ALL PASS，静态姿态 torque_ff 偏差 ≤0.0003 Nm、运动中 ≤0.0013 Nm。

## 相关路径

- `src/a3_hardware_interface/src/gravity_compensation_controller.cpp`（控制器插件）
- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（effort 模式切换/write）
- `src/a3_bringup/launch/edge_ros2_control_vcan.launch.py`（vcan 标准栈，free_drive --inactive 常驻）
- `src/a3_description/config/el_a3_controllers.yaml`（zero_torque_controller 条目）
- `scripts/a3_test/vcan_motor_sim.py`（effort 动力学 + 外力注入文件）
- `scripts/a3_test/f73_gravity_comp_vcan_acceptance.py`（41 项验收）
