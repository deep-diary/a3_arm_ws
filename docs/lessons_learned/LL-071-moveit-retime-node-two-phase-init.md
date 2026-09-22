# LL-071 — MoveIt 重定时服务节点：trajectory_processing 无 Python 绑定 + 两阶段初始化/空 vector/jerk 绑定五连坑

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble + MoveIt 2.5.9 + ros-humble-ruckig

## 现象

F68 需要在 Python 编排层（a3_arm_controller）对示教稠密点列做 Ruckig/TOTG 重定时。依次踩了：

1. `from moveit.trajectory_processing ...` 在 Python 侧根本不存在——Humble 的 moveit_core `trajectory_processing` 只有 C++ 接口（无 moveit_py 绑定）。
2. C++ 节点用 launch 注入嵌套参数（`velocity_limits.<joint>` map），构造函数里 `declare_parameter` 直接抛 `ParameterAlreadyDeclaredException`。
3. 改成 `NodeOptions().automatically_declare_parameters_from_overrides(true)` 后，构造函数里构造 `RobotModelLoader(shared_from_this(), ...)` 抛 `std::bad_weak_ptr` 崩溃。

## 根因

1. Humble 的 `RuckigSmoothing::applySmoothing` / `TimeOptimalTrajectoryGeneration` 只在 libmoveit_trajectory_processing 的 C++ 头文件暴露；Python 侧无任何绑定，import 必然失败。
2. `automatically_declare_parameters_from_overrides(true)` 会把 launch parameters 列表里的参数**预声明**进节点，构造函数再 declare 同名参数即二次声明。
3. `shared_from_this()` 要求对象已被 `shared_ptr` 接管（控制块已安装）；在 `make_shared` 调用的构造函数体内，控制块尚未装好，此时取 weak_ptr 抛 `bad_weak_ptr`。

## 正确做法 / 规避

1. 新建独立 C++ 包（`a3_trajectory_processing`）暴露 ROS 服务（`/a3/arm/retime_trajectory`，`a3_msgs/srv/RetimeTrajectory`），Python 侧以 service client 调用。CMake 里 `find_package(moveit_core moveit_ros_planning ruckig CONFIG)` 并 `target_link_libraries(... ruckig::ruckig)`。
2. 参数读取统一用守卫写法：
   ```cpp
   v = has_parameter(name) ? get_parameter(name).as_double()
                           : declare_parameter<double>(name, default);
   ```
   嵌套 map 参数（`velocity_limits.L1_joint`）由 launch 的 `{"velocity_limits": {name: val}}` dict 自动展开成 dot 参数，`has_parameter` 可直接命中。
3. 两阶段初始化：构造函数只做参数读取；新增 `void init()` 放 RobotModelLoader / service 创建，main 里 `auto node = std::make_shared<T>(options); node->init();` 之后再 spin。
4. 纯重定时不需要运动学求解器：`RobotModelLoader::Options{}.load_kinematics_solvers_ = false`，启动更快且不依赖 kinematics.yaml。
5. 多会话/多栈共用同一 ROS_DOMAIN_ID 时，仿真栈同名节点（sim_motor/move_group）会互相抢 /joint_states；启动前先 `ps -eo pid,etime,cmd` 确认，不要 pkill 匹配不到来源的进程（另见 LL-066/LL-069）。

## 追加坑 4：`std::vector<double>(0.0)` 是空 vector，不是含 0.0 的 vector

服务首次调用直接 SIGSEGV，gdb 定位在 `build_seeded_trajectory` 的 `v[i][j]`。

`std::vector<double>(0.0)` 的单参数构造签名是 `explicit vector(size_type count)`——`0.0` 被截断成 `size_t 0`，得到**空** vector，后续 `operator[]` 越界段错误。正确写法 `std::vector<double>(m, 0.0)`（count + value）。模板/重载场景下浮点字面量当 count 是 C++ 经典脚枪，gcc 无警告。

## 追加坑 5：Ruckig 单步 update 的 jerk 绑定 → 全局时长拉伸（4s → 29.6s）

崩溃修复后，201 点 / 录制 4.0s 的路径，Ruckig 返回 **29.61s**（每个 0.02s 段被统一拉到 0.148s），而 vs=1.0 时正常 4.01s。

根因在 Humble `RuckigSmoothing::runRuckig` 的机制：

1. `Ruckig(num_dof, timestep)` 用**平均段时长**作为固定控制周期，每个路点只调一次 `update()`——必须在一个常 jerk 周期内**精确**到达下一点的 (p, v, a)；返回非 `Finished` 就把**所有**段时长 ×1.1 并整体重算 v/a，从头再来（上限 10×）。
2. vs=0.2 时 jerk 上限 = `default_jerk_scale(5.0) × a_limit × 0.2` = 20 rad/s³。差分播种的三态彼此只近似一致，单步精确跟踪所需 jerk 远高于加速度的差分 jerk（实测需求峰值 33–55 rad/s³），任何一点不达标即触发全局拉伸。

正确做法（`relax_jerk_for_seed`）：逐段计算三个端点方程各自隐含的单步 jerk
`j_a=da/dt`、`j_v=2(dv−a0·dt)/dt²`、`j_p=6(dp−v0·dt−.5·a0·dt²)/dt³`，取峰值；逐关节 jerk 上限 = max(配置值, 需求峰值 × `seed_jerk_margin`(默认 1.5))。margin 不可省：jerk 恰好卡在代数需求时 Ruckig 的最小时间解仍会越过一个周期（实测 1.0 margin → 5.33s，1.5 margin → 4.41s）。语义上录制时刻已包含操作者选择的加减速节奏，Ruckig 在回放中只应兜底 v/a 越限，不应被一个拍脑袋的 jerk 上限拉长时长。

## 相关路径

- `src/a3_trajectory_processing/src/retime_trajectory_node.cpp`
- `src/a3_trajectory_processing/CMakeLists.txt`
- `src/a3_msgs/srv/RetimeTrajectory.srv`
- `src/a3_bringup/launch/a3_bringup.launch.py`
- `src/a3_bringup/launch/edge_web_sim.launch.py`
