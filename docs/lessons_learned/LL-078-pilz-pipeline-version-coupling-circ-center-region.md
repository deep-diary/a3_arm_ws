# LL-078 — F76：MoveIt/Pilz/pick-ik 精确版本耦合、CIRC「center」约束必须用覆盖整弧的大区域、陈旧 CMake 缓存

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，MoveIt 2.5.9→2.5.10，pilz-industrial-motion-planner 2.5.10，pick-ik 1.1.2，mock_components/GenericSystem
> **关联：** [[LL-071-moveit-retime-node-two-phase-init]]、[[LL-076-moveit-arm-group-leaves-l7-jtc-clamps-one-sided-limits]]、F76

## 现象

F76 把 Pilz 工业运动规划器作为第二条规划管线接入 move_group，调试中依次踩到：

1. MoveIt 2.5.9 升级到 2.5.10 后，旧栈（升级前已启动的进程组）里 pick_ik 报 `libmoveit_robot_state.so.2.5.9: cannot open shared object file`，retime 节点 exit 127 找不到 `libmoveit_robot_model_loader.so.2.5.9`。
2. CIRC 请求稳定返回 error_code=-2「Position constraint violated」，日志显示弧上点（距 center 60 mm）与「期望」差 60 mm。
3. Sequence 返回 error_code=-31「Could not solve request / Can not determine start state from empty sequence」。
4. 重编自研 C++ 包时 CMake 缓存仍硬链接 `libmoveit_constraint_sampler_manager_loader.so.2.5.9`，增量构建必失败。

## 根因

### 1. MoveIt 插件按精确次版本号链接（.so.2.5.9 vs .so.2.5.10），且运行中进程持有已被卸载的库路径

MoveIt 升级只替换 `/opt/ros/humble/lib` 下的 `.so.2.5.10`；第三方 IK 插件 pick-ik 的已装 build（`1.1.2-2jammy.20260804.*`）链接的是 `.so.2.5.9`，pluginlib 加载即失败。**升级 move_group/moveit_core 后，所有链接 MoveIt 的第三方插件与自研 C++ 包都必须同步换链接 2.5.10 的版本**；升级前启动的进程组还持有旧库路径，必须整组重启（旧 .so 文件已不存在）。

### 2. CIRC 的 center 辅助点经 path_constraints 传递，但通用管线仍把该约束当路径区域校验全程

Pilz 的约定：`MotionPlanRequest.path_constraints` 内名为 `center` 的 PositionConstraint，读取 `constraint_region.primitive_poses[0]` 作为圆心（另有 `interim` 约定）。但 move_group 的通用请求校验**不管这个约定**——它拿约束 region 对整条规划轨迹做碰撞/约束校验。若 region 做成 1 mm 小盒（直觉上的「一个点」），弧上每一点都在盒外，code=-2 必现（日志里 Desired 与 current 之差正好等于半径）。

### 3. Sequence 首项没有可解的起始状态：多段程序对起始位姿有隐性几何要求

LIN/CIRC 验收结束后末端停在远离 ready 的位置，此时三角形目标（ready 附近 +80 mm）对当前起始位姿不可达，Pilz 对整条 sequence 求解放大成 -31，且空/退化序列会报「Can not determine start state from empty sequence」。混合程序下发前必须先 PTP 回到程序几何可解的位姿。

### 4. CMakeCache 记录了升级前库的绝对 SONAME

`build/a3_trajectory_processing/CMakeCache.txt` 内残留 `libmoveit_*.so.2.5.9` 的链接探测结果，增量构建不会重新探测，重编必然失败。

## 正确做法 / 规避

- **升级 MoveIt 的标准动作清单**：① `apt upgrade ros-humble-moveit*`；② 同步升级所有第三方 MoveIt 插件（本次 pick-ik 换 `1.1.2-2jammy.20260910.012631`，Pilz 随 `ros-humble-pilz-industrial-motion-planner` 2.5.10）；③ 重编自研链接 MoveIt 的 C++ 包（a3_trajectory_processing、a3_hardware_interface）；④ `kill -INT -<PGID>` 整组重启升级前启动的栈。用 `ldd install/<pkg>/lib/<node> | grep moveit` 核对链接版本。
- **重编 C++ 包缓存失配时用 `colcon build --symlink-install --cmake-clean-cache --packages-select <pkg>`**，不要删 build/install 树（`--cmake-clean-cache` 让 CMake 重新探测库路径，本次 21 s 通过）。
- **CIRC center 约束的 region 必须是覆盖整段圆弧的大盒**（本次半边长 0.5 m）：Pilz 只用 `primitive_poses[0]` 当圆心，region 大小只影响通用校验；看到「Position constraint violated」且差值≈半径，就是 region 做成了点。
- **Sequence/混合程序前先 PTP 归位**到程序几何可解的起点；每个 MotionSequenceItem 的 blend_radius 末项给 0（不 blend 到下一段），中间项按工艺给混合半径。
- 管线选择在请求内：统一服务 `/plan_kinematic_path`（GetMotionPlan）按 `pipeline_id`（"ompl"/"pilz"）+ `planner_id`（"PTP"/"LIN"/"CIRC"）路由，不存在独立命名的 pilz 服务；序列走 `/plan_sequence_path` 与 `/sequence_move_group` action。

## 相关路径

- `src/a3_moveit_config/config/pilz_industrial_motion_planner.yaml`、`pilz_cartesian_limits.yaml`
- `src/a3_bringup/launch/edge_full_mock.launch.py`（双管线 + sequence 能力）
- `scripts/a3_test/f76_pilz_acceptance.py`（CIRC 大 region center、PTP 归位后发 sequence，12/12 验收）
