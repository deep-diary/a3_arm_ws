# F108 — 位置模式 pinocchio 重力前馈（对标 EDULITE gravity_feedforward_ratio；消除稳态下垂）


- **说明：** 真机插件 `A3MITHardwareInterface` 位置帧（JTC/夹爪默认模式）的 MIT t_ff 字段当前恒为 0。MIT 位置模式由电机端 kp/kd 闭环，关节受恒定重力矩 τ_g 时稳态下垂 δ = τ_g/kp（F49 记录大伸展位形可达 ~0.06 rad），轨迹落位后只能靠抬高 kp 硬压——工业做法是在指令帧前馈重力矩，让电机只承担动态分量。EDULITE 参考实现 `el_a3_hardware.cpp` 默认 `gravity_feedforward_ratio = 1.0` 且每个控制周期用 pinocchio RNEA 更新位置帧 t_ff，本需求对齐该行为；F49 标定惯量与 F89 逐关节 τ_scale 直接复用，不新增标定流程。
- **设计：**
  1. **模型与标定复用**：插件 on_init 一次性构建 pinocchio 模型（`robot_description` 硬件参数 → `urdf_path` 文件兜底）；`use_calibrated_inertia`（默认 true）时套用 F49 `inertia_params.yaml`（`use_calibrated_params: true` 才生效，逻辑同 `gravity_compensation_controller.cpp`）；F89 逐关节 `tau_scale`（6 值 L1–L6 顺序）从 `gravity_scales_file`（默认 `~/.a3/gravity_scales.yaml`，不存在或尺寸不符 → 1.0）读取；L7 恒为 1.0。模型构建失败不阻断插件加载，仅将前馈比例视为 0 并打一条错误日志（可用性优先）
  2. **前馈注入**：硬件参数 `gravity_feedforward_ratio`（默认 1.0，clamp 到 [0,1]）、`use_pinocchio_gravity`（默认 true）。write() 位置模式分支在 fb_mutex_ 内以当前 hw_pos 拼 q（逐关节 idx_q），`rnea(q,0,0)` 求重力矩，帧 t_ff = `clamp(ratio * tau_scale[i] * g_tau[idx_v], ±torque_max) * direction`。以下帧保持零前馈不变：effort 模式帧、soft-start 阻尼帧、freeze-hold 冻结帧、on_activate/on_deactivate 零指令帧。速度前馈不在本需求范围
  3. **运行时可调**：ratio 经 `hardware_interface` 的标准 param 声明暴露，参考实现支持在线改比例（调试时逐步加大），本需求保持同一参数名
- **改动：** `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（模型/标定/scale/注入）；`src/a3_description/urdf/el_a3_ros2_control.xacro`（真机块新增参数）；新增 `scripts/a3_test/f108_gravity_ff_vcan_acceptance.py`；`docs/shared/SAFETY.md`、`docs/edge/QUICKSTART.md` 同步。CMake 无需改（pinocchio/yaml-cpp/ament_index 已链接）
- **验收标准（仿真；机械臂保持断电；vcan0 + vcan_motor_sim；脚本 `scripts/a3_test/f108_gravity_ff_vcan_acceptance.py`）：**
  1. **ratio=0**：多点位形保持时各电机位置帧 t_ff ≈ 0（≤ 0.05 N·m）
  2. **ratio=1**：home/ready/伸展位形保持时，逐电机 t_ff = 独立 RNEA 重力矩 × scale × direction，误差 ≤ 0.02 N·m（量化+tmax 差异内）；运动过程中抽样同样吻合（按瞬时实际关节角计算）
  3. **中间比例**：ratio=0.5 时 t_ff ≈ 0.5 × RNEA，误差 ≤ 0.02
  4. **模式切换回归**：arm_controller ↔ zero_torque_controller STRICT 双向切换正常，effort 帧 kp=0、力矩 = RNEA（zero_torque 路径不受影响）
  5. **运动回归**：JTC home→ready→home 全程 SUCCESSFUL、跟踪正常；L7 夹爪帧 t_ff=0（无重力模型关节）
  6. 结束后无残留进程，退出码 0
- **关联：** F49（标定惯量同源）、F73（zero_torque 控制器 + vcan RNEA 验收基建）、F89（tau_scale）、F89b（STRICT 切换）、EDULITE `el_a3_hardware.cpp`（对标）
- **状态：** `completed`（2026-09-24，vcan 验收 23/23：`scripts/a3_test/f108_gravity_ff_vcan_acceptance.py`。P1 ratio=0 保持帧 t_ff 最大 0.0002；P2 ratio=1 home→ready→mid→home 保持帧 + 134 个运动抽样吻合独立 RNEA（最大偏差 0.0067）；P3 ratio=0.5 保持帧半值（0.0004）；P4 STRICT arm↔zero_torque effort 帧 kp=0、力矩=RNEA，切回位置帧正常；P5 JTC 全 SUCCESSFUL、位置模式 L7 帧 t_ff=0。排查中发现参数回调 handle 未持有导致在线 ratio 不生效，见 LL-126；验收脚本区分 GAC 空闲 effort 帧与 L7 位置帧）。真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
