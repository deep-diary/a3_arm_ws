# F89 — pinocchio 重力模型标定/核验（静态多姿态测量 → 逐关节重力矩比例因子 + R²；对标 EDULITE_A3 pinocchio_gravity_calibration.py）


- **说明：** 参考仓库 `el_a3_ros/scripts/pinocchio_gravity_calibration.py` 提供了一套轻量「重力模型核验」流程：多姿态静止、测电机维持力矩、与 pinocchio RNEA 预测做逐关节最小二乘，给出比例因子 + RMSE/R²。我们已有更重的 F49 十二参数惯性标定（mass+CoM，L-BFGS-B，真机小时级）和 C++ RNEA 自由拖动控制器，但缺三样：① 快速、可重复的**模型-实物一致性核验**（调试/出厂 commissioning 标准动作）；② F49 之后残余误差的廉价单参数修正；③ 修正因子被运行时消费的闭环。本项按工业 commissioning 路径补齐（与参考脚本同一测量原理：位置控制器保持静态姿态时电机维持力矩即重力负载，平衡态 τ_motor = g，无需切换控制器——早期「每姿态切自由驱动」方案在低惯量 sim 下残差重力导致姿态漂移，且 SIGTERM 中断会留下互换的控制器，已弃用）：标定工具走 F88 两点 FJT 到位，**保持 arm_controller 抱位**，静止窗口直接采样 `/joint_states` 电机反馈力矩（电机域，按 JOINT_SIGNS 转 URDF 域），预测力矩在**同一实测位姿**上用独立 pinocchio RNEA 计算（默认套用 F49 标定惯量），逐关节 LSQ 比例（clip [0.5,2.0]）+ RMSE/R²；结果直接写成 controller_manager 可用的 ParameterFile，由 bringup 加载后自由拖动控制器按比例缩放重力矩。vcan 仿真器同步增加 URDF 重力物理模型（可注入真实比例 + 测量噪声），使全流程可在断电条件下闭环验收。
- **改动：**
  1. 新增产品工具 `scripts/gravity_scale_calibration.py`（ROS 2 节点）：测试构型 = home + L2/L3 单关节扫掠 + L2×L3 网格 + L4/L5/L6 扰动（去重 atol、按 URDF 限位 clamp 并留安全余量，`--quick` 缩减）；到位用 F88 两点 FJT（两点显式零 v/a）；每构型到位后保持 arm_controller 抱位 → settle → 30 样本均值/标准差（不切换控制器）；启动时校验 arm_controller active。
  2. 拟合：`scale = Σ(g·τ)/Σ(g²)`（τ 为 URDF 域实测、g 为同位姿 RNEA），clip [0.5,2.0]，逐关节 RMSE 与 R²；输出 `~/.a3/gravity_scales.yaml`（格式 = CM ParameterFile：`zero_torque_controller.ros__parameters.tau_scale`，元数据另置顶层 key），终端打印汇总；`--out/--quick/--no-calibrated-inertia` 等参数。
  3. C++ `GravityCompensationController` 增参 `tau_scale`（double 数组，默认全 1，尺寸不符回退全 1），update() 内按比例缩放 RNEA 力矩；configure 时另经 yaml-cpp 解析 `a3_description/config/inertia_params.yaml`，把 F49 fitted mass/CoM 套用到 5 个 link（`use_calibrated_inertia` 默认 true，可用 `inertia_params_file` 覆盖路径）——运行时 RNEA 模型必须与标定工具/仿真真值同模型，否则比例因子相对另一个模型回归（首版残差最大 0.40 Nm）。a3_description 已 exec-depend 本包，反向不声明 package depend（循环），改由 ament_index 运行时定位 share。
  4. `a3_bringup.launch.py` 增参 `gravity_scales_file`（默认 ""；为空且 `~/.a3/gravity_scales.yaml` 存在则自动采用，语义同 named-pose 用户覆盖），存在时作为追加 ParameterFile 供 ros2_control_node 加载。
  5. `vcan_motor_sim.py` 增 `--gravity-model urdf`（+ `--gravity-scales` 六值、`--gravity-noise`、`--gravity-urdf`）：effort 模式下按当前姿态实时 RNEA 计算重力负载（电机域），物理净力矩 = 施加力矩 − 重力负载 + 推力 − 黏性，反馈力矩 = 施加力矩 + 高斯噪声；position 模式下内层刚性伺服一阶跟随到位，静态平衡时反馈力矩 = 电机域重力负载 + 高斯噪声（F89 抱位测量配对；运动中动态力矩不模拟，测量只在 settle 后进行）。
  6. 验收脚本 `scripts/a3_test/f89_gravity_scale_acceptance.py`：A 段注入已知比例跑标定工具，断言恢复精度与 R²；B 段以产出文件重启栈，经 `~/gravity_torque` 话题断言自由拖动力矩 = scale×独立 RNEA（≤0.03 Nm，实测 0.0000），切模式 settle 1 s 后测稳态漂移（1 s 内 ≤0.02 rad，实测最差 L3 0.0038 rad——模式切换瞬态在 settle 前，见 LL-099）；C 段无 scales 文件时日志确认 tau_scale 回退 1.0。
- **验收标准（仿真；机械臂保持断电，vcan0 闭环；脚本 `scripts/a3_test/f89_gravity_scale_acceptance.py`）：**
  1. A 段：工具全流程成功（两点 FJT 到位、arm_controller 全程抱位不切换、静态采样），产出 yaml 含 6 个比例与逐关节 R²/RMSE
  2. 注入比例（如 0.92/1.08/0.95/1.10/1.00/1.05，噪声 0.01 Nm）恢复误差 ≤ 0.05；被激发关节 R² ≥ 0.95（重力幅度过小的关节允许标记 low-excitation 豁免）
  3. B 段：加载产出文件后，zero_torque_controller 在 home/ready 姿态下发（`~/gravity_torque`）的力矩与 `scale × 独立 RNEA` 一致（≤0.03 Nm，实测 0.0000）；切模式 settle 1 s 后稳态 1 s 位姿漂移 ≤ 0.02 rad（实测最差 0.0038）
  4. 默认（无 scales 文件）行为不变：C++ 控制器 tau_scale=1.0、bringup 不追加文件；legacy 栈 gravity_torque_node 不受影响
- **关联：** F49（十二参数惯性标定，本项默认套用其结果并修正残余）、F73（RNEA 自由拖动控制器）、F88（两点 FJT 规约）、F85（自由拖动阻尼）；参考 `EDULITE_A3/el_a3_ros/scripts/pinocchio_gravity_calibration.py`
- **状态：** 已实现（仿真 20/20，2026-09-23；真机验收 2026-10-04）。真机 full 标定 22 姿态（接触污染符号判据自动剔除 1 点，LL-145），得 tau_scale L2=1.0695 / L3=0.9964 / L4=0.9570（L1/L5/L6 低激励保持 1.0），示教态实测无下坠/上飘。输出收敛到包内 `src/a3_description/config/gravity_scales.yaml`（`~/.a3` 仅向后兼容回退），工具注册为 `ros2 run a3_bringup gravity_scale_calibration`。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
