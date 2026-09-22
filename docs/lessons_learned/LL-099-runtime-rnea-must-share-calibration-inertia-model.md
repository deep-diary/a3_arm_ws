# LL-099 — 运行时重力补偿 RNEA 必须与标定工具共用同一套惯性模型

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble

## 现象

F89 验收 A 段标定结果完美（R²≈1），但 B 段加载产出文件后：自由拖动力矩与独立 RNEA×scale 残差最大 **0.40 Nm**（阈值 0.03），1 s 姿态漂移 **0.099 rad**（阈值 0.02）。

## 根因

三方用了两个不同的模型：

- 标定 Python 工具与仿真真值都套用 F49 fitted mass/CoM（来自 `a3_description/config/inertia_params.yaml`）；
- C++ `GravityCompensationController` 从安装 URDF 构建 pinocchio，用的是**名义惯量**（如 l3_lnik_urdf_asm 名义质量 0.262025 vs F49 0.2712），且从不读 F49 覆盖。

于是 scale 是相对「F49 模型」回归出来的，却乘在「名义模型」的 RNEA 输出上，残差系统性偏大。

## 正确做法 / 规避

- **一条模型链原则**：标定工具、运行时补偿控制器、仿真真值必须从同一份惯量参数构建。C++ 控制器在 configure（buildModel 之后）用 yaml-cpp 解析同一个 inertia_params.yaml，逐 link 替换 mass/CoM（保留原转动惯量），日志打印 applied 数量便于核验。
- package 依赖注意：a3_description 已 exec-depend a3_hardware_interface，反向再声明 depend 会形成循环依赖导致 colcon 无法拓扑排序。改为不声明依赖、运行时用 `ament_index_cpp::get_package_share_directory("a3_description")` 定位（同 workspace 必然存在），找不到则 WARN + 名义模型兜底。
- **模式切换瞬态不是稳态漂移**：position→effort 切换瞬间有速度/力矩交接，立即测量会拿到 0.03~0.1 rad 位移；settle 1 s 后再测，稳态漂移仅 0.0038 rad。验收应判稳态，不判切换瞬间。

## 相关路径

- `src/a3_hardware_interface/src/gravity_compensation_controller.cpp`（ApplyCalibratedInertia）
- `scripts/gravity_scale_calibration.py`、`scripts/a3_test/vcan_motor_sim.py`（同一份 inertia_params.yaml）
- `src/a3_description/config/inertia_params.yaml`
