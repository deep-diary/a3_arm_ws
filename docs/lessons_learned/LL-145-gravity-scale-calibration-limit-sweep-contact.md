# LL-145 — 重力比例标定扫掠越限与接触污染

> **日期：** 2026-10-04
> **产品线：** Edge
> **环境：** RK3588 板 + ROS 2 Humble

## 现象

F89 单关节重力比例标定（`gravity_scale_calibration.py`）真机标定时：

1. full 模式 L2/L3 单关节扫掠用对称 `±0.8 rad`，但 home 是「半抬」位姿（L2=0.785、L3=-0.785），越界后姿态被 `clamp_pose` 压回限位边缘（贴 0.15 边距），采样点退化、贴近行程极限。
2. full 首跑网格含「大 L2 + L3 接近伸直」组合（L2=1.285、L3=-0.285），末端下探撞桌面，该点实测力矩被桌面反作用力主导，`tau` 与模型重力 `g` **反号**，污染最小二乘拟合——L3 的 R² 从 0.879 崩到 0.463、RMSE 从 0.185 涨到 0.660，整组结果劣于 quick。

## 根因

- home 对 L2/L3 不对称：L2 下限=0、L3 上限=0，二者距极限仅 0.785 rad，对称 `±0.8` 必然越界（clamp 兜底保安全，但采样点不按设计意图）。
- 网格里「L2 大 + L3 接近 0（伸直）」会让末端过度伸展，超出当前安装环境物理空间，夹爪/末端碰桌面；接触力让电机维持力矩不再等于重力负载，平衡方程 `τ_motor = g` 失效。
- 纯位置阈值 `|pose - target|` 会被振动/过冲误触发，不能可靠判接触。

## 正确做法 / 规避

1. **扫掠幅度按关节实际行程非对称设计**：L2 只能往正、L3 只能往负，full 单扫用 `[-0.6, -0.3, 0.3, 0.6]`（均留 ≥0.185 rad 余量，不触发 clamp）。
2. **网格 L3 正向（往伸直）限幅**：L3 网格取 `{-1.285, -0.785, -0.485}`，避开「大 L2 + L3 伸直」的碰撞姿态。
3. **接触污染用符号判据剔除，不用位置阈值**：对激励足够（`|g| > 0.2 Nm`）的关节，`tau·g < 0` 即判外力污染、跳过该 pose。振动/摩擦只加噪声不会让重力力矩反号，所以不误伤正常点。
4. **标定结果写包内**（`src/a3_description/config/gravity_scales.yaml`，版本控制、多机 `git pull` 同步），`~/.a3` 仅作向后兼容回退；工具注册为 `ros2 run a3_bringup gravity_scale_calibration` 而非裸跑 scripts/。

## 相关路径

- `scripts/gravity_scale_calibration.py`（源，与包内副本保持一致）
- `src/a3_bringup/a3_bringup/gravity_scale_calibration.py`（ros2 run 入口）
- `src/a3_bringup/setup.py`（console_scripts 注册）
- `src/a3_description/config/gravity_scales.yaml`（标定输出）
- `src/a3_bringup/launch/a3_bringup.launch.py`（auto-load + gravity_torque_node 前馈接入 tau_scale）
