# F94 — 两点轨迹统一速度限幅（URDF velocity 地板时长；堵住绕过规划器的无限速 jog/park）


- **说明：** MoveIt goto / playback 路径由 OMPL + TOTG/Ruckig 强制速度/加速度限幅，但编排层所有「两点轨迹」（`_two_point_trajectory`：web set_joint_positions jog、set_jog、disable safe-park、F40 park 等 6 处）是直接把 2 点（端点 v=a=0）丢给 JTC spline，**没有任何速度上限**：web 端 duration 最小 0.05 s，若 Δq 大（如 1 rad / 0.05 s = 20 rad/s），JTC VARIABLE_DEGREE_SPLINE 生成的峰值速度可超过关节 URDF `velocity` 限（33/50 rad/s 是电机空载极限，不是安全作业速度）。工业做法：**限幅只有一个权威来源（URDF joint limit velocity，MoveIt joint_limits.yaml 与之同源）**，所有下发路径（规划路径由 TOTG 保证、原始两点路径由编排层地板时长保证）都不得超过同一限值。
  - `_load_joint_limits()` 同时解析 URDF 每关节 `velocity`（新增 `self._joint_vel_limits: Dict[str,float]`；零/缺失不采用，保持原有位置限加载行为）
  - 新参数 `joint_velocity_scale: 1.0`：现场收紧余量的统一旋钮（0.5 = 全路径减半；规划路径如需同步收紧调 MoveIt joint_limits.yaml），默认 1.0 = 不改变规划路径现状、只堵「无限制」漏洞
  - 新辅助 `_velocity_floor_duration(q0, q1, joint_names)` = `_SPLINE_PEAK_FACTOR × max_i |Δq_i| / (vmax_i × scale)`（未知关节名跳过；Δq=0 为 0）。**形状系数来源**：JTC VARIABLE_DEGREE_SPLINE 两点（端点 v/a=0）实测峰值/平均速度≈2.0–2.09（mock 全栈、T=0.5/1.0 s、200 Hz 微分数得），取 `_SPLINE_PEAK_FACTOR=2.2` 留 ~5% 余量；topic 线性后端按此只更保守
  - **集中在 `_two_point_trajectory()` 内强制** `duration = max(duration, floor)`——6 个调用点全覆盖、零漏网；扩展时 WARN 节流日志（关键字 `F94 duration extended`，含原值/地板值/最慢关节）
  - 响应语义：jog 服务响应回显实际采用时长（`/a3/arm/set_joint_positions` message 已含 duration；jog 定时器在 jog 调用点同样用地板后的时长，避免 back-to-ready 提前触发）
- **改动：**
  1. `src/a3_arm_controller/a3_arm_controller/arm_controller.py`：URDF velocity 解析、`joint_velocity_scale` 参数、`_velocity_floor_duration()`、`_two_point_trajectory()` 集中强制 + 日志；jog 调用点计时同步
  2. 新增 `scripts/a3_test/f94_velocity_limit_acceptance.py`：mock 全自动验收（不触碰 CAN）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f94_velocity_limit_acceptance.py`）：**
  1. 7 关节 URDF velocity 全部加载（L1–L3=33、L4–L7=50 rad/s）
  2. 超限请求：L1 Δq=3.0 rad、duration=0.05 s（vmax=33 → 地板 2.2×3.0/33=0.20 s）→ 响应回显 duration ≥ 0.20 s；从 `/joint_states` 中心差分实测运动窗口内 L1 峰值速度 ≤ vmax×scale（容差 5%；中心差分容忍 DDS 成批投递的早到帧，前向差分实测有 2 倍假峰）
  3. 保守请求（Δq=0.1 rad、duration=2.0 s）→ duration 不被修改（=2.0 s）
  4. `joint_velocity_scale:=0.5` 重测同超限请求：地板翻倍（0.40 s）、实测 L1 峰值速度 ≤ 0.5×33（+容差）
  5. 回归：safe-park（disable 回 idle）正常完成、最终位姿在 idle 容差内
- **关联：** F88（两点标准轨迹取代手搓稠密插值）、F40/F41（park/move_to 时长语义）、[shared/SAFETY.md](../shared/SAFETY.md)；MoveIt joint_limits.yaml 与 URDF 必须同源（本需求不改二者数值）
- **状态：** completed（2026-09-23，mock 全栈验收 11/11：`scripts/a3_test/f94_velocity_limit_acceptance.py`。实测：7 关节速度限加载（L1–L3=33、L4–L7=50）；L1 Δq=3.0/0.05s → 回显 duration=0.200s、中心差分峰值 28.87 rad/s；保守 Δq=0.1/2.0s → 2.000s 不变；scale=0.5 → 0.400s、峰值 14.94 rad/s；disable safe-park 成功、最终 worst=0.010 rad。形状系数 2.2 与中心差分的踩坑见 LL-106）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
