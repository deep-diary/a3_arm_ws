# F68 — 示教回放走 Ruckig/TOTG 在线重定时（保几何、退役手搓平滑）


- **说明：** 现状回放对点列做中心滑动平均（`_smooth_points`）+ 弧长重采样/加速度限时长（`_time_warp_points`）——手搓链路复杂、仍非连续加加速度，用户评价"各种折腾，效果反而不好"。改为工业标准：**几何保持的重定时（re-timing）**。50 Hz 稠密录制点不得交 OMPL 重规划（几何路径会变），只重算时间/速度/加速度：新增 C++ 包 `a3_trajectory_processing`（moveit_core `trajectory_processing` Python 不可绑定，无 moveit_py），服务 `/a3/arm/retime_trajectory`（`a3_msgs/srv/RetimeTrajectory`：输入 `trajectory_msgs/JointTrajectory`（仅位置）+ backend `ruckig|totg` + v/a 缩放；输出重定时 JointTrajectory，含 velocities/accelerations/稠密 time_from_start）。默认 backend=**Ruckig**（jerk-limited，起止零速、加加速度有界，最丝滑；系统已装 ros-humble-ruckig），TOTG 备选。限位取自 `joint_limits.yaml`（launch 注入 v/a map，jerk 默认 5×accel）。**关于录制速度：不需要保存**——Ruckig/TOTG 只吃位置点 + 关节限位，速度/加速度/加加速度全部由算法重算；已存 yaml 的 `time_from_start_sec` 时间戳在重定时中丢弃。回放流程：载入点列 → 末端拼接「当前位→首点」短 ramp（同样送重定时，保证整段连续）→ retime 服务 → 下发；**L7 夹爪排除在重定时外**——retime 节点组从 `arm_with_gripper` 改为 `arm`（L1–L6），L7 用原始录制时间戳单独下发（GripperCommand 抢占式步进，20ms + 相邻点线性插值 + ≤0.6 rad/s 硬限速），夹爪无需 Ruckig。新参数：`playback_retime: true`（false 时保留旧 smooth/time_warp 作兜底，默认走新链路；旧参数保留不删）。
- **验收标准：**
  1. 示教一段任意轨迹（含变速/停顿）→ Square 回放：逐关节几何路径与录制点一致（重采样后位置偏差 ≤ 0.01 rad），但时间曲线重排：起止速度 ≈ 0、|a| ≤ joint_limits×缩放、jerk 有界
  2. retime 服务单测（ros2 service call）：输入线性/折线点列 → 返回带 velocities/accelerations 的轨迹，总时长随 v_scaling 单调变化
  3. ruckig 失败自动降级 totg；retime 节点/服务不可用 → 回退旧链路（WARN），回放不中断
  4. 录制 yaml 仍只存 positions + time_from_start_sec（格式不变，旧文件可直接回放）
- **关联：** F67（goto 同体系）、F22（示教/回放门面）、F38b（ramp 段）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；LL-071
- **状态：** `done（仿真）`（2026-09-22，14/14 ALL PASS：Ruckig 回放 4.41s vs 录制 4.0s，101/101 几何点匹配、最大偏差 0.0034 rad、端点误差 0.0001 rad，无超速/超加速；旧链路几何 0.0045 rad 作回归对照；Ruckig 单步 jerk 绑定致 29.6s 拉伸的根因与修复见 LL-071 坑 5；真机验收待上电。2026-10-04：retime 排除 L7——组改 `arm`(L1–L6)，L7 原始时间戳 + 20ms 线性插值 + ≤0.6 rad/s 限速。2026-10-05 真机：backend 默认切 `totg`——ruckig 对去噪/均匀时间轨迹仍拉长（14.65s→61s），totg 匹配录制时长且 JTC splines 兜最终平滑；`playback_retime_backend: totg`、`velocity/acceleration_scaling: 0.5`）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
