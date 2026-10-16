# F175 — 末端位姿话题 `/a3/ee_pose`（base_link → end_effector 的 FK 直读）

- **说明：** 联调/复核时常要"读一个话题就拿到末端位姿"——例如把臂挪到某个位姿后核对
  「末端触地 ⇒ TCP 的 base_link z ≈ 底座抬高量（负值）」。此前**没有**这样的话题，只能
  `ros2 run tf2_ros tf2_echo base_link end_effector`（脚本里不方便、且依赖 TF 链路）。
  本需求新增一个轻量节点 `a3_bringup/ee_pose_publisher`：对**系统级模型配置**（`a3_robot_model`）
  指定的静态 URDF 做 Pinocchio FK（与 `gravity_torque_node` / 编排层同一份模型），
  订阅 `/joint_states`，按频率发布 `geometry_msgs/PoseStamped`。

- **验收标准：**
  1. 起栈（真机 `a3_stack.sh start` 或仿真 `--sim`）后存在话题 `/a3/ee_pose`
     （`geometry_msgs/PoseStamped`，`header.frame_id = base_link`），默认 20 Hz
  2. 该话题数值与 `tf2_echo base_link end_effector` 一致（同一 FK，误差 < 1e-6 m）
  3. 物理核对：末端贴桌面 + 底座抬高约 7 mm 时，发布的 `pose.position.z ≈ -0.007`（实测 -6.8~-7.0 mm）
  4. 可运行时改 `ee_frame`（如 `l5_l6_urdf_asm`）与 `rate_hz`；`/joint_states` 陈旧（>0.5 s）
     时不发布并告警，不发出错误位姿

- **实现：**
  - `src/a3_bringup/a3_bringup/ee_pose_publisher.py`：新节点（参数 `joint_states_topic` /
    `pose_topic` / `ee_frame` / `rate_hz` / `urdf_path`；URDF 默认取 `a3_robot_model` 静态 URDF）
  - `src/a3_bringup/setup.py`：注册 `ee_pose_publisher` 入口
  - `src/a3_bringup/launch/a3_bringup.launch.py`（真机/仿真统一入口）与
    `src/a3_bringup/launch/edge_web_sim.launch.py`（仿真底座）：随栈启动该节点

- **关联：** [LL-166](../../lessons_learned/LL-166-new-mesh-mm-units-foreign-frame.md)（新 mesh 摆位与 TCP 物理核对法）、
  F49/F108（重力前馈同样依赖"模型 + /joint_states"，本节点与其同源）

- **状态：** `implemented-pending-hw`（2026-10-16；真机 6 轴 + 真实末端件验证：
  `/a3/ee_pose` z 与 `tf2_echo` 一致、末端触地时 z ≈ -6.8 mm）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
