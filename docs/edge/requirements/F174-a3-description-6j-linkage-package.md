# F174 — 6 轴 + 连杆形态：独立描述包 `a3_description_6j` + 系统级模型配置（默认档接入真机/仿真栈）

- **说明：** 机械形态从「7 轴（L1–L6 + L7 夹爪电机）」改为「**6 轴 + 末端连杆件**」：
  L7 电机取消；轴 6（L6 法兰输出）后接**真实末端件**（CAD 导出 `el05_l6.stl`：法兰盘 + φ46 段
  + φ10 细杆，贴法兰面伸出 **150 mm**），TCP（`end_effector`）在细杆端面（z = −0.150 m）。
  去掉的只有**夹爪**（`jaw.stl`，属 L7 输出）；旧电机壳网格留档 `meshes/el05_l6_bk.stl`。
  三件事一起落地：
  1. **独立描述包** `a3_description_6j`（原 7 轴包 `a3_description` 零改动、与主仓库逐字一致），
     6 轴 + 连杆的 URDF 与配置在此独立演进——F168 的「同包 `include_gripper` + `*_6j` 后缀切换」
     改为**整包隔离**，避免改 6 轴误伤 7 轴；
  2. **系统级模型配置** `a3_robot_model`（`config/robot_model.yaml`，可被 `~/.a3/robot_model.yaml`
     字段级覆盖）指定**默认描述包**与配套档位文件（SRDF / controllers / motor_map / joint_names），
     真机栈与仿真栈读同一份，替代各处写死 `a3_description`；
  3. **真机栈与仿真栈默认切到该描述**（`include_gripper` 默认跟随系统配置）。

- **验收标准：**
  1. `a3_description_6j` 可独立构建；xacro 解析出恰 6 个可动关节（`L1_joint`..`L6_joint`），
     无 `L7_joint`/`gripper_link`；轴 6 法兰后 `end_effector_joint`（fixed，`0 0 -0.150`）→ `end_effector`
     （TCP 在真实末端件细杆端面）；Pinocchio 载入 `nq=6`
  2. `python3 -m a3_robot_model.robot_model` 打印默认档：`description_package=a3_description_6j`、
     `joint_names` 6 个、`include_gripper=false`、`srdf=el_a3_6j.srdf`、`motor_map=motor_map_6j.yaml`；
     `~/.a3/robot_model.yaml` 可字段级覆盖
  3. **真机/仿真统一入口** `a3_bringup.launch.py`（不传参数）起栈：`/joint_states` 恰 `L1..L6`、
     `arm_controller ready: joints=6`、`zero_torque configured: 6 joints, pinocchio nq=6`、
     `gravity_torque ... urdf=<a3_description_6j>/urdf/el_a3.urdf`、控制器列表无 `gripper_controller`
  4. `a3_stack.sh` 默认档取自系统配置（当前 6 轴）；`--7j` 可回到 7 轴形态（`a3_description` +
     `el_a3.srdf` + `motor_map.yaml` + L7）
  5. 原包回归：`src/a3_description` 与 HEAD/主仓库一致（本次未改，且已回退此前的未提交差异）

- **实现：**
  - **新包** `src/a3_description_6j/`（整包隔离：`urdf/` + `config/` + `meshes/` + `launch/`）
    - `urdf/el_a3.urdf.xacro`：无 `include_gripper` 分支、无 L7；
      `l5_l6_urdf_asm` 的 visual/collision = **真实末端件 `el05_l6.stl`**，用 `scale` + `origin` 摆位
      （该 CAD mesh 单位 mm、坐标系外置：件自轴沿 mesh x 位于 (y,z)≈(0.5,135.5)mm，贴合面 = mesh x=−42.15mm；
      URDF 参数 `linkage_mesh_scale` / `linkage_mesh_origin` / `linkage_mesh_rpy`）；
      `end_effector_joint`（fixed，`0 0 linkage_tcp_z`，默认 −0.150）挂 TCP；
      早期版本用的 **圆柱占位连杆 `linkage_link` 已删除**（末端几何由真实件承担）
    - `urdf/el_a3_ros2_control.xacro`：只声明 L1–L6，参数路径指向本包
    - `urdf/el_a3.urdf`：由 xacro 生成的静态 URDF（Pinocchio / 限位读取）
    - `config/`、`launch/` 6 轴化：`el_a3_controllers.yaml` 用无夹爪版（删 `gripper_controller` 与
      `zero_torque_controller` 的 L7）、删 `gripper_{position,effort}_plugin.yaml` 与冗余
      `el_a3_controllers_6j.yaml`、`named_poses.yaml`/`gravity_scales.yaml`/`friction_params.yaml`/
      `master_slave_config.yaml`/`multi_arm_controllers.yaml`/`el_a3_dual_view.rviz` 去 L7、
      `launch/` 去 gripper spawner
  - **新包** `src/a3_robot_model/`：`config/robot_model.yaml`（默认档）+ `a3_robot_model/robot_model.py`
    （`load()` / `description_share()` / `urdf_path()` / `xacro_path()` / `controllers_path()` /
    `joint_names()` / `include_gripper()`；`~/.a3/robot_model.yaml` 覆盖；`python3 -m ... --get KEY`
    供 shell 用）+ README
  - `src/a3_bringup/launch/a3_bringup.launch.py`：`include_gripper` 默认值改为「留空=跟随系统配置」；
    xacro / controllers / SRDF / motor_map / 编排层 `joint_names`+`switch_controllers` 全部取自
    `robot_model`；无夹爪形态不加 gripper 插件覆盖、不 spawn `gripper_controller`
  - `src/a3_bringup/a3_bringup/`：`gravity_torque_node`（关节名/URDF/标定文件）、`move_to_pose_ik_node`、
    `zero_to_ready_publisher`、`gravity_scale_calibration`（关节集合、输出路径、tau_scale 长度）改读系统配置
  - `src/a3_arm_controller/a3_arm_controller/arm_controller.py`：新增 `model_description_package()` /
    `model_urdf_path()` / `model_include_gripper()`；限位读取、Pinocchio 载荷模型、`named_poses_pkg`
    （`arm_controller.yaml` 置空=跟随配置）、`switch_controllers` 默认值随形态
  - `src/a3_bringup/launch/*`（其余 12 个）+ `src/a3_moveit_config/launch/*` + `src/a3_cloud_edge/launch/*`：
    描述包/URDF/controllers 路径来源改为 `robot_model` API；三个 `package.xml` 增 `a3_robot_model` 依赖
  - `scripts/a3_stack.sh`：默认档从 `robot_model.yaml` 读取（`--6j` / `--7j` 可覆盖），帮助文本更新
  - `src/a3_description`：**不改动**（并回退此前的未提交空白差异，与主仓库一致）

- **验证记录：**
  - **mock 栈（2026-10-15，`ROS_DOMAIN_ID=77`）**：`hardware:=mock`（不传任何档位参数）起栈后
    `/joint_states` 关节数 = **6**；`a3_arm_controller ready: joints=6 state=IDLE`；
    `zero_torque_controller: configured: 6 joints, pinocchio nq=6`；
    `gravity_torque_node: Pinocchio model nq=6 ... urdf=install/a3_description_6j/share/.../el_a3.urdf`；
    `ros2 control list_controllers` = joint_state_broadcaster(active) + arm_controller + zero_torque_controller
    （**无 gripper_controller**）；`switch_controllers=['arm_controller']`；xacro 在 `use_mock_hardware` /
    `use_real_hardware` / `linkage_mesh_origin` 覆盖三种参数下均解析出 6 个可动关节 + 固定末端。
  - **真机（2026-10-16，can1 + 6 电机在线）**：电机探测 6/6（ID 1–6）→ 自动 6 关节档；
    `power_sequence F52 档位：6 关节`；`arm_controller ready: joints=6`；`/joint_states` = L1–L6；
    控制器无 `gripper_controller`；CAN `ERROR-ACTIVE`（berr-counter 0/0）。
    首次实测发现 MoveIt 起点无效（连杆碰撞对未登记，方形长按 PTP 回放 error 99999），
    补 SRDF 后 `moveit_msgs/GetStateValidity → valid=true`、`/plan_kinematic_path` 到路点第 1 点
    `error_code=1 (SUCCESS)`

- **未覆盖项（有意）：**
  1. **末端件质量待复核**：几何已是真实 CAD STL（150 mm）；`l5_l6_urdf_asm` 惯量沿用腕部组件
     标定值（含末端件，F169/F170 标定即该形态实测）。若实测质量差异明显，再跑
     `calibrate model`；换件或重导出 CAD 后按新 mesh 的 bbox/自轴/贴合面重算
     `linkage_mesh_origin` / `linkage_mesh_rpy` / `linkage_tcp_z`（方法与陷阱见
     [LL-166](../../lessons_learned/LL-166-new-mesh-mm-units-foreign-frame.md)）
  2. **标定文件即该形态标定值（无需重标）**：新包 `inertia_params.yaml` / `gravity_scales.yaml` /
     `friction_params.yaml` / `kp_kd_gains.yaml` 沿用现有 6 关节标定数据——该批标定（F169/F170）
     **本就是在装好连杆的状态下做的**（用户 2026-10-15 确认）。仅当连杆换 STP 后实测质量/几何
     与实测值差异明显时，再复核 `calibrate model|scale`
  3. **MoveIt `moveit_controllers.yaml` 仍列 `gripper_controller`**（F168 已知项，日志一条
     "Added GripperCommand controller"，无功能影响）。
     SRDF 碰撞对：末端件挂在 `l5_l6_urdf_asm` 本体后，`l5_l6_urdf_asm↔end_effector`（Adjacent）
     与 `part_9↔end_effector`（Never）已够；此前圆柱占位件 `linkage_link` 的三条登记已随占位件删除
     （真机 2026-10-16 实测：不登记时 MoveIt 判起点无效、所有走 move_group 的动作 `error 99999`，
     见 [LL-165](../../lessons_learned/LL-165-fixed-link-needs-srdf-disable-collisions.md)）
  4. **真机操作层验证进行中**：起栈/使能/规划已通过（见上）；示教回放、巡游、servo jog 等
     依赖 move_group 的动作需按操作员流程逐个复测（操作手册见 `QUICKSTART` / `PS4_OPERATOR_GUIDE`）

- **状态：** `implemented-pending-hw`（2026-10-15/16；独立包 + **真实末端件 STL** + 系统级配置 +
  真机/仿真默认档接入完成，mock 栈验证通过；真机已起栈、修掉末端件 SRDF 碰撞对导致
  `error 99999` 的问题，操作层复测中）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
