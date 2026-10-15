# F174 — 6 轴 + 连杆形态：独立描述包 `a3_description_6j` + 系统级模型配置（默认档接入真机/仿真栈）

- **说明：** 机械形态从「7 轴（L1–L6 + L7 夹爪电机）」改为「**6 轴 + 末端连杆**」：
  L7 电机取消；轴 6（L6 法兰输出）后接一根**连杆**作为末端，相对轴 6 偏移 `(0, 0, -0.14)` m
  （14 cm 打印件，待实测/STP 修正），TCP（`end_effector`）位于连杆末端；连杆尚无 STP，
  先用 **φ8mm × 140mm 圆柱**占位（`radius=0.004`、`length=0.14`、占位质量 `0.02 kg`）；
  去掉的只有**夹爪**（`jaw.stl`，属 L7 输出）；轴 6 法兰处那台 **EL05 电机壳体 `el05_l6.stl`
  （φ46×106mm，一台）保留**（2026-10-15 对照实物确认仍在，连杆与其共轴穿出）。
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
     无 `L7_joint`/`gripper_link`；轴 6 后有 `linkage_joint`（fixed，origin `0 0 -0.14`）→ `linkage_link`
     （圆柱占位）→ `end_effector`（TCP）；Pinocchio 载入 `nq=6`
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
    - `urdf/el_a3.urdf.xacro`：无 `include_gripper` 分支、无 L7；新增 `linkage_link`
      （`<inertial>` 圆柱惯量 + `visual`/`collision` cylinder）+ `linkage_joint`（fixed，父
      `l5_l6_urdf_asm`，origin `0 0 $(arg linkage_offset_z)`，默认 `-0.14`）；`end_effector_joint`
      挂连杆末端（TCP）；圆柱几何/质量由 xacro arg 参数化（`linkage_offset_z` / `linkage_radius` /
      `linkage_length` / `linkage_mass`），换 STP 只改这几处
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

- **验证记录（2026-10-15，WSL mock 栈，`ROS_DOMAIN_ID=77`）：**
  `hardware:=mock`（不传任何档位参数）起栈后：`/joint_states` 关节数 = **6**；
  `a3_arm_controller ready: joints=6 state=IDLE`；`zero_torque_controller: configured: 6 joints, pinocchio nq=6`；
  `gravity_torque_node: Pinocchio model nq=6 ... urdf=install/a3_description_6j/share/.../el_a3.urdf`；
  `ros2 control list_controllers` = joint_state_broadcaster(active) + arm_controller + zero_torque_controller
  （**无 gripper_controller**）；`switch_controllers=['arm_controller']`；xacro 在 `use_mock_hardware` /
  `use_real_hardware` / `linkage_offset_z` 覆盖三种参数下均解析出 6 个可动关节 + 固定连杆。

- **未覆盖项（有意）：**
  1. **连杆几何/质量待实测**：当前为 φ8×140mm 圆柱 + 0.02 kg 占位；STP 到位后替换
     `linkage_radius` / `linkage_length` / `linkage_mass` 与几何即可（TCP 偏移随之修正）。
     注意连杆与腕部 EL05 电机壳体（0…106mm）共轴，实测后按实际装配修正 `linkage_offset_z`；
     模型相对 7 轴原形态**只少夹爪 `jaw.stl`**（`el05_l6.stl` 电机壳体保留）
  2. **标定文件即该形态标定值（无需重标）**：新包 `inertia_params.yaml` / `gravity_scales.yaml` /
     `friction_params.yaml` / `kp_kd_gains.yaml` 沿用现有 6 关节标定数据——该批标定（F169/F170）
     **本就是在装好连杆的状态下做的**（用户 2026-10-15 确认）。仅当连杆换 STP 后实测质量/几何
     与占位（0.02 kg、φ8×140mm）差异明显时，再复核 `calibrate model|scale`
  3. **MoveIt `moveit_controllers.yaml` 仍列 `gripper_controller`**（F168 已知项，日志一条
     "Added GripperCommand controller"，无功能影响）；`el_a3_6j.srdf` 未登记 `linkage_link` 的
     adjacent 碰撞对（连杆为圆柱、不与自身碰撞，暂无影响）
  4. **真机未上机验证**：本次仅 mock 栈验证；上板需确认 CAN 上只剩电机 1..6、`motor_map_6j.yaml`
     与 power_sequence 档位一致

- **状态：** `implemented-pending-hw`（2026-10-15；独立包 + 圆柱连杆 + 系统级配置 + 真机/仿真默认档接入完成，mock 栈验证通过）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
