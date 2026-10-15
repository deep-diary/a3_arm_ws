# F174 — 独立描述包 `a3_description_6j`（6 轴 + 连杆形态，配置隔离）

- **说明：** 机械形态从「7 轴（L1–L6 + L7 夹爪电机）」改为「**6 轴 + 连杆**」：L7 电机取消，
  轴 6（L6 法兰输出）后接一根**固定连杆**作为末端，相对轴 6 偏移 `(0, 0, -0.014)` m
  （-14 mm 为初步估算，待实测修正），TCP（`end_effector`）移到连杆末端。
  此前 6 关节只有 F168 的「产品栈原生 6J 档」：**同一个 `a3_description` 包**里按
  `include_gripper:=false` + `*_6j` 后缀文件切换，URDF/config/SRDF/motor_map **共用一套**，
  改 6J 容易误伤 7J。本需求把 6 轴形态**整包隔离**到独立描述包 `a3_description_6j`：
  7 轴原包 `a3_description` 零改动，6 轴 + 连杆的 URDF 与配置独立演进。

- **验收标准：**
  1. `src/a3_description_6j/` 为可独立构建的 ROS 2 包（`package.xml`/`CMakeLists.txt`/`meshes`/`config`/`launch`/`urdf` 齐备），`colcon build --packages-select a3_description_6j` 通过
  2. xacro 解析后关节恰为 6 个可动关节（`L1_joint`..`L6_joint`），**无** `L7_joint`/`gripper_link`
  3. 轴 6 后存在固定连杆：`linkage_joint`（fixed，父 `l5_l6_urdf_asm`，origin `0 0 -0.014`）→ `linkage_link`；`end_effector` 挂在连杆末端（TCP）
  4. Pinocchio 从新包静态 URDF 加载 `nq=6`；`base_link → end_effector` 链路完整
  5. 原包回归不受影响：`src/a3_description` 逐字不变（7 轴默认档照旧）

- **关联：** F168（产品栈原生 6 关节档，本需求的前置形态）、F169（标定 6J 适配）、
  F162（弹琴 app，6 关节臂为主）、[LL-157](../../lessons_learned/LL-157-product-6j-profile-include-gripper.md)（`include_gripper` 档位切换踩坑）

- **实现：**
  - `src/a3_description_6j/package.xml`、`CMakeLists.txt`：包名 `a3_description_6j`（install `urdf meshes launch config`）
  - `src/a3_description_6j/urdf/el_a3.urdf.xacro`：去 `include_gripper` 分支与 L7/`gripper_link`；
    新增 `linkage_link` + `linkage_joint`（fixed，父 `l5_l6_urdf_asm`，origin `0 0 $(arg linkage_offset_z)`，
    arg 默认 `-0.014`），`end_effector_joint` 改挂连杆末端（TCP）；
    包内引用改 `package://a3_description_6j/...` / `$(find a3_description_6j)/...`
  - `src/a3_description_6j/urdf/el_a3_ros2_control.xacro`：删 L7 关节声明与 `include_gripper` 形参；参数路径指向新包
  - `src/a3_description_6j/urdf/el_a3.urdf`：由新 xacro 生成的静态 URDF（Pinocchio/限位读取用）
  - `src/a3_description_6j/config/*`、`meshes/*`：整包复制，6 轴形态后续在此独立演进
  - `src/a3_description_6j/launch/*.py`：`FindPackageShare` 改指新包
  - `src/a3_description`：**不改动**

- **未覆盖项（有意，待接线时处理）：**
  1. **消费方仍指向原包**：`a3_bringup`（launch / `gravity_torque_node` / `move_to_pose_ik_node`）、
     `a3_arm_controller`（限位读取）、`a3_moveit_config`（SRDF/xacro）、`a3_can_bridge`（motor_map）、
     `scripts/*.py` 目前都写死 `a3_description` / `el_a3.urdf`，新包尚未接入任何 launch 或档位选择
     （未做 `a3_stack.sh` 第三档）
  2. **连杆几何待实测**：`linkage_link` 暂无 mesh / 碰撞体 / 惯量（缺 `inertial` 视为零质量，不参与重力模型），
     偏移 `-0.014` 为初步估算
  3. **配套件未做 6 轴化**：`config/el_a3_controllers.yaml`（仍含 `gripper_controller`/L7）、
     `launch/el_a3_control.launch.py`（仍 spawn `gripper_controller`）、SRDF、`motor_map`、
     `inertia_params.yaml` / `gravity_scales.yaml` / `kp_kd_gains.yaml` 均为原样复制，未按新形态清理或重标定

- **状态：** `partial`（2026-10-15；独立包复制 + URDF 6 轴 + 连杆改造完成，未接 launch、未上真机）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
