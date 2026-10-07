# F160 实现计划 — MIT 位置环速度/加速度前馈 + 力矩分量分解遥测

## 1. 摘要

把 F108 的「只重力前馈」扩展为工业标准 computed-torque 全模型前馈：

```
τ = kp·(q_des−q_meas) + kd·(q̇_des−q̇_meas) + M(q)·q̈_des + C(q,q̇_des)·q̇_des + G(q)
```

- **速度前馈 VFF**：MIT 帧 velocity 域填 `q̇_des`（来自 JTC splines 的 velocity 命令接口）。
- **加速度前馈 AFF + 科氏/离心**：`rnea(q_meas, q̇_des, q̈_des)` 一次性算 `M·a+C·v+G`，并入 t_ff。
- 新增 `feedforward_mode`（`gravity`|`full`），默认 `gravity` 保底，`full` 真机验证后启用。
- 同时把力矩分量分解（重力 / 科氏+离心 / 惯量 AFF / 总前馈 / PD 反馈 / 总指令）加到 `/a3/arm_status` 便于记录分析。

关键结论（探索确认）：
- 硬件插件 [a3_mit_hardware_interface.cpp](file:///home/cat/a3_arm_ws/src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp) 已导出 velocity 命令接口（`cmd_vel`）但 `write()` 位置帧 velocity 域硬编码 0、t_ff 只算 `rnea(q,0,0)`（F108）。
- 未导出 acceleration 命令接口，需新增（用户已确认走 JTC acceleration 命令接口方案）。
- JTC（Humble）头文件确认支持 `acceleration` 命令接口，`interpolation_method: splines`（=VARIABLE_DEGREE_SPLINE）会写出 q̈_des。
- `el_a3_controllers.yaml` 被 mock/can 共用；mock `GenericSystem` + `calculate_dynamics` 不支持同时 claim position+velocity（文件内注释已写明），因此必须按 mock/can 拆分控制器命令接口配置。
- `/a3/arm_status` 的 `efforts` 是实测关节力矩（来自 /joint_states），无分量分解；前馈在 C++ 插件算，`/a3/arm_status` 在 Python `arm_controller` 发，跨进程需经新 topic 中转。

---

## 2. 现状分析（已读文件）

| 文件 | 现状 |
|---|---|
| `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp` | `JointMapping` 有 `cmd_pos/cmd_vel/cmd_eff` 无 `cmd_acc`；`export_command_interfaces` 导出 position/velocity/effort；`write()` 位置分支 velocity 域恒 0、t_ff=`rnea(q,0,0)`×ratio×scale×direction；`on_init/on_configure` 有 `gravity_feedforward_ratio`/`use_pinocchio_gravity` 参数与 set_parameters 回调（LL-126 保留句柄） |
| `src/a3_description/urdf/el_a3_ros2_control.xacro` | 真机块声明 `gravity_feedforward_ratio`/`use_pinocchio_gravity`；每关节 `<command_interface>` 有 position/velocity/effort，`<state_interface name="temperature">` 已用 `use_real_hardware` 条件包裹 |
| `src/a3_description/config/el_a3_controllers.yaml` | `arm_controller.command_interfaces: [position]`、`state_interfaces: [position, velocity]`、`interpolation_method: splines`；gripper/zero_torque 不变 |
| `src/a3_description/config/gripper_effort_plugin.yaml` | 仅覆盖 `controller_manager.ros__parameters.gripper_controller.type`——证明 rcl 参数文件后者可深合并覆盖嵌套键 |
| `src/a3_bringup/launch/a3_bringup.launch.py` | 主产品入口，`hardware:=mock|can`；控制器用 `el_a3_controllers.yaml`；`gripper_plugin_file` 是条件 ParameterFile 覆盖样板 |
| `src/a3_bringup/launch/edge_ros2_control_vcan.launch.py` | F108/F72 vcan 验收入口，恒 `use_real_hardware:=true`，直接用 `el_a3_controllers.yaml` |
| `src/a3_msgs/msg/ArmStatus.msg` | 已有 `efforts`（实测）等字段，无前馈分量 |
| `src/a3_msgs/CMakeLists.txt` | `rosidl_generate_interfaces` 注册 msg |
| `src/a3_arm_controller/a3_arm_controller/arm_controller.py` | `_build_status_msg()` 组装 ArmStatus；`_on_js` 从 /joint_states 填 efforts；订阅用 best_effort `js_qos` |
| `src/a3_hardware_interface/package.xml` + `CMakeLists.txt` | 已 depend `a3_msgs`，可直接用新 msg |
| `scripts/a3_test/f108_gravity_ff_vcan_acceptance.py` | vcan 数值验收样板（CanSniffer/Recorder/send_jtc/pinocchio 参考模型） |

---

## 3. 变更清单（按文件）

### 3.1 硬件插件（核心）

**`src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`**

1. `JointMapping` 增加 `double cmd_acc{0.0};`
2. `export_command_interfaces()`：追加 `acceleration`（`j.name, "acceleration", &j.cmd_acc`），reserve ×4。
3. `on_init`：
   - 新增 `feedforward_mode_ = GetParam(hp, "feedforward_mode", "gravity")`；非法值回落 `"gravity"`。
4. `on_configure`：
   - `declare_parameter("feedforward_mode", feedforward_mode_)`（string）。
   - set_parameters 回调增加 `feedforward_mode` 分支（`PARAMETER_STRING`，取值 `gravity`/`full` 才写成员，否则忽略保持原值）。
5. `on_activate` / `perform_command_mode_switch` 位置重锚处：把 `j.cmd_acc = 0.0` 一并清零（与 `cmd_vel=0.0` 并列）。
6. `InitGravityModel()`：预分配成员 `v_des_ = Eigen::VectorXd::Zero(model_.nv)`、`a_des_ = ...`（避免 write() 内每周期分配）。
7. `write()` 位置模式分支（当前 929–998 行）重构：
   - `const bool full_ff_active = gravity_ff_active && feedforward_mode_ == "full";`
   - 在 `fb_mutex_` 内：拼 `q_`（现有）+ 拼 `v_des_`/`a_des_`（full 时用 `v_index_[i]` 写入 `cmd_vel`/`cmd_acc`）。
   - 前馈力矩：`full` 用 `rnea(model_, data_, q_, v_des_, a_des_)`；`gravity` 用现有 `rnea(model_, data_, q_, v_zero_, a_zero_)`。
   - 位置帧 velocity 域：`full_ff_active ? (j.direction * j.cmd_vel) : 0.0`（VFF，motor 坐标 = direction×joint，无 offset）。
   - t_ff 公式不变：`clamp(gravity_ff_ratio_ * ff_scale_[i] * ff_tau[v_index_[i]], ±torque_max) * j.direction`（`gravity_ff_ratio_` 作为整体前馈比例，full 下作用于 rnea 全模型输出）。
   - effort 分支、soft-start、freeze-hold、on_activate/on_deactivate 零指令帧一律保持 velocity=0、无 FF（不动）。
8. 力矩分量遥测（见 3.4）：在 fb_mutex_ 内计算并缓存分量，供定时发布。

### 3.2 xacro（命令接口 + 参数）

**`src/a3_description/urdf/el_a3_ros2_control.xacro`**

- 真机硬件参数块（F108 附近）新增：
  ```xml
  <param name="feedforward_mode">gravity</param>
  ```
- L1–L7 每个 `<joint>` 内，在现有 velocity/effort 命令接口之后，新增（用 `use_real_hardware` 条件包裹，对齐 temperature state 接口写法，避免 mock 侧多声明）：
  ```xml
  <xacro:if value="${use_real_hardware}">
    <command_interface name="acceleration">
      <param name="min">-100.0</param>
      <param name="max">100.0</param>
    </command_interface>
  </xacro:if>
  ```

### 3.3 控制器命令接口拆分（mock/can）

**新增 `src/a3_description/config/arm_controller_ff_plugin.yaml`**（只覆盖 arm_controller 命令接口，深合并样板同 gripper 覆盖文件）：
```yaml
# F160: 真机栈 arm_controller 增加 velocity/acceleration 命令接口，
# 供 JTC splines 的 v/a 前馈；mock GenericSystem(calculate_dynamics) 不支持。
arm_controller:
  ros__parameters:
    command_interfaces:
      - position
      - velocity
      - acceleration
```

**`src/a3_bringup/launch/a3_bringup.launch.py`**
- 新增 `arm_ff_plugin_file`：`hardware:=can` 时指向上述新文件，`mock` 时回退到 `controllers_yaml`（避免覆盖），复用 `gripper_plugin_file` 的 `ParameterFile(PythonExpression([...]), allow_substs=True)` 写法；追加进 `controller_manager` 的 `parameters` 列表（放在 `gripper_plugin_file` 之前或之后均可，因 key 不重叠）。

**`src/a3_bringup/launch/edge_ros2_control_vcan.launch.py`**
- 恒真机，直接在 `controller_manager` 的 `parameters` 追加该新文件路径（`os.path.join(desc_share, "config", "arm_controller_ff_plugin.yaml")`），在 `controllers_yaml` 之后。

### 3.4 力矩分量分解遥测

**新增 `src/a3_msgs/msg/TorqueComponents.msg`**（硬件插件 → arm_controller 的跨进程载体）：
```
std_msgs/Header header
string[] joint_names        # L1_joint..L7_joint
float64[] gravity           # G(q)
float64[] coriolis          # C(q,v)·v（含离心）
float64[] inertia           # M(q)·a（AFF）
float64[] feedforward       # 总前馈 = gravity+coriolis+inertia（rnea 全模型输出×scale×ratio，关节空间未符号）
float64[] pd_feedback       # kp·(q_des−q_meas)+kd·(q̇_des−q̇_meas) 估计（关节空间）
float64[] command           # feedforward + pd_feedback
float64[] measured          # hw_eff（实测关节力矩）
```

**`src/a3_msgs/CMakeLists.txt`**：`rosidl_generate_interfaces` 追加 `"msg/TorqueComponents.msg"`。

**`src/a3_msgs/msg/ArmStatus.msg`**：追加（`measured` 复用已有 `efforts`，不重复）：
```
# 力矩分量分解（关节空间 N·m；L7 无前馈恒 0；F160）
float64[] ff_gravity
float64[] ff_coriolis
float64[] ff_inertia
float64[] ff_total
float64[] pd_feedback
float64[] command_torque
```

**`src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`**（配合 3.1）：
- include `a3_msgs/msg/torque_components.hpp`。
- 成员：`rclcpp::Publisher<a3_msgs::msg::TorqueComponents>::SharedPtr torque_components_pub_;` + 缓存 `std::vector<double> tele_gravity_/tele_coriolis_/tele_inertia_/tele_feedforward_/tele_pd_/tele_command_/tele_measured_`（size=joints_.size()）。
- `on_configure`：在 `health_node_` 上 `create_publisher<a3_msgs::msg::TorqueComponents>("/a3/hardware/torque_components", states_qos)`（best_effort，与 motor_states 一致）。
- `write()` fb_mutex_ 内（当 `gravity_ff_ready_` 时）计算分量并缓存：
  - full：`g = rnea(q,0,0)`、`gv = rnea(q,v_des,0)`、`gva = rnea(q,v_des,a_des)`；则 gravity=g、coriolis=gv−g、inertia=gva−gv、feedforward=gva；各乘 `gravity_ff_ratio_ * ff_scale_[i]`。
  - gravity：gravity=`rnea(q,0,0)`×ratio×scale，coriolis/inertia=0，feedforward=gravity。
  - pd_feedback（仅 position 模式关节）`= j.kp*(j.cmd_pos−j.hw_pos) + j.kd*(j.cmd_vel−j.hw_vel)`；effort 关节记 0。
  - command = feedforward + pd_feedback；measured = j.hw_eff。
- 新增 `PublishTorqueComponents()`，在现有 `motor_states_timer_` 回调里与 `PublishMotorStates()` 一并调用（锁 fb_mutex_ 读取缓存组装 msg，填 `joint_names`=joints_.name）。

**`src/a3_arm_controller/a3_arm_controller/arm_controller.py`**
- import `TorqueComponents`。
- 声明参数 `torque_components_topic: /a3/hardware/torque_components`。
- 订阅（best_effort `js_qos`，`callback_group=self._cb_group`）→ `_on_torque_components`：按 `joint_names` 顺序缓存 6 个分量 list（缺省 0）。
- `_build_status_msg()` 追加 6 个数组（ff_gravity/ff_coriolis/ff_inertia/ff_total/pd_feedback/command_torque），值来自缓存，长度与 `joint_names` 对齐。

**`src/a3_arm_controller/config/arm_controller.yaml`**：追加 `torque_components_topic: /a3/hardware/torque_components`（与其他 topic 参数并列）。

### 3.5 验收脚本

**新增 `scripts/a3_test/f160_velocity_accel_ff_vcan_acceptance.py`**（复用 f108 的 CanSniffer/Recorder/send_jtc/pinocchio 参考模型基建；`f72_ros2_control_vcan_acceptance` 的 `CanSniffer`、`f73` 的 `apply_f49_inertia`）：
- P1 `gravity` 模式回归：velocity 域恒 0、t_ff=rnea(q,0,0)（复用 f108 断言思路）。
- P2 `full` 模式数值：跑快速 quintic 轨迹，抓帧断言 velocity 域=`q̇_des`×direction、t_ff=独立 `rnea(q_meas,v_des,a_des)`×scale×direction（≤0.02 N·m 或速度域量化容差）。
- P3 跟踪误差 A/B：同轨迹同 kp/kd，`full` 的 `max|e|`/稳态滞后显著低于 `gravity`（阈值：稳态滞后 `e≈(kd/kp)·q̇` 下降，例如峰值跟踪误差下降 ≥ 某比例）。
- P4 运行时切换：`/a3_hardware_health` set `feedforward_mode` full↔gravity 生效（抓帧验证）。
- P5 回归：L7 夹爪帧 velocity=0/t_ff=0；effort 帧（zero_torque）不受影响；STRICT arm↔zero_torque 切换正常；退出码 0、无残留进程。

### 3.6 文档同步（architecture-sync / docs-update 规则）

- **`docs/edge/requirements/F160-mit-position-velocity-accel-feedforward.md`**：状态 `proposed` → `implemented`（仿真 vcan 验收，附日期+摘要，真机待通电），沿用 F108 写法。
- **`docs/edge/REQUIREMENTS.md`**：F160 条目状态同步。
- **`docs/shared/TOPIC_CONTRACT.md`**：新增 `/a3/hardware/torque_components`（TorqueComponents）说明；`/a3/arm_status`（ArmStatus）新增 6 个力矩分量字段。
- **`docs/shared/CONTROL_ROADMAP.md`**：控制能力对标表更新「位置环前馈」能力（F108 重力 → F160 全模型 computed-torque）。
- （可选）`docs/edge/QUICKSTART.md`：真机开启 `full` 前馈与遥测查看方式。

---

## 4. 假设与决策

- **AFF 来源**：JTC acceleration 命令接口（用户已确认），非数值微分。
- **力矩遥测**：本次落地进 `/a3/arm_status`（用户已确认），经硬件插件新 topic 中转。
- **前馈比例复用**：`gravity_ff_ratio_` 作为整体前馈比例，`full` 模式下作用于 rnea 全模型输出（不做 coriolis/inertia 独立比例，保持最小复杂度）。
- **符号/坐标**：t_ff 与 velocity 沿用 F108 的 motor 坐标约定（`× direction`，velocity 无 offset）；遥测分量统一关节空间（`× direction` 之前），`efforts`=实测。
- **rnea 次数**：full 模式下为得到独立 coriolis/inertia 分量，write() 内最多 3 次 rnea（n=6，微秒级，200Hz 下可忽略）；gravity 模式仅 1 次。
- **L7**：无 pinocchio 关节映射（`ff_scale_` 恒 0），前馈/遥测分量恒 0。
- **默认安全**：`feedforward_mode` 默认 `gravity`，异常（fault/限位/超温回落逻辑已由既有 FSM 处理）不额外加回落代码——full 仅在真机 A/B 验证后手动/参数启用。

---

## 5. 验证步骤

1. 编译：`colcon build --packages-up-to a3_msgs a3_hardware_interface a3_arm_controller a3_description a3_bringup`（a3_msgs 新增 msg 需先编译）。
2. vcan 闭环：
   - `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up`
   - `python3 scripts/a3_test/vcan_motor_sim.py --interface vcan0`
   - `ROS_DOMAIN_ID=59 ros2 launch a3_bringup edge_ros2_control_vcan.launch.py`
   - `ROS_DOMAIN_ID=59 python3 scripts/a3_test/f160_velocity_accel_ff_vcan_acceptance.py`，退出码 0。
3. 遥测查看：`ros2 topic echo /a3/arm_status` 应出现 ff_gravity/ff_coriolis/ff_inertia/ff_total/pd_feedback/command_torque；`ros2 topic echo /a3/hardware/torque_components` 有 full 分解。
4. 回归：mock 栈 `edge_full_mock.launch.py` 仍正常（mock 不加载 velocity/acceleration 命令接口，`arm_controller` 仍 `[position]`）。
