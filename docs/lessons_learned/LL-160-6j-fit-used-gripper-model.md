# LL-160 — 6J 档拟合用了「带夹爪」的模型：拟合质量被压低 → 消费端系统性欠补偿到处下塌

> **日期：** 2026-10-14
> **产品线：** Edge
> **环境：** lubancat（RK3588）Ubuntu 22.04 + ROS 2 Humble，产品 6 关节档（拆夹爪）

## 现象

真机拆掉 L7 夹爪改 6 关节档后，重力标定（F49）**通过验收**（RMSE 0.148 Nm），但**示教（`ZERO_TORQUE`，kp=0）下大量位姿仍持续下塌**——L2、L3 都有，且是「到处、系统性」而非个别位姿。用户怀疑拟合算法有问题。

## 根因

F49 脚本（`scripts/gravity_calibration.py`）加载的 URDF 是 `install/a3_description/share/a3_description/urdf/el_a3.urdf`，它**仍含 `L7_joint` + `gripper_link`（mass=0.05）**、`nq=7`；而真机已无夹爪，消费端（`a3_mit_hardware_interface` / `gravity_compensation_controller`）用的是**无夹爪的 6 关节模型**（`nq=6`）。

- 拟合在「带夹爪」的模型上最小二乘去拟合**无夹爪真机**的实测力矩 → 为抵消模型里那个并不存在的 0.05 kg，最优解把 **L2–L6 质量整体压低（合计 ≈ −0.06 kg）**；
- 消费端（6J，无夹爪）套用这套被压低的参数 → **系统性欠补偿**（各关节都少一点）→ 自由拖动下到处漂/塌；
- F169 的「6 关节适配」只改了 `JOINT_NAMES`/符号/位姿长度，**没改 URDF**，所以模型与真机不一致这一条被漏掉。

**佐证**：用无夹爪 6J 模型重拟合同一批数据，RMSE 完全相同（0.1483）但**质量每根都抬高**（L2 0.658→0.679、L3 0.183→0.189、L4 0.410→0.429、L5 0.016→0.018、L6 0.482→0.500，合计 +~0.06 kg）——正是被压低的夹爪量。

## 正确做法 / 规避

**拟合模型必须与真机一致**。`--joints 6` 时：

- （已实现）在 `init_pinocchio` 里**清零 `gripper_link` 惯量**（`model.inertias[frames[gid].parentJoint] = pin.Inertia.Zero()`），拟合模型即与真机对齐；
- （等价替代）`xacro el_a3.urdf.xacro include_gripper:=false > 6j.urdf` 后 `--urdf`；
- **已采数据可离线重拟合、无需重采**：`--joints 6 --optimize-only --data-file <jsonl>`。

**另注**：修掉这点系统性误差（≈0.1–0.3 Nm）后，L2 仍有 **±0.170 Nm 方向性迟滞**（齿轮/线束，硬件地板）。自由拖动（kp=0、无位置弹簧）下**任何残余力矩都会转成稳态漂移**（`v ≈ Δτ/kd`），所以**静态刚体模型无法让示教完全不塌**——要彻底改善需摩擦/迟滞补偿，或示教时给一点刚度/加大阻尼。

## 相关路径

- `scripts/gravity_calibration.py`（`init_pinocchio` 的 6J 清零、`FIT_LINK_MAP`、`load_params_yaml`）
- `install/a3_description/share/a3_description/urdf/el_a3.urdf`（含 L7，nq=7）
- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（`kF49JointToLink` 消费端映射）
- `docs/edge/requirements/F170-fullspace-calibration-grid.md`
