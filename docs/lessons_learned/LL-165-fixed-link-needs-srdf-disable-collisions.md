# LL-165 — 新增末端固定件必须同步登记 SRDF 碰撞对：否则 MoveIt 起点无效，所有 move_group 动作 error 99999

> **日期：** 2026-10-16
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ 真机 6 轴（L1–L6 + 连杆）

## 现象

6 轴 + 连杆形态真机起栈正常（关节数、控制器、CAN 全对），但 PS4 **长按方形**（路点 PTP 回放）
每次都失败，编排层反复重试后回 READY：

```
[move_group] [WARN] ompl: arm/arm: Skipping invalid start state (invalid state)
[move_group] [ERROR] ompl: RRTConnect: Motion planning start tree could not be initialized!
[move_group] [INFO] move_action_capability: Catastrophic failure
[arm_controller] state -> READY (waypoint ptp 1/5: error 99999)
```

任何走 `move_group` 的动作（PTP/LIN 回放、巡游、回航）同样全灭；顶层报 `error 99999`，
但**看不出是碰撞**（OMPL 只说起点无效）。

## 根因

新增的连杆 `linkage_link`（轴 6 法兰后固定连杆，φ8×140mm）**与腕部 EL05 电机壳共轴**——
而电机壳 `el05_l6.stl` 是母件 `l5_l6_urdf_asm` 的碰撞体，所以连杆从法兰往下 14cm 里，
前 10.6cm 完全落在电机壳内部（物理上正是从电机轴穿出）。`el_a3_6j.srdf` 没登记连杆的
`disable_collisions` → MoveIt 认为"当前状态自碰"，起点无效 → 规划必然失败。

定位手段（一行命令，不用猜）：

```bash
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity \
  "{group_name: arm, robot_state: {joint_state: {name: [...], position: [...]}}}"
# valid=False，contacts 里直接给出碰撞对与穿深：
#   l5_l6_urdf_asm <-> linkage_link  depth=0.0440
#   part_9         <-> linkage_link  depth=0.0000
```

对照排查：`grep -n "error 99" /tmp/a3_real_stack.log`（99999 = OMPL 起点无效/规划catastrophic；
99966 之类另查 `moveit_msgs/MoveItErrorCodes`）。

## 正确做法 / 规避

- **凡新增/替换末端固定件（连杆、夹爪、工具、相机支架）**，改完 URDF 后必须同步在 SRDF 里登记
  与母件/相邻件/远链件的 `disable_collisions`（7 轴原 SRDF 对 `gripper_link` 就是这么做的）。
  本例最小集（`src/a3_moveit_config/config/el_a3_6j.srdf`）：

  ```xml
  <disable_collisions link1="l5_l6_urdf_asm" link2="linkage_link" reason="Adjacent"/>
  <disable_collisions link1="linkage_link" link2="end_effector" reason="Adjacent"/>
  <disable_collisions link1="part_9" link2="linkage_link" reason="Never"/>
  ```

  只关"必然重叠/相切"的对，**不要**无脑把所有臂件都设 Never——连杆与其余臂件的真实碰撞应当被拦住。
- SRDF 是启动时读入的：改完要**重启栈**（`a3_stack.sh restart --rviz`，且臂须先失能）。
- 验证不必动机器人：`check_state_validity → valid=true`，再 `/plan_kinematic_path`
  （目标给路点文件 `~/.a3/trajectories/waypoints/latest.yaml` 的第 1 点）拿 `error_code=1`。
- 相关工具：`moveit_msgs/srv/GetStateValidity`（查碰撞）、`GetMotionPlan`（只规划不执行，
  物理调试时优先用它）。

## 相关路径

- `src/a3_moveit_config/config/el_a3_6j.srdf`（新增连杆碰撞对）
- `src/a3_description_6j/urdf/el_a3.urdf.xacro`（`linkage_link` / `linkage_joint`）
- 需求：[F174](../edge/requirements/F174-a3-description-6j-linkage-package.md)；相关：[LL-164](LL-164-mesh-identification-dont-trust-names.md)（同一批网格辨识）

> **后续（2026-10-16 当天）**：真实末端件 CAD STL 到位后，圆柱占位件 `linkage_link` 已删除，
> 末端几何改由 `l5_l6_urdf_asm` 直接挂真实件（见 F174 / [LL-166](LL-166-new-mesh-mm-units-foreign-frame.md)）；
> 那三条 `linkage_link` 登记随之移除。**教训不变**——凡末端件换型（占位↔真实件、换工具），
> 都要重跑 `check_state_validity` 并按需补/删 SRDF 碰撞对。
