# F113 — 点位改名 home→idle + 弃用 `~/.a3/poses.yaml` 用户覆盖（点位统一收敛到包内 named_poses.yaml）


- **说明：** 真机标定的折叠自然下垂位（重力稳定，F40 失能保护位）原名 `home`，只存在于仓外 `~/.a3/poses.yaml`（F39 用户层覆盖）；包内 `named_poses.yaml` 的 `home` 还是 EDULITE 半抬遗留值——同名不同值，靠覆盖文件才正确，仓外文件不可版本管理。本次：① `home` 整体改名 `idle`，值 = 2026-09-13 真机标定 `[-0.0002, 0.0006, 0.0002, 0.3351, 0.0121, -0.0002, -0.0002]`（位置不变）；EDULITE 半抬 `home` 退役删除；② 点位唯一定义收回包内 `a3_description/config/named_poses.yaml`，MoveIt `el_a3.srdf` group_state 同步改名改值；`~/.a3/poses.yaml` 弃用（改名 `.bak` 留底）；③ Circle（default 映射）/D-pad 左（simple 映射）与 F40/F44 safe-park 目标（三份 `arm_controller*.yaml` + 代码默认值 `disable_home_pose_name`）同步改指 `idle`。遗留注意：`/a3/arm/save_named_pose`（F39）被调用时会重建 `~/.a3/poses.yaml` 形成覆盖——约定不再使用，或后续单独收敛该服务。
- **验收标准：**
  1. `named_poses.yaml` / `el_a3.srdf` 含 `idle`（折叠自然下垂值）且不再含 `home`；三份 `arm_controller*.yaml` 与代码默认值 `disable_home_pose_name: idle`
  2. Circle（default）/ D-pad 左（simple）→ `idle`；`goto_named_pose {pose_name: idle}` 成功，`{pose_name: home}` 返回 unknown pose
  3. 真机回归：R3 safe-park 收敛到 idle ±0.15 rad 后失能（行为与改名前逐位一致）；Triangle→ready、Square 回放不受影响
- **关联：** F39（save_named_pose 用户层覆盖）、F40（失能保护）、F60/F64（PS4 映射）、F109（ready 重定义）；[shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)（命名姿态表）
- **状态：** `implemented`（2026-09-26 改名落地；真机回归待上电）




> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
