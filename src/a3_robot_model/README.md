# a3_robot_model — 系统级机器人模型配置

**一句话职责：** 指定 A3 的**默认机器人描述**（URDF/xacro 所在包）与配套档位文件（SRDF /
controllers / motor_map / joint_names），让真机栈、仿真栈与各节点从**同一份配置**取描述，
不再各处写死 `a3_description`。

## 配置文件与覆盖顺序

| 顺序 | 文件 | 说明 |
|---|---|---|
| 1 | `config/robot_model.yaml`（本包） | 仓内默认：**6 轴（L1–L6）+ 末端连杆**，`a3_description_6j`（F174） |
| 2 | `~/.a3/robot_model.yaml` | 本机覆盖（字段级 merge），例如某台机器换回 7 轴形态 |

```yaml
robot_model:
  description_package: a3_description_6j
  urdf_xacro: urdf/el_a3.urdf.xacro
  urdf_static: urdf/el_a3.urdf
  controllers: config/el_a3_controllers.yaml
  joint_names: [L1_joint, ..., L6_joint]
  include_gripper: false
  srdf: el_a3_6j.srdf              # 相对 a3_moveit_config/config
  motor_map: motor_map_6j.yaml     # 相对 a3_can_bridge/config
```

**换形态**（例如回到 7 轴）：写 `~/.a3/robot_model.yaml` 覆盖 `description_package:
a3_description`、`include_gripper: true`、`joint_names` 含 `L7_joint`、`srdf: el_a3.srdf`、
`motor_map: motor_map.yaml` 即可，无需改代码。

## 用法

```bash
# 打印解析后的完整配置（含最终生效值）
python3 -m a3_robot_model.robot_model
# 只取某个字段（shell 脚本用）
python3 -m a3_robot_model.robot_model --get description_package
# 打印实际生效的配置文件路径
python3 -m a3_robot_model.robot_model --path
```

```python
from a3_robot_model import robot_model

cfg = robot_model.load()
share = robot_model.description_share(cfg)   # .../share/a3_description_6j
urdf = robot_model.urdf_path(cfg)            # .../urdf/el_a3.urdf
joints = robot_model.joint_names(cfg)        # ['L1_joint', ..., 'L6_joint']
```

## 消费方

| 消费方 | 用途 |
|---|---|
| `a3_bringup`（`a3_bringup.launch.py` 与各节点） | xacro / controllers / SRDF / motor_map / 编排层 joint_names |
| `a3_arm_controller` | 关节限位读取、静态 URDF（Pinocchio）、`named_poses_pkg` |
| `scripts/a3_stack.sh` | 起栈默认档位（6 轴 / 7 轴） |

> 需求：[F174](../../docs/edge/requirements/F174-a3-description-6j-linkage-package.md)
