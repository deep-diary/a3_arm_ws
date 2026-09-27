# LL-130 — CLIK「假收敛」：残差达标不等于真解，必须独立 data FK 复核 + 多种子

> **日期：** 2026-09-26
> **产品线：** Edge
> **环境：** WSL2 Ubuntu 22.04 + ROS 2 Humble（pinocchio 2.6.20 探针，el_a3.urdf）

## 现象

F115 为生成 home 周边笛卡尔偏移点（home_up/down/front/back），用 Pinocchio 写了
CLIK（微分逆解 + 6D 任务残差）探针。第一版探针对「home 末端上/下平移 20cm、姿态不变」
全部报告 OK 并输出关节解，例如 `[.217,1.595,-2.546,.753,.601,-1.035]`；
用**另起的独立 FK**复核，该解末端姿态与 home 偏差 **68°/28°**（位置却接近），
属典型「看起来收敛、腕翻成另一构型」的假解。另外 home 正下 20cm 在 home seed 下
迭代 200 次残差仍 0.418（根本没收敛），探针差一点把它当有效解采纳。

## 根因

1. **误差计算复用同一个 pinocchio data 对象：** 位置帧与姿态帧（以及多任务）在同一
   `data` 上先后 `framesForwardKinematics/updateFramePlacement`，后写覆盖前写，
   残差读到陈旧/错帧；容差判定恒真 →「假收敛」。
2. **单种子 + 软容差：** 不可达/边界目标（−Z 15/20cm 严格保姿态不可达、+X 20cm
   越过水平臂展）会把求解器推入奇异附近的局部解，位置接近但腕翻转，单种子无法发现。
3. 只看迭代终止时的标量残差，没有对输出做独立 6D 复核。

## 正确做法 / 规避

1. **每个候选解用独立 model/data（或至少独立 data）重跑 FK**，同时核对位置与姿态
   （本次门槛：位置 1e-4 m、姿态 1e-4 量级，角度误差单独判）。
2. **多起点（200 随机种子）+ 去重**，要求所有采纳解彼此一致；平面构型优先
   （L1=L5=L6=0），避免拿到腕翻转边缘分支（home +Z 20cm 唯一「解」在 L4 翻转限位边缘）。
3. **不可达就判不可达**，按物理包络缩小距离（最终：后 20cm / 前 15cm / 上 15cm / 下 10cm），
   不要放最近似解。
4. 生产节点 `a3_bringup/move_to_pose_ik_node.py` 是同型 CLIK，存在同样风险；
   后续改进位姿 IK（笛卡尔服务）时必须按上述三点加固，不能只信求解器残差。

## 相关路径

- `src/a3_description/config/named_poses.yaml`（home_back/home_front/home_up/home_down 最终值，注释含探针日期）
- `src/a3_moveit_config/config/el_a3.srdf`（4 个同名 group_state）
- `src/a3_bringup/a3_bringup/move_to_pose_ik_node.py`（生产同型 CLIK，待加固点）
- `scripts/a3_test/f116_random_tour_sim_acceptance.py`（含逐点到点 0.02 rad 有序核对，可作点位正确性回归）
