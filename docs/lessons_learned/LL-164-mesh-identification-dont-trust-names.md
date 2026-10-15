# LL-164 — 换形态删网格别按名字猜：A3 腕部 mesh 对照表 + 无 RViz 的 mesh 辨识方法

> **日期：** 2026-10-15
> **产品线：** Edge
> **环境：** WSL2 Ubuntu 22.04 + ROS 2 Humble

## 现象

把描述改成「6 轴 + 连杆」时，按 mesh 名字推断 `el05_l6.stl` 是"第 7 个（夹爪）电机壳体"，
从 `l5_l6_urdf_asm` 的 `visual`/`collision` 里删掉 → 用户看 RViz 立刻反馈"电机壳没了、少了 2 个"，
实际那台电机**物理上还在**（连杆与它共轴穿出）。来回两轮才纠正。

## 根因

- mesh 名字不可靠：`el05_l6` 读作"EL05 · L6"容易被当成第 7 电机；`l6.stl` 也不是电机而是结构件。
- 该 mesh 是**一台**电机的一体壳体：实测 bbox `46×47×106 mm`，连通分量是两个互相套叠的壳体
  （7362 + 4536 三角面）+ 4 个螺栓（各 598），**不能按连通分量拆成两台电机**。
- RViz 只认 `/robot_description`（URDF）里的 visual；删错网格 = 模型缺件，但运动学/惯量不受影响
  （`<inertial>` 与 visual 是两套东西，别一起删）。

## 正确做法 / 规避

**本臂 mesh ↔ 部件对照（A3，2026-10-15 实测 bbox）：**

| mesh | 尺寸 (mm) | 装在哪 | 是什么 |
|---|---|---|---|
| `l1.stl` | — | `l1_urdf_urdf_asm` | 底座结构件 |
| `l2.stl` | — | `l1_link_urdf_asm` | 上臂结构件 |
| `l3.stl` | — | `l2_l3_urdf_asm` | 结构件 |
| `l4.stl` | — | `l3_lnik_urdf_asm` | 前臂结构件 |
| `l5.stl` | 100×54×112 | `l4_l5_urdf_asm` | 腕部结构件（两个圆柱相交） |
| `l6.stl` | 32×35×57 | `part_9` | 腕部法兰件（两面螺栓孔） |
| `el05_l6.stl` | 46×47×106 | `l5_l6_urdf_asm`（轴 6 输出） | **腕部 EL05 电机壳体（一台，含输出轴段）** |
| `jaw.stl` | 191×55×104 | `gripper_link`（L7） | 夹爪（6 轴形态下已拆） |

**换形态/删网格前的"三查"：**

1. **查几何**：算 bbox 与连通分量（判断是不是多件合体、是不是一台电机）。
2. **查渲染**：不依赖 RViz 也能看——本机 `matplotlib` 因 pyparsing ABI 不匹配已坏（见 LL-152/156），
   直接用 `numpy + PIL` 写几十行光栅渲染（读二进制 STL → 取三角形 → 相机基投影 → 按深度排序
   填充多边形 + 平光着色）；要看真实 RViz 画面时用 `DISPLAY=:0` + `xdotool search --name RViz`
   + `import -window <id>` 截屏（比让用户描述快得多）。
3. **查实物**：形状对不上就问用户（他能直接看机械臂），不要替他判断"哪个电机被拆了"。

**删除姿态**：只删 visual/collision，保留 `<inertial>`（惯量参与重力前馈，标定数据对应的是实物状态）。

## 相关路径

- `src/a3_description_6j/meshes/*.stl`、`src/a3_description_6j/urdf/el_a3.urdf.xacro`（`l5_l6_urdf_asm`）
- 需求：[F174](../edge/requirements/F174-a3-description-6j-linkage-package.md)；相关：[LL-163](LL-163-default-description-switch-hidden-hardcodes.md)
