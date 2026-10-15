# LL-166 — 换新 STL 先量单位与坐标系：CAD 件常是 mm + 外置坐标系，URDF 要用 scale+origin 显式摆位

> **日期：** 2026-10-16
> **产品线：** Edge
> **环境：** RK3588（lubancat）/ ROS 2 Humble（A3 6 轴 + 真实末端件）

## 现象

拿到真实末端件的 CAD STL（`el05_l6.stl`，覆盖原占位圆柱）后，直接按原 mesh 的写法
`<mesh filename=".../el05_l6.stl"/>`（无 scale、沿用旧 origin）挂到 URDF 上会**完全错位**：
新件 bbox 是 `x∈[−192,−42] mm、y∈±23 mm、z∈[111,159] mm`——**单位是毫米**（旧 mesh 是米），
且**坐标系不是链接系**（件自轴沿 mesh x，离 mesh 原点 135 mm；贴合面在 x=−42 mm 平面）。

## 根因

- 本仓 mesh 惯例是「米 + 链接系直接可用」（如 `l6.stl`），但**CAD 导出件常是 mm 且带装配坐标系**，
  两者混用必错位。
- 只看 bbox 数值无法判断"哪个面贴合、哪条是轴线"——需要算**自轴**（各 x 切片截面中心）与
  **贴合面**（带螺栓孔的法兰盘那端）。

## 正确做法 / 规避

1. 先量：单位（数量级）、bbox、连通分量、**沿长轴的截面半径剖面**（判阶梯/法兰/细杆）。
2. 定坐标：找出件的自轴（本例 = mesh x，位于 mesh (y,z)≈(0.5,135.5)mm）与贴合面
   （本例 = mesh x=−42.15mm 平面）；远端 = 另一极端面（x=−192.15mm，件长 150mm）。
3. 写 URDF：`<mesh ... scale="0.001 0.001 0.001"/>` + 显式 `<origin>`：
   - `rpy`：把 mesh 长轴对到 link 的末端方向（本例 mesh +x → link +z ⇒ `rpy="0 -1.5708 0"`）
   - `xyz`：把自轴/贴合面搬到 link 轴/法兰面（本例 `0.1355 -0.0005 0.04215` m）
   - TCP 偏移随之改（本例 `end_effector` z = −0.150）
   全部做成 xacro arg（`linkage_mesh_scale/origin/rpy/tcp_z`），换件只改这几个值。
4. 验：用 Pinocchio 打印 link/TCP 位置（可达性自检）、本地渲染整机、起栈后 `check_state_validity`；
   **换件后务必复查 SRDF 碰撞对**（见 LL-165）。
5. 不要为了"省事"把 mm 网格重导出成米制——CAD 后续还会更新，参数化 origin/scale 更稳。

## 相关路径

- `src/a3_description_6j/urdf/el_a3.urdf.xacro`（`linkage_mesh_*` / `linkage_tcp_z`）
- `src/a3_description_6j/meshes/el05_l6.stl`（真实末端件）、`el05_l6_bk.stl`（旧件留档）
- 需求：[F174](../edge/requirements/F174-a3-description-6j-linkage-package.md)；
  相关：[LL-164](LL-164-mesh-identification-dont-trust-names.md)（mesh 辨识）、[LL-165](LL-165-fixed-link-needs-srdf-disable-collisions.md)（换件后 SRDF 碰撞对）
