# F145 — Pinocchio + MeshCat 运动学/重力可视化工具【P2】


- **说明：** 对标 reBot Pinocchio+MeshCat 教程：浏览器 3D 显示 A3 模型、FK/IK 结果、各关节重力矩向量与 tau_scale 标定前后对比，辅助 F89 重力标定、示教调试与教学。复用本仓 URDF 与 `inertia_params.yaml`/`gravity_scales.yaml`，只读工具、不接 CAN。
- **验收标准：**
  1. 一条命令在浏览器打开 A3 MeshCat 视图，关节角与 `/joint_states`（或日志回放）一致
  2. 可可视化 RNEA 重力矩（标定前/后叠加对比）；FK/IK 示例可交互
- **关联：** F89（tau_scale 标定）、F49（惯性标定）、F8（重力力矩）；[Pinocchio Guide](https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/)
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
