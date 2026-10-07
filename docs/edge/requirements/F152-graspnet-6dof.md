# F152 — GraspNet 6-DoF / OBB 抓取姿态估计与视觉抓取闭环（A1）【P1】


- **说明：** 对标 reBot 两条抓取路径：① YOLO 分割 + OBB 最小外接矩形抓取姿态（轻）；② YOLO11n-seg 过滤 + GraspNet 6-DoF（重，服务器跑）。抓取姿态经本仓 IK/FJT（或 F141 pick_place 链）执行，形成「看见→抓→放」闭环；重推理放服务器、轻姿态边缘。
- **验收标准：**
  1. 指定桌面目标可给出可达抓取位姿并完成抓取放置（先服务器 GraspNet，后评估 NPU 轻量）
  2. 不可达/无抓取点时显式失败，不发运动指令；全程受 SAFETY 门控
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A1；F150/F151、F141、C1；[reBot-DevArm-Grasp](https://github.com/EclipseaHime017/reBot-DevArm-Grasp)
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
