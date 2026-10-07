# F159 — Isaac Sim USD 数字孪生与 sim-to-real（A5）【P2】


- **说明：** reBot 的 Isaac Sim 集成（USD 模型 + 仿真遥操作 + 合成数据）已对 DM/RS 双机型 **Done**（DLI 课程 + reBot-Isaacsim）。A3 需在带 NVIDIA GPU 的服务器/云上构建 USD 模型与仿真遥操作，用于合成数据与策略 sim-to-real（RK3588 本地不跑）；与 F147（轻量浏览器孪生）按算力分工并存。
- **验收标准：**
  1. A3 USD 模型在 Isaac Sim 中与 URDF 关节/惯量一致，可仿真遥操作
  2. 产出一批合成数据并验证至少一个感知/策略环节的 sim-to-real 迁移
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A5/6.1；F147（MuJoCo 轻方案）；[reBot-Isaacsim](https://github.com/Seeed-Projects/reBot-Isaacsim)
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
