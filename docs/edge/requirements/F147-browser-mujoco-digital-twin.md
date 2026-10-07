# F147 — 浏览器 MuJoCo 数字孪生（免安装演示/远程验收）【P3】


- **说明：** 对标 reBot 在线 MuJoCo 孪生（标准臂/AGV 配置、关节与 TCP 控制、顶视/腕部相机、归位/码垛自动 demo）。RK3588 无 NVIDIA 生态，MuJoCo（CPU/WebAssembly）是比 Isaac Sim 更轻的演示与远程验收手段：基于 A3 URDF/mesh 生成 MJCF，浏览器可视关节状态（订阅经 Web 侧转发）与简单轨迹。与 deep-trace 3D 渲染（F20）定位区分：本项为物理仿真孪生，F20 为实时状态渲染。
- **验收标准：**
  1. 浏览器打开即可见 A3 模型并可拖动关节/发简单位姿目标，零本地安装
  2. 模型姿态与 URDF 一致；可回放录制轨迹（F54）做离线演示
- **关联：** F20（Web 3D 实时渲染）、F159（Isaac Sim，重方案）；[reBot online demo](https://yang-ci.github.io/Rebot_Arm_AGV/)
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
