# F150 — RGB-D 深度相机接入与 TSAI 手眼标定（A1）【P0】


- **说明：** 对标 reBot 深度相机集成（Orbbec Gemini2 / RealSense D405/D435i；eye-to-hand 与 D405 eye-in-hand）。接入相机驱动、完成 TSAI 手眼标定并发布相机→基座 TF，提供标定精度验收；腕部相机 TF 可先做 RViz 可视化（对标社区 D405 eye-in-hand TF）。
- **验收标准：**
  1. RGB-D 画面与深度流稳定（USB 供电/带宽不掉帧），相机系→`base_link` TF 发布
  2. 手眼标定重投影/位姿误差在文档容差内，标定文件可复用
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A1；F151/F152（检测/抓取消费方）
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
