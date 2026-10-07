# F151 — YOLO/YOLOE 目标检测分割与 RKNN 适配（A1）【P1】


- **说明：** 对标 reBot YOLO/YOLOE 检测分割（官方走 Jetson TensorRT）。A3 在 RK3588 NPU 上以 RKNN 落地轻量检测/分割与开放词汇提示，输出目标 2D mask/bbox + 深度投影，作为抓取姿态与 WRC 语言定位（F156）的感知输入。
- **验收标准：**
  1. NPU 推理可对桌面常见物体输出检测/分割，帧率满足抓取节拍，CPU 卸载明显
  2. 输出坐标经 F150 TF 转到基座系可复现；模型/标签可配置
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A1/NPU 分层决策；F150（相机/TF）、F152（抓取姿态）
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
