<!-- cursor-meta: description=CloudEdge 固件路径探测：引用 xiaozhi-esp32 deep-dog 固件前先探测本机实际路径，勿写死 | alwaysApply=true -->

# CloudEdge ESP32 固件路径探测

A3 CloudEdge 的 ESP32-S3（deep-dog 板型）固件在**独立仓库** `xiaozhi-esp32` 开发，路径随机器不同。
引用或修改固件前，**先探测本机实际存在的路径，不要写死**。

## 远程与探测

- 远程：https://github.com/deep-diary/xiaozhi-esp32 （板级 `main/boards/deep-dog/`）
- 本仓（WSL 开发机）通常没有该仓，多在 Windows / 其他机器工作区。引用前先 `ls`/`Test-Path` 探测，
  多个克隆存在时以 `git -C <path> log -1` 提交时间最新者为准；不确定就问用户。
- 仅在探测确认后，才引用具体子路径。

## 关键子目录

- 板级实现：`…/main/boards/deep-dog/`
- 固件端需求 SWRS：`…/deep-dog/swrs/`（如有），与本仓 `docs/cloud_edge/REQUIREMENTS.md` 对齐
- micro-ROS 组件：https://github.com/micro-ROS/micro_ros_espidf_component（**Humble** 分支，勿用 rolling）

## 本仓边界

本仓 `src/a3_cloud_edge/` 只放服务器侧（micro-ROS Agent / mock client）与契约文档，
**不含固件源码**。不要在本仓创建或修改固件 `.c/.cpp/.h` 文件（见根目录 `AGENT.md` 仓库边界）。
