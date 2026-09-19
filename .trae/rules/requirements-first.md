<!-- cursor-meta: description=需求优先：开发前先查 docs/<产品线>/REQUIREMENTS.md；缺失则先写需求；有歧义先与用户确认 | alwaysApply=true -->

# 需求优先开发流程

动手写代码或改实现前，必须先处理需求文档。

## 检查清单

1. 在 `docs/` 下搜索是否已有相关需求
   - 总看板：`docs/README.md`
   - Edge（主线）：`docs/edge/REQUIREMENTS.md`
   - CloudEdge（支线）：`docs/cloud_edge/REQUIREMENTS.md`
   - 跨线契约：`docs/shared/`（`TOPIC_CONTRACT.md`、`SAFETY.md`、`CONTROL_ROADMAP.md`）
   - 踩坑与换机备忘：`docs/lessons_learned/`（**不是需求文档**）
2. **无对应文档** → 先在对应产品线目录下编写或更新需求，再开发
3. **需求不清晰、有歧义或与现有规格冲突** → 先向用户确认，得到答复后再开发

## 需求条目格式

每条需求包含：**ID**（如 S7、E9、F18）、**说明**、**验收标准**、**关联文档**。
实现完成后在需求文档标注实现状态（如 `implemented`、`外置固件`）。

## 工作流

1. 在 `docs/<产品线>/REQUIREMENTS.md` 追加需求 ID 与验收标准
2. 若涉及话题/安全契约，同步更新 `docs/shared/`
3. **先查参考项目是否有现成实现**（见 `reference-first.md`），有则优先借鉴
4. 再创建或修改 `src/` 代码（固件在外部仓库时不在本仓改固件）
5. 补充 `QUICKSTART` 或包内 `README` 中的验证步骤

## 上下文

- Edge 主线：RK3588 板 + SocketCAN + 7 个 MIT 电机（`L1_joint`..`L7_joint`，L7 为夹爪）
- CloudEdge：内网服务器 + ESP32-S3 micro-ROS CAN 桥（固件在外置 xiaozhi-esp32 仓库）
- 话题/关节契约：`docs/shared/TOPIC_CONTRACT.md`；控制对标：`docs/shared/CONTROL_ROADMAP.md`

## 禁止

- 不要跳过需求文档直接实现新功能或大改行为
- 不要自行假设未确认的业务规则
