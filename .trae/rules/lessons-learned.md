<!-- cursor-meta: description=经验教训：确认修复后同轮回复记录 LL-NNN 到 docs/lessons_learned/；新任务前先检索 | alwaysApply=true -->

# 经验教训

## 何时必须记录

在问题**已经确认修好**（复测通过，或用户明确说已恢复）之后，**同一轮回复里**写入 `docs/lessons_learned/`，不要等用户再催。
未确认的猜测、一次性笔误、改文档用词，不必建条。

典型场景：环境 / overlay / `source` 顺序导致找不到包或 RMW 被换掉；缺 apt/ROS 插件（RViz Display、MoveIt planner、controller_manager）；
launch 参数声明了但没接到节点；`DISPLAY` / SSH / HDMI、CRLF、`PYTHONNOUSERSITE`、QoS 不匹配（RELIABLE vs BEST_EFFORT）；
硬件/电机/CAN/串口类易复发根因。修完后若再遇到同类问题，应能靠该文档独立复现规避。

## 记录规范

1. 看 `docs/lessons_learned/README.md` 索引，取下一个 `LL-NNN`；能并入已有条目就改那一篇，不要复制粘贴新文件
2. 按 `docs/lessons_learned/TEMPLATE.md`：现象 / 根因 / 正确做法 / 相关路径
3. 文件名：`docs/lessons_learned/LL-NNN-short-slug.md`（slug 英文短横线）
4. 在 `docs/lessons_learned/README.md` 的索引表加一行
5. 脚本必须是 LF；新建 `*.sh` 后用 `bash -n` 抽查

## 任务前检索

开始新任务前，按任务关键词检索 `docs/lessons_learned/` 清单，避免重复踩坑。

## 不要做

- 不要改 `docs/lessons_learned` 以外的「总结散文」代替正式条目
- 不要把未修复、未验证的假设写成根因
- 需求仍走 `docs/<产品线>/REQUIREMENTS.md`（本规则不替代需求先行）
