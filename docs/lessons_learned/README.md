# Lessons Learned（踩坑与换机备忘）

> **用途：** 记录开发中踩过的坑、环境差异、耗时操作的替代方案，方便换机器 / 新人少走弯路。  
> **不是需求文档：** 正式需求仍写在 `docs/<产品线>/REQUIREMENTS.md`。

## 怎么写

每条一篇独立文件，文件名建议：

```
LL-NNN-short-slug.md
```

模板见 [TEMPLATE.md](TEMPLATE.md)。智能体在**确认修复后**按仓库规则 `.cursor/rules/lessons-learned.mdc` 自动补条目并更新本表。

写入时：现象写可检索的报错原文关键词；根因写「为什么」不是「改了哪一行」；正确做法必须是换机可执行的命令。

## 索引

| ID | 标题 | 产品线 | 日期 |
|----|------|--------|------|
| [LL-001](LL-001-microros-host-setup.md) | micro-ROS Agent / host 构建慢、apt 不可用、XRCE 消息缓冲 | CloudEdge | 2026-08-25 |
| [LL-002](LL-002-edge-sim-rviz-moveit-env.md) | Edge 仿真 RViz / MoveIt demo：HDMI、colcon overlay、缺包 | Edge | 2026-08-26 |
| [LL-003](LL-003-arm-trotbot-port-leftovers.md) | trotbot→arm 移植遗留：限位表越界把目标 clamp 成 0、脚本 CRLF 报 pipefail | Edge | 2026-09-01 |
| [LL-004](LL-004-lubancat-can0-can1-physical.md) | LubanCat-4-V1 CAN 物理口：软件 can0 无收发器、can1（芯片 CAN2）才能直连电机 | Edge | 2026-09-01 |
| [LL-005](LL-005-humble-timer-qos-python-node.md) | Humble Python 节点：create_timer 无 oneshot；/joint_states 需 best-effort 订阅 | Edge | 2026-09-02 |
