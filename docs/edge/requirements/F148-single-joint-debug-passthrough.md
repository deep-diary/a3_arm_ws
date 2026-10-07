# F148 — 编排层单关节调试 passthrough 服务统一【P3】


- **说明：** 对标 reBot `/rebotarm` 命名空间下的 `JointMitCmd`/`JointPosVelCmd` 单关节调试服务：本仓单电机服务分散在 `a3_can_bridge`（MotorCommand/MotorMitCommand/SetMotorParam 等），标准栈（F78）后缺少对应的统一调试入口。在编排层/维护命名空间提供受控的单关节 MIT/位置速度直通服务（仅维护态可调，运动态互斥），并补一页 API 速查。
- **验收标准：**
  1. 维护模式下可对指定单关节发 MIT（pos/vel/kp/kd/tau）或 posvel 指令并收到反馈；READY/运动/示教态调用被拒
  2. 与 F91 电机零点/参数维护节点、F138 整定工具的权限模型一致（控制器活动联锁）
- **关联：** F91（电机维护节点）、F53（指令×状态×模式矩阵）、`a3_can_bridge/srv`
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
