# LL-127 — 订阅零样本先核对 topic 字符串本身（下划线/斜杠），勿在字符串核对前深挖 rmw

> **日期：** 2026-09-25
> **产品线：** Edge
> **环境：** RK3588 (lubancat), ROS 2 Humble, rclpy, 隔离域 ROS_DOMAIN_ID=110

## 现象

F110 验收脚本（`f110_mock_power_acceptance.py`）唯一 FAIL：`FsmState` 类对
`/a3/arm_status`（a3_msgs/ArmStatus）的订阅始终 0 收消息。同节点上语义相同的
内联类/lambda 订阅全部正常（30/30 收到），甚至同节点同进程 publisher 的 loopback，
FsmState 也收不到，而内联订阅能收到。Python 层逐项检查全部正常：handle 非空、
QoS 相同、callback 绑定方法正确、callback group `has_entity=True`、
`inspect.signature` 同为 `(msg)`；字节级副本（换路径/换模块名导入）同样 FAIL。
据此一度定性为「rclpy/rmw C reader 由文件字节内容触发的孤立怪癖」。

## 根因

FsmState 订阅的 topic 字符串实际写成了 **`/a3_arm_status`（下划线）**，
而产品 FSM 发布的话题是 **`/a3/arm_status`（斜杠）**——一个完全不存在的话题。
合法 QoS、合法 handle、正常的 C reader，只是名字错了一个字符：

- loopback 实验中 publisher 发在正确的 `/a3/arm_status`，FsmState 听的是
  `/a3_arm_status`，自然 0 样本；内联订阅用的是正确名字，所以全收到。
- 「字节级副本复现、内联重打不复现」并非内容玄学：重打的文本恰好打对了斜杠。
- hexdump 显示纯 ASCII「无隐藏字符」、dis 反汇编正常——都属实，但没人逐字符
  核对字符串里的 `_` 与 `/`。

## 正确做法 / 规避

1. **订阅零消息的排查顺序**（自上而下，不可跳级）：
   ① `ros2 topic list` / `topic info` 确认名字逐字符一致（`_` vs `/`、大小写）；
   ② QoS（reliability/durability）兼容；③ 发现是否完成（late joiner）；
   ④ 才是 rcl/rmw 内部机制。字符串没核对之前，不要进 C 层。
2. 脚本里订阅名与发布名尽量从同一常量/同一来源取，避免手写两份。
3. 「换个写法就正常」不能证明底层玄学——先 diff 两段文本的**字符串字面量**，
   再怀疑框架。所有「正常」的旁证（handle/QoS/签名）对错误 topic 同样成立，
   不构成 topic 正确的证据。

## 相关路径

- `scripts/a3_test/f110_mock_power_acceptance.py`（FsmState 订阅行）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（/a3/arm_status 发布方）
