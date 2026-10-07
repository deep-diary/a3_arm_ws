# F141 — 参数化 pick & place MoveIt demo（规划场景物体 + 全 YAML 参数）【P1】


- **说明：** 对标 `rebotarm_moveit_demos` 的 `pick_place`：ready/pick/place 位姿、TCP RPY、物体尺寸全部 YAML 参数化，MoveIt 规划场景加入碰撞物体，完成「接近→下降→合爪→抬起→转移→放置→开爪」整链；与画矩形 demo 同级，作为真机 Plan+Execute 回归与演示基线。A3 现有 `draw_rectangle_demo.py` 仅覆盖画线。
- **验收标准：**
  1. mock 栈一键启动 demo 并全程成功，RViz 可见规划场景物体与抓取路径
  2. 参数（位姿/尺寸/夹爪开合）改 YAML 即可适配不同桌面布局，无需改代码
  3. 真机执行受 gate/限位/容差约束；回归脚本可跑
- **关联：** F11（统一 MoveIt Execute launch + 矩形 demo）、F76（Pilz PTP/LIN）、F98（夹爪限位标定）；[reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2) `rebotarm_moveit_demos`
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
