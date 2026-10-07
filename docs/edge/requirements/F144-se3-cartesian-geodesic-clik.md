# F144 — SE(3) 笛卡尔测地线轨迹 + CLIK 跟踪【P2】


- **说明：** 对标 `reBotArm_control_py` 的 trajectory 模块：位姿目标走 SE(3) 测地线（位置线性、姿态四元数球面插值）+ CLIK（闭环逆解）实时跟踪，生成平滑笛卡尔路径。本仓 F76 Pilz LIN 提供直线工业路径，本项补「自由姿态插值 + 速度级 IK 跟踪」的轻量实现/评估，用于手柄笛卡尔连续运动与笛卡尔示教。
- **验收标准：**
  1. 给定起止位姿生成 SE(3) 测地线采样，姿态无翻滚跳变、路径误差有界
  2. CLIK 跟踪在奇异点附近降速/报错而不是发散；与 Servo/FJT 路径的仲裁明确
  3. 先仿真（vcan/mock）验收，真机列为后续
- **关联：** F76（Pilz LIN/CIRC，工业路径基线）、C4（Servo 笛卡尔）、F12（MoveToPoseIK）；[reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) `trajectory/`
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
