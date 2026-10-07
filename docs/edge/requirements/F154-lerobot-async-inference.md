# F154 — LeRobot 异步推理（PolicyServer / RobotClient gRPC）（A2/A3）【P1】


- **说明：** 对标 LeRobot 官方 async inference（单机/LAN/云三模式）：大策略在 GPU 服务器跑 PolicyServer，RK3588 作 RobotClient，动作块流水化避免空转。gRPC/pickle 接口**仅限内网**（VPN/SSH/安全组限源），不暴露公网；断线时执行层进安全态。
- **验收标准：**
  1. LAN 模式下服务器推理、边缘执行连续不卡 chunk；断网/服务不可用时机械臂安全停车
  2. 接口默认绑定内网 + 鉴权/限源配置；延迟与抖动有实测记录
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A2/A3/6.4 安全边界；F153/F155
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
