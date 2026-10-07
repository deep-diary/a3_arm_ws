# F63 — joynet Linux DS4 独立布局（pygame/SDL 6 轴 + hat，对齐 joy_node 协议）


- **说明：** joynet 在 RK3588（Ubuntu 22.04）上经 pygame/SDL 读取 DS4，真机实测（2026-09-20，蓝牙 `Wireless Controller`）与 Windows 开发环境不同：`axes=6`（0/1 左摇杆、2=L2、3/4 右摇杆、5=R2；摇杆上/左为负、右/下为正；扳机静息 −1、按满 +1），`buttons=13`（0 cross、1 circle、2 triangle、3 square、4 l1、5 r1、8 share、9 options、10 ps、11 l3、12 r3；6/7 是扳机数字点击），十字键是 **hat 0** 不是轴。原 `DS4_LINUX` 表把十字键放在轴 6/7（该 8 轴形态只存在于 joydev/`joy_node` 侧）且 `detect_layout` 对 6 轴手柄错判为 `ds4_sdl`，导致用户实测键位全错。修正：`DS4_LINUX` 改 `dpad="hat"`、摇杆 polarity +1（SDL 已是右/下正），扳机仍走 `(v+1)/2`；`detect_layout` 对「Wireless Controller + 6 轴 + 轴 2 静息 −1（扳机）/ 1 hat」选 `ds4_linux`。joynet 以 **TCP client** 接入 `a3_teleop_ps4` 现成的 `ds4_tcp_joy_node`（127.0.0.1:8890），由其发布标准 ds4_linux 协议 `/joy`（8 轴 / 14 键），ROS 侧不新增代码。
- **验收标准：**
  1. `layout: auto` 时本机手柄自动选中 `ds4_linux`
  2. `ds4_linux` 抽象快照：摇杆推上/左输出负、右/下正，扳机静息 0 / 按满 1，十字键四方向具名按钮正确
  3. joynet 自带 pytest 全绿；端到端 `ds4_tcp_joy_node` 发布的 `/joy` 与 `a3_teleop_ps4/config/ds4_linux.yaml` 契约一致（axes `[LX,LY,L2,RX,RY,R2,DPAD_X,DPAD_Y]`、buttons `[cross,circle,triangle,square,l1,r1,l2,r2,share,options,ps,l3,r3,touch]`）
- **关联：** F60（PS4 键位）、F62（合成 /joy 仿真）；LL-060（SDL 6 轴 + hat vs joydev 8 轴、ds4_sdl 误判、venv pytest 污染）
- **状态：** `in-progress`（2026-09-20，代码+pytest 61 全绿、auto 检测/快照极性/dump 均已实测；evdev 原始事件证实 joydev 侧契约上=−1/下=+1（LL-061）。TCP 桥端到端仅完成分段验证+空闲帧，剩一次带按键的实时帧采集）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
