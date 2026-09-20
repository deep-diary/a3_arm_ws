# LL-060 Linux DS4 两套原始协议不可混用：pygame/SDL 是 6 轴+hat（扳机静息 −1），joydev/joy_node 是 8 轴（扳机静息 +1）；ROS PYTHONPATH 污染 venv pytest 报缺 lark

- **日期**：2026-09-20
- **产品线**：Edge（joynet 真机蓝牙 DS4 联调，F63）
- **严重度**：中高（按键/摇杆/十字键整体错位，用户实测即发现；另有换机必踩的测试环境坑）

## 现象

joynet 在 RK3588（蓝牙 DS4，pygame 2.6.1 / SDL 2.28.4，设备名
`'Wireless Controller'`）上以 `layout: auto` 启动后自动选中了 `ds4_sdl`（Windows
表），用户实测反馈「按键分布跟之前 win 开发的时候不一样」：摇杆方向、十字键、面键
索引全错位。同时在 venv 内跑 `pytest` 收集阶段直接崩：
`ModuleNotFoundError: No module named 'lark'`，traceback 指向
`/opt/ros/humble/.../launch/frontend/parse_substitution.py`。

## 根因

**两套 Linux 原始协议被当成一套：**

| | pygame/SDL（joynet 实际使用） | joydev `/dev/input/js0`（ros-humble-joy 的 joy_node） |
|---|---|---|
| 轴数 | **6** | **8** |
| 轴序 | 0 LX,1 LY,**2 L2**,3 RX,4 RY,**5 R2** | 0 LX,1 LY,2 L2,3 RX,4 RY,5 R2,**6 dpad_x,7 dpad_y** |
| 摇杆极性 | 右/下为正（SDL 标准） | raw 左/上为正 |
| 扳机静息 | **−1**，按满 +1 | **+1**，按满 −1 |
| 十字键 | **hat 0** | 轴 6/7（虚拟按键） |
| 按键数 | 13（6/7 是 L2/R2 数字点击） | 13/14 |

旧的 joynet `DS4_LINUX` 表按 joydev 形态写（dpad 放轴 6/7、摇杆 polarity 写反），
在 pygame 设备上轴 6/7 根本不存在、十字键 hat 被忽略；`detect_layout` 对
6 轴/13 键/1 hat 的设备又落进 Windows `ds4_sdl` 分支（该表右摇杆在轴 2、L2 在轴 4）。

**pytest 坑**：shell source 过 ROS 环境后 `PYTHONPATH` 含
`/opt/ros/humble/lib/python3.10/site-packages`，pytest 启动时按 setuptools
entrypoints 从该路径加载 ROS 侧 pytest 插件 → import `launch.frontend` → `lark`
（deb 装在 `/usr/lib/python3/dist-packages`）不在 venv 的 sys.path 里，收集即崩，
测试本身一个都没跑到。

## 修复 / 规避

- joynet 新增独立 `ds4_linux` 表：6 轴（摇杆 polarity +1、扳机 2/5）、13 键、
  `dpad="hat"`；`detect_layout` 用「≥6 轴 ≥13 键且 idle axis2 < −0.5」判定 SDL
  形态，按设备名兜底（hat≥1 → linux）。两表 docstring 互相警告不可混用。
- ROS 侧**不新增桥**：joynet 以 TCP client 接入现成 `ds4_tcp_joy_node`，由它把
  抽象快照（右/下正、l2/r2 ∈[0,1]）转成 `config/ds4_linux.yaml` 的 joydev 8 轴
  /joy——极性转换只允许存在于这一个边界函数里。
- venv 跑 pytest 必须剥离 ROS 变量：
  `env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH .venv/bin/python -m pytest -q`。

## 教训

- **手柄原始枚举必须先写一个 60 s 事件打印脚本实测，再写映射表**；名字相同
  （"Wireless Controller"）的设备在不同输入栈（SDL vs joydev）下轴数/极性/静息值
  都不一样，「参考 joy_node 协议」时要明确参考的是哪一层，不能照抄索引。
- 判定扳机轴别看索引，看**静息值**（SDL −1 vs joydev +1）；判定十字键来源看
  **轴数和 hat 数**。这两条不受平台/连接方式（蓝牙/有线）影响。
- ROS source 的环境与项目自带 venv 天然冲突（PYTHONPATH 注入 + site-packages
  隔离），在 venv 中执行任何带插件机制的 Python 工具（pytest 等）都应 `-u PYTHONPATH`。

## 相关路径

- `src/third_party/joynet/src/joynet/layout.py`（DS4_LINUX 表、detect_layout）
- `src/third_party/joynet/tests/test_layout.py`（真机形态回归用例）
- `src/a3_teleop_ps4/a3_teleop_ps4/ds4_tcp_joy_node.py`（快照→joydev 8 轴 /joy 边界）
- `src/a3_teleop_ps4/config/ds4_linux.yaml`（A3 栈 /joy 契约）
- 关联：F63、F60；LL-059（同批真机联调）
