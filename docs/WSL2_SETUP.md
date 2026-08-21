# WSL2 Ubuntu 22.04 开发环境搭建清单（给 Agent / 人工执行）

> **目标读者：** 在本机（Windows）上执行环境搭建的 Agent 或开发者。  
> **目标结果：** WSL2 内 Ubuntu 22.04 + ROS 2 Humble 可编译 `a3_arm_ws`，并能用 **mock 硬件** 跑通 RViz / MoveIt demo（不接真电机）。  
> **明确不做：** 在 WSL 里调板载 SocketCAN / 真电机（留给 RK3588 或 USB-CAN 台架）。

相关文档：

- 工作区总览：[`../README.md`](../README.md)
- 上板 CAN：[`PLATFORM_CAN.md`](PLATFORM_CAN.md)
- 架构：[`ARCHITECTURE.md`](ARCHITECTURE.md)

---

## 0. 验收标准（做完必须全部打勾）

在 WSL Ubuntu 里执行后应满足：

1. `lsb_release -a` 显示 **Ubuntu 22.04**
2. `echo $ROS_DISTRO` 为 **humble**
3. 源码位于 **Linux 文件系统**（例如 `~/dev/a3_arm_ws`），**不是**长期在 `/mnt/d/...` 上编译
4. `colcon build` 至少成功编译：`a3_description`、`a3_moveit_config`、`a3_bringup`、`a3_can_bridge`、`a3_teleop_ps4`（`a3_lerobot_config` 可选）
5. 能启动 MoveIt mock demo 或 description 显示，RViz 可见机械臂模型（WSLg 图形）
6. 本文末尾「验收命令」全部通过

---

## 1. Windows 侧：安装 / 启用 WSL2 + Ubuntu 22.04

在 **Windows PowerShell（管理员）** 执行：

```powershell
wsl --status
wsl --list --verbose
```

若尚未安装：

```powershell
wsl --install -d Ubuntu-22.04
```

若已安装其他发行版，仍建议单独装 22.04：

```powershell
wsl --install -d Ubuntu-22.04
```

设置默认版本为 WSL2（如需要）：

```powershell
wsl --set-default-version 2
wsl --set-default Ubuntu-22.04
```

首次启动 Ubuntu，按提示创建 UNIX 用户名/密码。

**可选（推荐）：** 确认 WSLg 可用（Win11 通常自带），便于 RViz：

```powershell
wsl -d Ubuntu-22.04 -- echo $DISPLAY
```

应有非空输出（常见如 `:0`）。

---

## 2. 进入 WSL，基础系统准备

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y \
  curl gnupg lsb-release ca-certificates \
  build-essential cmake git wget \
  python3-pip python3-venv \
  locales

sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
```

---

## 3. 安装 ROS 2 Humble（官方 apt）

按 [ROS 2 Humble Ubuntu 安装](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 执行（摘要）：

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-humble-desktop \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  python3-colcon-common-extensions python3-rosdep python3-vcstool

sudo rosdep init 2>/dev/null || true
rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO   # 必须是 humble
```

若 `ros-humble-desktop` 体积过大、磁盘紧张，可改为：

```bash
sudo apt install -y ros-humble-ros-base \
  ros-humble-rviz2 ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  python3-colcon-common-extensions python3-rosdep
```

---

## 4. 放置源码（关键：用 Linux 盘，不要用 /mnt/d 编译）

Windows 工作区当前在：`d:\working\dev\a3_arm_ws`  
WSL 里对应盘符一般是：`/mnt/d/working/dev/a3_arm_ws`

**推荐做法：复制到 `~/dev/`（ext4），再开发。**

```bash
mkdir -p ~/dev
# 首次同步（排除 Windows 构建垃圾）
rsync -a --delete \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  --exclude '.git/' \
  /mnt/d/working/dev/a3_arm_ws/ ~/dev/a3_arm_ws/

# vendor 可选（首期 mock 不强制）
rsync -a --delete \
  --exclude '.git/' \
  /mnt/d/working/dev/a3_arm_vendor/ ~/dev/a3_arm_vendor/ || true

cd ~/dev/a3_arm_ws
```

说明：

- `src/third_party` 在 Windows 上可能是 **junction**；rsync 到 Linux 后若为空或异常，首期可忽略（目录内已有 `COLCON_IGNORE`）。
- 日常改代码：可继续在 Windows Cursor 编辑 `/mnt/d/...`，改完再 `rsync` 进 `~/dev`；或直接用 Cursor Remote/WSL 打开 `~/dev/a3_arm_ws`。

**禁止：** 在 `/mnt/d/working/dev/a3_arm_ws` 上长期 `colcon build`（慢、权限/可执行位问题多）。

---

## 5. 依赖与编译

```bash
source /opt/ros/humble/setup.bash
cd ~/dev/a3_arm_ws

# 仅解析自有包（忽略 third_party）
rosdep install --from-paths src/a3_description src/a3_moveit_config \
  src/a3_can_bridge src/a3_bringup src/a3_teleop_ps4 src/a3_lerobot_config \
  --ignore-src -r -y

colcon build --symlink-install --packages-select \
  a3_description \
  a3_moveit_config \
  a3_can_bridge \
  a3_bringup \
  a3_teleop_ps4 \
  a3_lerobot_config

source install/setup.bash
```

若 `a3_can_bridge` 因缺系统头文件失败，安装：

```bash
sudo apt install -y libsocketcan-dev
# 或确保 linux-libc-dev 已装
sudo apt install -y linux-libc-dev
```

若暂时编不过 `a3_can_bridge`，**至少保证** mock 相关包通过：

```bash
colcon build --symlink-install --packages-select \
  a3_description a3_moveit_config a3_bringup
```

---

## 6. Mock / 仿真冒烟（WSL 内必须跑通）

### 6.1 仅显示 URDF（最轻）

```bash
source /opt/ros/humble/setup.bash
source ~/dev/a3_arm_ws/install/setup.bash

ros2 launch a3_description el_a3_control.launch.py use_mock_hardware:=true use_rviz:=true
```

若该 launch 参数名略有差异，可用：

```bash
ros2 launch a3_description el_a3_control.launch.py --show-args
```

按实际参数启动；核心是 **`use_mock_hardware:=true`** 且 RViz 能显示臂。

### 6.2 MoveIt Demo（推荐验收）

```bash
source /opt/ros/humble/setup.bash
source ~/dev/a3_arm_ws/install/setup.bash

ros2 launch a3_moveit_config demo.launch.py
```

预期：RViz + MoveIt 规划组，拖动目标可规划（mock，无真实电机）。

### 6.3 图形若黑屏 / 无 DISPLAY

```bash
echo $DISPLAY
# Win11 WSLg 一般自动有 :0
# 若为空，更新 WSL：在 Windows PowerShell 执行  wsl --update
```

可先无 GUI 做话题级检查：

```bash
ros2 launch a3_moveit_config demo.launch.py use_rviz:=false
# 另开终端
source /opt/ros/humble/setup.bash && source ~/dev/a3_arm_ws/install/setup.bash
ros2 topic list
ros2 topic echo /joint_states --once
```

---

## 7. WSL 明确不做的事（避免 Agent 误做）

| 事项 | 原因 |
|------|------|
| `ip link set can0 up` / 真 SocketCAN | WSL2 通常无板载 CAN；不要当作 RK3588 验证 |
| 同时开 MotorBridge 抢总线 | 真机台架阶段再做，且与板端互斥 |
| 在 `/mnt/d` 上 colcon | 性能与权限差 |
| 把 Windows 编好的 `install/` 拷到板子 | ABI/架构不对，必须在目标环境重编 |

真 CAN / 电机请走：

1. PC USB-CAN 台架（可选），或  
2. RK3588 + [`PLATFORM_CAN.md`](PLATFORM_CAN.md)

---

## 8. 建议的日常工作流（写给后续开发）

```text
Windows Cursor 改代码（可选）
    → rsync 到 WSL ~/dev/a3_arm_ws
    → WSL: colcon build + mock/MoveIt 验证
    → 通过后把同一源码同步到 RK3588
    → 板上: can-up + 只验证 a3_can_bridge / 实机
```

上板最小集（提醒，非本清单执行范围）：

- 拷贝/同步 `~/dev/a3_arm_ws` 源码（不要依赖 Windows junction）
- 按 `PLATFORM_CAN.md` 起 `can0`
- `colcon build` 后：`ros2 launch a3_bringup a3_bringup.launch.py`

---

## 9. Agent 执行顺序（照做即可）

1. 检查/安装 WSL2 + `Ubuntu-22.04`
2. 进入发行版，`apt update/upgrade`，装基础工具
3. 安装 ROS 2 Humble + MoveIt + ros2_control + colcon + rosdep
4. `rsync`：`/mnt/d/working/dev/a3_arm_ws` → `~/dev/a3_arm_ws`
5. `rosdep install` + `colcon build`（包列表见第 5 节）
6. 跑 `demo.launch.py` 或 mock description launch
7. 把本节「验收命令」输出保存/回报给用户

---

## 10. 验收命令（复制整段执行）

```bash
set -e
source /opt/ros/humble/setup.bash
source ~/dev/a3_arm_ws/install/setup.bash

echo "=== OS ==="
lsb_release -a | grep -E 'Description|Codename'

echo "=== ROS ==="
test "$ROS_DISTRO" = "humble" && echo "ROS_DISTRO=humble OK"

echo "=== Workspace path ==="
pwd
realpath ~/dev/a3_arm_ws | grep -v '^/mnt/' && echo "Not on /mnt (good)" || echo "WARN: still under /mnt"

echo "=== Packages ==="
ros2 pkg prefix a3_description
ros2 pkg prefix a3_moveit_config
ros2 pkg prefix a3_bringup

echo "=== Launch files exist ==="
ros2 pkg prefix a3_moveit_config
test -f $(ros2 pkg prefix a3_moveit_config)/share/a3_moveit_config/launch/demo.launch.py && echo "demo.launch.py OK"

echo "ALL CHECKS DONE"
```

可选（有显示器时）：

```bash
timeout 30s ros2 launch a3_moveit_config demo.launch.py use_rviz:=false \
  || true
# 能拉起节点、不因缺包立刻崩溃即可；完整 GUI 需人工看 RViz
```

---

## 11. 故障速查

| 现象 | 处理 |
|------|------|
| `rosdep init` 已存在 | 忽略，继续 `rosdep update` |
| `colcon` 找不到包 | 是否在 `~/dev/a3_arm_ws` 且已 `source install/setup.bash` |
| RViz 无法显示 | `wsl --update`；检查 `$DISPLAY`；先 `use_rviz:=false` |
| `a3_can_bridge` 编译失败 | 装 `linux-libc-dev`；mock 验收可不依赖真 CAN |
| `/mnt/d` 下极慢 | 立刻迁到 `~/dev` |
| `third_party` 空 | 正常可忽略；需要时在 WSL 内重新 `git clone` 到 `~/dev/a3_arm_vendor` |

---

## 12. 完成后给用户的简短回报模板

```text
- WSL 发行版: Ubuntu-22.04 (WSL2)
- ROS: humble
- 源码路径: ~/dev/a3_arm_ws
- 已编译包: a3_description, a3_moveit_config, a3_bringup, ...
- Mock/MoveIt: 已启动 / 失败原因
- 未做: 真 CAN、RK3588、MotorBridge
```
