# LL-002 — Edge 仿真 RViz / MoveIt demo：HDMI 显示、colcon overlay、缺包

> **日期：** 2026-08-26  
> **产品线：** Edge（仿真 / mock，无 CAN）  
> **环境：** LubanCat aarch64 · Ubuntu 22.04 · ROS 2 Humble · 板载 HDMI + SSH

## 现象

1. `ros2 launch a3_bringup edge_sim_wave_a.launch.py` **不弹 RViz**（即便传了 `use_rviz:=true` 也曾无效）。
2. SSH 进板子后设 `DISPLAY=${DISPLAY:-:0}`，RViz **画不到 HDMI**（或画到笔记本 / 失败）。
3. 新开终端跑 `ros2 launch a3_moveit_config demo.launch.py use_rviz:=true`：
   - `package 'controller_manager' not found`，搜索路径里全是 `micro_ros_host/...` 和 `trotbot_ws`，`/opt/ros/humble` 在末尾仍找不到包。
   - 装上 `controller_manager` 后：`pick_ik`、`ompl_interface/OMPLPlanner`、`MoveItSimpleControllerManager` 插件缺失，`move_group` 崩溃。
   - 栈能起、日志有 `You can start planning now!`，但 RViz：**MotionPlanning 插件加载失败**，画面没有机械臂实际模型、没有拖动球 / Plan / Execute。
4. 用户 pip 为 NumPy 2.x 时 `import pinocchio` 崩溃（Wave A 重力节点）。

## 根因

### RViz 没起来 / 没有拖动球

| 表现 | 原因 |
|------|------|
| Wave A 不弹 RViz | `edge_sim_wave_a.launch.py` 曾声明 `use_rviz` 但未接线；默认 `false` 是给无屏脚本用的 |
| Wave A 没有 Plan/Execute | 该 launch **没有** `move_group`，用的是 `el_a3_view.rviz`（仅 RobotModel/TF）。拖动球属于 MoveIt 插件 |
| demo 有 RViz 但无 MotionPlanning | 未装 `ros-humble-moveit-ros-visualization`；RViz 声明的 Display 只有 `rviz_default_plugins/*` |
| demo 看不见「实际模」 | `moveit.rviz` 关掉了独立 `RobotModel`（避免与 Scene Robot 叠两层）。插件没加载时 **两边都没有** |

### HDMI vs SSH 的 DISPLAY

- SSH `-X`/`-Y` 会把 `DISPLAY` 设成 `localhost:10.0`。
- `${DISPLAY:-:0}` **只在变量为空时**才落到 `:0`；已有转发值时不会改，RViz 不会画到板载 `:0`。
- 看板载屏必须 **写死** `export DISPLAY=:0`，且 SSH **不要** `-X`/`-Y`。本机图形会话在 `tty2` / `/tmp/.X11-unix/X0`。

### `controller_manager not found` 其实是两件事叠在一起

1. **包真没装：** `ros-humble-joint-trajectory-controller` 在，不等于 `controller_manager` 在。demo 还要：
   - `ros-humble-controller-manager` / `ros-humble-ros2-control`
   - `ros-humble-pick-ik`
   - `ros-humble-moveit-planners-ompl`
   - `ros-humble-moveit-simple-controller-manager`
   - `ros-humble-moveit-ros-visualization`（RViz MotionPlanning）
2. **`install/setup.bash` 会链 underlay：** colcon 生成该文件时，当时环境里已经 source 了 micro-ROS host 和 `trotbot_ws`。之后每次 `source ~/a3_arm_ws/install/setup.bash` 都会把整串 prefix 加回来。Edge 仿真不需要 XRCE，却被这串 overlay 污染。

对比：

| 文件 | 作用 |
|------|------|
| `install/setup.bash` | 本工作区 **+ 生成时记下的所有 underlay**（humble / micro-ROS / trotbot） |
| `install/local_setup.bash` | **仅本工作区**，不把 micro-ROS 再链进来 |

`package.xml` 里的 `<exec_depend>` **不会**自动 `apt install`。

### 其它

- **CRLF：** Windows/Cursor 写入的 `*.sh` 会出现 `/usr/bin/env: 'bash\r'` 或 `case $- in\r`。仓库 `.gitattributes` 约束 `*.sh` → LF。
- **Pinocchio：** `ros-humble-pinocchio` 对系统 NumPy 1.x；`~/.local` 的 NumPy 2.x 会崩。Wave 脚本与 `a3_shell_env.sh` 设 `PYTHONNOUSERSITE=1`。
- **wmctrl：** `.rviz` 只能写死窗口宽高；真正最大化要 `wmctrl`。没装时 launch 里的 maximize 步骤静默跳过。

## 正确做法 / 规避

1. **新开终端用 Edge 环境（已写入 `~/.bashrc`）：**
   ```bash
   source ~/a3_arm_ws/scripts/a3_shell_env.sh
   # 内部：humble → install/local_setup.bash，并剥掉 AMENT 里的 micro_ros_host
   # DISPLAY=:0（存在 /tmp/.X11-unix/X0 时）；GUI 去笔记本则 A3_KEEP_DISPLAY=1
   ```
   **不要**再 `source src/third_party/micro_ros_host/setup_microros.bash`（CloudEdge XRCE 专用）。

2. **HDMI 上看 RViz（SSH）：**
   ```bash
   export DISPLAY=:0    # 写死，不用 ${DISPLAY:-:0}
   ```

3. **只看轨迹播放（无拖动球）：**
   ```bash
   ros2 launch a3_bringup edge_sim_wave_a.launch.py duration_s:=3.0 use_rviz:=true
   ```

4. **拖动球 + Plan / Execute（mock，不发 CAN）：**
   ```bash
   sudo apt install -y ros-humble-ros2-control ros-humble-controller-manager \
     ros-humble-pick-ik ros-humble-moveit-planners-ompl \
     ros-humble-moveit-simple-controller-manager \
     ros-humble-moveit-ros-visualization wmctrl
   ros2 launch a3_moveit_config demo.launch.py use_rviz:=true
   ```
   Interact → 拖橙色球 → 面板 Plan / Execute。橙色半透明 = 目标模；Scene Robot = `/joint_states` 实际模。

5. **换机 / 重编后若 `setup.bash` 又链上 micro-ROS：** 用 `local_setup.bash` 或 `a3_shell_env.sh`；干净重建时先只 `source /opt/ros/humble/setup.bash` 再 `colcon build`。

6. **脚本换行：** `sed -i 's/\r$//' scripts/*.sh`；`bash -n scripts/a3_shell_env.sh`。

## 相关路径

- [`scripts/a3_shell_env.sh`](../../scripts/a3_shell_env.sh)
- [`src/a3_bringup/launch/edge_sim_wave_a.launch.py`](../../src/a3_bringup/launch/edge_sim_wave_a.launch.py)
- [`src/a3_moveit_config/launch/demo.launch.py`](../../src/a3_moveit_config/launch/demo.launch.py)
- [`src/a3_moveit_config/config/moveit.rviz`](../../src/a3_moveit_config/config/moveit.rviz)
- [`README.md`](../../README.md)（无硬件仿真运行）
- [`docs/edge/QUICKSTART.md`](../edge/QUICKSTART.md)
- [`LL-001`](LL-001-microros-host-setup.md)（micro-ROS 构建本身；本条强调 **Edge 不要 source 它**）
