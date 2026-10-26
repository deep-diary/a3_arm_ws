# LL-169 — 改 `default.yaml` / melodion stream 不生效：这两个包安装树是**拷贝**不是符号链接

> **日期：** 2026-10-16
> **产品线：** Edge
> **环境：** lubancat（RK3588）Ubuntu 22.04 + ROS 2 Humble（真机栈）

## 现象

改了 `src/a3_teleop_ps4/config/mappings/default.yaml`（PS4 D-pad 重绑、Triangle 长按改指 stream）与
`src/app_melodion/config/streams/*.yaml`，`a3_stack.sh restart` 后**完全不生效**：

- 手柄按 D-pad 跑的还是老映射（`melodion_F3_up/down`、甚至更早的 `home_up/home_down`），长按 Triangle 仍跑 `modes_sweep`；
- 新生成的 `streams/melodion_pts.yaml` 在 `install/` 里根本不存在 → `playback {name: ...}` 报找不到；
- 安装树里还留着已删的 `scale.yaml.baked.yaml` / `inspection.yaml.baked.yaml`（旧烘焙缓存会被 `_play_stream` 优先命中）。

`ps4_mapper ready mapping=...` 日志显示消费路径正是 `install/a3_teleop_ps4/share/.../mappings/default.yaml`。

## 根因

本仓多数包用 `colcon build --symlink-install` 构建（例如 `a3_description` / `a3_description_6j` 的
`config/named_poses.yaml` 是指向 `src/` 的符号链接，改源码即时生效）。但 **`a3_teleop_ps4` 与
`app_melodion` 的安装树里是普通文件拷贝**（时间戳停在更早的构建），且 `--symlink-install` 重新构建
`app_melodion` 时对新增文件报 `error: [Errno 2] No such file or directory: .../install/.../melodion_pts.yaml`
而整包构建失败。

因此「改源码 → `a3_stack.sh restart`」这条路径对这两个包的**配置类文件无效**，必须重新构建；
且 `colcon` 重新安装**不会删除**已被移除的旧文件（`.baked.yaml` 残留仍会被消费）。

## 正确做法 / 规避

1. 改 `a3_teleop_ps4`（mapping/action_registry）或 `app_melodion`（`config/streams/*`、`playing_geometry.yaml` 等）
   后，**必须** `colcon build --packages-select a3_teleop_ps4 app_melodion`，再重启栈（mapping 在节点启动时加载）。
   该类配置改动后**不能**只 restart。
2. 若 `--symlink-install` 对该包报错，可去掉该 flag 用拷贝安装（这两包历史上就是拷贝安装）：
   `rm -rf install/app_melodion install/a3_teleop_ps4 && colcon build --packages-select app_melodion a3_teleop_ps4`。
3. 构建后核对安装树：
   `grep -n 'name: <期望值>' install/a3_teleop_ps4/share/a3_teleop_ps4/config/mappings/default.yaml`；
   `ls install/app_melodion/share/app_melodion/config/streams/`（新文件在、旧 `*.baked.yaml` 不在）。
4. 删过 `*.baked.yaml`（预烘焙缓存）时，`install/` 侧要一并删（或整包重装），否则 playback 仍用旧缓存
   （`_play_stream` 命中 `<path>.baked.yaml` 优先）。

> 判据：改完配置后，**先看 `install/` 里的那份是不是符号链接**（`ls -l`）。
> 是链接 → 改源码即生效；是普通文件 → 必须重建。

## 追加：`a3_bringup` 的 launch 也是拷贝安装（2026-10-26）

改 `src/a3_bringup/launch/a3_bringup.launch.py` 里的节点参数（例：把
`{"max_ik_jump": 0.05}` 收到 `0.01`）后只 `restart`，运行时 `ros2 param get` 仍是 **0.05**：

- `install/a3_bringup/share/a3_bringup/launch/…` 是**普通文件**（`readlink -f` 指回 install 自身），
  源码改动不会同步；
- 更隐蔽的是：**launch 里的显式参数会覆盖节点代码里的 `declare_parameter` 默认值**——
  只改 C++ 默认值同样无效，必须两处一起改（launch + 代码默认值），否则排查时会一直盯着代码。

做法：`colcon build --packages-select a3_bringup --symlink-install`（该包 `--symlink-install` 能过，
之后 install 侧变符号链接）→ 重启栈 → `ros2 param get <node> <param>` 核对。
判据同前：先 `readlink -f` 看 install 侧是不是链接，再重新读参数确认落地。

## 相关路径

- `src/a3_teleop_ps4/config/mappings/default.yaml`、`src/a3_teleop_ps4/config/action_registry.yaml`
- `src/app_melodion/config/streams/`、`src/app_melodion/config/playing_geometry.yaml`
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_stream_path`、`_play_stream`、`_prepare_playback`）
- 对照：`src/a3_description[_6j]/config/named_poses.yaml`（安装树为符号链接，改源码即生效）
