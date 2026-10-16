# LL-170 — `a3_stack.sh set-zero` 必然失败：`ros2 service call` 不接受 `--no-daemon`

> **日期：** 2026-10-16
> **产品线：** Edge
> **环境：** lubancat（RK3588）Ubuntu 22.04 + ROS 2 Humble（真机栈）

## 现象

`./scripts/a3_stack.sh set-zero 6`（F91 单电机设零）稳定失败：

```
[*] 启动 motor_maintenance（can1 独占；栈已停、控制器 inactive，互锁放行）…
[*] 设零（写零点，不带使能）：电机 6
[x] set_zero 失败：usage: ros2 [-h] ... ros2: error: unrecognized arguments: --no-daemon
```

脚本随即把维护节点 TERM 掉，**零点没写进电机**（`/joint_states` 里 L6 读数不变）。

## 根因

`do_set_zero` 里两条 `ros2 service call … --no-daemon` 用了 Humble **不支持的选项**：
`--no-daemon` 只对 `ros2 topic echo` / `ros2 service list` 等有效，`ros2 service call` 没有该参数
（本机 `ros2 service call -h` 无此项）。同一脚本其它位置（`topic echo`/`service list`）用
`--no-daemon` 是合法的，所以混用长期没暴露；F91 验收脚本走的是 vcan 仿真直连服务，**没覆盖
`a3_stack.sh set-zero` 这条封装分支**，因此 CI/验收全绿但真机第一次用就挂。

## 正确做法 / 规避

1. 已修：`scripts/a3_stack.sh` 的 `do_set_zero` 去掉两处 `ros2 service call … --no-daemon`（仅保留
   `ros2 service list --no-daemon` 的等待轮询）。修后 `./scripts/a3_stack.sh set-zero <id>` 一次通过
   （`set_zero 完成` + `save_parameters 完成`）。
2. 需要绕过 ros2 daemon 发现缓存时：只有 `topic echo` / `service list` 支持 `--no-daemon`；
   `service call` 改为 `timeout N ros2 service call …`，必要时先 `ros2 daemon stop`。
3. 手工等价流程（栈必须已停；维护节点独占 can1）：
   ```bash
   ros2 run a3_hardware_interface motor_maintenance --ros-args \
     -p can_interface:=can1 -p enforce_controller_interlock:=true &
   ros2 service call /a3/maintenance/set_zero a3_msgs/srv/MotorIdCommand "{motor_id: 6}"
   ros2 service call /a3/maintenance/save_parameters a3_msgs/srv/MotorIdCommand "{motor_id: 6}"
   ```
   验证零位：`python3 scripts/mit_noenable_stream.py probe --iface can1 --ids 1..6`
   （目标电机 angle 应 ≈ 0，其余不变）。
4. **`init` ≠ 单电机设零**：产品栈（`motor_service_backend=controller_switch`）下 `/a3/arm/init`
   （L1+R1 长按）走 `/a3/hardware/set_zero {motor_id: 255}` = **把当前整臂位姿设为全部电机零点**。
   单电机重标定后**不要**随手按 init，否则会把其余关节标定一起打乱（只有臂确实摆在 URDF 零位时才可）。

## 相关路径

- `scripts/a3_stack.sh`（`do_set_zero`）
- `src/a3_hardware_interface/src/motor_maintenance_node.cpp`（`/a3/maintenance/set_zero|save_parameters`）
- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（`/a3/hardware/set_zero`，255=全部）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_init_cb` → `_call_hw_set_zero(255)`）
- `src/a3_msgs/srv/MotorIdCommand.srv`（`motor_id` 1..7 单个 / 255 全部）
- `docs/edge/requirements/F91-motor-zero-param-maintenance.md`
