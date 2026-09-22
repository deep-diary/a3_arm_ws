# LL-097 — rcl --params-file：每个顶层 key 必须是节点名且含 ros__parameters

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble

## 现象

F89 标定工具 `scripts/gravity_scale_calibration.py` 首版产出的 yaml 在顶层放了一个裸元数据 map（`gravity_calibration_metadata: {date: ..., r_squared: ...}`）。a3_bringup 把该文件作为追加 ParameterFile 传给 ros2_control_node 后，节点启动即 abort：

```
Couldn't parse params file: ... Cannot have a value before ros__parameters at line 11
```

ros2_control_node 死亡，spawner 一直 list_controllers 重试；表面症状是使能服务反复报「position check failed: no /joint_states received yet」。

## 根因

rcl 的 params 文件只接受一种结构：**每个顶层 key 是节点名，其下必须有 `ros__parameters` map**。顶层直接出现值（裸 map / 标量）非法。另有两条连带约束：

- 数组必须同质（全 double 或全 string），混用报 parse error；
- `null` / `~` 叶子非法（Python yaml 里 `None` 会被 dump 成 null）。

## 正确做法 / 规避

- 工具生成参数文件时，载荷一律 `{节点名: {"ros__parameters": {...}}}`。
- 元数据塞不进目标节点时，挂到一个**合成节点名**下（如 `gravity_calibration_metadata`）：加载时没有同名节点，rcl 静默忽略，不报错。
- 无意义的浮点值（如 low-excitation 关节没有 R²）编码成哨兵浮点（`-1.0`），不要用 null。

## 相关路径

- `scripts/gravity_scale_calibration.py`（write_output）
- `src/a3_bringup/launch/a3_bringup.launch.py`（gravity_scales_file ParameterFile）
