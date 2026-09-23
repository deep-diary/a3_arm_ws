# LL-126 — add_on_set_parameters_callback 返回的 handle 必须持有，否则参数服务「假成功」

> **日期：** 2026-09-24  
> **产品线：** Edge  
> **环境：** RK3588 (lubancat) Ubuntu 22.04 + ROS 2 Humble

## 现象

F108 位置模式重力前馈的在线比例 `gravity_feedforward_ratio` 不生效：

```text
$ ros2 param set /a3_hardware_health gravity_feedforward_ratio 0.0
Set parameter successful
$ ros2 param get /a3_hardware_health gravity_feedforward_ratio
Double value is: 0.0
```

参数服务器与读回都已是 0.0，但 200 Hz 持续发出的 MIT 位置帧 t_ff 仍是 ratio=1 的 RNEA 全值（L2 −0.778、L3 +3.367、L4 −0.796，与独立 pinocchio RNEA 经 direction 映射后逐位一致）。排除了二进制过期、重复发送节点、模拟器杂帧之后，问题锁定在插件成员没被更新。

## 根因

插件代码只调用、不保存返回值：

```cpp
health_node_->add_on_set_parameters_callback(
  [this](const std::vector<rclcpp::Parameter> & params) { ... });
```

rclcpp 内部以 **weak_ptr** 保存 `OnSetParametersCallbackHandle`。返回的 SharedPtr 当场析构 → 弱引用失效 → 参数服务照常更新存储值并回复 successful，但回调被整体跳过，回调里的成员写入永远不执行。表现就是「set/get 都对、行为不变」，极难从外部察觉。

## 正确做法 / 规避

节点（或插件）以成员持有 handle：

```cpp
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
  on_set_parameters_handle_;

on_set_parameters_handle_ =
  health_node_->add_on_set_parameters_callback(...);
```

通用排查口径：

- 参数 set 成功但行为不变，先查调用方是否持有 callback handle，再查回调所在节点有没有 executor 在 spin（本插件 health 节点由 RxLoop 里 `spin_some(0ns)` 驱动）。
- `ros2 param set/get` 只证明参数存储正确，不证明业务侧已响应；验收必须观测实际行为（本 F108 即直接比对 CAN 帧 t_ff）。

附带一个验收语义坑：GripperActionController 空闲时持续发 **kp=0 的 effort 帧**（t_ff=max_effort=1.0 N·m），与位置模式帧（kp≈80）混在同一总线。检查「L7 位置帧不带重力前馈」必须先按 kp 过滤，否则会把控制器合法的保持力当成失败。

## 相关路径

- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（`on_set_parameters_handle_`）
- `scripts/a3_test/f108_gravity_ff_vcan_acceptance.py`（P1–P5，L7 按 kp>50 过滤）
