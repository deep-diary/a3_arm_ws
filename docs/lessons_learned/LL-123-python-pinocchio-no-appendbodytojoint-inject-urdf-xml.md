# LL-123 — Python pinocchio 不暴露 appendBodyToJoint，负载模型改为向 URDF XML 注入 link + fixed joint

> **日期：** 2026-09-24  
> **产品线：** Edge  
> **环境：** RK3588 (lubancat) Ubuntu 22.04 + ROS 2 Humble + ros-humble-pinocchio（Python 绑定）

## 现象

F107 要在既有机械臂模型上动态挂载末端点质量（负载可变、需运行时重建），按 C++ pinocchio 常见写法调 `pin.appendBodyToJoint(model, joint_id, inertia, placement)`，Python 绑定直接 `AttributeError: module 'pinocchio' has no attribute 'appendBodyToJoint'`。

## 根因

apt 安装的 ros-humble-pinocchio Python 绑定只暴露部分建模 API，`appendBodyToJoint` / `appendBodyToJoint` 这类增量建模函数未绑定（版本/打包裁剪），不是参数写错。

## 正确做法 / 规避

不增量改模型，直接改 URDF 源文本再整体重建：

1. 从 `robot_description` 参数拿 URDF XML 字符串；
2. 在 `</robot>` 前注入负载 link（`<mass>` + 极小惯性张量 1e-6）与 fixed joint（parent 为挂载点 `gripper_link`，`<origin xyz="质心偏移">`）；
3. `pin.buildModelFromXML(xml)` 重建；
4. 关节索引不要假设顺序，用 `model.idx_qs[model.getJointId(name)]`（位置）/ `model.idx_vs[jid]`（速度）建映射；el_a3 上两映射恰为恒等 [0..6]，但换构型不能依赖。

改负载 = 重新注入 + 重建（模型很轻，单次服务调用内完成）。零负载时不注入任何东西，用原 URDF。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_ensure_pin_model`）
- `src/a3_msgs/srv/SetPayload.srv`
