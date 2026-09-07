# LL-010 — GripperCommand.timeout_s 被忽略：软物体力环未收敛就 ERR_GRASP_TIMEOUT

> **日期：** 2026-09-06
> **产品线：** Edge
> **环境：** RK3588 真机（lubancat）+ ROS 2 Humble，单电机台架 can1 / ID7，泡棉软物体

## 现象

泡棉夹取力控 0.3 Nm，命令里传了 `timeout_s: 12.0`，服务返回 success，但约 5 s 后进 FAULT：

```
state: FAULT, error_code: 2（ERR_GRASP_TIMEOUT）
```

而实测力矩已达 0.286 Nm（接近目标 0.3，±10% 带内），只是没来得及进 GRASPED 判定窗口。

## 根因

`_cmd_cb` 正确算出 `timeout` 并传入 `_start_force(tau, timeout)`，但 `_start_force` **没有存储该参数**；`_tick_force` 的超时判断恒用配置参数 `self._grasp_timeout_s`（5.0 s）。命令级 `timeout_s` 形同虚设。软物体（泡棉）压缩刚度低，力环收敛慢（>5 s），于是超时误报。

## 正确做法 / 规避

- 服务命令里凡是「本次生效」的参数，回调里要落成实例变量并在执行循环中引用，不能只做函数形参。
- 软物体力控测试时接触检测阈值（`contact_detect_min_nm`=0.1、`0.35×target`）低于自由闭合的动态力矩时会提前切入 PI、进一步拖慢收敛——调小目标力档位（如 0.3 Nm）时尤其要给足 `timeout_s`（本次实测 12 s 够用）。
- 修复点：`gripper_controller_node.py` —— `_start_force` 存 `self._force_timeout_s`（缺省/非正回落 `grasp_timeout_s`），`_tick_force` 改用它。

## 相关路径

- `src/a3_gripper_controller/a3_gripper_controller/gripper_controller_node.py`
- `src/a3_gripper_controller/config/gripper_config.yaml`（`grasp_timeout_s` / `contact_detect_*`）
- `docs/edge/REQUIREMENTS.md`（F30）
