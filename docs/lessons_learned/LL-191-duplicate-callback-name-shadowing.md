# LL-191 — 新增 service 回调被同名的订阅回调覆盖（`_on_status` 重名）

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble

## 现象

给 `face_track_controller` 新增 `/a3/face_track/status` 服务后，调用它导致整节点崩溃：

```
TypeError: FaceTrackController._on_status() takes 2 positional arguments but 3 were given
[ERROR] [face_track_controller]: process has died
```

随后其它 `/a3/face_track/*` 调用都报 `rcl node's context is invalid`。

## 根因

类里早就有订阅回调 `_on_status(self, msg: ArmStatus)`（订阅 `/a3/arm_status`）。
新加的服务回调也叫 `_on_status(self, req, resp)` —— **Python 类体里后定义的同名方法
直接覆盖前者**，`create_service` 拿到的是订阅版签名，服务调用传入 `(req, resp)`
两个参数 → TypeError → 节点退出。

## 正确做法 / 规避

1. 命名区分：服务用 `_on_track_status` / `_on_set_target`，订阅保持 `_on_status`。
2. 新增回调后除了编译，**实际 call 一次服务**（`ros2 service call ... Trigger`）验证；
   只 `service list` 有名字不代表能跑。
3. 节点"莫名死掉"先看日志 `Traceback` + `process has died`，不要只看 `ros2 node list`
   （launch 不会因此退出，其余节点照常运行，容易误判"服务没实现"）。

## 相关路径

- `src/a3_perception/a3_perception/face_track_controller.py`
- `docs/edge/requirements/F191-face-track-calibration.md`
