# LL-034 看门狗节点两个隐蔽坑：回调内同步服务死锁 + 时间戳纪元混用

- **日期**：2026-09-13
- **产品线**：Edge（F50 a3_arm_monitor 看门狗节点，Python 20 Hz 节拍内同步调服务）
- **严重度**：高（第一坑让所有故障处置动作静默超时=看门狗形同虚设；第二坑让 STALE_JS 判据永不触发）

## 现象

F50 看门狗冒烟时两个怪象：

1. **服务全「调用超时」但服务器其实执行了**：monitor 的 stop/reset 调用日志全部 timeout，紧接着 UNEXPECTED_DISABLE 却触发了——证明 reset 服务其实在服务器端执行成功（电机真失能了），只是客户端收不到响应。看门狗的处置动作全部落空。
2. **STALE_JS 永不触发**：SIGSTOP 冻结 sim_motor 后 js 停发几十秒，看门狗毫无反应——js 过期判据像不存在。

## 根因

1. **回调内 `spin_until_future_complete` 死锁**：`rclpy.spin_until_future_complete(future)` 会一直 spin 直到 future 完成。在**同一个单线程 executor 的 timer 回调里**调用它时，服务响应回调永远排不进 executor 队列——服务端早已执行完并把响应发回来，但客户端没人处理，直到 timer 回调返回才轮到。于是每个同步调用都「超时」，而服务端动作照常执行（首轮错判为「服务端没响应」，实际是客户端自己堵死了自己）。
2. **时间纪元混用**：`now - js_msg.header.stamp.sec` 里 `now = time.monotonic()`（开机秒数）而消息 stamp 是墙钟（1970 纪元）——差值恒为巨大负数，`js_age > 1.0` 永远不成立。冻结几十秒被算成「-50 年的过期」，判据静默失效。

## 修复

1. **服务客户端全部挂独立 `ReentrantCallbackGroup` + `MultiThreadedExecutor` + 纯轮询等 future**（复刻 arm_controller 已验证模式）：

```python
self._client_group = ReentrantCallbackGroup()
self._stop_cli = self.create_client(MotorStop, ..., callback_group=self._client_group)

def _call_service(self, cli, req):
    future = cli.call_async(req)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and rclpy.ok():
        if future.done():
            break
        time.sleep(0.005)   # 纯轮询，绝不 spin
```

    ReentrantCallbackGroup 允许响应回调在 executor 的其它线程并发执行，不受 timer 回调阻塞；轮询只 sleep 不 spin，回调队列保持流动。改后 stop/reset 全部返回 `success=True`。

2. **过期判据用接收时刻的 monotonic 时间戳**：`_on_js` 里记 `self._js_last_mono = time.monotonic()`（收到即记，与消息 stamp 解耦），判据改 `js_age = now - self._js_last_mono`。改后 SIGSTOP 冻结 1 s 后 STALE_JS 准点触发。

## 验证与判据教训

- 判据教训 1：**看门狗节点冒烟必须验证「处置动作真正送达」**——不能只看触发日志，要看服务调用的返回码和服务的实际效果（电机 mode_status 变化）。触发日志对而动作全超时的看门狗等于没有。
- 判据教训 2：时间判据类代码 review 时**对任何时间差运算问一句「两个时间戳各自什么纪元」**——monotonic 与墙钟混减不出错也不报错，只是静默失效，冒烟测不出来（除非专门测冻结场景）。

## 关联

- arm_controller.py 87-90/456-464 行的既有注释就是坑 1 的文档化修复——新节点写同步服务调用先抄这个模式。
- [[LL-020]] js 冻结判据（STALE_JS 阈值设计依据）；[[LL-030]] 双 QoS 订阅。
- F50（a3_arm_monitor，本条目两坑的出处）。
