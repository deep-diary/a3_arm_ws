# LL-012 — rclcpp Humble 延迟服务响应：三参数 (header,req,resp) 回调返回即自动回包

> **日期：** 2026-09-06
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ ROS 2 Humble rclcpp

## 现象

`/a3/motor/scan_and_collect`（F32 扫描聚合）需要在发探针后等 1.5 s 收集窗口结束再回包。
按直觉把回调改成「三参数 header 版 + 显式 `send_response`」后，客户端**立即**（0.001 s）
收到 `success=false, message=''` 的默认响应；而节点日志显示 1.5 s 后收集正确完成、也调用了
`send_response`（第二次发送被 rmw 丢弃，客户端无感）。

## 根因

rclcpp Humble 的 `AnyServiceCallback::dispatch` 按回调签名分派，`Service::handle_request`
在 dispatch 返回非空 response 时**自动发送**：

- `(req, resp)` → `SharedPtrCallback`：返回 response → 回调返回即自动回包（同步服务，正确姿势）。
- `(header, req, resp)` → `SharedPtrWithRequestHeaderCallback`：**同样返回 response、同样自动回包**。
  所谓「header 版延迟响应」是误读——header 参数只是给了请求 id，并不延迟发送。
- 真正的延迟响应是**两参数 `(header, req)`** → `SharedPtrDeferResponseCallback`：dispatch 返回
  nullptr，不自动发送；由回调方自行持有 `Service` 指针与 header，稍后显式
  `srv->send_response(*header, *resp)`（Response 对象也需自己 `make_shared` 创建）。

## 正确做法 / 规避

延迟响应服务的注册与实现：

```cpp
// 注册：两参数 (header, req) = DeferResponse 形式
scan_collect_srv_ = this->create_service<MotorScanCollect>(
  "/a3/motor/scan_and_collect",
  [this](const std::shared_ptr<rmw_request_id_t> header,
         const std::shared_ptr<MotorScanCollect::Request> req) {
    HandleScanCollectService(header, req);
  });

// 处理：自行创建 Response、立即拒绝路径显式 send、异步路径存入状态待定时器回包
void HandleScanCollectService(
  const std::shared_ptr<rmw_request_id_t> & header,
  const std::shared_ptr<MotorScanCollect::Request> & req)
{
  auto resp = std::make_shared<MotorScanCollect::Response>();
  if (scan_collect_.active) {           // 立即拒绝
    resp->success = false; resp->message = "scan busy";
    scan_collect_srv_->send_response(*header, *resp);
    return;
  }
  scan_collect_.header = header;        // 挂起，稍后 FinishScanCollect 里
  scan_collect_.pending = resp;         // scan_collect_srv_->send_response(*header, *resp);
  // ...
}
```

判别依据：`/opt/ros/humble/include/rclcpp/rclcpp/any_service_callback.hpp` 的 `set()` 里
`same_arguments` if-constexpr 链（SharedPtrCallback → WithRequestHeader → DeferResponse →
DeferResponseWithServiceHandle），及 `service.hpp` 的 `handle_request`：
`if (response) send_response(...)`——dispatch 返回非空即自动回包。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`（`HandleScanCollectService` / `FinishScanCollect` / 服务注册）
- `/opt/ros/humble/include/rclcpp/rclcpp/any_service_callback.hpp`、`service.hpp`
