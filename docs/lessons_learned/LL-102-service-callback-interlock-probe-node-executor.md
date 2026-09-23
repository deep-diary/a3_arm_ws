# LL-102 — 服务回调里查其他服务：用独立探针节点 + 专用 executor；不要在自己的 executor 上递归 spin

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble（rclcpp）

## 现象

F91 独立维护节点要在执行 SetZero/SaveParam 前查 `/controller_manager/list_controllers` 做联锁。初版把 client 建在本节点上，在服务回调里调 `rclcpp::spin_until_future_complete(shared_from_this(), future)`，结果维护服务本身的请求永远等不到响应（回调整体卡死，客户端超时）。

改成 MultiThreadedExecutor + ReentrantCallbackGroup 后依然不派发回调（节点在线、参数服务正常，自定义服务的回调入口日志一条没有）；最终把主执行器换成 **SingleThreadedExecutor + 默认回调组**、联锁 client 放在**独立探针节点**上才正常。

## 根因与正确做法

1. **一个节点不能在自己被 spin 的 executor 上递归 spin。** Humble 的 `spin_until_future_complete(node, ...)` 内部新建临时 executor 并 `add_node`，而该节点已经挂在外层 executor 上，等待条件永远不满足/不被处理，回调死等。

2. **标准模式：探针独立成节点，自带 SingleThreadedExecutor。** 回调里在另一个 executor 上 spin 另一个节点，与外层 executor 完全无交集：
   ```cpp
   class MyNode : public rclcpp::Node {
     rclcpp::Node::SharedPtr probe_node_ = std::make_shared<rclcpp::Node>("probe");
     rclcpp::executors::SingleThreadedExecutor probe_exec_;
     rclcpp::Client<OtherSrv>::SharedPtr cli_;
     // ctor:
     //   cli_ = probe_node_->create_client<OtherSrv>("/other/service");
     //   probe_exec_.add_node(probe_node_);
     // callback:
     //   auto fut = cli_->async_send_request(std::make_shared<OtherSrv::Request>());
     //   probe_exec_.spin_until_future_complete(fut, 2s);
   };
   ```
   主节点用 SingleThreadedExecutor 即可：维护类回调短、彼此互斥不是缺陷；只有真正需要并发回调时才上 MultiThreadedExecutor + Reentrant 组。

3. **排障顺序记住：** 参数服务（`ros2 param get /node name`）能响应而自定义服务不派发 → 问题在回调组/executor 组合，不在 DDS 发现。先在回调入口打日志确认「有没有进来」，再怀疑下游阻塞。

4. **附带坑：** `pkill -f` 自匹配包装 shell 导致复合命令 144 退出——详见 LL-101；本任务再次踩到，杀节点优先 `pgrep -f` 后排除 `$$/$PPID` 再 kill。
