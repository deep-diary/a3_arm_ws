# LL-119 — CycloneDDS 0.10 套接字缓冲配置在 Internal 下且只有 min

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble（ros-humble-cyclonedds 0.10.5-2jammy）

## 现象

F105 按新版 CycloneDDS 文档把套接字缓冲写在 General 下：

```xml
<General>
  <SocketReceiveBufferSize min="4MiB" max="4MiB"/>
</General>
```

节点启动即致命解析错误：`General: SocketReceiveBufferSize: unknown element`，随后 `rmw_create_node` 失败，所有节点无法创建。

## 根因

CycloneDDS 配置 schema 随版本变化。0.10.x 中套接字缓冲只有一个路径：

- `CycloneDDS/Domain/Internal/SocketReceiveBufferSize[@min]`
- `CycloneDDS/Domain/Internal/SocketSendBufferSize[@min]`

在 `<Internal>` 下，且只有 `min` 一个属性；没有 `max`，放在 `<General>` 下是未知元素（fatal）。新版本的 `General/SocketReceiveBufferSize[min,max]` 形式在 0.10 不存在。

可用 `strings /opt/ros/humble/lib/aarch64-linux-gnu/libddsc.so.0 | grep Socket` 核对本机实际支持的路径。

## 正确做法 / 规避

```xml
<Internal>
  <SocketReceiveBufferSize min="4MiB"/>
  <SocketSendBufferSize min="4MiB"/>
</Internal>
```

`min` 是对内核的申请值（实际 SO_RCVBUF/SO_SNDBUF），需配合 `net.core.rmem_max/wmem_max` 上调，否则申请被静默截断。尺寸单位 `4MiB`/`65500B` 均接受。改 XML 后先手工起一个节点验证解析，再铺开整栈。

## 相关路径

- `src/a3_bringup/config/cyclonedds.xml`
- `systemd/60-a3-dds.conf`
- `scripts/setup/setup_dds_network.sh`
