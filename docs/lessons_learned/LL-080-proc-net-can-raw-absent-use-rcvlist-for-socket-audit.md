# LL-080 — F78：审计「栈是否打开了 CAN socket」时 `/proc/net/can/raw` 在本机不存在，改用 `/proc/net/can/rcvlist_all` / `rcvlist_fil`

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Linux 6.1.84，SocketCAN（can0/can1/vcan0 CAN FD）
> **关联：** F78、F72

## 现象

F78 要证明 mock 栈（`hardware:=mock`）完全不碰 CAN、can 栈只打开指定接口。直觉去看经典文档里的 `/proc/net/can/raw`：

```bash
$ cat /proc/net/can/raw
（空——文件存在但无任何条目，且无法区分接口）
```

拿不到每个接口的 raw socket 清单，没法证明「mock 栈在 can0/can1/vcan0 零 socket」。

## 根因

该内核（6.1.84）的 CAN procfs 未提供有用的 `raw` 条目；AF_CAN 的接收钩子清单在：

- `/proc/net/can/rcvlist_all` — 混杂（promiscuous）raw socket，每接口一行，带 `matches` 累计匹配计数；
- `/proc/net/can/rcvlist_fil` — 带 CAN_RAW_FILTER 的 EFF/SFF raw 条目，同样按接口列出 can_id/mask 与计数；
- 另有 `rcvlist_sff`、`rcvlist_eff`、`rcvlist_err`、`rcvlist_fil` 等同族文件。

模拟器持有的「收全部帧」rx_all socket 出现在 `rcvlist_all`；硬件插件带过滤器的 socket 出现在 `rcvlist_fil`。

## 正确做法 / 规避

- **审计某接口有哪些 CAN 接收者**：
  ```bash
  grep -w can1 /proc/net/can/rcvlist_all /proc/net/can/rcvlist_fil
  ```
  无输出 = 该接口没有任何 raw 接收 socket。
- **证明某接口零流量**：记录 `matches` 计数，隔一段时间再读；计数冻结 = 期间无帧送达（F78 用此法证明 mock 栈运行期间模拟器计数不增长）。
- **核对硬件插件过滤器**：`rcvlist_fil` 行的 `can_id`/`mask` 即 `CAN_RAW_FILTER` 内容，可验证只收 type-2 反馈帧（F78 实测插件条目：can_id `0x98000000`、mask `0x9f000000`，EFF）。
- 排障顺序：先 `ip -details link show <iface>` 确认 UP / mtu（CAN FD=72），再看 rcvlist 确认接收者与计数，最后才上 candump。

## 相关路径

- `scripts/a3_test/vcan_motor_sim.py`（rx_all 混杂接收，体现在 rcvlist_all）
- `src/a3_hardware_interface/`（A3MITHardwareInterface 的过滤 raw socket，体现在 rcvlist_fil）
