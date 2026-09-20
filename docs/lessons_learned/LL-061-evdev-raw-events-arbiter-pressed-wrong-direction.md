# LL-061 极性争议的最终裁判是 evdev 原始事件；「按键不响应」先排除按错方向/窗口错位

**日期：** 2026-09-20
**产品线：** Edge
**关联：** [[LL-060-pygame-sdl-6axis-vs-joydev-8axis]]、F63

## 现象

joynet Linux 映射改完、61 pytest 全绿后，端到端验收反复「失败」：在 `/joy` 20 s 录制窗口里请用户按十字键上，分析结果 `DPAD_Y min=0 max=+1`，而 `config/ds4_linux.yaml` 契约写的是上 = −1。先后怀疑过多发布者报文污染、桥节点极性转换写反，沿整条链路逐层排查。

## 根因

直接读内核原始事件（Python 打开 `/dev/input/event12`，解码 `struct input_event`，过滤 `EV_ABS`），用户依次按上、下：

```
EV_ABS code=17 value=4294967295   # ABS_HAT0Y，0xFFFFFFFF 即有符号 -1 → 上
EV_ABS code=17 value=0
EV_ABS code=17 value=1            # 下
EV_ABS code=17 value=0
```

内核极性完全标准（上 −1 / 下 +1），joy_node 原样透传。之前窗口里的 +1 是**实际按到了「下」**，不是代码缺陷。排查链最上游的「输入事实」没有第一时间取证，导致在中间层（ROS 话题、发布者列表）上消耗了大量时间。

## 教训 / 规则

1. **极性/键位争议，第一步直接读 evdev 原始事件**（`/dev/input/eventN`，20 行 Python 即可，无需安装 evtest/jstest）：内核输入是整条链的源头，源头对了再往下查，源头错了（驱动/设备差异）就直接改映射假设。
2. 录制窗口式验收必须同时记录「用户实际操作了什么」：文字指令「按上」不等于报文里的就是上，分析结果与预期矛盾时，先在分析脚本里同时打印原始 value 和时间戳，与用户确认，不要直接假设代码错。
3. 同设备多个 `joy_node` 发布者（读同一个 js0）输出一致，不构成污染；异源发布者（如 TCP 桥）混入才会污染——判断发布者要看**数据源**而不是数量。
4. 给用户的快速判据：内核级确认可用一行命令完成——
   ```bash
   timeout 15 python3 -c "
   import struct,time
   f=open('/dev/input/event12','rb'); fmt='llHHI' if struct.calcsize('ll')==16 else 'qHHI'; s=struct.calcsize(fmt)
   end=time.time()+14
   while time.time()<end:
       d=f.read(s)
       if len(d)<s: break
       _,_,t,c,v=struct.unpack(fmt,d)
       if t==3: print(c, v if v < 2**31 else v-2**32)"
   ```
