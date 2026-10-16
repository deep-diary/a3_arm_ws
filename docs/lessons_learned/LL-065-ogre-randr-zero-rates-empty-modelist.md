# LL-065 — X RANDR 报告尺寸但零刷新率 → Ogre 视频模式表为空 → RViz 启动即崩

> **日期：** 2026-09-21
> **产品线：** Edge
> **环境：** RK3588（lubancat）/ ROS 2 Humble / rviz2 + Ogre 1.12.1 / F62 双模型 RViz

## 现象

`rviz2`（legacy GL 与 GL3Plus 皆然）启动瞬间 SIGSEGV，日志在加载 RenderSystem 后即断：

```
Loading library /opt/ros/humble/lib/librviz_rendering.so
[INFO] [rviz]: OpenGl version: <崩溃，打不出来>
Segmentation fault
```

gdb 栈顶在 `std::string::_M_assign` / `std::vector<std::string>::operator[]`，调用路径为
`OgreGLRenderSystemCommon::initConfigOptions()` → 读 RANDR 视频模式列表 `possibleValues[0]`。

## 根因

Ogre 1.12 的 `GLXGLSupport` 用 **X RANDR 扩展枚举显示器视频模式**（刷新率/分辨率组合）填充配置项。本机 X 服务器（rockchip 驱动 + HDMI）的 RANDR 应答里：

- `XRRSizes` **有尺寸**；
- `XRRRates` 对每个尺寸返回 **零条刷新率**。

于是 Ogre 得到的视频模式 vector 是**空表**，而 `initConfigOptions()` 无条件取 `possibleValues[0]` → 对空 `std::string` 解引用 → 段错误。GLX 共享代码在 legacy GL 与 GL3Plus 两条 RenderSystem 路径都会走，换 plugin 无效。

## 正确做法 / 规避

用 LD_PRELOAD 小 shim 拦截 `XQueryExtension`，对 `"RANDR"` 返回 False，逼 Ogre 走「单 DisplayWidth×Height」非 RANDR 回退路径：

```c
#define _GNU_SOURCE
#include <X11/Xlib.h>
#include <dlfcn.h>
#include <string.h>
typedef Bool (*xqe_t)(Display *, const char *, int *, int *, int *);
Bool XQueryExtension(Display *dpy, const char *name, int *mi, int *fe, int *ferr)
{
    static xqe_t real = NULL;
    if (!real) real = (xqe_t)dlsym(RTLD_NEXT, "XQueryExtension"); /* 别直接调自身：无限递归 */
    if (name && strcmp(name, "RANDR") == 0) return False;
    return real(dpy, name, mi, fe, ferr);
}
```

```bash
gcc -shared -fPIC -o ~/.a3/hide_randr/libhide_randr.so hide_randr.c -lX11 -ldl
LD_PRELOAD=$HOME/.a3/hide_randr/libhide_randr.so LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

shim 已接入 `edge_teleop_full_sim.launch.py` 的 RViz prefix（存在才加），修复后 RViz 稳定输出
`OpenGl version: 4.5 (GLSL 4.5)`，双 RobotModel 长期运行不崩。

**2026-10-16 补充（真机入口漏了同一 shim）**：`a3_bringup.launch.py`（真机/仿真统一产品入口）
的 RViz prefix 只加了 `LIBGL_ALWAYS_SOFTWARE=1`，**没有** `LD_PRELOAD=hide_randr` → 真机栈起栈时
`rviz2` 立刻 `process has died [exit code -11]`（同一 RANDR 空模式表崩溃），而仿真栈正常，容易误判成
"真机没显示"。修：该文件的 rviz 节点按 `edge_teleop_full_sim` 同款写法 preload（文件存在才加），
已随栈重启验证 RViz 正常（模型 = 6 轴 + 连杆）。**两处前缀必须同时维护**。

## 排查附注（同场另外两个本机坑）

- **进程枚举盲区**：读 `/proc/<pid>/environ` 对 root 属主进程直接 EACCES，脚本若静默 skip 就会漏判「root 起的 ROS 节点在同域」。查域内节点先用 `ps aux`（root 进程也可见）再按 pid 补 environ。
- **截图黑屏**：DPMS 休眠时 `xset q` 显示 `Monitor is Off`，`import -window root` 只能拿到黑/1-bit 图（rockchip X scanout）。先 `DISPLAY=:0 xset s off; xset s noblank; xset -dpms`，再按窗口 ID 截：`DISPLAY=:0 import -window 0x3c00106 out.png`。
- **跨域误判**：调查「仿真里出现真机报错文案」先确认 CLI 的 `ROS_DOMAIN_ID`——shell 不保留上次 export，domain 0（真机栈）与 domain 45（仿真）只差一个环境变量，读错域会把真机 FAULT 当成仿真故障。

## 相关路径

- `~/.a3/hide_randr/hide_randr.c` / `libhide_randr.so`（shim 源码与产物，不入仓）
- `src/a3_bringup/launch/edge_teleop_full_sim.launch.py`（仿真入口 RViz prefix 自动 LD_PRELOAD）
- `src/a3_bringup/launch/a3_bringup.launch.py`（真机/仿真统一产品入口，同样 preload，2026-10-16 补）
- 关联：[[LL-027-rviz-panfrost-freeze-software-render]]（RViz 在本机的另一前缀；注意该条 2026-10-16 已复测：
  默认改走 panfrost 硬件 GL，`LIBGL_ALWAYS_SOFTWARE=1` 仅作卡死回退；而本条的 `hide_randr` **两种模式都必须加**）
