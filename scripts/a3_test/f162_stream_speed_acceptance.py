#!/usr/bin/env python3
"""F162 轨迹流速度缩放验收：ptp/lin × speed(0.1~1.0) 运行时长扫描。

前置：a3_bringup.launch.py hardware:=mock 已起，且已 init（READY）。
用法：python3 f162_stream_speed_acceptance.py
退出码 0 = 速度缩放生效（时长随 speed 单调递减）且全部组合规划成功。

验收点：
  1. 规划正常：ptp/lin × speed 全组合 playback success=True；
  2. 反比关系：时长随 speed 增大单调递减，且 0.1/1.0 时长比 ≥ 1.5（速度缩放生效）。
注：比值不会到理论 10x——加减速段与到位容差是固定时间开销，仅匀速段严格反比。

实现说明：用 subprocess 调 ros2 service（每次独立进程，天然带进程启动/DDS 发现
缓冲，规避 move_group current-state 滞后导致的相邻段规划失真，见 LL-103）。
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import time

import yaml

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
# A→B：ptp 用 zero→ready（关节，长距离，速度/加速度主导）；lin 用 home→ready（笛卡尔）。
READY_Q = [0.0, 1.05, -1.575, 0.0, 0.0, 0.0, 0.0]
READY_POSE = {
    "position": {"x": -0.144, "y": 0.0, "z": 0.499},
    "orientation": {"x": 0.613, "y": 0.613, "z": 0.353, "w": 0.353},
}
SPEEDS = [0.1, 0.2, 0.3, 0.5, 0.7, 1.0]


def _streams_dir() -> str:
    p = subprocess.run(["ros2", "pkg", "prefix", "app_melodion"],
                       capture_output=True, text=True)
    prefix = (p.stdout or "").strip()
    if prefix:
        return os.path.join(prefix, "share", "app_melodion", "config", "streams")
    return os.path.expanduser("~/.a3/trajectories/streams")


STREAMS_DIR = _streams_dir()
os.makedirs(STREAMS_DIR, exist_ok=True)


def ros(args):
    return subprocess.run(["ros2"] + args, capture_output=True, text=True)


def goto(name: str) -> None:
    ros(["service", "call", "/a3/arm/goto_named_pose",
         "a3_msgs/srv/GotoNamedPose", f"{{pose_name: {name}}}"])
    time.sleep(2.0)


def run_seg(mode, speed, target):
    doc = {
        "kind": "stream",
        "joint_names": JOINTS,
        "segments": [{"target": target, "mode": mode, "speed": speed, "sleep": 0.0}],
    }
    path = os.path.join(STREAMS_DIR, "speedtest.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)

    r = ros(["service", "call", "/a3/arm/playback", "a3_msgs/srv/PlaybackTrajectory",
             "{name: speedtest, type: stream}"])
    out = (r.stdout + r.stderr).strip()
    success = "success=True" in out
    dur = None
    m = re.search(r"(\d+\.\d+)s", out)
    if m:
        dur = float(m.group(1))
    return success, dur


def main():
    rows = []
    for mode, a_name, target in [
        ("ptp", "zero", {"positions": READY_Q}),
        ("lin", "home", {"pose": READY_POSE}),
    ]:
        for spd in SPEEDS:
            goto(a_name)
            ok, dur = run_seg(mode, spd, target)
            rows.append((mode, spd, ok, dur))
            print(f"[{mode}] speed={spd:.1f} -> ok={ok} dur={dur}")

    print("\n===== F162 轨迹流速度缩放验收 =====")
    print("dur = MoveIt 轨迹运行时长(秒)；ptp=zero→ready(长距离)，lin=home→ready(短距离)")
    print(f"{'mode':5s} | " + " | ".join(f"{s:.1f}" for s in SPEEDS))
    print("-" * 60)
    by_mode = {}
    for mode in ("ptp", "lin"):
        durs = [r[3] for r in rows if r[0] == mode]
        by_mode[mode] = durs
        print(f"{mode:5s} | " + " | ".join(
            f"{d:.2f}" if d is not None else " FAIL" for d in durs))

    all_ok = all(r[2] for r in rows)
    print("\n----- 判定 -----")
    print(f"全部组合规划成功 = {all_ok}")
    passed = all_ok
    for mode in ("ptp", "lin"):
        durs = by_mode[mode]
        ok_durs = [d for d in durs if d is not None]
        if len(ok_durs) >= 2:
            mono = all(ok_durs[i] >= ok_durs[i + 1] - 0.05 for i in range(len(ok_durs) - 1))
            ratio = ok_durs[0] / ok_durs[-1] if ok_durs[-1] > 0 else 0.0
            print(f"{mode}: 时长单调递减={mono}, 0.1/1.0 时长比={ratio:.1f}x")
            passed = passed and mono and ratio >= 1.5
        else:
            print(f"{mode}: 有效样本不足")
            passed = False

    print(f"\n结果: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
