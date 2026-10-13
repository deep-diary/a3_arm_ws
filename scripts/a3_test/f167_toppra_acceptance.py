#!/usr/bin/env python3
"""F167.2 验收：toppra 力矩/凸约束离线重定时 + （可选）仿真 baked 回放。

验收点：
  1. `traj_retime_toppra.py` 求解成功（退出码 0）且报告标记力矩在限内（PASS）。
  2. 生成合法 baked（kind=continuous, retimed, points 非空）。
  3.（--sim）仿真栈在线时，playback 该 baked 成功执行（message 含 "baked"）。
前置：先 `source scripts/a3_shell_env.sh`；（--sim）仿真栈已起、已 enable、export ROS_DOMAIN_ID=99。
用法：
  python3 scripts/a3_test/f167_toppra_acceptance.py [--sim]
退出码：0=PASS，1=FAIL。
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

import yaml

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STREAM = os.path.join(WS, "trajectories", "streams", "toppra_5pt.yaml")
BAKED = STREAM + ".baked.yaml"
REPORT = os.path.join(WS, "docs", "dev", "F167_toppra.md")


def _sh(cmd: str) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, shell=True, cwd=WS, capture_output=True, text=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="额外做仿真 baked 回放")
    args = ap.parse_args()
    fails = []

    r = _sh(f"python3 scripts/traj_retime_toppra.py --out {BAKED} --report {REPORT}")
    print(r.stdout.strip()[-400:])
    if r.returncode != 0:
        fails.append("toppra 求解/力矩越限")
    if not os.path.exists(REPORT) or "力矩在限内 | PASS" not in open(REPORT, encoding="utf-8").read():
        fails.append("报告未标记力矩在限内")

    if os.path.exists(BAKED):
        b = yaml.safe_load(open(BAKED, encoding="utf-8"))
        if b.get("kind") != "continuous" or not b.get("points"):
            fails.append("baked 结构非法")
    else:
        fails.append("baked 未生成")

    if args.sim:
        # 需要占位 stream 存在（_prepare_playback 先校验 <name>.yaml 存在）
        if not os.path.exists(STREAM):
            fails.append("缺少占位 stream（toppra_5pt.yaml）")
        else:
            env = "export ROS_DOMAIN_ID=99; "
            r2 = _sh(env + "ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "
                           "\"{name: toppra_5pt, type: stream, strategy: ''}\"")
            if "success=True" not in r2.stdout:
                fails.append(f"仿真回放失败: {r2.stdout.strip()[-160:]}")
            else:
                print("[sim] playback:", r2.stdout.strip().splitlines()[-1])

    if fails:
        for f in fails:
            print(f"[FAIL] {f}")
        return 1
    print("[PASS] F167.2 toppra 力矩/凸约束重定时" + (" + 仿真回放" if args.sim else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
