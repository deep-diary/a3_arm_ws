#!/usr/bin/env python3
"""5 模式 TCP 轨迹对比——在线采集（/joint_states 实测关节 → 供报告 FK 出 TCP）。

对 5 个模式（pilz/blend/l1p5/l2/l3）各跑一遍同一条 5 点路径，采集实测关节序列：
  * 每个模式一个 stream：先 ptp 回起点（保证各模式同一起点），再 <mode> 过 5 点；
  * 逐模式 playback，期间订阅 /joint_states 记录 (t, q[7])；用 /a3/arm_status 判 TRAJ 起止。
pilz/blend 用 pose（笛卡尔）；l1p5/l2/l3 用 positions（关节）。输出 JSON 给报告脚本。

用法（仿真栈已起、已 enable；先 source a3_shell_env.sh）：
  export ROS_DOMAIN_ID=99
  python3 scripts/a3_test/traj_modes_tcp_capture.py \
      --waypoints trajectories/waypoints/latest.yaml --speed 0.4 \
      --out /tmp/traj_modes_tcp.json
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from typing import Dict, List, Optional

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import PlaybackTrajectory

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(WS, "scripts"))
JOINTS = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint", "L7_joint"]
# F166：9 模式（规范名）+ 复用 gen_stream_modes 的段构造（含 ptp 逐腿展开 / circ 3 点弧）
from gen_stream_modes import MODES, CARTESIAN_MODES, build_mode_segments  # noqa: E402
STREAM_DIR = os.path.join(WS, "trajectories", "streams")


def build_streams(wp_path: str, speed: float) -> List[str]:
    """生成 9 个 stream（每模式一条，末尾一段 ptp 回起点），返回 (stream 名, 段总数) 列表。"""
    with open(wp_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    wps = data["waypoints"]
    jnames = data.get("joint_names") or JOINTS
    names: List[str] = []
    os.makedirs(STREAM_DIR, exist_ok=True)
    for mode in MODES:
        if mode in CARTESIAN_MODES and any(not w.get("pose") for w in wps):
            print(f"[warn] {mode}: 有路点缺 pose", file=sys.stderr)
        segs = build_mode_segments(mode, wps, speed)
        segs.append({"mode": "ptp", "speed": 0.5,
                     "target": {"positions": list(wps[0]["positions"])}, "sleep": 0.5})
        doc = {"kind": "stream", "joint_names": list(jnames), "segments": segs}
        name = f"tcp_cmp_{mode}"
        with open(os.path.join(STREAM_DIR, name + ".yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
        names.append((name, len(segs)))
    return names


class Capture(Node):
    def __init__(self):
        super().__init__("traj_modes_tcp_capture")
        self._state = ""
        self._mode = ""
        self._seg = 0  # 当前 stream 段号（从 ArmStatus.message 的 "N/2" 解析）
        self.capture = False
        self.buf: List[List[float]] = []  # [t, q0..q6, seg]
        self._idx = {jn: i for i, jn in enumerate(JOINTS)}
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_status, 10)
        self.playback_cli = self.create_client(PlaybackTrajectory, "/a3/arm/playback")

    def _on_status(self, msg: ArmStatus):
        self._state = msg.state
        self._mode = msg.mode
        # message 形如 "stream st_tcp_cmp_pilz 2/2 via" → 取段号分子
        mm = re.search(r"\b(\d+)/(\d+)\b", msg.message or "")
        if mm:
            self._seg = int(mm.group(1))

    def _on_js(self, msg: JointState):
        if not self.capture:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        q = [0.0] * 7
        for jn, i in self._idx.items():
            if jn in msg.name:
                j = msg.name.index(jn)
                if j < len(msg.position):
                    q[i] = float(msg.position[j])
        self.buf.append([t] + q + [float(self._seg)])


    def call_playback(self, name: str) -> tuple:
        if not self.playback_cli.wait_for_service(timeout_sec=5):
            return False, "playback svc unavailable"
        req = PlaybackTrajectory.Request()
        req.name = name
        req.type = "stream"
        req.strategy = ""
        fut = self.playback_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=200)
        r = fut.result()
        return (bool(r.success), r.message) if r else (False, "timeout")

    def wait_ready(self, timeout: float = 8.0) -> bool:
        """等臂回到 READY（非 TRAJ_RUNNING），避免上一段收尾期被 busy 拒绝。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._state == "READY" and self._mode != "TRAJ_RUNNING":
                return True
            if self._state == "FAULT":
                return False
        return self._state == "READY"

    def run_mode(self, name: str, n_total: int) -> Dict:
        for attempt in range(2):
            self.wait_ready()
            self.buf = []
            self._seg = 0  # 重置段号（在 wait_ready 之后，避免上一条尾消息残留污染切分）
            self.capture = True
            t0 = time.monotonic()
            ok, msg = self.call_playback(name)
            # call_playback 同步阻塞到整条 stream 跑完（各段内部到位同步）；再等 0.2s 收尾采样
            end = time.monotonic() + 0.2
            while time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
            self.capture = False
            if ok and len(self.buf) > 20:
                break
            self.get_logger().warn(f"{name} attempt {attempt+1} 失败({msg})，重试")
        return {"ok": ok, "msg": msg, "n": len(self.buf), "wall": time.monotonic() - t0,
                "n_total": n_total, "samples": self.buf}


def main() -> None:
    ap = argparse.ArgumentParser(description="5 模式 TCP 轨迹在线采集")
    ap.add_argument("--waypoints", default=os.path.join(WS, "trajectories", "waypoints", "latest.yaml"))
    ap.add_argument("--speed", type=float, default=0.4)
    ap.add_argument("--out", default="/tmp/traj_modes_tcp.json")
    args = ap.parse_args()

    names = build_streams(args.waypoints, args.speed)
    print(f"生成 {len(names)} 个 stream: {[n for n, _ in names]}")

    rclpy.init()
    node = Capture()
    rclpy.spin_once(node, timeout_sec=0.5)

    out: Dict = {"speed": args.speed, "modes": {}}
    for mode, (name, n_total) in zip(MODES, names):
        r = node.run_mode(name, n_total)
        out["modes"][mode] = r
        print(f"[{mode}] ok={r['ok']} n={r['n']} segs={n_total} wall={r['wall']:.1f}s  {r['msg']}")

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f)
    print(f"采集 -> {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
