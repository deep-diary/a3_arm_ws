#!/usr/bin/env python3
"""F193 DEMO 巡游验收：四种模式的几何特征。

推荐在 **mock 硬件 + 生产 servo 链（a3_servo_target）** 上跑（domain 77）：

    ROS_DOMAIN_ID=77 ros2 launch a3_bringup a3_bringup.launch.py \
        hardware:=mock use_servo_target:=true use_teleop:=false use_camera:=false \
        use_rviz:=false use_mqtt:=false use_diagnostics:=false
    ROS_DOMAIN_ID=77 ros2 service call /a3/arm/enable std_srvs/srv/Trigger
    ROS_DOMAIN_ID=77 ros2 run a3_bringup angular_cruise        # 另一个终端
    ROS_DOMAIN_ID=77 python3 scripts/a3_test/f193_demo_tour_accept.py

注意：纯仿真栈（edge_teleop_full_sim）用的是 moveit_servo，在当前姿态大量报
`Could not find IK solution (error -31)`，无法用它验收末端几何；这里用 mock 硬件
+ a3_servo_target（与真机同一条链）。

判定：
    translate_xyz : X→Y→Z 依次平移（各轴 ±6cm / z ±4.5cm），姿态变化 < 3°
    translate     : 连续随机平移，留在半径内，姿态 < 16°
    xyz           : 只旋转，末端位移 < 10 mm，姿态最大变化 > 40°
    random        : 只旋转，末端位移 < 10 mm，姿态有变化
"""

import math
import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "99")

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from a3_msgs.msg import ArmStatus  # noqa: E402
from a3_msgs.srv import AngularCruise, GotoNamedPose  # noqa: E402

AMP = 0.06


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("f193_demo_tour_accept")
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.samples = []
        self.state = ""
        self.create_subscription(PoseStamped, "/a3/ee_pose", self._on_ee, rel)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._on_st, rel)
        self.cli = self.create_client(AngularCruise, "/a3/demo/angular_cruise")
        self.goto_cli = self.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")

    def goto_home(self) -> None:
        if not self.goto_cli.wait_for_service(timeout_sec=5.0):
            print("[!] goto_named_pose 不可用，跳过回 home")
            return
        req = GotoNamedPose.Request()
        req.pose_name = "home"
        fut = self.goto_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        try:
            resp = fut.result()
            print("goto home:", resp.success, resp.message)
        except Exception as exc:  # noqa: BLE001
            print("goto home 失败:", exc)
        self.spin_for(8.0)

    def _on_ee(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.samples.append((time.monotonic(),
                             (p.x, p.y, p.z),
                             (q.x, q.y, q.z, q.w)))

    def _on_st(self, msg: ArmStatus) -> None:
        self.state = msg.state

    def spin_for(self, sec: float) -> None:
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, action: int, mode: str = "", duration: float = 0.0,
             vmax: float = 0.0, bound: float = 0.0, rot: float = 0.0,
             peak: float = 0.0):
        if not self.cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("angular_cruise 服务不可用")
        req = AngularCruise.Request()
        req.action = int(action)
        req.mode = str(mode)
        req.duration_s = float(duration)
        req.vmax_mps = float(vmax)
        req.bounds_m = float(bound)
        req.rot_bound_deg = float(rot)
        req.peak_dps = float(peak)
        fut = self.cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result()

    def stats(self):
        if not self.samples:
            return None
        p0 = self.samples[0][1]
        q0 = self.samples[0][2]
        max_dp = 0.0
        ext = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        max_ang = 0.0
        for _, p, q in self.samples:
            dp = [p[i] - p0[i] for i in range(3)]
            max_dp = max(max_dp, math.sqrt(sum(v * v for v in dp)))
            for i in range(3):
                ext[i][0] = min(ext[i][0], dp[i])
                ext[i][1] = max(ext[i][1], dp[i])
            dot = abs(sum(a * b for a, b in zip(q0, q)))
            max_ang = max(max_ang, 2.0 * math.acos(min(1.0, dot)))
        return dict(n=len(self.samples), max_dp=max_dp,
                    ext=ext, max_ang=math.degrees(max_ang))


def fmt(s) -> str:
    if not s:
        return "no samples"
    ex = ", ".join(f"{v[0] * 1000:+.0f}..{v[1] * 1000:+.0f}" for v in s["ext"])
    return (f"n={s['n']} max|dp|={s['max_dp'] * 1000:.1f}mm "
            f"ext=[{ex}]mm max_rot={s['max_ang']:.1f}°")


def main() -> int:
    rclpy.init()
    n = Probe()
    print("[*] 等 READY …")
    t0 = time.monotonic()
    while n.state != "READY" and time.monotonic() - t0 < 20:
        n.spin_for(0.2)
    print("state:", n.state)
    if n.state != "READY":
        print("[x] 仿真臂不是 READY（先 init/enable）")
        return 2
    # DEMO 进入时会先回 home；验收同样先到 home（零位附近是 IK 奇异）
    n.goto_home()
    print("home ee:", tuple(round(v, 3) for v in n.samples[-1][1])
          if n.samples else "n/a")

    results = {}

    r = n.call(1, "translate_xyz", vmax=0.04, bound=AMP, rot=15.0)
    print("start translate_xyz:", r.success, r.message)
    n.samples.clear()
    n.spin_for(30.0)
    results["translate_xyz"] = n.stats()

    r = n.call(1, "translate", duration=12.0, vmax=0.05, bound=AMP, rot=15.0)
    print("start translate:", r.success, r.message)
    n.samples.clear()
    n.spin_for(14.0)
    results["translate"] = n.stats()

    r = n.call(1, "xyz", peak=30.0)
    print("start xyz:", r.success, r.message)
    n.samples.clear()
    n.spin_for(40.0)
    results["xyz"] = n.stats()

    r = n.call(1, "random", peak=45.0)
    print("start random:", r.success, r.message)
    n.samples.clear()
    n.spin_for(12.0)
    n.call(0)
    results["random"] = n.stats()

    print("\n=== 结果 ===")
    for k, s in results.items():
        print(f"{k:14s} {fmt(s)}")

    s = results["translate_xyz"]
    c1 = bool(s) and s["max_ang"] < 3.0 and abs(s["ext"][0][0]) > 0.04 \
        and abs(s["ext"][1][1]) > 0.04 and abs(s["ext"][2][1]) > 0.03
    s = results["translate"]
    c2 = bool(s) and s["max_ang"] < 16.0 and s["max_dp"] < AMP * 1.3 + 0.01
    s = results["xyz"]
    c3 = bool(s) and s["max_dp"] < 0.010 and s["max_ang"] > 40.0
    s = results["random"]
    c4 = bool(s) and s["max_dp"] < 0.010 and s["max_ang"] > 5.0

    ok = True
    for name, cond in (("translate_xyz", c1), ("translate", c2),
                       ("xyz", c3), ("random", c4)):
        print(f"  {name:14s} {'PASS' if cond else 'FAIL'}")
        ok = ok and bool(cond)
    print("=== 总判定:", "PASS" if ok else "FAIL", "===")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
