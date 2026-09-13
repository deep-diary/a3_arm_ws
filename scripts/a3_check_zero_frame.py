#!/usr/bin/env python3
"""A3 开机零位校验（F48）：/joint_states 读数 vs URDF 限位（硬检查）+ 期望位姿（软检查）。

背景（LL-019）：MIT 电机上电用单圈编码器 + 多圈推算恢复绝对角度；zero_sta=1
（-π~π 窗口）下正常断电不丢，但超 ±180° 的断电转动或参数异常仍会 +2π 环绕，
读数远超关节限位。此时使能会 kp×误差瞬间猛拉——所以上电后、使能前跑本脚本。

  硬检查（决定退出码）：7 关节读数全部落在 URDF 限位内且 /joint_states 消息
    新鲜（stamp 距今 ≤ --max-age，默认 1 s）；任一越限、消息陈旧或超时无消息
    → FAIL exit 1。越限 = 疑似环绕，先恢复 URDF 零位再 init；陈旧 = 桥异常
    （refresh 停发，LL-020），旧值校验形同虚设。
  软检查（仅 WARN，默认容差 ±20°）：期望位姿模板——L2/L3/L5/L6/L7 ≈ 0；
    L4 ≈ 0（URDF 零位，泡沫垫水平）或 ≈ 0.34（折叠自然下垂）；L1 自由（水平
    转动 ±178° 由机械限位约束）。模板外说明臂被留在其它位姿，注意 poses/FK
    语义。软检查不阻断——判断是否环绕（危险）的唯一标准是硬检查。

用法（真机：先启动硬件栈，再）：
  source scripts/a3_shell_env.sh
  python3 scripts/a3_check_zero_frame.py                 # 默认 --timeout 3
  python3 scripts/a3_check_zero_frame.py --tol 0.2 --timeout 5

依赖：ROS 2 Humble + a3_description（限位从 install/share/a3_description/urdf/el_a3.urdf 读）。
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState

# 期望位姿模板（软检查）：(关节名, [允许值列表])；L1 自由不检查。
# L4 两个允许值：0 = URDF 零位（泡沫垫水平）；0.34 ≈ 折叠自然下垂（home 实测 0.3408）。
EXPECT_TEMPLATE = {
    "L2_joint": [0.0],
    "L3_joint": [0.0],
    "L4_joint": [0.0, 0.34],
    "L5_joint": [0.0],
    "L6_joint": [0.0],
    "L7_joint": [0.0],
}
FREE_JOINTS = ("L1_joint",)


def load_urdf_limits():
    """从 a3_description 的 el_a3.urdf 读关节限位（与 arm_controller._load_joint_limits 同源）。"""
    share = get_package_share_directory("a3_description")
    path = os.path.join(share, "urdf", "el_a3.urdf")
    limits = {}
    order = []
    for joint in ET.parse(path).getroot().iter("joint"):
        name = joint.get("name", "")
        limit = joint.find("limit")
        if limit is None:
            continue
        try:
            lower = float(limit.get("lower"))
            upper = float(limit.get("upper"))
        except (TypeError, ValueError):
            continue
        limits[name] = (lower, upper)
        order.append(name)
    return limits, order


class CheckNode(Node):
    def __init__(self, topic: str, timeout_s: float):
        super().__init__("a3_check_zero_frame")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, topic, self._on_js, qos)
        self.timeout_s = timeout_s
        self.msg = None

    def _on_js(self, msg: JointState):
        self.msg = msg

    def wait_for_js(self) -> JointState:
        deadline = self.get_clock().now().nanoseconds + int(self.timeout_s * 1e9)
        while rclpy.ok() and self.msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                break
        return self.msg


def main() -> int:
    parser = argparse.ArgumentParser(description="A3 开机零位校验（F48）")
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--timeout", type=float, default=3.0, help="等待 /joint_states 的秒数")
    parser.add_argument(
        "--tol", type=float, default=0.35,
        help="软检查容差 (rad)，默认 0.35 ≈ ±20°——软检查仅提示性，不阻断",)
    parser.add_argument(
        "--margin", type=float, default=0.001,
        help="硬检查限位裕量 (rad)——编码器量化噪声 ±0.0002（L2/L3/L7 下界为 0）",)
    parser.add_argument(
        "--max-age", type=float, default=1.0,
        help="/joint_states 最大陈旧时长 (s)——桥异常时 js 冻结旧值（LL-020），旧值校验形同虚设",)
    args = parser.parse_args()

    rclpy.init()
    node = CheckNode(args.topic, args.timeout)
    try:
        limits, urdf_order = load_urdf_limits()
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL: cannot load URDF limits: {exc}")
        return 1
    joint_names = [n for n in urdf_order if n in limits]

    msg = node.wait_for_js()
    if msg is None:
        print(f"FAIL: no {args.topic} within {args.timeout}s — is a3_can_bridge running?")
        print("DO NOT ENABLE the arm.")
        return 1

    # 新鲜度检查（LL-020）：桥 refresh 停发时 js 冻结在旧值，旧值校验形同虚设
    stamp = Time.from_msg(msg.header.stamp)
    age = (node.get_clock().now() - stamp).nanoseconds * 1e-9
    if age > args.max_age:
        print(f"FAIL: stale {args.topic}: stamp {age:.1f}s old (max {args.max_age}s) — "
              "CAN feedback not flowing, check a3_can_bridge")
        print("DO NOT ENABLE the arm.")
        return 1

    name_to_idx = {n: i for i, n in enumerate(msg.name)}
    positions = {n: float(msg.position[name_to_idx[n]]) for n in joint_names if n in name_to_idx}
    missing = [n for n in joint_names if n not in name_to_idx]

    # ---- 硬检查：URDF 限位 ----
    hard_fail = []
    for n in joint_names:
        if n in missing:
            hard_fail.append(f"{n}: not in {args.topic} message")
            continue
        lo, hi = limits[n]
        p = positions[n]
        if p < lo - args.margin or p > hi + args.margin:
            hard_fail.append(f"{n}={p:+.4f} limit={lo:.4f}..{hi:.4f}")

    # ---- 软检查：期望位姿模板 ----
    soft_warn = []
    for n, allowed in EXPECT_TEMPLATE.items():
        if n in positions and all(abs(positions[n] - a) > args.tol for a in allowed):
            soft_warn.append(
                f"{n}={positions[n]:+.4f} (expected " + " or ".join(f"{a:+.2f}" for a in allowed) + ")"
            )

    # ---- 输出 ----
    print(f"=== A3 开机零位校验 (F48), tol={args.tol} rad ===")
    for n in joint_names:
        tag = "FREE" if n in FREE_JOINTS else "  "
        print(f"  {n:<10} {positions.get(n, float('nan')):+9.4f}   limit "
              f"{limits[n][0]:+.4f}..{limits[n][1]:+.4f}  {tag}")
    if hard_fail:
        print("\nHARD FAIL（严禁使能）: " + "; ".join(hard_fail))
        print("疑似断电多圈环绕（LL-019）：把臂摆回 URDF 零位（工装/泡沫垫）后 /a3/arm/init 恢复零位，")
        print("或检查 zero_sta 参数与电源回路。")
        return 1
    if soft_warn:
        print("\n软检查 WARN（硬检查通过，可安全使能，但注意位姿语义）: " + "; ".join(soft_warn))
        print("臂被留在模板外位姿——poses.yaml / FK 按 URDF 零位或折叠 home 语义使用前先确认实际位姿。")
    else:
        l4 = positions.get("L4_joint", 0.0)
        pose = "URDF 零位" if abs(l4) <= abs(l4 - 0.34) else "折叠 home（L4 重力下垂）"
        print(f"\n软检查 PASS：位姿匹配「{pose}」")
    print("\nPASS：读数全部在 URDF 限位内，可以 /a3/arm/enable。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
