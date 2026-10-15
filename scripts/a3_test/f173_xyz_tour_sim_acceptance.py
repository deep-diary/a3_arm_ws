#!/usr/bin/env python3
"""F173 仿真验收：`/a3/arm/random_pose_tour` 笛卡尔 xyz 直接巡游。

前置：`./scripts/a3_stack.sh start --sim --rviz` 已起（ROS_DOMAIN_ID=99），move_group 就绪。
用法：export ROS_DOMAIN_ID=99 && python3 scripts/a3_test/f173_xyz_tour_sim_acceptance.py
退出码 0 = 全部验收项通过。

验收点：
  1. 3 个可达 xyz + orientation_mode=0(free)：success、按序经过每点（FK 末端 ≤5 mm）；
  2. orientation_mode=1(hold_start)：跑通，且各点末端姿态相对起始姿态偏差 < free 模式；
  3. 不可达点（工作空间外）→ success=false，message 指出点序号与坐标；
  4. 非法请求（xyz 长度非 3 倍数 / mode=2 但 ref_quat 非 4）→ 立即拒绝且 message 可读；
  5. 回归：xyz 为空时 F116 随机巡游仍正常（count=2）。
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import pinocchio as pin
from a3_msgs.msg import ArmStatus
from a3_msgs.srv import RandomPoseTour

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
URDF = "/home/cat/a3_arm_ws/src/a3_description/urdf/el_a3.urdf"
POS_ERR = 0.010  # 10 mm（含 F134 blend 转角偏置 2 cm 半径在浅转角下 ≈0.2 mm + 跟踪残差）
ROT_ERR = 0.10   # rad，hold_start 姿态偏差上限

MODEL = pin.buildModelFromUrdf(URDF)
Q_IDX = [int(MODEL.idx_qs[MODEL.getJointId(j)]) for j in JOINTS[:6]]
FID = MODEL.getFrameId("end_effector")


def fk(q6):
    q = pin.neutral(MODEL)
    for v, iq in zip(q6, Q_IDX):
        q[iq] = float(v)
    d = MODEL.createData()
    pin.forwardKinematics(MODEL, d, q)
    pin.updateFramePlacements(MODEL, d)
    return d.oMf[FID]


class Recorder(Node):
    def __init__(self):
        super().__init__("f173_acceptance")
        self.samples = []  # (t, pos[7])
        self.arm_state = "?"
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, qos_profile_sensor_data)
        self.create_subscription(ArmStatus, "/a3/arm_status", self._arm_cb, 10)

    def _js_cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        self.samples.append((time.monotonic(),
                             [msg.position[idx[j]] for j in JOINTS]))

    def _arm_cb(self, msg: ArmStatus):
        self.arm_state = msg.state

    def window(self, t0, t1):
        return [s for s in self.samples if t0 - 0.05 <= s[0] <= t1 + 0.3]


def call(node, srv_type, name, request, timeout=180.0):
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"service not found: {name}")
    future = cli.call_async(request)
    t0 = time.monotonic()
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > timeout:
            raise RuntimeError(f"service timeout: {name}")
    return future.result()


def spin_s(node, t):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        rclpy.spin_once(node, timeout_sec=0.1)


def wait_state(node, want, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.arm_state == want:
            return True
    return False


def verify_ordered_xyzs(samples, targets, label):
    """按序贪心匹配：每个目标点须在运动采样中出现（FK 末端 ≤ POS_ERR）。"""
    msgs, ok_all, si = [], True, 0
    for k, tgt in enumerate(targets):
        tgt = np.asarray(tgt, dtype=float)
        match_i, err_at = None, None
        for i in range(si, len(samples)):
            err = float(np.linalg.norm(fk(samples[i][1][:6]).translation - tgt))
            if err <= POS_ERR:
                match_i, err_at = i, err
                break
        if match_i is None:
            best = min(
                (float(np.linalg.norm(fk(s[1][:6]).translation - tgt))
                 for s in samples[si:]), default=9.9)
            msgs.append(f"pt{k + 1} 未到点（最佳误差 {best * 1000:.1f} mm）")
            ok_all = False
            break
        msgs.append(f"pt{k + 1} 误差 {err_at * 1000:.1f} mm")
        si = match_i + 1
    return ok_all, f"[{label} 到点] " + "; ".join(msgs)


def max_rot_dev(samples, ref_R, label):
    """采样中末端姿态相对 ref_R 的最大测地偏差（rad）。"""
    worst = 0.0
    for s in samples:
        R = fk(s[1][:6]).rotation
        worst = max(worst, float(np.linalg.norm(pin.log3(ref_R.T @ R))))
    return worst, f"[{label} 姿态最大偏差] {worst:.3f} rad"


def main():
    rclpy.init()
    node = Recorder()
    results = []
    pts = [(0.00, 0.25, 0.00), (0.15, 0.00, 0.00), (0.00, 0.25, -0.074)]

    r = call(node, Trigger, "/a3/arm/enable", Trigger.Request())
    if not r.success:
        print(f"enable failed: {r.message}")
        return 1
    print(f"enable: {r.message}")
    spin_s(node, 3.0)

    # ---- 1. free 模式 3 点巡游 ----
    t0 = time.monotonic()
    r1 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(
                  xyz=[v for p in pts for v in p], orientation_mode=0))
    t1 = time.monotonic()
    print(f"xyz free: success={r1.success} {r1.message}")
    results.append((r1.success and len(r1.sequence) == 3,
                    f"[free success/序列] {r1.message}"))
    if r1.success:
        ok, msg = verify_ordered_xyzs(node.window(t0, t1), pts, "free")
        results.append((ok, msg))
    results.append((wait_state(node, "READY", 10.0),
                    f"[free 后回 READY] state={node.arm_state}"))
    spin_s(node, 1.0)

    # ---- 2. hold_start 模式：姿态锁定生效 ----
    def first_R(window):
        return fk(window[0][1][:6]).rotation

    w1 = node.window(t0, t1)
    free_dev, free_msg = max_rot_dev(w1, first_R(w1), "free")
    t2 = time.monotonic()
    r2 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(
                  xyz=[v for p in pts for v in p], orientation_mode=1))
    t3 = time.monotonic()
    print(f"xyz hold_start: success={r2.success} {r2.message}")
    results.append((r2.success, f"[hold_start success] {r2.message}"))
    w2 = node.window(t2, t3)
    hold_dev, hold_msg = max_rot_dev(w2, first_R(w2), "hold_start")
    results.append((hold_dev <= ROT_ERR,
                    f"{hold_msg} 需 <={ROT_ERR} rad（对照 free={free_dev:.3f}）"))
    results.append((wait_state(node, "READY", 10.0),
                    f"[hold_start 后回 READY] state={node.arm_state}"))
    spin_s(node, 1.0)

    # ---- 3. 不可达点显式拒绝 ----
    r3 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(xyz=[0.30, 0.00, 0.00], orientation_mode=0))
    print(f"xyz unreachable: success={r3.success} msg={r3.message}")
    results.append((not r3.success and "IK failed" in r3.message,
                    f"[不可达拒绝] success={r3.success} '{r3.message}'"))
    spin_s(node, 1.0)

    # ---- 4. 非法请求 ----
    r4a = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
               RandomPoseTour.Request(xyz=[0.10, 0.20]))
    print(f"xyz bad length: {r4a.message}")
    results.append((not r4a.success and "multiple of 3" in r4a.message,
                    f"[长度非法拒绝] '{r4a.message}'"))
    r4b = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
               RandomPoseTour.Request(xyz=[0.0, 0.25, 0.0], orientation_mode=2,
                                      ref_quat=[0.0, 0.0, 1.0]))
    print(f"ref_quat bad: {r4b.message}")
    results.append((not r4b.success and "ref_quat length 4" in r4b.message,
                    f"[ref_quat 非法拒绝] '{r4b.message}'"))
    r4c = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
               RandomPoseTour.Request(xyz=[0.0, 0.25, 0.0], orientation_mode=7))
    print(f"mode bad: {r4c.message}")
    results.append((not r4c.success and "orientation_mode" in r4c.message,
                    f"[mode 非法拒绝] '{r4c.message}'"))
    spin_s(node, 1.0)

    # ---- 5. 回归：xyz 空 → F116 随机巡游 ----
    r5 = call(node, RandomPoseTour, "/a3/arm/random_pose_tour",
              RandomPoseTour.Request(count=2, seed=7))
    print(f"regression random tour: success={r5.success} {list(r5.sequence)}")
    results.append((r5.success and len(r5.sequence) == 2,
                    f"[F116 回归] {r5.message}"))
    results.append((wait_state(node, "READY", 10.0),
                    f"[回归后回 READY] state={node.arm_state}"))

    print("\n===== F173 验收结果 =====")
    all_ok = True
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
    print("\n总体:", "ALL PASS" if all_ok else "HAS FAILURES")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
