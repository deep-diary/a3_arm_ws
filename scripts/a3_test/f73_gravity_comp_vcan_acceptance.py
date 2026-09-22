#!/usr/bin/env python3
"""F73 vcan 数值验收：力矩指令模式 + 标准重力补偿自由拖动控制器。

前置：
  1. python3 scripts/a3_test/vcan_motor_sim.py --interface vcan0
  2. ROS_DOMAIN_ID=59 ros2 launch a3_bringup edge_ros2_control_vcan.launch.py \
       adaptive_kd_enabled:=false
用法：ROS_DOMAIN_ID=59 python3 f73_gravity_comp_vcan_acceptance.py
退出码 0 = 全部验收项通过。

验收链（全程无自研节点参与，机械臂保持断电，vcan0 闭环）：
  1. zero_torque_controller 以 inactive 状态常驻（list_controllers）
  2. 多姿态（home / ready / 中间姿态）标准 switch_controllers 互斥切换：
     arm_controller(deactivate) + zero_torque_controller(activate) 原子完成
     CAN 抓包：kp≈0、kd≈zero_torque_kd(0.3 固定兜底，F85 adaptive 关闭)、vel=0、位置字段=当前测量位、
     torque_ff = 独立 Python-Pinocchio RNEA 重力矩 × direction（≤0.02 Nm）
  3. ready 姿态外力注入（sim ext 文件 motor3 +0.6 Nm）：关节单调跟随、
     全程 kp≈0、torque_ff 实时跟随 RNEA；撤力后在新位姿零力矩保持（漂移≤0.03）
  4. 切回 arm_controller：kp≈80、kd≈2、torque_ff=0
  5. 切回后 JTC home→ready→home 仍 ALL PASS（落位误差 ≤0.02）
"""

import json
import math
import os
import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node

import pinocchio

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from f72_ros2_control_vcan_acceptance import (  # noqa: E402
    ARM_JOINTS,
    DIRECTION,
    CanSniffer,
    load_package_poses,
    send_jtc,
    spin_s,
    angdiff,
    GOTO_S,
)

FREE_DRIVE = "zero_torque_controller"
ARM_CTRL = "arm_controller"
TORQUE_TOL = 0.02
KD_EFFORT = 0.3
KP_POSITION = 80.0
KD_POSITION = 2.0

# 中间姿态（L1–L6），提供与 home/ready 不同的重力矩工况
MID = [0.30, -0.40, 0.60, -0.35, 0.25, 0.45]
MID_TARGET_TOL = 0.03


def render_urdf():
    desc = "/home/cat/a3_arm_ws/src/a3_description/urdf/el_a3.urdf.xacro"
    out = subprocess.run(
        ["xacro", desc, "use_real_hardware:=true", "can_interface:=vcan0"],
        capture_output=True, text=True, check=True,
    )
    return out.stdout


def gravity_torques(urdf, joint_positions):
    """独立参考实现：Python-Pinocchio RNEA（v=a=0），返回 L1–L6 重力矩。"""
    model = pinocchio.buildModelFromXML(urdf)
    data = pinocchio.Data(model)
    q = pinocchio.neutral(model)
    for k, joint in enumerate(ARM_JOINTS):
        idx = model.joints[model.getJointId(joint)].idx_q
        q[idx] = joint_positions[k]
    tau = pinocchio.rnea(model, data, q,
                         pinocchio.utils.zero(model.nv),
                         pinocchio.utils.zero(model.nv))
    return [tau[model.joints[model.getJointId(j)].idx_v] for j in ARM_JOINTS]


class SwitchClient(Node):
    def __init__(self):
        super().__init__("f73_acceptance")
        self.switch_cli = self.create_client(
            SwitchController, "/controller_manager/switch_controller")

    def switch(self, activate, deactivate, strict=True):
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            return False, "switch_controller 服务不可用"
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = (SwitchController.Request.STRICT if strict
                          else SwitchController.Request.BEST_EFFORT)
        req.activate_asap = False
        req.timeout = rclpy.duration.Duration(seconds=3.0).to_msg()
        fut = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        res = fut.result()
        if res is None:
            return False, "switch 调用超时"
        return res.ok, "ok"

    def controller_states(self):
        from controller_manager_msgs.srv import ListControllers
        cli = self.create_client(ListControllers,
                                 "/controller_manager/list_controllers")
        if not cli.wait_for_service(timeout_sec=5.0):
            return {}
        fut = cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        return {c.name: c.state for c in fut.result().controller}


def move_and_settle(node, target6, duration, tol, label):
    start = node.current()
    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, start[:6], target6, duration)
    spin_s(node, duration + 2.0)
    settled = node.current()
    err = max(abs(angdiff(settled[j], target6[j])) for j in range(6))
    return ok and err <= tol, f"{label}: 发送 {msg}，落位误差 {err:.4f}", settled


def check_effort_frames(node, sniffer, urdf, label):
    spin_s(node, 0.6)
    settled = node.current()
    cmd, _ = sniffer.snapshot()
    expect_tau = gravity_torques(urdf, settled[:6])

    results = []
    kp_vals, kd_vals, max_vel = {}, {}, 0.0
    max_tau_err = 0.0
    max_pos_err = 0.0
    for motor in range(1, 7):
        j = motor - 1
        frame = cmd.get(motor)
        if frame is None:
            results.append((False, f"{label}: motor {motor} 无控制帧"))
            continue
        kp_vals[motor] = frame["kp"]
        kd_vals[motor] = frame["kd"]
        max_vel = max(max_vel, abs(frame["vel"]))
        expect_motor_torque = expect_tau[j] * DIRECTION[j]
        max_tau_err = max(max_tau_err, abs(frame["t_ff"] - expect_motor_torque))
        expect_motor_pos = DIRECTION[j] * settled[j]
        max_pos_err = max(max_pos_err, abs(frame["pos"] - expect_motor_pos))

    got = len(kp_vals)
    results.append((
        got == 6 and all(abs(v - 0.0) <= 0.05 for v in kp_vals.values()),
        f"[{label} CAN kp≈0] { {m: round(v, 3) for m, v in kp_vals.items()} }"))
    results.append((
        got == 6 and all(abs(v - KD_EFFORT) <= 0.05 for v in kd_vals.values()),
        f"[{label} CAN kd≈{KD_EFFORT}] { {m: round(v, 3) for m, v in kd_vals.items()} }"))
    results.append((max_vel <= 0.01, f"[{label} CAN vel=0] 最大 {max_vel:.5f}"))
    results.append((
        max_pos_err <= 0.01,
        f"[{label} CAN 位置字段=当前测量位] 最大偏差 {max_pos_err:.5f}"))
    results.append((
        max_tau_err <= TORQUE_TOL,
        f"[{label} torque_ff=RNEA×dir] 最大偏差 {max_tau_err:.4f} Nm"))
    return results


def external_force_phase(rec, sniffer, urdf):
    """外力注入：motor3 +0.6 Nm 持续 1.5 s，关节跟随；撤力后零力矩保持。"""
    ext_path = os.environ.get("F73_EXT_FILE", "/tmp/f73_ext.json")
    ji = 2  # L3
    motor = 3
    push = 0.6

    def latest():
        return rec.samples[-1][1][ji] if rec.samples else rec.current()[ji]

    start = latest()
    with open(ext_path, "w") as f:
        json.dump({"motor": motor, "torque": push}, f)

    end_pose = start
    same_sign_steps = 0
    total_steps = 0
    max_tau_err = 0.0
    max_kp = 0.0
    try:
        prev = start
        for _ in range(10):
            spin_s(rec, 0.15)
            p = latest()
            d = angdiff(p, prev)  # new − old
            total_steps += 1
            if abs(d) > 1e-4 and (d > 0) == (push * DIRECTION[ji] > 0):
                same_sign_steps += 1
            prev = p

            cmd, _ = sniffer.snapshot()
            frame = cmd.get(motor, {})
            max_kp = max(max_kp, frame.get("kp", 0.0))
            # RNEA expected value at the joints embedded in THIS snapshot's
            # CAN frames (motor pos / direction), not at the median-delayed
            # rec.current(): the arm is moving fast here.
            if all(m in cmd for m in range(1, 7)):
                frame_q = [cmd[m]["pos"] / DIRECTION[m - 1] for m in range(1, 7)]
                expect_tau = gravity_torques(urdf, frame_q)
                max_tau_err = max(
                    max_tau_err,
                    abs(frame.get("t_ff", 0.0) - expect_tau[ji] * DIRECTION[ji]))
        end_pose = latest()
    finally:
        with open(ext_path, "w") as f:
            json.dump({}, f)

    total = angdiff(end_pose, start)
    results = [
        (abs(total) >= 0.10 and
         (total > 0) == (push * DIRECTION[ji] > 0),
         f"[外力注入 关节跟随] L3 位移 {total:.3f} rad（期望同号幅度≥0.10）"),
        (same_sign_steps >= max(1, int(0.8 * total_steps)),
         f"[外力注入 单调跟随] {same_sign_steps}/{total_steps} 步同方向"),
        (max_kp <= 0.05,
         f"[外力注入 全程无力控] kp 最大 {max_kp:.3f}"),
        (max_tau_err <= 0.05,
         f"[外力注入 torque_ff 实时=RNEA×dir] 最大偏差 {max_tau_err:.4f} Nm"),
    ]

    # Settle past current()'s ~2 s median window, then compare two fully
    # settled windows: pure holding drift, independent of push-end samples.
    spin_s(rec, 2.5)
    hold1 = rec.current()[ji]
    spin_s(rec, 1.5)
    hold2 = rec.current()[ji]
    drift = abs(angdiff(hold2, hold1))
    results.append((drift <= 0.03,
                    f"[撤力 零力矩保持] 稳定窗口间漂移 {drift:.4f} rad"))
    return results


def main():
    rclpy.init()
    node = SwitchClient()
    # 复用 F72 Recorder 的 /joint_states 记录（current()）
    from f72_ros2_control_vcan_acceptance import Recorder
    rec = Recorder()

    sniffer = CanSniffer(os.environ.get("F73_CAN_IF", "vcan0"))
    try:
        sniffer.start()
    except OSError as e:
        print(f"vcan 抓包 socket 起不来：{e}（先起 vcan0 + 电机模拟器？）")
        return 1

    spin_s(rec, 1.0)
    poses = load_package_poses()
    home = poses["home"][:6]
    ready = poses["ready"][:6]

    if max(abs(angdiff(rec.current()[j], home[j])) for j in range(6)) > 0.02:
        print(f"当前位不是包内 home，先核对栈/模拟器：{rec.current()}")
        return 1

    urdf = render_urdf()
    results = []

    states = node.controller_states()
    results.append((
        states.get(FREE_DRIVE) == "inactive" and states.get(ARM_CTRL) == "active",
        f"[初始控制器状态] arm={states.get(ARM_CTRL)}, free_drive={states.get(FREE_DRIVE)}"))

    test_poses = [("home", home, None), ("ready", ready, GOTO_S),
                  ("mid", MID, GOTO_S)]
    for label, pose, move_dur in test_poses:
        if move_dur is not None:
            tol = 0.02 if label == "ready" else MID_TARGET_TOL
            ok, msg, _ = move_and_settle(rec, pose, move_dur, tol, label)
            results.append((ok, f"[JTC → {label}] {msg}"))

        ok, msg = node.switch(activate=[FREE_DRIVE], deactivate=[ARM_CTRL])
        results.append((ok, f"[switch → 自由拖动 @{label}] {msg}"))
        if ok:
            results.extend(check_effort_frames(rec, sniffer, urdf, label))
        if ok and label == "ready":
            results.extend(external_force_phase(rec, sniffer, urdf))

        ok, msg = node.switch(activate=[ARM_CTRL], deactivate=[FREE_DRIVE])
        results.append((ok, f"[switch → 位置模式 @{label}] {msg}"))
        if ok:
            spin_s(rec, 0.5)
            cmd, _ = sniffer.snapshot()
            kp_ok = all(abs(cmd.get(m, {}).get("kp", -1) - KP_POSITION) <= 0.5
                        for m in range(1, 7))
            kd_ok = all(abs(cmd.get(m, {}).get("kd", -1) - KD_POSITION) <= 0.05
                        for m in range(1, 7))
            tff_ok = all(abs(cmd.get(m, {}).get("t_ff", 1.0)) <= 0.05
                         for m in range(1, 7))
            results.append((kp_ok, f"[切回 @{label} kp≈80]"))
            results.append((kd_ok, f"[切回 @{label} kd≈2]"))
            results.append((tff_ok, f"[切回 @{label} torque_ff=0]"))

    # 切换回归：标准 JTC home→ready→home 仍正常
    ok, msg, _ = move_and_settle(rec, ready, GOTO_S, 0.02, "回归 ready")
    results.append((ok, f"[切回后 JTC home→ready] {msg}"))
    ok, msg, _ = move_and_settle(rec, home, GOTO_S, 0.02, "回归 home")
    results.append((ok, f"[切回后 JTC ready→home] {msg}"))

    sniffer.stop()

    print("\n===== F73 验收结果 =====")
    all_ok = True
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
    print("\n总体:", "ALL PASS" if all_ok else "HAS FAILURES")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
