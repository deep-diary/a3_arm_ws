#!/usr/bin/env python3
"""F108 vcan 数值验收：位置模式 pinocchio 重力前馈。

对标 EDULITE el_a3_hardware.cpp gravity_feedforward_ratio：MIT 位置帧
t_ff = ratio * tau_scale * RNEA(q)，消除自重稳态下垂。

前置（机械臂保持断电）：
  1. sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
  2. python3 scripts/a3_test/vcan_motor_sim.py --interface vcan0
  3. ROS_DOMAIN_ID=59 ros2 launch a3_bringup edge_ros2_control_vcan.launch.py
用法：ROS_DOMAIN_ID=59 python3 f108_gravity_ff_vcan_acceptance.py
退出码 0 = 全部验收项通过。

验收链：
  P1 ratio=0：home 保持帧 t_ff ≈ 0（含 L7）
  P2 ratio=1：home→ready→mid→home，保持帧与运动抽样 t_ff =
     独立参考 RNEA（F49 标定惯量同源）× direction，≤0.02 N·m
  P3 ratio=0.5：ready 保持帧 t_ff ≈ 0.5 × RNEA
  P4 STRICT arm↔zero_torque 切换：effort 帧 kp=0、力矩=RNEA；
     切回后位置帧 t_ff=RNEA（zero_torque 路径不受 F108 影响）
  P5 JTC home→ready→home 全 SUCCESSFUL；全程 L7 帧 t_ff=0
"""

import os
import sys

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

import pinocchio

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from f72_ros2_control_vcan_acceptance import (  # noqa: E402
    ARM_JOINTS,
    DIRECTION,
    CanSniffer,
    Recorder,
    load_package_poses,
    send_jtc,
    spin_s,
    angdiff,
    GOTO_S,
)
from f73_gravity_comp_vcan_acceptance import (  # noqa: E402
    MID,
    apply_f49_inertia,
)

TORQUE_TOL = 0.02
ZERO_TOL = 0.05
HEALTH_NODE = "/a3_hardware_health"


def build_reference_model(urdf):
    """Independent reference: Python pinocchio + F49 fitted inertia, same as plugin."""
    return apply_f49_inertia(pinocchio.buildModelFromXML(urdf))


def reference_torques(model, joint_positions):
    data = pinocchio.Data(model)
    q = pinocchio.neutral(model)
    for k, joint in enumerate(ARM_JOINTS):
        q[model.joints[model.getJointId(joint)].idx_q] = joint_positions[k]
    tau = pinocchio.rnea(
        model, data, q,
        pinocchio.utils.zero(model.nv), pinocchio.utils.zero(model.nv))
    return [tau[model.joints[model.getJointId(j)].idx_v] for j in ARM_JOINTS]


def render_urdf():
    import subprocess
    out = subprocess.run(
        ["xacro",
         "/home/cat/a3_arm_ws/src/a3_description/urdf/el_a3.urdf.xacro",
         "use_real_hardware:=true", "can_interface:=vcan0"],
        capture_output=True, text=True, check=True)
    return out.stdout


def snap_q(sniffer):
    """Snapshot frames and joint configs.

    q_cmd comes from control-frame position fields; q_fb from feedback-frame
    angles — the same source the plugin feeds to RNEA (hw state interfaces).
    Motion checks must use q_fb; holds may use either since they coincide.
    """
    cmd, fb = sniffer.snapshot()
    q_cmd = (
        [cmd[m]["pos"] / DIRECTION[m - 1] for m in range(1, 7)]
        if all(m in cmd for m in range(1, 7)) else None)
    q_fb = (
        [fb[m]["angle"] / DIRECTION[m - 1] for m in range(1, 7)]
        if all(m in fb for m in range(1, 7)) else None)
    return cmd, q_cmd, q_fb


class ParamSetter(Node):
    def __init__(self):
        super().__init__("f108_acceptance")
        self.cli = self.create_client(
            SetParameters, HEALTH_NODE + "/set_parameters")

    def set_ratio(self, ratio):
        # Humble rclpy has no SyncParametersClient; call the node's standard
        # set_parameters service directly.
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetParameters.Request()
        req.parameters = [ParameterMsg(
            name="gravity_feedforward_ratio",
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(ratio)))]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        res = fut.result()
        return bool(res and res.results and res.results[0].successful)


def fmt_worst(w):
    return "无帧" if w is None else f"{w:.4f}"


def hold_error(sniffer, ref_model, ratio):
    """Max |t_ff - ratio*RNEA*dir| over arm motors from one settled snapshot."""
    cmd, _q_cmd, q_fb = snap_q(sniffer)
    if q_fb is None:
        return None, {m: cmd.get(m, {}).get("t_ff") for m in range(1, 7)}
    expect = reference_torques(ref_model, q_fb)
    errs = {}
    worst = 0.0
    for motor in range(1, 7):
        e = abs(cmd[motor]["t_ff"] - ratio * expect[motor - 1] * DIRECTION[motor - 1])
        errs[motor] = e
        worst = max(worst, e)
    return worst, errs


def set_hardware_state(node, target_id):
    # Humble switch_controller does NOT activate INACTIVE-boot hardware
    # (el_a3_controllers.yaml hardware_components_initial_state, LL-086):
    # only this service drives the component lifecycle, which runs the
    # vendor enable choreography in the plugin's on_activate().
    from controller_manager_msgs.srv import SetHardwareComponentState
    cli = node.create_client(
        SetHardwareComponentState,
        "/controller_manager/set_hardware_component_state")
    if not cli.wait_for_service(timeout_sec=5.0):
        return False
    req = SetHardwareComponentState.Request()
    req.name = "RsA3System"
    req.target_state.id = target_id
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    res = fut.result()
    return bool(res and res.ok and res.state.id == target_id)


def switch(node, activate, deactivate):
    from controller_manager_msgs.srv import SwitchController
    cli = node.create_client(
        SwitchController, "/controller_manager/switch_controller")
    if not cli.wait_for_service(timeout_sec=5.0):
        return False
    req = SwitchController.Request()
    req.activate_controllers = list(activate)
    req.deactivate_controllers = list(deactivate)
    req.strictness = SwitchController.Request.STRICT
    req.timeout = rclpy.duration.Duration(seconds=3.0).to_msg()
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    return fut.result() is not None and fut.result().ok


def jtc_with_motion_samples(node, sniffer, ref_model, start6, target6, dur):
    """Send JTC goal; while moving, sample t_ff vs RNEA at instantaneous q."""
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    cli = rclpy.action.ActionClient(
        node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not cli.wait_for_server(timeout_sec=5.0):
        return False, "no server", 1.0
    traj = JointTrajectory(joint_names=ARM_JOINTS)
    n_pts = 31
    for i in range(1, n_pts + 1):
        u = i / n_pts
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        traj.points.append(JointTrajectoryPoint(
            positions=[start6[j] + (target6[j] - start6[j]) * s
                       for j in range(6)],
            time_from_start=rclpy.duration.Duration(seconds=dur * u).to_msg()))
    goal = FollowJointTrajectory.Goal(trajectory=traj)
    goal.goal_time_tolerance = rclpy.duration.Duration(seconds=0.0).to_msg()
    gh_f = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gh_f, timeout_sec=10)
    gh = gh_f.result()
    if not gh.accepted:
        return False, "rejected", 1.0
    res_f = gh.get_result_async()

    worst = 0.0
    samples = 0
    while not res_f.done():
        rclpy.spin_once(node, timeout_sec=0.05)
        cmd, _q_cmd, q_fb = snap_q(sniffer)
        if q_fb is None:
            continue
        moving = max(abs(angdiff(q_fb[j], start6[j])) for j in range(6)) > 0.01
        if not moving:
            continue
        expect = reference_torques(ref_model, q_fb)
        for m in range(1, 7):
            worst = max(
                worst,
                abs(cmd[m]["t_ff"] - expect[m - 1] * DIRECTION[m - 1]))
        samples += 1
    rclpy.spin_until_future_complete(node, res_f, timeout_sec=5)
    res = res_f.result().result
    ok = res.error_code == 0 and samples >= 5
    return ok, f"error_code={res.error_code}, {samples} motion samples", worst


def main():
    rclpy.init()
    node = ParamSetter()
    rec = Recorder()
    sniffer = CanSniffer(os.environ.get("F108_CAN_IF", "vcan0"))
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

    # Controllers spawn while the component stays INACTIVE (LL-086); drive it
    # to ACTIVE here. on_activate needs motor feedback, so vcan_motor_sim must
    # already be running before this call.
    if not set_hardware_state(node, 3):
        print("RsA3System -> ACTIVE 失败（先起 vcan_motor_sim？）")
        return 1
    spin_s(rec, 1.0)

    ref_model = build_reference_model(render_urdf())
    results = []
    max_l7_tff = 0.0

    def track_l7():
        nonlocal max_l7_tff
        cmd, _ = sniffer.snapshot()
        # GripperActionController idles with kp=0 effort frames (t_ff=1 N·m);
        # only position-mode L7 frames (kp≈80) must carry zero feedforward.
        if 7 in cmd and cmd[7].get("kp", 0.0) > 50.0:
            max_l7_tff = max(max_l7_tff, abs(cmd[7]["t_ff"]))

    # ---- P1 ratio=0 ----
    results.append((node.set_ratio(0.0), "[P1 ratio=0 设置]"))
    spin_s(rec, 0.6)
    track_l7()
    worst, _ = hold_error(sniffer, ref_model, 0.0)
    results.append((worst is not None and worst <= ZERO_TOL,
                    f"[P1 ratio=0 保持帧 t_ff≈0] 最大 {worst if worst is not None else '无帧'}"))

    # ---- P2 ratio=1: holds + motion samples ----
    results.append((node.set_ratio(1.0), "[P2 ratio=1 设置]"))
    spin_s(rec, 0.4)

    cur = rec.current()[:6]
    ok, msg, w_motion = jtc_with_motion_samples(
        node, sniffer, ref_model, cur, ready, GOTO_S)
    results.append((ok, f"[P2 JTC home→ready + 运动抽样] {msg}"))
    results.append((w_motion <= TORQUE_TOL,
                    f"[P2 运动中 t_ff=RNEA×dir] 最大偏差 {w_motion:.4f}"))
    spin_s(rec, 2.0)
    worst, _ = hold_error(sniffer, ref_model, 1.0)
    results.append((worst is not None and worst <= TORQUE_TOL,
                    f"[P2 ready 保持帧] 最大偏差 {fmt_worst(worst)}"))
    track_l7()

    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, ready, MID, GOTO_S)
    spin_s(rec, GOTO_S + 2.0)
    settled = rec.current()[:6]
    mid_err = max(abs(angdiff(settled[j], MID[j])) for j in range(6))
    worst, _ = hold_error(sniffer, ref_model, 1.0)
    results.append((ok and mid_err <= 0.03,
                    f"[P2 JTC ready→mid] {msg}，落位误差 {mid_err:.4f}"))
    results.append((worst is not None and worst <= TORQUE_TOL,
                    f"[P2 mid 保持帧] 最大偏差 {fmt_worst(worst)}"))
    track_l7()

    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, MID, home, GOTO_S)
    spin_s(rec, GOTO_S + 2.0)
    worst, _ = hold_error(sniffer, ref_model, 1.0)
    results.append((ok, f"[P2 JTC mid→home] {msg}"))
    results.append((worst is not None and worst <= TORQUE_TOL,
                    f"[P2 home 保持帧] 最大偏差 {fmt_worst(worst)}"))
    track_l7()

    # ---- P3 ratio=0.5 ----
    results.append((node.set_ratio(0.5), "[P3 ratio=0.5 设置]"))
    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, home, ready, GOTO_S)
    spin_s(rec, GOTO_S + 2.0)
    worst, _ = hold_error(sniffer, ref_model, 0.5)
    results.append((ok, f"[P3 JTC home→ready] {msg}"))
    results.append((worst is not None and worst <= TORQUE_TOL,
                    f"[P3 ready 保持帧 t_ff≈0.5RNEA] 最大偏差 {fmt_worst(worst)}"))
    track_l7()

    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, ready, home, GOTO_S)
    spin_s(rec, GOTO_S + 2.0)
    results.append((ok, f"[P3 JTC ready→home] {msg}"))
    track_l7()

    # ---- P4 STRICT switch regression ----
    results.append((node.set_ratio(1.0), "[P4 ratio 恢复 1.0]"))
    ok = switch(node, ["zero_torque_controller"], ["arm_controller"])
    results.append((ok, "[P4 STRICT → zero_torque]"))
    spin_s(rec, 0.8)
    cmd, _q_cmd, q_fb = snap_q(sniffer)
    if q_fb is None:
        results.append((False, "[P4 effort 帧 kp≈0] 无帧（电机未应答？）"))
        results.append((False, "[P4 effort 帧 t_ff=RNEA×dir] 无帧（电机未应答？）"))
    else:
        expect = reference_torques(ref_model, q_fb)
        max_kp = max(cmd[m]["kp"] for m in range(1, 7))
        torque_err = max(
            abs(cmd[m]["t_ff"] - expect[m - 1] * DIRECTION[m - 1])
            for m in range(1, 7))
        results.append((max_kp <= 0.05, f"[P4 effort 帧 kp≈0] 最大 {max_kp:.3f}"))
        results.append((torque_err <= TORQUE_TOL,
                        f"[P4 effort 帧 t_ff=RNEA×dir] 最大偏差 {torque_err:.4f}"))
    track_l7()

    ok = switch(node, ["arm_controller"], ["zero_torque_controller"])
    results.append((ok, "[P4 STRICT → arm_controller]"))
    spin_s(rec, 0.8)
    worst, _ = hold_error(sniffer, ref_model, 1.0)
    results.append((worst is not None and worst <= TORQUE_TOL,
                    f"[P4 切回位置帧 t_ff=RNEA×dir] 最大偏差 {fmt_worst(worst)}"))
    track_l7()

    # ---- P5 motion regression + L7 ----
    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, home, ready, GOTO_S)
    results.append((ok, f"[P5 JTC home→ready SUCCESSFUL] {msg}"))
    spin_s(rec, GOTO_S + 2.0)
    track_l7()
    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, ready, home, GOTO_S)
    results.append((ok, f"[P5 JTC ready→home SUCCESSFUL] {msg}"))
    spin_s(rec, GOTO_S + 2.0)
    track_l7()
    results.append((max_l7_tff <= ZERO_TOL,
                    f"[P5 全程 L7 帧 t_ff=0] 最大 {max_l7_tff:.4f}"))

    sniffer.stop()

    print("\n===== F108 验收结果 =====")
    all_ok = True
    n_pass = 0
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
        n_pass += int(ok)
    print(f"\n总体: {'ALL PASS' if all_ok else 'HAS FAILURES'} ({n_pass}/{len(results)})")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
