#!/usr/bin/env python3
"""F160 vcan 数值验收：MIT 位置环速度/加速度前馈（VFF + AFF + 科氏/离心）。

把 F108 的「只重力前馈」扩展为 computed-torque 全模型前馈：
  τ = kp·(q_des−q) + kd·(q̇_des−q̇) + M(q)·q̈_des + C(q,q̇_des)·q̇_des + G(q)

本脚本用 vcan0 + vcan_motor_sim 闭环验证真机插件：
  P1 gravity 模式回归：MIT velocity 域恒 0，保持帧 t_ff = RNEA(q,0,0)
  P2 full 模式 VFF：运动中 velocity 域非 0，静止回落 0；保持帧 t_ff 仍 = 重力
  P3 AFF+科氏前馈数值：full 模式运动中 t_ff 相对纯重力有额外残差（科氏+惯量）
  P4 运行时切换 feedforward_mode gravity↔full 生效
  P5 回归：L7 夹爪 velocity/t_ff 恒 0；effort 帧（zero_torque）不受影响

前置（机械臂保持断电）：
  1. sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
  2. python3 scripts/a3_test/vcan_motor_sim.py --interface vcan0
  3. ROS_DOMAIN_ID=59 ros2 launch a3_bringup edge_ros2_control_vcan.launch.py
用法：ROS_DOMAIN_ID=59 python3 f160_velocity_accel_ff_vcan_acceptance.py
退出码 0 = 全部验收项通过。
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
from f73_gravity_comp_vcan_acceptance import MID, apply_f49_inertia  # noqa: E402

TORQUE_TOL = 0.02
ZERO_VEL_TOL = 0.06   # MIT velocity 量化 ~ speed_max/32768*2，略放宽
VEL_MOTION_MIN = 0.2  # rad/s：运动中 velocity 域应超过此阈（VFF 生效）
HEALTH_NODE = "/a3_hardware_health"


def build_reference_model(urdf):
    """独立参考：Python pinocchio + F49 标定惯量，同插件。"""
    return apply_f49_inertia(pinocchio.buildModelFromXML(urdf))


def reference_gravity(model, q):
    data = pinocchio.Data(model)
    qq = pinocchio.neutral(model)
    for k, joint in enumerate(ARM_JOINTS):
        qq[model.joints[model.getJointId(joint)].idx_q] = q[k]
    tau = pinocchio.rnea(
        model, data, qq,
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


class ParamSetter(Node):
    """运行时改 /a3_hardware_health 参数（沿用 f108 的 set_parameters 直调）。"""

    def __init__(self):
        super().__init__("f160_acceptance")
        self.cli = self.create_client(
            SetParameters, HEALTH_NODE + "/set_parameters")

    def set_param(self, name, value):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return False
        req = SetParameters.Request()
        p = ParameterMsg(name=name)
        if isinstance(value, str):
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=value)
        else:
            p.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        req.parameters = [p]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        res = fut.result()
        return bool(res and res.results and res.results[0].successful)

    def set_ratio(self, ratio):
        return self.set_param("gravity_feedforward_ratio", ratio)

    def set_mode(self, mode):
        return self.set_param("feedforward_mode", mode)


def snap_vel_ff(sniffer):
    """一次快照：返回 (max_abs_vel, max_abs_tff, max_l7_vel, max_l7_tff)。"""
    cmd, _ = sniffer.snapshot()
    max_vel = max((abs(cmd[m]["vel"]) for m in range(1, 7)), default=0.0)
    max_tff = max((abs(cmd[m]["t_ff"]) for m in range(1, 7)), default=0.0)
    l7_vel = abs(cmd[7]["vel"]) if 7 in cmd else 0.0
    # 仅 position 模式 L7 帧（kp>50）才算，GAC 空闲 effort 帧 t_ff=1 Nm 跳过
    l7_tff = abs(cmd[7]["t_ff"]) if (7 in cmd and cmd[7].get("kp", 0.0) > 50.0) else 0.0
    return max_vel, max_tff, l7_vel, l7_tff


def hold_tff_error(sniffer, ref_model):
    """保持帧 t_ff 与 RNEA(q,0,0) 的最大偏差（静止 v=a=0，full 应退化为重力）。"""
    cmd, _q_cmd, q_fb = None, None, None
    cmd, fb = sniffer.snapshot()
    q_fb = (
        [fb[m]["angle"] / DIRECTION[m - 1] for m in range(1, 7)]
        if all(m in fb for m in range(1, 7)) else None)
    if q_fb is None or any(m not in cmd for m in range(1, 7)):
        return None
    expect = reference_gravity(ref_model, q_fb)
    worst = 0.0
    for m in range(1, 7):
        worst = max(
            worst, abs(cmd[m]["t_ff"] - expect[m - 1] * DIRECTION[m - 1]))
    return worst


def run_fast_traj(node, sniffer, ref_model, start6, target6, dur):
    """发一条快速 quintic 轨迹，采样运动中 velocity 域峰值与 t_ff 相对纯重力的残差峰值。

    vcan_motor_sim 是位置一阶滞后模型，不建模 MIT 位置环动力学，故不能直接
    用跟踪误差 A/B 证明 AFF/VFF 收益（该收益待真机验收）；这里退而验证前馈
    通道数值正确：full 模式下运动中 t_ff 应额外携带科氏+惯量分量（残差>0）。
    """
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    cli = rclpy.action.ActionClient(
        node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not cli.wait_for_server(timeout_sec=5.0):
        return False, "no action server", 0.0, 0.0
    traj = JointTrajectory(joint_names=ARM_JOINTS)
    n_pts = 61
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
        return False, "rejected", 0.0, 0.0
    res_f = gh.get_result_async()

    max_vel = 0.0
    max_ff_residual = 0.0
    while not res_f.done():
        rclpy.spin_once(node, timeout_sec=0.05)
        cmd, fb = sniffer.snapshot()
        if not (all(m in cmd for m in range(1, 7)) and
                all(m in fb for m in range(1, 7))):
            continue
        max_vel = max(max_vel, max(abs(cmd[m]["vel"]) for m in range(1, 7)))
        q_fb = [fb[m]["angle"] / DIRECTION[m - 1] for m in range(1, 7)]
        g = reference_gravity(ref_model, q_fb)
        for m in range(1, 7):
            max_ff_residual = max(
                max_ff_residual,
                abs(cmd[m]["t_ff"] - g[m - 1] * DIRECTION[m - 1]))
    rclpy.spin_until_future_complete(node, res_f, timeout_sec=5)
    res = res_f.result().result
    ok = res.error_code == 0
    return ok, f"error_code={res.error_code}", max_vel, max_ff_residual


def main():
    rclpy.init()
    node = ParamSetter()
    rec = Recorder()
    sniffer = CanSniffer(os.environ.get("F160_CAN_IF", "vcan0"))
    try:
        sniffer.start()
    except OSError as e:
        print(f"vcan 抓包 socket 起不来：{e}（先起 vcan0 + 电机模拟器？）")
        return 1

    spin_s(rec, 1.0)
    poses = load_package_poses()
    seed = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]  # vcan_motor_sim 播种位
    ready = poses["ready"][:6]

    if max(abs(angdiff(rec.current()[j], seed[j])) for j in range(6)) > 0.02:
        print(f"当前位不是 vcan 播种位，先核对栈/模拟器：{rec.current()}")
        return 1

    # 硬件 INACTIVE 启动（LL-086），驱动到 ACTIVE 才跑 vendor 使能编排
    if not set_hardware_state(node, 3):
        print("RsA3System -> ACTIVE 失败（先起 vcan_motor_sim？）")
        return 1
    spin_s(rec, 1.0)

    ref_model = build_reference_model(render_urdf())
    results = []

    # ---- P1 gravity 模式回归 ----
    results.append((node.set_mode("gravity"), "[P1 feedforward_mode=gravity 设置]"))
    results.append((node.set_ratio(1.0), "[P1 ratio=1.0 设置]"))
    spin_s(rec, 0.5)
    ok, msg, max_vel, _res = run_fast_traj(node, sniffer, ref_model, seed, ready, 1.5)
    results.append((ok, f"[P1 gravity JTC seed→ready] {msg}"))
    results.append((max_vel <= ZERO_VEL_TOL,
                    f"[P1 gravity velocity 域恒 0] 运动中峰值 {max_vel:.4f}"))
    spin_s(rec, 1.0)
    worst = hold_tff_error(sniffer, ref_model)
    results.append((worst is not None and worst <= TORQUE_TOL,
                    f"[P1 gravity 保持帧 t_ff=RNEA(q,0,0)] 最大偏差 {worst if worst is not None else '无帧'}"))

    # ---- P2 full 模式 VFF ----
    results.append((node.set_mode("full"), "[P2 feedforward_mode=full 设置]"))
    spin_s(rec, 0.3)
    ok, msg, max_vel_full, _res = run_fast_traj(node, sniffer, ref_model, ready, MID, 1.5)
    results.append((ok, f"[P2 full JTC ready→mid] {msg}"))
    results.append((max_vel_full >= VEL_MOTION_MIN,
                    f"[P2 full velocity 域非 0（VFF）] 运动中峰值 {max_vel_full:.4f}"))
    spin_s(rec, 1.0)
    worst = hold_tff_error(sniffer, ref_model)
    results.append((worst is not None and worst <= TORQUE_TOL,
                    f"[P2 full 保持帧 t_ff 退化回重力] 最大偏差 {worst if worst is not None else '无帧'}"))

    # ---- P3 AFF+科氏前馈数值（同轨迹 mid→seed，full vs gravity 的 t_ff 残差）----
    results.append((node.set_mode("full"), "[P3 full 设置]"))
    ok, msg, _v, full_residual = run_fast_traj(node, sniffer, ref_model, MID, seed, 1.5)
    results.append((ok, f"[P3 full JTC mid→seed] {msg}"))
    results.append((node.set_mode("gravity"), "[P3 gravity 设置]"))
    spin_s(rec, 0.3)
    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, seed, MID, GOTO_S)
    spin_s(rec, GOTO_S + 1.0)
    ok, msg, _v, gravity_residual = run_fast_traj(node, sniffer, ref_model, MID, seed, 1.5)
    results.append((ok, f"[P3 gravity JTC mid→seed] {msg}"))
    results.append((full_residual > gravity_residual + TORQUE_TOL,
                    f"[P3 full 前馈含科氏+惯量] full残差={full_residual:.4f} > gravity残差={gravity_residual:.4f}"))

    # ---- P4 运行时切换（seed↔mid 同一段）----
    results.append((node.set_mode("full"), "[P4 运行时切 full]"))
    ok, msg, max_vel, _res = run_fast_traj(node, sniffer, ref_model, seed, MID, 1.2)
    results.append((ok and max_vel >= VEL_MOTION_MIN,
                    f"[P4 full velocity 域非 0] {msg}, 峰值 {max_vel:.4f}"))
    results.append((node.set_mode("gravity"), "[P4 运行时切回 gravity]"))
    ok, msg, max_vel, _res = run_fast_traj(node, sniffer, ref_model, MID, seed, 1.2)
    results.append((ok and max_vel <= ZERO_VEL_TOL,
                    f"[P4 gravity velocity 域回落 0] {msg}, 峰值 {max_vel:.4f}"))

    # ---- P5 回归：L7 与 effort 帧 ----
    results.append((node.set_mode("gravity"), "[P5 恢复 gravity]"))
    results.append((node.set_ratio(1.0), "[P5 ratio 恢复 1.0]"))
    ok, msg = send_jtc(node, "/arm_controller/follow_joint_trajectory",
                       ARM_JOINTS, seed, ready, GOTO_S)
    spin_s(rec, GOTO_S + 2.0)
    results.append((ok, f"[P5 JTC seed→ready SUCCESSFUL] {msg}"))
    _max_vel, _max_tff, l7_vel, l7_tff = snap_vel_ff(sniffer)
    results.append((l7_vel <= ZERO_VEL_TOL and l7_tff <= TORQUE_TOL,
                    f"[P5 L7 velocity/t_ff 恒 0] vel={l7_vel:.4f} tff={l7_tff:.4f}"))

    # STRICT arm↔zero_torque 切换（F127/LL-136：zero_torque 含 L7 effort，
    # 须与 gripper_controller 的 L7 effort 原子换出，否则 STRICT 被拒）
    ok = switch(node, ["zero_torque_controller"], ["arm_controller", "gripper_controller"])
    results.append((ok, "[P5 STRICT → zero_torque（L7 原子释放）]"))
    spin_s(rec, 0.6)
    cmd, _ = sniffer.snapshot()
    max_kp = max((cmd[m]["kp"] for m in range(1, 7)), default=0.0)
    results.append((max_kp <= 0.05, f"[P5 effort 帧 kp≈0] 最大 {max_kp:.3f}"))
    ok = switch(node, ["arm_controller", "gripper_controller"], ["zero_torque_controller"])
    results.append((ok, "[P5 STRICT → arm_controller]"))

    sniffer.stop()

    print("\n===== F160 验收结果 =====")
    all_ok = True
    n_pass = 0
    for ok, msg in results:
        print(("PASS " if ok else "FAIL ") + msg)
        all_ok = all_ok and ok
        n_pass += int(ok)
    print(f"\n总体: {'ALL PASS' if all_ok else 'HAS FAILURES'} ({n_pass}/{len(results)})")
    return 0 if all_ok else 1


def set_hardware_state(node, target_id):
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


if __name__ == "__main__":
    sys.exit(main())
