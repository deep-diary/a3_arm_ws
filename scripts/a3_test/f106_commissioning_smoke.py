#!/usr/bin/env python3
"""F106 — one-shot commissioning smoke test (对标 EDULITE startup_test_demo).

一条命令自动起栈（mock/real 统一走 F78 产品入口 a3_bringup.launch.py），
按六阶段验证整条产品链路并输出 PASS/FAIL 报告与退出码，结束自动清栈：

  P1 图健康      P2 控制器 + 产品使能      P3 反馈读取
  P4 基础运动    P5 夹爪                  P6 zero_torque 控制器切换

模式：
  --mode mock    默认；hardware:=mock，隔离域 ROS_DOMAIN_ID=106
  --mode real    hardware:=can（默认 can1），仅用户授权通电时
  --mode connect 附着当前已运行栈（不起/不停栈）

P6（zero_torque 实时切换）对标 EDULITE startup_test_demo 的 --test-zero-torque，
默认不执行；mock_components/GenericSystem 的 prepare_command_mode_switch
不接受 effort-only 模式（同 F88），mock 下即使加 flag 也只做静态核验。
真机/connect 的实时切换路径由 F89b vcan 20/20 覆盖。
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time

WS_DIR = "/home/cat/a3_arm_ws"
ARM_JOINTS = ["L1_joint", "L2_joint", "L3_joint",
              "L4_joint", "L5_joint", "L6_joint"]
ALL_JOINTS = ARM_JOINTS + ["L7_joint"]
MOCK_DOMAIN = "106"
POSITION_TOL = 0.05
MOVE_DT = 3.0
MOVE_AMP = 0.20
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.78
GRIPPER_TOL = 0.05

LAUNCH_PROC = None
WE_ENABLED = False


def ensure_env(mock: bool) -> None:
    """Re-exec under a3_shell_env.sh so rclpy/ament packages are importable."""
    if os.environ.get("A3_F106_ENVED") == "1":
        return
    out = subprocess.run(
        ["bash", "-c",
         f"source {WS_DIR}/scripts/a3_shell_env.sh >/dev/null 2>&1 && env"],
        capture_output=True, text=True, check=False,
    )
    env = {}
    for line in out.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            env[k] = v
    if "ROS_DOMAIN_ID" not in env and "ROS_DOMAIN_ID" in os.environ:
        env["ROS_DOMAIN_ID"] = os.environ["ROS_DOMAIN_ID"]
    if mock:
        env["ROS_DOMAIN_ID"] = MOCK_DOMAIN
    env["A3_F106_ENVED"] = "1"
    os.execvpe(sys.executable,
               [sys.executable, os.path.abspath(__file__), *sys.argv[1:]], env)


class JointRecorder:
    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self.msg = None
        from sensor_msgs.msg import JointState
        node.create_subscription(
            JointState, "/joint_states", self._cb, 20)

    def _cb(self, msg):
        with self._lock:
            self.msg = msg

    def latest(self):
        with self._lock:
            return self.msg

    def positions(self, joints):
        for _ in range(50):
            msg = self.latest()
            if msg and all(j in msg.name for j in joints):
                return [msg.position[msg.name.index(j)] for j in joints]
            time.sleep(0.1)
        raise RuntimeError(f"joint_states missing among {joints}")


class StateRecorder:
    def __init__(self, node):
        self._lock = threading.Lock()
        self.state = ""
        from a3_msgs.msg import ArmStatus
        node.create_subscription(
            ArmStatus, "/a3/arm_status", self._cb, 10)

    def _cb(self, msg):
        with self._lock:
            self.state = msg.state

    def get(self):
        with self._lock:
            return self.state


class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, ok, detail=""):
        mark = "PASS" if ok else "FAIL"
        line = f"  [{mark}] {name}"
        if detail:
            line += f"  ({detail})"
        print(line, flush=True)
        if ok:
            self.passed += 1
        else:
            self.failed += 1
        return ok

    def skip(self, name, detail=""):
        line = f"  [SKIP] {name}"
        if detail:
            line += f"  ({detail})"
        print(line, flush=True)

    def phase(self, title):
        print(f"\n=== {title} ===", flush=True)


def wait_future(fut, timeout):
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end:
        time.sleep(0.02)
    return fut.done()


def call_service(node, srv_type, srv_name, request, timeout=5.0, wait=0.5):
    cli = node.srv_clients.get(srv_name)
    if cli is None:
        cli = node.create_client(srv_type, srv_name)
        node.srv_clients[srv_name] = cli
    if not cli.wait_for_service(timeout_sec=wait):
        raise RuntimeError(f"service unavailable: {srv_name}")
    fut = cli.call_async(request)
    if not wait_future(fut, timeout):
        raise RuntimeError(f"service timeout: {srv_name}")
    return fut.result()


def list_controllers(node):
    from controller_manager_msgs.srv import ListControllers
    res = call_service(node, ListControllers,
                       "/controller_manager/list_controllers",
                       ListControllers.Request(), timeout=5.0)
    return {c.name: c for c in res.controller}


def wait_nodes(node, wanted, deadline_s):
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        names = node.get_node_names()
        if all(any(n == w for n in names) for w in wanted):
            return True
        time.sleep(0.5)
    return False


def send_fjt(node, recorder, target_positions, label, rep, mock):
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration as RDuration
    ac = node.fjt_ac
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = [float(x) for x in target_positions]
    pt.velocities = [0.0] * 6
    pt.time_from_start = RDuration(sec=int(MOVE_DT), nanosec=0)
    goal.trajectory.points = [pt]
    goal.goal_time_tolerance = RDuration(sec=1, nanosec=0)
    gh_fut = ac.send_goal_async(goal)
    if not wait_future(gh_fut, 5.0):
        rep.check(f"FJT {label} goal accepted", False, "goal timeout")
        return False
    gh = gh_fut.result()
    if not gh.accepted:
        rep.check(f"FJT {label} goal accepted", False, "rejected")
        return False
    res_fut = gh.get_result_async()
    if not wait_future(res_fut, MOVE_DT + 15.0):
        rep.check(f"FJT {label} result", False, "result timeout")
        return False
    result = res_fut.result().result
    ok = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    rep.check(f"FJT {label} SUCCESSFUL", ok, f"error_code={result.error_code}")
    if not ok:
        return False
    time.sleep(0.4)
    actual = recorder.positions(ARM_JOINTS)
    err = max(abs(a - t) for a, t in zip(actual, target_positions))
    rep.check(f"FJT {label} tracking err <{POSITION_TOL}", err < POSITION_TOL,
              f"max_err={err:.4f}")
    return err < POSITION_TOL


def send_gripper(node, recorder, position, label, rep):
    from control_msgs.action import GripperCommand
    ac = node.gripper_ac
    goal = GripperCommand.Goal()
    goal.command.position = float(position)
    goal.command.max_effort = 1.0
    gh_fut = ac.send_goal_async(goal)
    if not wait_future(gh_fut, 5.0):
        rep.check(f"gripper {label} goal", False, "goal timeout")
        return False
    gh = gh_fut.result()
    if not gh.accepted:
        rep.check(f"gripper {label} goal", False, "rejected")
        return False
    res_fut = gh.get_result_async()
    if not wait_future(res_fut, 15.0):
        rep.check(f"gripper {label} result", False, "result timeout")
        return False
    result = res_fut.result().result
    time.sleep(0.3)
    actual_l7 = recorder.positions(ALL_JOINTS)[6]
    ok = result.reached_goal and abs(actual_l7 - position) <= GRIPPER_TOL
    rep.check(
        f"gripper {label} reached {position:.2f}", ok,
        f"reached_goal={result.reached_goal} actual={actual_l7:.3f} "
        f"stalled={result.stalled}")
    return ok


def cleanup_launch(log_path):
    global LAUNCH_PROC
    proc = LAUNCH_PROC
    LAUNCH_PROC = None
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=12.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=5.0)
    print(f"launch stopped; log: {log_path}", flush=True)


def main():
    global LAUNCH_PROC, WE_ENABLED

    ap = argparse.ArgumentParser(description="F106 commissioning smoke test")
    ap.add_argument("--mode", choices=["mock", "real", "connect"],
                    default="mock")
    ap.add_argument("--can-interface", default="can1")
    ap.add_argument("--wait-sec", type=float, default=45.0,
                    help="P1 stack-up timeout")
    ap.add_argument("--test-zero-torque", action="store_true",
                    help="执行 P6 zero_torque 实时 STRICT 切换（mock 下因 "
                         "mock_components 不支持 effort-only 模式切换而跳过）")
    args = ap.parse_args()
    ensure_env(args.mode == "mock")

    import rclpy
    import rclpy.action
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import (
        ListControllers, LoadController, ConfigureController, SwitchController)

    rclpy.init()
    node = rclpy.create_node("f106_commissioning_smoke")
    node.srv_clients = {}
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    recorder = JointRecorder(node)
    state_rec = StateRecorder(node)
    node.fjt_ac = rclpy.action.ActionClient(
        node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    node.gripper_ac = rclpy.action.ActionClient(
        node, GripperCommand, "/gripper_controller/gripper_cmd")

    rep = Report()
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = f"/tmp/f106_launch_{ts}.log"

    def signal_handler(signum, frame):
        print(f"\nsignal {signum} received, cleaning up...", flush=True)
        cleanup_launch(log_path)
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if args.mode in ("mock", "real"):
        launch_args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
                       f"hardware:={'mock' if args.mode == 'mock' else 'can'}"]
        if args.mode == "real":
            launch_args.append(f"can_interface:={args.can_interface}")
        else:
            launch_args += ["use_rviz:=false", "use_mqtt:=false",
                            "use_teleop:=false"]
        logf = open(log_path, "w")
        LAUNCH_PROC = subprocess.Popen(
            launch_args, stdout=logf, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid)
        print(f"launch started (pid {LAUNCH_PROC.pid}): "
              f"{' '.join(launch_args)}", flush=True)

    try:
        # ---------------- P1 graph health ----------------
        rep.phase("P1 图健康：关键节点 / robot_description / /tf")
        nodes_ok = wait_nodes(
            node, ["controller_manager", "robot_state_publisher", "move_group"],
            args.wait_sec)
        rep.check("controller_manager / robot_state_publisher / move_group online",
                  nodes_ok,
                  ", ".join(n for n in
                            ["controller_manager", "robot_state_publisher",
                             "move_group"]
                            if n not in node.get_node_names()) or "all up")
        if not nodes_ok and args.mode == "connect":
            print("\nconnect mode: target stack not found, aborting.",
                  flush=True)
            print(f"\n==== F106 RESULT: {rep.passed} PASS / {rep.failed} FAIL",
                  flush=True)
            sys.exit(1)

        gp_cli = node.create_client(
            GetParameters, "/robot_state_publisher/get_parameters")
        rd_ok = False
        if gp_cli.wait_for_service(timeout_sec=2.0):
            req = GetParameters.Request()
            req.names = ["robot_description"]
            fut = gp_cli.call_async(req)
            if wait_future(fut, 3.0):
                vals = fut.result().values
                rd_ok = bool(vals and vals[0].string_value)
        rep.check("robot_description parameter readable & non-empty", rd_ok)
        rep.check("/tf topic present",
                  len(node.get_publishers_info_by_topic("/tf")) > 0)

        # ---------------- P2 controllers + enable ----------------
        rep.phase("P2 控制器状态 + 产品使能 /a3/arm/enable")
        end = time.monotonic() + 30.0
        ctrls = {}
        while time.monotonic() < end:
            ctrls = list_controllers(node)
            if all(k in ctrls for k in
                   ("joint_state_broadcaster", "arm_controller",
                    "gripper_controller")):
                break
            time.sleep(0.5)
        jsb = ctrls.get("joint_state_broadcaster")
        arm = ctrls.get("arm_controller")
        grip = ctrls.get("gripper_controller")
        rep.check("joint_state_broadcaster active pre-enable",
                  jsb is not None and jsb.state == "active",
                  jsb.state if jsb else "missing")
        rep.check("arm_controller configured/inactive pre-enable",
                  arm is not None and arm.state in ("inactive", "active"),
                  arm.state if arm else "missing")
        rep.check("gripper_controller configured/inactive pre-enable",
                  grip is not None and grip.state in ("inactive", "active"),
                  grip.state if grip else "missing")

        already_enabled = arm is not None and arm.state == "active"
        if not already_enabled:
            # FSM 使能前要求收到新鲜 /joint_states（_check_positions_in_limits）；
            # FSM 在 JSB 之后才启动，首次 enable 可能撞上「no /joint_states
            # received yet」，按反馈消息有界重试。
            en = None
            end = time.monotonic() + 15.0
            while time.monotonic() < end:
                try:
                    recorder.positions(ALL_JOINTS)
                except Exception:
                    pass
                en = call_service(node, Trigger, "/a3/arm/enable",
                                  Trigger.Request(), timeout=10.0, wait=15.0)
                if en.success or "joint_states" not in en.message:
                    break
                time.sleep(1.0)
            rep.check("/a3/arm/enable success", en.success, en.message)
            WE_ENABLED = en.success
        else:
            rep.check("/a3/arm/enable (already enabled)", True)

        end = time.monotonic() + 10.0
        while time.monotonic() < end and state_rec.get() != "READY":
            time.sleep(0.1)
        rep.check("FSM state READY", state_rec.get() == "READY",
                  f"state={state_rec.get()}")

        ctrls = list_controllers(node)
        arm = ctrls.get("arm_controller")
        grip = ctrls.get("gripper_controller")
        rep.check("arm_controller active post-enable",
                  arm is not None and arm.state == "active",
                  arm.state if arm else "missing")
        rep.check("gripper_controller active post-enable",
                  grip is not None and grip.state == "active",
                  grip.state if grip else "missing")
        claimed = len(arm.claimed_interfaces) + len(grip.claimed_interfaces) \
            if arm and grip else 0
        rep.check("hardware claimed interfaces > 0", claimed > 0,
                  f"claimed={claimed}")

        # ---------------- P3 feedback ----------------
        rep.phase("P3 反馈读取：/joint_states 7 关节位置有效")
        msg = None
        for _ in range(50):
            msg = recorder.latest()
            if msg and all(j in msg.name for j in ALL_JOINTS):
                break
            time.sleep(0.1)
        missing = [j for j in ALL_JOINTS if msg is None or j not in msg.name]
        finite = all(math.isfinite(msg.position[msg.name.index(j)])
                     for j in ALL_JOINTS) if not missing else False
        detail = "missing=" + ",".join(missing) if missing else "7 joints finite"
        rep.check("/joint_states has L1-L7 with finite positions",
                  not missing and finite, detail)

        # ---------------- P4 motion ----------------
        rep.phase("P4 基础运动：FJT 小相对运动（≤0.20 rad / ≥3 s）")
        base = recorder.positions(ARM_JOINTS)
        m1 = list(base)
        m1[1] += MOVE_AMP
        m2 = list(base)
        m2[0] += 0.15
        m2[2] += 0.15
        m2[4] += 0.10
        ok1 = send_fjt(node, recorder, m1, "L2 +0.20", rep, args.mode == "mock")
        ok2 = send_fjt(node, recorder, m2,
                       "L1/L3/L5 combined ≤0.20", rep, args.mode == "mock")
        ok3 = send_fjt(node, recorder, base, "return to base", rep,
                       args.mode == "mock")

        # ---------------- P5 gripper ----------------
        rep.phase("P5 夹爪：GAC 0.0 → 1.78 → 0.0")
        okg1 = send_gripper(node, recorder, GRIPPER_CLOSE, "close", rep)
        okg2 = send_gripper(node, recorder, GRIPPER_OPEN, "open", rep)

        # ---------------- P6 controller switching ----------------
        rep.phase("P6 控制器切换：arm ↔ zero_torque（静态核验 + 可选实时切换）")
        ctrls = list_controllers(node)
        zt = ctrls.get("zero_torque_controller")
        if zt is None:
            ld = call_service(node, LoadController,
                              "/controller_manager/load_controller",
                              LoadController.Request(
                                  name="zero_torque_controller"), timeout=5.0)
            rep.check("load zero_torque_controller", ld.ok)
            cf = call_service(node, ConfigureController,
                              "/controller_manager/configure_controller",
                              ConfigureController.Request(
                                  name="zero_torque_controller"), timeout=5.0)
            rep.check("configure zero_torque_controller", cf.ok)
        else:
            rep.check("zero_torque_controller loaded by spawner",
                      zt.state in ("inactive", "active"), f"state={zt.state}")

        if not args.test_zero_torque:
            rep.skip("live STRICT arm ↔ zero_torque switch",
                     "opt-in via --test-zero-torque; real path covered by "
                     "F89b vcan 20/20")
        elif args.mode == "mock":
            rep.skip("live STRICT arm ↔ zero_torque switch",
                     "mock_components/GenericSystem rejects effort-only mode "
                     "switch (F88); run with --mode real/connect")
        else:
            sw_req = SwitchController.Request(
                activate_controllers=["zero_torque_controller"],
                deactivate_controllers=["arm_controller"],
                strictness=SwitchController.Request.STRICT)
            sw = call_service(node, SwitchController,
                              "/controller_manager/switch_controller", sw_req,
                              timeout=5.0)
            rep.check("STRICT switch arm → zero_torque", sw.ok)
            ctrls = list_controllers(node)
            zt = ctrls.get("zero_torque_controller")
            arm = ctrls.get("arm_controller")
            rep.check("zero_torque active / arm inactive",
                      zt is not None and zt.state == "active"
                      and arm is not None and arm.state == "inactive",
                      f"zt={zt.state if zt else '?'} "
                      f"arm={arm.state if arm else '?'}")

            sw_req = SwitchController.Request(
                activate_controllers=["arm_controller"],
                deactivate_controllers=["zero_torque_controller"],
                strictness=SwitchController.Request.STRICT)
            sw = call_service(node, SwitchController,
                              "/controller_manager/switch_controller", sw_req,
                              timeout=5.0)
            rep.check("STRICT switch zero_torque → arm (restore)", sw.ok)
            ctrls = list_controllers(node)
            zt = ctrls.get("zero_torque_controller")
            arm = ctrls.get("arm_controller")
            rep.check("arm active / zero_torque inactive (restored)",
                      arm is not None and arm.state == "active"
                      and zt is not None and zt.state == "inactive",
                      f"arm={arm.state if arm else '?'} "
                      f"zt={zt.state if zt else '?'}")

    except Exception as exc:
        rep.check("phase execution", False, f"exception: {exc}")

    # ---------------- teardown ----------------
    if args.mode in ("mock", "real"):
        try:
            call_service(node, Trigger, "/a3/arm/disable",
                         Trigger.Request(), timeout=8.0)
        except Exception:
            pass
        cleanup_launch(log_path)
        time.sleep(3.0)

        resid = []
        for _ in range(15):
            chk = subprocess.run(
                ["bash", "-c",
                 f"source {WS_DIR}/scripts/a3_shell_env.sh >/dev/null 2>&1 && "
                 f"ROS_DOMAIN_ID={MOCK_DOMAIN} ros2 node list"],
                capture_output=True, text=True, timeout=10.0)
            resid = [ln for ln in chk.stdout.splitlines()
                     if ln.strip()
                     and ln.strip() != "/f106_commissioning_smoke"]
            if not resid:
                break
            time.sleep(1.0)
        rep.check("no residual nodes on isolated domain after teardown",
                  not resid, ",".join(resid) if resid else "clean")
    else:
        if WE_ENABLED:
            print("\n[connect] this script enabled the arm; stack left ENABLED "
                  "(FSM READY). Disable with: ros2 service call "
                  "/a3/arm/disable std_srvs/srv/Trigger", flush=True)

    print(f"\n==== F106 RESULT: {rep.passed} PASS / {rep.failed} FAIL ====",
          flush=True)
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if rep.failed == 0 else 1)


if __name__ == "__main__":
    main()
