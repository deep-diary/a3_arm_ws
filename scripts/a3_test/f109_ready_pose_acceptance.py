#!/usr/bin/env python3
"""F109 — ready 点位重定义（折叠竖直、重心过底座中心）+ R2 夹爪链路 验收。

一条命令自动起栈（mock，产品入口 a3_bringup.launch.py hardware:=mock），
按五阶段验证并输出 PASS/FAIL 报告与退出码，结束自动清栈：

  P1 使能                P2 goto ready 规划/到位
  P3 独立 RNEA CoM/力矩  P4 GripperCommand 开合（R2 链路）
  P5 goto home 回归

隔离域 ROS_DOMAIN_ID=109。
"""
import os
import signal
import subprocess
import sys
import threading
import time

WS_DIR = "/home/cat/a3_arm_ws"
URDF = f"{WS_DIR}/src/a3_description/urdf/el_a3.urdf"
ARM_JOINTS = ["L1_joint", "L2_joint", "L3_joint",
              "L4_joint", "L5_joint", "L6_joint"]
ALL_JOINTS = ARM_JOINTS + ["L7_joint"]
DOMAIN = "109"

LAUNCH_PROC = None


def load_effective_poses():
    """Replicate FSM merge: package named_poses + ~/.a3/poses.yaml override."""
    import yaml
    with open(f"{WS_DIR}/src/a3_description/config/named_poses.yaml") as f:
        pkg = yaml.safe_load(f)
    poses = {}
    for name, body in (pkg.get("poses") or {}).items():
        poses[name] = list(body["positions"])
    user_path = os.path.expanduser("~/.a3/poses.yaml")
    if os.path.exists(user_path):
        with open(user_path) as f:
            user = yaml.safe_load(f)
        for name, vals in (user.get("poses") or {}).items():
            poses[name] = list(vals)
    return poses


POSES = load_effective_poses()
READY = POSES["ready"][:6]
HOME = POSES["home"][:6]


def ensure_env() -> None:
    """Re-exec under a3_shell_env.sh so rclpy/ament packages are importable."""
    if os.environ.get("A3_F109_ENVED") == "1":
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
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["A3_F109_ENVED"] = "1"
    os.execvpe(sys.executable,
               [sys.executable, os.path.abspath(__file__), *sys.argv[1:]], env)


class JointRecorder:
    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self.msg = None
        from sensor_msgs.msg import JointState
        node.create_subscription(JointState, "/joint_states", self._cb, 20)

    def _cb(self, msg):
        with self._lock:
            self.msg = msg

    def positions(self, joints):
        for _ in range(50):
            msg = self.msg
            if msg and all(j in msg.name for j in joints):
                with self._lock:
                    return [msg.position[msg.name.index(j)] for j in joints]
            time.sleep(0.1)
        raise RuntimeError(f"joint_states missing among {joints}")


class StateRecorder:
    def __init__(self, node):
        self._lock = threading.Lock()
        self.state = ""
        from a3_msgs.msg import ArmStatus
        node.create_subscription(ArmStatus, "/a3/arm_status", self._cb, 10)

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


def wait_state(state_rec, target, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if state_rec.get() == target:
            return True
        time.sleep(0.1)
    return False


def goto_pose(node, state_rec, name):
    wait_state(state_rec, "READY", 10.0)
    from a3_msgs.srv import GotoNamedPose
    req = GotoNamedPose.Request(pose_name=name)
    return call_service(node, GotoNamedPose, "/a3/arm/goto_named_pose", req,
                        timeout=40.0, wait=30.0)


def wait_joints(rec, target, joints, tol, timeout, label=""):
    end = time.monotonic() + timeout
    last = None
    while time.monotonic() < end:
        last = rec.positions(joints)
        if all(abs(a - b) <= tol for a, b in zip(last, target)):
            return True, last
        time.sleep(0.2)
    return False, last


def rnea_metrics(q6):
    import numpy as np
    import pinocchio as pin
    model = pin.buildModelFromUrdf(URDF)
    jid = {n: model.getJointId(n) for n in ALL_JOINTS}
    q = pin.neutral(model)
    for n, v in zip(ARM_JOINTS, q6):
        q[model.joints[jid[n]].idx_q] = v
    d = model.createData()
    tau = pin.rnea(model, d, q, np.zeros(model.nv), np.zeros(model.nv))
    pin.forwardKinematics(model, d, q)
    m_total, c = 0.0, np.zeros(3)
    for i in range(1, model.njoints):
        inert = model.inertias[i]
        if inert.mass > 0:
            m_total += inert.mass
            c += inert.mass * d.oMi[i].act(inert.lever)
    com = c / m_total
    t = [tau[model.joints[jid[n]].idx_v] for n in ARM_JOINTS]
    return com, float(np.sum(np.abs(t)))


def gripper_action(node, rec, rep, position):
    from control_msgs.action import GripperCommand
    ac = node.action_clients.get("gripper")
    if ac is None:
        from rclpy.action import ActionClient
        ac = ActionClient(node, GripperCommand,
                          "/gripper_controller/gripper_cmd")
        node.action_clients["gripper"] = ac
    if not ac.wait_for_server(timeout_sec=15.0):
        rep.check(f"gripper action server reachable (target {position:.2f})",
                  False, "no server")
        return
    goal = GripperCommand.Goal()
    goal.command.position = float(position)
    goal.command.max_effort = 0.0
    gh_fut = ac.send_goal_async(goal)
    ok = wait_future(gh_fut, 5.0) and gh_fut.result().accepted
    rep.check(f"gripper goal accepted (position={position:.2f})", ok)
    if not ok:
        return
    res_fut = gh_fut.result().get_result_async()
    done = wait_future(res_fut, 10.0)
    if done:
        rep.check(f"gripper action result (position={position:.2f})",
                  res_fut.result().result.reached_goal,
                  f"effort={res_fut.result().result.effort:.2f}")
    else:
        rep.check(f"gripper action result (position={position:.2f})", False,
                  "result timeout")
    settled, last = wait_joints(rec, [position], ["L7_joint"], 0.05, 8.0)
    rep.check(f"L7 reaches {position:.2f} rad", settled,
              f"actual={last[0]:.3f}" if last else "no feedback")


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
    global LAUNCH_PROC
    ensure_env()

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node("f109_ready_pose_acceptance")
    node.srv_clients = {}
    node.action_clients = {}
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    rec = JointRecorder(node)
    state_rec = StateRecorder(node)
    rep = Report()
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_path = f"/tmp/f109_launch_{ts}.log"

    def signal_handler(signum, frame):
        print(f"\nsignal {signum} received, cleaning up...", flush=True)
        cleanup_launch(log_path)
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    launch_args = ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
                   "hardware:=mock", "use_rviz:=false", "use_mqtt:=false",
                   "use_teleop:=false"]
    logf = open(log_path, "w")
    LAUNCH_PROC = subprocess.Popen(
        launch_args, stdout=logf, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)
    print(f"launch started (pid {LAUNCH_PROC.pid}): {' '.join(launch_args)}",
          flush=True)

    try:
        # ---------------- P1 enable ----------------
        rep.phase("P1 使能：mock 栈 enable → FSM READY")
        end = time.monotonic() + 60.0
        en = None
        while time.monotonic() < end:
            try:
                en = call_service(node, Trigger, "/a3/arm/enable",
                                  Trigger.Request(), timeout=10.0, wait=20.0)
                if en.success:
                    break
            except Exception:
                pass
            time.sleep(1.0)
        rep.check("/a3/arm/enable success at zero pose",
                  en is not None and en.success,
                  en.message if en else "enable unreachable")
        end = time.monotonic() + 15.0
        ready_state = False
        while time.monotonic() < end:
            if state_rec.get() == "READY":
                ready_state = True
                break
            time.sleep(0.1)
        rep.check("FSM state READY", ready_state, f"state={state_rec.get()}")

        # ---------------- P2 goto ready ----------------
        rep.phase("P2 goto ready：MoveIt 规划 + 执行 SUCCESSFUL")
        r = goto_pose(node, state_rec, "ready")
        rep.check("goto_named_pose(ready) success", r.success, r.message)
        settled, last = wait_joints(rec, READY, ARM_JOINTS, 0.03, 25.0)
        rep.check("joints settle at [0,1.05,-1.575,0,0,0] ±0.03",
                  settled,
                  "actual=" + ",".join(f"{v:.3f}" for v in (last or [])))

        # ---------------- P3 independent RNEA ----------------
        rep.phase("P3 独立 pinocchio RNEA：CoM 过底座中心 + 静态力矩")
        try:
            com, tsum = rnea_metrics(READY)
            rep.check("|CoM_x| <= 10 mm", abs(com[0]) <= 0.010,
                      f"CoM_x={com[0]*1000:+.1f} mm")
            rep.check("sum|tau| <= 2.2 Nm", tsum <= 2.2,
                      f"sum|tau|={tsum:.2f} Nm")
        except Exception as exc:
            rep.check("RNEA evaluation", False, str(exc))

        # ---------------- P4 gripper action (R2 link) ----------------
        rep.phase("P4 R2 链路：GripperCommand 闭合/打开 L7")
        gripper_action(node, rec, rep, 1.79)
        gripper_action(node, rec, rep, 0.0)

        # ---------------- P5 home regression ----------------
        rep.phase("P5 回归：goto home 仍正常")
        r = goto_pose(node, state_rec, "home")
        rep.check("goto_named_pose(home) success", r.success, r.message)
        settled, last = wait_joints(rec, HOME, ARM_JOINTS, 0.03, 25.0)
        rep.check("joints settle at home ±0.03", settled,
                  "actual=" + ",".join(f"{v:.3f}" for v in (last or [])))

    finally:
        cleanup_launch(log_path)

    print(f"\n{'='*50}")
    print(f"TOTAL: {rep.passed} passed, {rep.failed} failed")
    if rep.failed:
        print("RESULT: FAIL")
        sys.exit(1)
    print("RESULT: PASS")
    sys.exit(0)


if __name__ == "__main__":
    main()
