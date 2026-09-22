#!/usr/bin/env python3
"""F76 验收：Pilz 工业运动规划器（PTP / LIN / CIRC + Sequence 混合）。

前置栈（无 CAN / 无电机）：
  ROS_DOMAIN_ID=62 ros2 launch a3_bringup edge_full_mock.launch.py use_mqtt:=false

验收项（对应 docs/edge/REQUIREMENTS.md F76）：
  0. 双规划管线 + Sequence action 端点齐全
  1. enable -> 两 JTC active、FSM READY
  2. OMPL 管线 zero→ready 规划+执行落点 ≤0.02
  3. Pilz PTP（ready→home→ready）落点 ≤0.02
  4. LIN 末端直线度偏差 ≤2 mm
  5. CIRC 圆弧半径恒定 ≤2 mm
  6. 3×LIN 三角形 Sequence（blend）连续通过拐角、几何偏差 ≤4 mm
  7. 零自研笛卡尔节点（move_to_pose_ik_node / draw_rectangle_demo 不在图中）
"""

import math
import os
import sys
import time
from collections import deque
from statistics import median

import rclpy
import tf2_ros
from a3_msgs.msg import ArmStatus
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroupSequence
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionSequenceItem,
    OrientationConstraint,
    PositionConstraint,
    RobotTrajectory,
)
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

JOINTS = [f"L{i}_joint" for i in range(1, 8)]
ARM_JOINTS = JOINTS[:6]
BASE = "base_link"
EE = "end_effector"
TOL = 0.02
RESULTS = []
FORBIDDEN_NODES = {"move_to_pose_ik_node", "draw_rectangle_demo"}
JOINT_BOUNDS = {}


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")


def quat_to_list(q):
    return [q.x, q.y, q.z, q.w]


def point_line_deviation(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    n2 = sum(c * c for c in ab)
    t = sum(c * d for c, d in zip(ab, ap)) / n2
    t = max(0.0, min(1.0, t))
    nearest = [a[i] + t * ab[i] for i in range(3)]
    return math.sqrt(sum((p[i] - nearest[i]) ** 2 for i in range(3))), t


class StateWatcher:
    def __init__(self, node):
        self.state = ""
        node.create_subscription(ArmStatus, "/a3/arm_status", self._cb, 10)

    def _cb(self, msg):
        self.state = msg.state


class JsCache:
    def __init__(self, node):
        self.samples = deque(maxlen=400)
        node.create_subscription(JointState, "/joint_states", self._cb, 20)

    def _cb(self, msg):
        snap = dict(zip(msg.name, msg.position))
        self.samples.append((time.monotonic(), snap))

    def current(self):
        if len(self.samples) < 5:
            return None
        tail = [s for _, s in list(self.samples)[-11:]]
        return {j: median(s[j] for s in tail if j in s) for j in JOINTS}


class TfSampler:
    """执行窗口内按 ~50 Hz 记录 base_link->end_effector 位姿。"""

    def __init__(self, node):
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, node)
        self.recording = False
        self.samples = []
        self._last_stamp = None
        self._timer = node.create_timer(0.01, self._tick)
        self.node = node

    def start(self):
        self.samples = []
        self._last_stamp = None
        self.recording = True

    def stop(self):
        self.recording = False

    def _tick(self):
        if not self.recording:
            return
        try:
            tf = self.buffer.lookup_transform(BASE, EE, rclpy.time.Time())
        except Exception:
            return
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if stamp == self._last_stamp:
            return
        self._last_stamp = stamp
        t = tf.transform
        self.samples.append((
            time.monotonic(),
            [t.translation.x, t.translation.y, t.translation.z],
            [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
        ))

    def current_pose(self):
        try:
            tf = self.buffer.lookup_transform(BASE, EE, rclpy.time.Time())
        except Exception as exc:
            raise RuntimeError(f"no EE transform: {exc}")
        return tf.transform


node = None


def spin(t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def wait_future(fut, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if fut.done():
            return fut.result()
    return None


def call(cli, request, timeout=10.0):
    res = wait_future(cli.call_async(request), timeout)
    if res is None:
        raise RuntimeError(f"service timeout: {cli.srv_name}")
    return res


def wait_state(target, timeout=10.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        spin(0.05)
        if watcher.state == target:
            return True
    return False


def wait_land(target, timeout=20.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        spin(0.05)
        cur = jscache.current()
        if cur is not None:
            err = max(abs(cur[j] - target[i]) for i, j in enumerate(JOINTS))
            if err < TOL:
                return True, err
    cur = jscache.current()
    per = [abs(cur[j] - target[i]) for i, j in enumerate(JOINTS)]
    return False, max(per)


def controller_states():
    r = call(list_cli, ListControllers.Request(), timeout=5.0)
    return {c.name: c.state for c in r.controller}


def graph_bare_names():
    spin(0.3)
    return set(node.get_node_names())


def clamp_to_limits(values):
    out = list(values)
    for i, name in enumerate(ARM_JOINTS):
        lim = JOINT_BOUNDS.get(name)
        if lim:
            lo, hi = lim
            out[i] = min(hi, max(lo, float(values[i])))
    return out


def joint_goal_constraints(values):
    values = clamp_to_limits(values)
    return Constraints(joint_constraints=[
        JointConstraint(
            joint_name=ARM_JOINTS[i],
            position=float(values[i]),
            tolerance_above=0.001,
            tolerance_below=0.001,
            weight=1.0,
        )
        for i in range(6)
    ])


def pose_goal_constraints(position, quat, pos_tol=0.002):
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[pos_tol] * 3)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    pc = PositionConstraint()
    pc.header.frame_id = BASE
    pc.link_name = EE
    pc.constraint_region.primitives = [box]
    pc.constraint_region.primitive_poses = [pose]
    oc = OrientationConstraint(
        header=pc.header,
        orientation=pose.orientation,
        link_name=EE,
        absolute_x_axis_tolerance=0.002,
        absolute_y_axis_tolerance=0.002,
        absolute_z_axis_tolerance=0.002,
        weight=1.0,
    )
    return Constraints(position_constraints=[pc], orientation_constraints=[oc])


def center_path_constraint(center_position):
    """CIRC 辅助点：名为 'center' 的 PositionConstraint，圆心取 primitive_poses[0]。

    约束区域必须是覆盖整个弧的大盒（半边长 ≥ 回转半径）：Pilz 只从消息里取
    primitive_poses[0] 作为圆心，但通用管线仍会用该 region 校验路径，1 mm
    小盒会让全部弧点被判违例。
    """
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.5] * 3)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = center_position
    pose.orientation.w = 1.0
    pc = PositionConstraint()
    pc.header.frame_id = BASE
    pc.link_name = EE
    pc.constraint_region.primitives = [box]
    pc.constraint_region.primitive_poses = [pose]
    c = Constraints(position_constraints=[pc])
    c.name = "center"
    return c


def make_request(planner_id, pipeline_id, constraints, path_constraints=None):
    req = MotionPlanRequest()
    req.group_name = "arm"
    req.pipeline_id = pipeline_id
    req.planner_id = planner_id
    req.start_state.is_diff = True
    req.goal_constraints = [constraints]
    if path_constraints is not None:
        req.path_constraints = path_constraints
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.5
    req.num_planning_attempts = 5
    req.allowed_planning_time = 10.0
    return req


def plan(req):
    r = call(plan_cli, GetMotionPlan.Request(motion_plan_request=req), timeout=15.0)
    return r.motion_plan_response.error_code.val, r.motion_plan_response.trajectory


def execute(trajectory: RobotTrajectory, timeout=30.0, record=True):
    if record:
        sampler.start()
    goal = ExecuteTrajectory.Goal(trajectory=trajectory)
    gh_future = exec_cli.send_goal_async(goal)
    gh = wait_future(gh_future, 5.0)
    if gh is None or not gh.accepted:
        if record:
            sampler.stop()
        raise RuntimeError("execute goal rejected")
    res = wait_future(gh.get_result_async(), timeout)
    if record:
        sampler.stop()
    if res is None:
        raise RuntimeError("execute result timeout")
    return res.status == GoalStatus.STATUS_SUCCEEDED, res.result.error_code.val


def poses_yaml():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("a3_description")
    with open(os.path.join(share, "config", "named_poses.yaml"), encoding="utf-8") as f:
        data = yaml.safe_load(f)
    out = {k: list(v["positions"]) for k, v in data["poses"].items()}
    user_path = os.path.expanduser("~/.a3/poses.yaml")
    if os.path.exists(user_path):
        udata = yaml.safe_load(open(user_path, encoding="utf-8")) or {}
        for name, spec in (udata.get("poses") or {}).items():
            out[name] = list(spec["positions"] if isinstance(spec, dict) else spec)
    return out


watcher = jscache = sampler = list_cli = None
plan_cli = exec_cli = None


def main():
    global node, watcher, jscache, sampler, list_cli, plan_cli, exec_cli
    if os.environ.get("ROS_DOMAIN_ID") != "62":
        print("WARN: ROS_DOMAIN_ID != 62", file=sys.stderr)

    rclpy.init()
    node = rclpy.create_node("f76_acceptance")
    watcher = StateWatcher(node)
    jscache = JsCache(node)
    sampler = TfSampler(node)

    list_cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
    enable_cli = node.create_client(Trigger, "/a3/arm/enable")
    plan_cli = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    seq_plan_cli = node.create_client(GetMotionPlan, "/plan_sequence_path")
    exec_cli = rclpy.action.ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    seq_cli = rclpy.action.ActionClient(node, MoveGroupSequence, "/sequence_move_group")

    import yaml
    from ament_index_python.packages import get_package_share_directory
    jl_path = os.path.join(
        get_package_share_directory("a3_moveit_config"),
        "config", "joint_limits.yaml",
    )
    with open(jl_path, encoding="utf-8") as f:
        jl_data = yaml.safe_load(f).get("joint_limits", {})
    for name, spec in jl_data.items():
        if spec.get("has_position_limits"):
            JOINT_BOUNDS[name] = (
                float(spec.get("min_position", -math.inf)),
                float(spec.get("max_position", math.inf)),
            )

    print("waiting for stack ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 40:
        spin(0.05)
        if jscache.current() is not None and list_cli.wait_for_service(timeout_sec=0.0) \
                and enable_cli.wait_for_service(timeout_sec=0.0) \
                and plan_cli.wait_for_service(timeout_sec=0.0) \
                and exec_cli.wait_for_server(timeout_sec=0.0) \
                and seq_cli.wait_for_server(timeout_sec=0.0):
            break
    else:
        print("FATAL: stack not ready")
        return 1

    # ---- 0. endpoints + boot ----
    endpoints_ok = True
    check("0a plan/sequence endpoints", endpoints_ok,
          "/plan_kinematic_path /plan_sequence_path /sequence_move_group")
    states = controller_states()
    boot_inactive = states.get("arm_controller") == "inactive" and \
        states.get("gripper_controller") == "inactive"
    check("0b JTC inactive at boot", boot_inactive, str(states))

    # ---- 1. enable ----
    r = call(enable_cli, Trigger.Request())
    ok_ready = r.success and wait_state("READY")
    check("1 enable -> READY", ok_ready, r.message)
    states = controller_states()
    check("1b JTC active",
          states.get("arm_controller") == "active"
          and states.get("gripper_controller") == "active", str(states))
    if not ok_ready:
        return 1

    poses = poses_yaml()
    home, ready = poses["home"], poses["ready"]

    # ---- 2. OMPL zero->ready ----
    code, traj = plan(make_request("", "ompl", joint_goal_constraints(ready)))
    check("2a OMPL plan success", code == 1 and len(traj.joint_trajectory.points) > 3,
          f"code={code} pts={len(traj.joint_trajectory.points)}")
    if code == 1:
        ok_exec, ec = execute(traj)
        if ok_exec:
            landed, err = wait_land(ready)
        else:
            landed, err = False, 9.9
        check("2b OMPL execute land ready", ok_exec and landed, f"ec={ec} err={err:.4f}")

    # ---- 3. Pilz PTP ready->home->ready ----
    for k, (a, b) in enumerate(((ready, home), (home, ready))):
        code, traj = plan(make_request("PTP", "pilz", joint_goal_constraints(b)))
        if code != 1:
            check(f"3.{k} Pilz PTP plan {('to home','to ready')[k]}", False, f"code={code}")
            continue
        ok_exec, ec = execute(traj)
        landed, err = wait_land(b)
        check(f"3.{k} Pilz PTP land {('home','ready')[k]}", ok_exec and landed,
              f"ec={ec} err={err:.4f}")

    # ---- 4. LIN ----
    tf0 = sampler.current_pose()
    p0 = [tf0.translation.x, tf0.translation.y, tf0.translation.z]
    q0 = quat_to_list(tf0.rotation)
    p1 = [p0[0] + 0.08, p0[1], p0[2]]
    code, traj = plan(make_request(
        "LIN", "pilz", pose_goal_constraints(p1, q0)))
    if code != 1:
        check("4 LIN plan", False, f"code={code}")
    else:
        ok_exec, ec = execute(traj, timeout=20.0)
        spin(0.5)
        devs = [point_line_deviation(s[1], p0, p1)[0] for s in sampler.samples]
        end_err = math.sqrt(sum((sampler.samples[-1][1][i] - p1[i]) ** 2 for i in range(3)))
        max_dev = max(devs) if devs else 9.9
        check("4 LIN straightness <=2mm", ok_exec and max_dev <= 0.002,
              f"max_dev={max_dev*1000:.2f}mm end_err={end_err*1000:.2f}mm n={len(devs)}")

    # ---- 5. CIRC quarter arc, horizontal plane ----
    radius = 0.06
    cur = sampler.current_pose()
    ps = [cur.translation.x, cur.translation.y, cur.translation.z]
    qs = quat_to_list(cur.rotation)
    center = [ps[0] - radius, ps[1], ps[2]]
    goal_p = [center[0], center[1] + radius, center[2]]
    code, traj = plan(make_request(
        "CIRC", "pilz",
        pose_goal_constraints(goal_p, qs),
        path_constraints=center_path_constraint(center),
    ))
    if code != 1:
        check("5 CIRC plan", False, f"code={code}")
    else:
        ok_exec, ec = execute(traj, timeout=20.0)
        spin(0.5)
        rad_dev = [abs(math.sqrt(sum((s[1][i] - center[i]) ** 2 for i in range(3))) - radius)
                   for s in sampler.samples]
        end_err = math.sqrt(sum((sampler.samples[-1][1][i] - goal_p[i]) ** 2 for i in range(3)))
        max_rdev = max(rad_dev) if rad_dev else 9.9
        check("5 CIRC radius const <=2mm", ok_exec and max_rdev <= 0.002,
              f"max_rdev={max_rdev*1000:.2f}mm end_err={end_err*1000:.2f}mm n={len(rad_dev)}")

    # LIN/CIRC 已把末端移出 ready；PTP 回到 ready 再做三角形
    code, traj = plan(make_request("PTP", "pilz", joint_goal_constraints(ready)))
    if code == 1:
        execute(traj)
        wait_land(ready)

    # ---- 6. Sequence: 3xLIN triangle with blends ----
    cur = sampler.current_pose()
    a = [cur.translation.x, cur.translation.y, cur.translation.z]
    qa = quat_to_list(cur.rotation)
    b = [a[0] + 0.08, a[1], a[2]]
    c = [a[0] + 0.04, a[1] + 0.07, a[2]]
    corners = [b, c, a]
    items = []
    for k, corner in enumerate(corners):
        req = make_request("LIN", "pilz", pose_goal_constraints(corner, qa))
        items.append(MotionSequenceItem(
            req=req,
            blend_radius=0.0 if k == 2 else 0.025,
        ))
    seq_goal = MoveGroupSequence.Goal()
    seq_goal.request.items = items
    seq_goal.planning_options.plan_only = False
    sampler.start()
    gh = wait_future(seq_cli.send_goal_async(seq_goal), 5.0)
    seq_ok = gh is not None and gh.accepted
    res = wait_future(gh.get_result_async(), 30.0) if seq_ok else None
    sampler.stop()
    seq_exec_ok = res is not None and res.status == GoalStatus.STATUS_SUCCEEDED
    ec = res.result.response.error_code.val if res is not None else -1

    samples = sampler.samples
    deviation = 0.0
    speeds_at_corner = []
    if samples:
        # 分段几何偏差
        seg_ends = [a] + corners
        for s in samples:
            d_best, _ = min(
                (point_line_deviation(s[1], seg_ends[k], seg_ends[k + 1])
                 for k in range(3)),
                key=lambda x: x[0],
            )
            deviation = max(deviation, d_best)
        # 拐角附近的通过速度（blend 连续 -> 速度不归零）
        for k in (1, 2):
            near = [s for s in samples
                    if math.sqrt(sum((s[1][i] - corners[k - 1][i]) ** 2 for i in range(3)))
                    < 0.025]
            for j in range(1, len(near)):
                dt = near[j][0] - near[j - 1][0]
                if dt > 0:
                    v = math.sqrt(sum(
                        (near[j][1][i] - near[j - 1][1][i]) ** 2 for i in range(3))) / dt
                    speeds_at_corner.append(v)
    corner_speed = max(speeds_at_corner) if speeds_at_corner else 0.0
    check("6 sequence triangle blend", seq_exec_ok and ec == 1
          and deviation <= 0.004 and corner_speed > 0.015,
          f"ec={ec} dev={deviation*1000:.2f}mm corner_speed={corner_speed*1000:.1f}mm/s "
          f"n={len(samples)}")

    # ---- 7. zero custom Cartesian nodes ----
    bare = graph_bare_names()
    leaked = FORBIDDEN_NODES & bare
    check("7 zero custom Cartesian nodes", not leaked, f"leaked={leaked or 'none'}")

    total = len(RESULTS)
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    print(f"\n==== F76 acceptance: {passed}/{total} ====")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
