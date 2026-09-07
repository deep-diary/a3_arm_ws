#!/usr/bin/env python3
"""夹爪（L7）力控回归测试（F28）。

两种模式：
  --sim  （默认，无硬件可跑）：本脚本自起 gripper_controller 节点（关闭固件硬限写入），
         在测试进程内跑一个假「电机+物体」植物：订阅 L7 轨迹得到 q_cmd，
         一阶位置跟随模拟电机内环，接触后按物体刚度反算握力，发布 /joint_states，
         形成闭环，验证：
           (a) 配置越界拒绝 / 合法接受
           (b) POSITION 开合
           (c) FORCE 对软/硬物体均收敛到目标 ±10% 且 GRASPED
           (d) 看门狗：停止反馈 -> FAULT
           (e) 超硬限：反馈超力矩 -> FAULT
  真机模式（不加 --sim，需 gripper_controller + motor_protocol 在 can1/ID7 运行）：
         仅做服务存在性、配置下发、POSITION/RELEASE 等安全检查；力控阶跃需人工放置
         海绵/硬阻挡后观察，不在脚本内自动断言真机力矩。

真机安全：本脚本不直接发 CAN，所有运动经 gripper 节点；真机力控目标默认取小值。
"""

import math
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Reporter, call_service  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

from a3_msgs.srv import GripperCommand, GripperSetConfig  # noqa: E402

ALL_JOINTS = [
    "L1_joint", "L2_joint", "L3_joint", "L4_joint",
    "L5_joint", "L6_joint", "L7_joint",
]
L7_IDX = 6
# 2026-09-06 标定：全开位设零、闭合为正（与 gripper_config.yaml / 节点 _DEFAULTS 对齐）
Q_OPEN = 0.0
Q_CLOSE = 1.79


class FakeGripperPlant(Node):
    """假电机 + 物体：订阅 L7 轨迹，模拟位置跟随与接触握力，发布 /joint_states。"""

    def __init__(self, contact_q=0.6, contact_k=20.0):
        super().__init__("a3_fake_gripper_plant")
        self.q = Q_OPEN            # 实际 L7 角
        self.q_cmd = Q_OPEN        # 最近指令
        self.contact_q = contact_q  # 物体表面位置（夹爪闭合到此处接触）
        self.contact_k = contact_k  # 物体刚度：软物小、硬物大
        self.tau = 0.0
        self.publish_feedback = True
        self.force_tau_override = None  # 置非 None 时强制反馈该力矩（测超硬限）
        self.create_subscription(JointTrajectory,
                                 "/joint_group_effort_controller/joint_trajectory",
                                 self._on_traj, 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.02, self._tick)  # 50 Hz

    def _on_traj(self, msg):
        if "L7_joint" in msg.joint_names and msg.points:
            i = msg.joint_names.index("L7_joint")
            if i < len(msg.points[-1].positions):
                self.q_cmd = float(msg.points[-1].positions[i])

    def _tick(self):
        # 电机位置内环：一阶快速跟随
        self.q += (self.q_cmd - self.q) * 0.35
        self.q = max(Q_OPEN, min(Q_CLOSE, self.q))
        # 接触握力：闭合为正方向，q 越过物体表面后，穿透量 × 刚度
        penetration = max(0.0, self.q - self.contact_q)
        self.tau = self.contact_k * penetration
        if not math.isfinite(self.tau):
            self.tau = 0.0

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(ALL_JOINTS)
        js.position = [0.0] * 7
        js.position[L7_IDX] = self.q
        js.velocity = [0.0] * 7
        js.effort = [0.0] * 7
        if self.force_tau_override is not None:
            js.effort[L7_IDX] = float(self.force_tau_override)
        else:
            js.effort[L7_IDX] = self.tau
        if self.publish_feedback:
            self.js_pub.publish(js)


class GripperStatusReader(Node):
    def __init__(self):
        super().__init__("a3_gripper_status_reader")
        self.state = ""
        self.contact = False
        self.actual = 0.0
        self.target = 0.0
        self.error_code = 0
        from a3_msgs.msg import GripperStatus
        self.create_subscription(GripperStatus, "/a3/gripper_status", self._cb, 10)

    def _cb(self, msg):
        self.state = msg.state
        self.contact = msg.contact
        self.actual = msg.actual_torque_nm
        self.target = msg.target_torque_nm
        self.error_code = msg.error_code


def wait_sec(node, sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def run_sim(rep):
    """无硬件闭环：起节点 + 假植物，跑全断言。"""
    env = dict(os.environ)
    # 独立域，避免与真机/其它图互扰；测试进程与被测节点必须同域
    domain = env.get("A3_GRIPPER_TEST_DOMAIN", "77")
    env["ROS_DOMAIN_ID"] = domain
    os.environ["ROS_DOMAIN_ID"] = domain
    overrides_dir = "/tmp/a3_gripper_test_overrides"
    os.makedirs(overrides_dir, exist_ok=True)
    cmd = [
        "ros2", "run", "a3_gripper_controller", "gripper_controller",
        "--ros-args",
        "-p", "apply_firmware_torque_limit:=false",
        "-p", "require_gate:=false",
        "-p", "overrides_dir:=" + overrides_dir,
        "-p", "grasp_timeout_s:=10.0",
        "-p", "feedback_fresh_timeout_s:=0.3",
    ]
    rep.info("启动 gripper_controller（sim，ROS_DOMAIN_ID=%s）" % env["ROS_DOMAIN_ID"])
    node_log = open("/tmp/a3_gripper_node.log", "w")
    node_proc = subprocess.Popen(
        cmd, env=env, stdout=node_log, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    try:
        rclpy.init()
        plant = FakeGripperPlant(contact_q=0.6, contact_k=20.0)
        reader = GripperStatusReader()
        cli_node = Node("a3_gripper_test_client")
        # 单一 MultiThreadedExecutor 管理全部本地节点（植物 50 Hz 定时器、状态订阅、
        # 服务客户端回调）；不要再对其中任一 node 单独 spin_once（会与 executor 冲突）。
        import threading
        from rclpy.executors import MultiThreadedExecutor
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(plant)
        ex.add_node(reader)
        ex.add_node(cli_node)
        threading.Thread(target=ex.spin, daemon=True).start()
        time.sleep(2.0)  # 等节点起来

        def _call(srv_type, name, req, timeout=5.0):
            # cli_node 由后台 executor 维护，这里只发请求并轮询 future，不另 spin
            cli = cli_node.create_client(srv_type, name)
            if not cli.wait_for_service(timeout_sec=timeout):
                cli_node.destroy_client(cli)
                return None
            fut = cli.call_async(req)
            end = time.time() + timeout
            while time.time() < end and not fut.done():
                time.sleep(0.05)
            resp = fut.result() if fut.done() else None
            cli_node.destroy_client(cli)
            return resp

        def cmd(mode, **kw):
            req = GripperCommand.Request()
            req.mode = mode
            req.position = float(kw.get("position", 0.0))
            req.torque_nm = float(kw.get("torque", 0.0))
            req.preset = str(kw.get("preset", ""))
            req.timeout_s = float(kw.get("timeout", 0.0))
            return _call(GripperCommand, "/a3/gripper/command", req)

        def cfg(key, value):
            req = GripperSetConfig.Request()
            req.key = key
            req.value = float(value)
            return _call(GripperSetConfig, "/a3/gripper/set_config", req)

        # (a) 配置校验
        r = cfg("max_torque_nm", 99.0)
        rep.check("配置越界(99Nm)被拒", bool(r and not r.success),
                  getattr(r, "message", "无响应"))
        r = cfg("max_torque_nm", -1.0)
        rep.check("配置非法(负值)被拒", bool(r and not r.success),
                  getattr(r, "message", "无响应"))
        r = cfg("max_torque_nm", 1.0)
        rep.check("配置合法(1.0Nm)接受并落盘", bool(r and r.success),
                  getattr(r, "message", "无响应"))
        r = cfg("max_torque_nm", 1.5)
        rep.check("配置超硬上限(1.5Nm)被拒", bool(r and not r.success),
                  getattr(r, "message", "无响应"))

        # (b) POSITION 开合
        r = cmd("position", position=0.0)
        rep.check("POSITION 闭合命令成功", bool(r and r.success),
                  getattr(r, "message", "无响应"))
        time.sleep(1.5)
        r = cmd("position", position=1.0)
        rep.check("POSITION 张开命令成功", bool(r and r.success),
                  getattr(r, "message", "无响应"))
        time.sleep(1.5)

        # (c) FORCE 软物体（小刚度）收敛
        plant.contact_k = 8.0
        r = cmd("force", torque=0.4)
        rep.check("FORCE 软物抓取命令成功", bool(r and r.success),
                  getattr(r, "message", "无响应"))
        _wait_grasped(reader, rep, target=0.4, label="软物(kc=8)")

        # 切换目标到 0.8 Nm（软物），应继续收敛
        r = cmd("force", torque=0.8)
        time.sleep(0.2)
        _wait_grasped(reader, rep, target=0.8, label="软物(kc=8) 0.8Nm")

        # (c2) 硬物体（大刚度）小目标
        cmd("release")
        time.sleep(2.0)
        plant.contact_k = 60.0
        r = cmd("force", torque=0.5)
        rep.check("FORCE 硬物抓取命令成功", bool(r and r.success),
                  getattr(r, "message", "无响应"))
        _wait_grasped(reader, rep, target=0.5, label="硬物(kc=60)")

        # (e) 超硬限：强制反馈一个超限力矩
        cmd("release")
        time.sleep(1.5)
        r = cmd("force", torque=0.5)
        time.sleep(0.5)
        plant.force_tau_override = 5.0  # 远超 FAULT 阈值 1.5Nm（=1.0×overtorque_ratio）
        time.sleep(1.0)  # 等后台 executor 处理故障状态
        rep.check("超硬限 -> FAULT(error_code=4)",
                  reader.error_code == 4,
                  "state=%s err=%s" % (reader.state, reader.error_code))
        plant.force_tau_override = None

        # (d) 看门狗：停止反馈
        cmd("release")
        time.sleep(1.5)
        r = cmd("force", torque=0.4)
        time.sleep(0.5)
        plant.publish_feedback = False
        time.sleep(1.2)  # 超过 feedback_fresh_timeout 0.3s
        rep.check("反馈看门狗 -> FAULT(error_code=3)",
                  reader.error_code == 3,
                  "state=%s err=%s" % (reader.state, reader.error_code))
        plant.publish_feedback = True

    finally:
        try:
            os.killpg(os.getpgid(node_proc.pid), signal.SIGINT)
            time.sleep(1.0)
            os.killpg(os.getpgid(node_proc.pid), signal.SIGKILL)
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


def _wait_grasped(reader, rep, target, label, timeout=10.0):
    # reader 由后台 MultiThreadedExecutor spin，这里只等待并读取其字段
    end = time.time() + timeout
    grasped = False
    converged = False
    while time.time() < end:
        time.sleep(0.1)
        if reader.state == "GRASPED":
            grasped = True
        if grasped and abs(reader.actual - target) <= 0.10 * target + 0.02:
            converged = True
            break
    rep.check(
        "%s 力控收敛 ±10%% 且 GRASPED" % label,
        grasped and converged,
        "state=%s actual=%.3f target=%.3f" % (reader.state, reader.actual, target),
    )


def run_hw(rep):
    """真机模式：服务与安全检查（力控阶跃需人工放置负载观察）。"""
    rclpy.init()
    node = Node("a3_gripper_hw_test")
    rep.info("真机模式：仅做服务/配置/开合安全检查")

    r = call_service(node, GripperSetConfig, "/a3/gripper/set_config",
                     _cfg("max_torque_nm", 99.0), timeout=5.0)
    rep.check("越界最大握力被拒", bool(r and not r.success), getattr(r, "message", "无服务"))

    r = call_service(node, GripperCommand, "/a3/gripper/command",
                     _cmd("release"), timeout=8.0)
    rep.check("release 服务可用", bool(r and r.success), getattr(r, "message", "无服务"))

    r = call_service(node, GripperCommand, "/a3/gripper/command",
                     _cmd("position", position=1.0), timeout=8.0)
    rep.check("POSITION 张开服务可用", bool(r and r.success), getattr(r, "message", "无服务"))

    rep.warn("真机力控阶跃（弱/中/强 × 海绵/硬阻挡）需人工放置负载后观察 grip_actual_torque")
    node.destroy_node()
    rclpy.shutdown()


def _cmd(mode, position=0.0, torque=0.0, preset="", timeout=0.0):
    req = GripperCommand.Request()
    req.mode = mode
    req.position = float(position)
    req.torque_nm = float(torque)
    req.preset = str(preset)
    req.timeout_s = float(timeout)
    return req


def _cfg(key, value):
    req = GripperSetConfig.Request()
    req.key = key
    req.value = float(value)
    return req


def main():
    sim = "--sim" in sys.argv or os.environ.get("A3_GRIPPER_TEST_MODE", "sim") == "sim"
    rep = Reporter("F28 夹爪力控回归 (%s)" % ("SIM 闭环" if sim else "真机服务检查"))
    if sim:
        run_sim(rep)
    else:
        run_hw(rep)
    ok = rep.summary()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
