#!/usr/bin/env python3
"""F51/LL-039 事故场景回归：motor_protocol_node ↔ mock 电机（无 CAN 硬件、无 root）。

背景（2026-09-14 真机事故，见 docs/lessons_learned/LL-039）：
  ① 示教退出把 MIT 目标重锚到拖动位姿（L2=1.98 rad）；
  ② 看门狗假触发 HOLD_DRIFT → /a3/motor/stop 写 NaN + 卸力帧——但卸力帧不改变
     mode（仍=2），refresh 播种分支 5 ms 后把 NaN 覆盖、目标锚回拖动位姿；
  ③ 看门狗升级 reset → 电机 mode 0（人工把臂搬回 home，实际 0.03 rad）；
  ④ /a3/arm/enable → kp=80 对 2 rad 误差满输出 → 3 s 甩到位，甩断 L6 打印关节。

本测试断言修复后的行为（F51）：
  T1 stop 后执行层不得再续发 kp>0 的旧目标（hold 抑制生效，只发零增益保活）；
  T2 reset 后目标清空；
  T3 使能必须重锚到反馈位（命令位置 ≈ 实际位置），且 kp 软起步（斜坡）；
  T4 全程 mock 关节位置不被甩动（|Δq| 小）。

接线方式：不经过 SocketCAN——直接对接 motor_protocol_node 的
/can_tx_frames（执行层发出）与 /can_rx_frames（本脚本伪造电机反馈），
因此覆盖的是真 C++ 执行层逻辑（仿真栈的 sim_motor_node 替换了整个 C++ 节点，
不覆盖本次改动）。

安全隔离：本测试默认跑在独立 ROS_DOMAIN_ID（默认 57，`--domain N` 可改），
与真机栈（domain 0）DDS 完全不可见——否则本脚本伪造的反馈帧会喂给真机执行层、
/a3/motor/enable 客户端可能打到真机执行层的服务上（禁止！）。

用法：
  source scripts/a3_shell_env.sh
  python3 scripts/a3_test/incident_regression_test.py          # 自动拉起执行层
  python3 scripts/a3_test/incident_regression_test.py --no-spawn   # 执行层已在跑
  python3 scripts/a3_test/incident_regression_test.py --domain 58
"""

import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from a3_can_bridge.srv import MotorCommand, MotorStop
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger

MOTORS = [1, 2, 3, 4, 5, 6, 7]
JOINTS7 = [
    "L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint", "L7_joint",
]
TORQUE_MAX = {1: 14.0, 2: 14.0, 3: 14.0, 4: 6.0, 5: 6.0, 6: 6.0, 7: 6.0}
SPEED_MAX = {1: 33.0, 2: 33.0, 3: 33.0, 4: 50.0, 5: 50.0, 6: 50.0, 7: 50.0}
P_MIN, P_MAX = -12.57, 12.57
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
CMD_CONTROL, CMD_FEEDBACK, CMD_ENABLE = 0x01, 0x02, 0x03
CMD_RESET, CMD_SET_ZERO = 0x04, 0x06

BUS_CAN1 = 1
FEEDBACK_HZ = 50.0
PLANT_MAX_SPEED = 6.0      # rad/s，一阶跟随的限速（模拟真实关节被驱动时的运动）

Q_DRAG = 1.98    # 示教拖动后的 L2 位姿（真机事故值 1.98462）
Q_HOME = 0.03    # 人工搬回 home 后（真机 0.0286）
TOL_CMD = 0.10   # 使能后命令位置与反馈位允许的最大偏差（rad）


def f2u(x, lo, hi):
    x = max(lo, min(hi, float(x)))
    return int((x - lo) / (hi - lo) * 65535.0)


def u2f(u, lo, hi):
    return u * (hi - lo) / 65535.0 + lo


def pack_frame(bus, can_id, data):
    b = bytearray(15)
    b[0] = bus
    b[1] = 1
    b[2] = len(data)
    b[3] = (can_id >> 24) & 0xFF
    b[4] = (can_id >> 16) & 0xFF
    b[5] = (can_id >> 8) & 0xFF
    b[6] = can_id & 0xFF
    b[7:7 + len(data)] = data
    msg = UInt8MultiArray()
    msg.data = list(bytes(b))
    return msg


class Harness(Node):
    """mock 电机组 + 场景驱动（同一节点，便于直接读写内部状态）。"""

    def __init__(self, motors=None):
        super().__init__("f51_incident_test")
        # F52：mock 电机的集合 = 当前档位在线电机（默认 7 台）。5J 档传 [1..5]。
        self.motors = list(motors) if motors else list(MOTORS)
        be = QoSProfile(
            depth=400, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(UInt8MultiArray, "/can_tx_frames", self._on_tx, be)
        # 执行层 /can_tx_frames 是 RELIABLE depth=4000，双订阅覆盖两种 QoS（LL-030 同款）
        self.create_subscription(UInt8MultiArray, "/can_tx_frames", self._on_tx, 400)
        self._rx_pub = self.create_publisher(UInt8MultiArray, "/can_rx_frames", be)

        self._lock = threading.Lock()
        self.mode = {m: 0 for m in self.motors}
        self.pos = {m: 0.0 for m in self.motors}
        self.vel = {m: 0.0 for m in self.motors}
        self.torque = {m: 0.0 for m in self.motors}
        self.temp = {m: 35.0 for m in self.motors}
        self.cmd_p = {m: 0.0 for m in self.motors}
        self.cmd_kp = {m: 0.0 for m in self.motors}
        self.frames = []     # (t, motor, kind, p, kp, kd, tau)
        self._t0 = time.monotonic()

        self.create_timer(1.0 / FEEDBACK_HZ, self._feedback_tick)
        self.create_timer(1.0 / FEEDBACK_HZ, self._plant_tick)

    # ---------------------------------------------------------------- mock 电机

    def t(self):
        return time.monotonic() - self._t0

    def set_pose(self, motor_id, q):
        """模拟「人搬动」：直接改实际位置（只在失能/kp=0 时代表物理搬动）。"""
        with self._lock:
            self.pos[motor_id] = float(q)
            self.vel[motor_id] = 0.0

    def _on_tx(self, msg):
        d = bytes(msg.data)
        if len(d) != 15:
            return
        bus = d[0]
        can_id = int.from_bytes(d[3:7], "big")
        data = d[7:15]
        cmd = (can_id >> 24) & 0x1F
        mid = can_id & 0xFF
        if mid not in self.mode or bus != BUS_CAN1:
            return
        now = self.t()
        with self._lock:
            if cmd == CMD_ENABLE:
                self.mode[mid] = 2
                self.frames.append((now, mid, "enable", 0.0, 0.0, 0.0, 0.0))
            elif cmd == CMD_RESET:
                self.mode[mid] = 0
                self.cmd_kp[mid] = 0.0
                self.frames.append((now, mid, "reset", 0.0, 0.0, 0.0, 0.0))
            elif cmd == CMD_SET_ZERO:
                self.pos[mid] = 0.0
                self.frames.append((now, mid, "set_zero", 0.0, 0.0, 0.0, 0.0))
            elif cmd == CMD_CONTROL:
                tau = u2f((can_id >> 8) & 0xFFFF, -TORQUE_MAX[mid], TORQUE_MAX[mid])
                p = u2f((data[0] << 8) | data[1], P_MIN, P_MAX)
                v = u2f((data[2] << 8) | data[3], -SPEED_MAX[mid], SPEED_MAX[mid])
                kp = u2f((data[4] << 8) | data[5], KP_MIN, KP_MAX)
                kd = u2f((data[6] << 8) | data[7], KD_MIN, KD_MAX)
                self.cmd_p[mid] = p
                self.cmd_kp[mid] = kp
                self.frames.append((now, mid, "mit", p, kp, kd, tau))

    def _plant_tick(self):
        dt = 1.0 / FEEDBACK_HZ
        with self._lock:
            for m in self.motors:
                if self.mode[m] != 2 or self.cmd_kp[m] <= 0.5:
                    continue
                err = self.cmd_p[m] - self.pos[m]
                step = max(-PLANT_MAX_SPEED * dt, min(PLANT_MAX_SPEED * dt, err))
                self.pos[m] += step
                self.vel[m] = step / dt

    def _feedback_tick(self):
        with self._lock:
            for m in self.motors:
                can_id = (CMD_FEEDBACK << 24) | (self.mode[m] << 22) | (m << 8) | 0xFD
                pu = f2u(self.pos[m], P_MIN, P_MAX)
                vu = f2u(self.vel[m], -SPEED_MAX[m], SPEED_MAX[m])
                tu = f2u(self.torque[m], -TORQUE_MAX[m], TORQUE_MAX[m])
                tq = int(self.temp[m] * 10)
                data = bytes([
                    pu >> 8, pu & 0xFF, vu >> 8, vu & 0xFF,
                    tu >> 8, tu & 0xFF, tq >> 8, tq & 0xFF,
                ])
                self._rx_pub.publish(pack_frame(BUS_CAN1, can_id, data))

    # ---------------------------------------------------------------- 查询辅助

    def mit_frames(self, motor_id, t_from=0.0, t_to=None):
        with self._lock:
            return [
                f for f in self.frames
                if f[1] == motor_id and f[2] == "mit" and f[0] >= t_from
                and (t_to is None or f[0] <= t_to)
            ]


_T_START = time.monotonic()


def log(msg):
    print(f"[{time.monotonic() - _T_START:6.2f}s] {msg}", flush=True)


def call(node, client, req, timeout=3.0):
    fut = client.call_async(req)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout:
        time.sleep(0.005)
    if not fut.done():
        return None
    return fut.result()


def wait_for_publisher(node, topic, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if node.get_publishers_info_by_topic(topic):
            return True
        time.sleep(0.1)
    return False


INSTALL_CFG = "install/a3_can_bridge/share/a3_can_bridge/config"
GAINS_7J = f"{INSTALL_CFG}/control_gains.yaml"
MAP_7J = f"{INSTALL_CFG}/motor_map.yaml"


def spawn_exec_node(gains_file=GAINS_7J, map_file=MAP_7J, log_path="/tmp/f51_regression_exec.log",
                    extra_args=()):
    """拉起真执行层（motor_protocol_node）。

    LL-039 判据教训 4：`ros2 run` 只是包装器，terminate() 杀不掉它孵化的节点
    （实测残留 PPID=1）——必须 start_new_session + 按进程组 killpg 收尸。
    返回 (proc, log_file_handle)；调用方负责 killpg 与关文件。
    """
    cmd = [
        "ros2", "run", "a3_can_bridge", "motor_protocol_node", "--ros-args",
        "--params-file", gains_file,
        "--params-file", map_file,
        "-p", "enable_power_sequence_gate:=false",
        "-p", "enable_gravity_compensation:=false",
        "-p", "enable_rx_decode_log:=false",
        "-p", "publish_feedback_joint_states:=true",
        *extra_args,
    ]
    fh = open(log_path, "w")
    proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, fh


def kill_exec_node(proc, fh):
    """按进程组收尸（见 spawn_exec_node 说明）。"""
    if proc is not None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                proc.kill()
    if fh is not None:
        fh.close()


def main():
    no_spawn = "--no-spawn" in sys.argv
    domain = "57"
    if "--domain" in sys.argv:
        domain = sys.argv[sys.argv.index("--domain") + 1]
    if os.environ.get("ROS_DOMAIN_ID", "") != domain:
        log(f"→ 隔离域 ROS_DOMAIN_ID={domain}（真机栈在 domain 0，DDS 互不可见）")
    os.environ["ROS_DOMAIN_ID"] = domain

    rclpy.init()
    node = Harness()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()

    proc = None
    exec_log = None
    failures = []
    try:
        # ---- 预检：不要和已在跑的执行层共用话题（本测试会伪造全部反馈）
        if node.get_publishers_info_by_topic("/can_tx_frames"):
            log("✗ /can_tx_frames 上已有发布者（残留执行层？）——先清理再跑本测试")
            return 2
        if not no_spawn:
            log("→ 拉起执行层（日志见 /tmp/f51_regression_exec.log）")
            proc, exec_log = spawn_exec_node()
        if not wait_for_publisher(node, "/can_tx_frames", 10.0):
            log("✗ 执行层未就绪（/can_tx_frames 无发布者）")
            return 2
        log("→ 执行层就绪")

        # 服务客户端
        cli = {
            "enable": node.create_client(MotorCommand, "/a3/motor/enable"),
            "reset": node.create_client(MotorCommand, "/a3/motor/reset"),
            "stop": node.create_client(MotorStop, "/a3/motor/stop"),
            "zt_start": node.create_client(Trigger, "/a3/zero_torque/start"),
            "zt_stop": node.create_client(Trigger, "/a3/zero_torque/stop"),
        }
        for name, c in cli.items():
            if not c.wait_for_service(timeout_sec=5.0):
                log(f"✗ 服务不可用: {name}")
                return 2
        log("→ 服务就绪")

        # 初始位姿：home，全部失能
        for m in node.motors:
            node.set_pose(m, Q_HOME)
        time.sleep(1.0)   # 等 refresh 保活帧 + 反馈新鲜度建立

        # ---- T0 使能 ----
        r = call(node, cli["enable"], MotorCommand.Request(motor_id=0))
        if r is None or not r.success:
            failures.append(f"T0 使能失败: {getattr(r, 'message', None)}")
        else:
            log(f"T0 使能: {r.message}")
            if "F51" not in r.message:
                failures.append("T0 使能响应缺少 F51 重锚摘要")
        time.sleep(0.6)

        # ---- T1 示教拖动 → 目标重锚到拖动位姿 ----
        r = call(node, cli["zt_start"], Trigger.Request())
        if r is None or not r.success:
            failures.append(f"T1 零力矩进入失败: {getattr(r, 'message', None)}")
        time.sleep(0.3)
        node.set_pose(2, Q_DRAG)     # 手搬（零力矩下）
        time.sleep(0.3)
        r = call(node, cli["zt_stop"], Trigger.Request())
        if r is None or not r.success:
            failures.append(f"T1 零力矩退出失败: {getattr(r, 'message', None)}")
        time.sleep(0.6)
        held = node.mit_frames(2, t_from=node.t() - 0.4)
        if not held or max(abs(f[3] - Q_DRAG) for f in held) > 0.15:
            failures.append(f"T1 示教退出未把目标重锚到拖动位姿: {held[-1:]}")
        else:
            log(f"T1 示教退出目标已锚定拖动位姿 ≈{Q_DRAG}（帧 {len(held)}）")

        # ---- T2 /a3/motor/stop：不得再续发满增益旧目标 ----
        # F58（LL-053）后语义更新：stop 后电机仍使能且反馈新鲜时允许「重力支撑保持」
        # （stop_hold_kp=25，目标逐帧锚定当前实测位）——禁止的是 kp=80 拉旧目标。
        t_stop = node.t()
        r = call(node, cli["stop"], MotorStop.Request(motor_id=0))
        if r is None or not r.success:
            failures.append(f"T2 motor_stop 调用失败: {getattr(r, 'message', None)}")
        time.sleep(1.2)
        after = node.mit_frames(2, t_from=t_stop + 0.2)
        if not after:
            failures.append("T2 stop 后完全停帧（保活流断，LL-020 会致 js 冻结）")
        else:
            bad_gain = [f for f in after if f[4] > 25.5]   # F58 上限 stop_hold_kp=25
            bad_pull = [f for f in after if abs(f[3] - node.pos[2]) > 0.10]
            if bad_gain or bad_pull:
                failures.append(
                    f"T2 F51/F58 失败：stop 后续发满增益/拉离实测位 "
                    f"gain={[(round(f[3], 3), round(f[4], 1)) for f in (bad_gain or bad_pull)[:3]]}"
                    f"（实测位 {node.pos[2]:.3f}）")
            else:
                log(f"T2 stop 后 {len(after)} 帧：kp≤25 重力保持且目标锚定实测位 "
                    f"{node.pos[2]:.3f}，未拉旧目标 ✓")

        # ---- T3 看门狗阶梯 reset → mode 0 ----
        r = call(node, cli["reset"], MotorCommand.Request(motor_id=0, command=2))
        time.sleep(0.5)
        with node._lock:
            mode_l2 = node.mode[2]
        if mode_l2 != 0:
            failures.append(f"T3 reset 后 motor2 mode={mode_l2}（应为 0）")
        else:
            log("T3 reset → motor2 mode=0 ✓")

        # ---- T4 人工搬回 home → 使能必须重锚 + 软起步 ----
        node.set_pose(2, Q_HOME)
        time.sleep(0.3)
        t_en = node.t()
        r = call(node, cli["enable"], MotorCommand.Request(motor_id=0))
        if r is None or not r.success:
            failures.append(f"T4 使能失败: {getattr(r, 'message', None)}")
        else:
            log(f"T4 使能: {r.message}")
        time.sleep(1.5)
        f2 = [f for f in node.mit_frames(2, t_from=t_en) if f[4] > 0.01]
        if not f2:
            failures.append("T4 使能后无 kp>0 帧（电机无力矩）")
        else:
            worst = max(abs(f[3] - Q_HOME) for f in f2)
            max_kp = max(f[4] for f in f2)
            first_kp = f2[0][4]
            log(f"T4 使能后 {len(f2)} 帧: 命令位置最大偏离反馈位 {worst:.4f} rad, "
                f"kp {first_kp:.1f}→{max_kp:.1f}")
            if worst > TOL_CMD:
                failures.append(
                    f"T4 F51 失败：使能后命令位置偏离反馈位 {worst:.3f} rad"
                    f"（陈旧目标未被丢弃 → 会甩动）")
            if max_kp < 40.0:
                failures.append(f"T4 kp 未升到额定（max {max_kp:.1f}）")
            if first_kp > 0.6 * max_kp:
                failures.append(f"T4 缺少 kp 软起步斜坡（首帧 kp={first_kp:.1f}）")
        q_after = node.pos[2]
        if abs(q_after - Q_HOME) > 0.05:
            failures.append(f"T4 关节被甩动：L2 实际位置 {q_after:.3f}（应停在 {Q_HOME}）")
        else:
            log(f"T4 关节未被甩动：L2 实际位置 {q_after:.4f} ✓")

        # ---- T5 部分点名轨迹不得复活「使能前的历史输入」（LL-039 同类补洞）----
        # 场景：先跑一条满关节轨迹（把 latest_input 写成 0.6）→ stop/reset → 人工搬回
        # 0.03 → 使能 → 来一条**只点名 L1** 的轨迹（真机对应夹爪 L7 单关节轨迹）。
        # 修复前：未点名关节回退到使能前的 latest_input=0.6，又被驱动成一次「执行历史」；
        # 修复后：使能时随重锚一起清掉回退目标，未点名关节由 refresh 保持在重锚位。
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: PLC0415
        Q_STALE = 0.6
        traj_pub = node.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10)

        def publish(names, positions):
            msg = JointTrajectory()
            msg.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            msg.points = [pt]
            for _ in range(3):
                traj_pub.publish(msg)
                time.sleep(0.1)

        t_sub = time.monotonic()
        while traj_pub.get_subscription_count() < 1 and time.monotonic() - t_sub < 5.0:
            time.sleep(0.1)
        home7 = [Q_HOME] * 7
        publish(JOINTS7, home7[:1] + [Q_STALE] + home7[2:])
        # 到位要等启动平滑走完（startup_smoothing_duration_s=2 s，起点 = 使能时反馈位），
        # 不是轨迹一发布就到位：轮询收敛，最多 8 s
        t_wait = time.monotonic()
        while time.monotonic() - t_wait < 8.0:
            fr = node.mit_frames(2)
            if fr and abs(fr[-1][3] - Q_STALE) <= 0.05:
                break
            time.sleep(0.2)
        stale_cmd = node.mit_frames(2)
        if not stale_cmd or abs(stale_cmd[-1][3] - Q_STALE) > 0.10:
            failures.append(f"T5 前置失败：满关节轨迹未把 L2 目标带到 {Q_STALE}: {stale_cmd[-1:]}")
        else:
            # 失能 + 人工搬回 home（与事故时间线一致）
            call(node, cli["stop"], MotorStop.Request(motor_id=0))
            call(node, cli["reset"], MotorCommand.Request(motor_id=0, command=2))
            time.sleep(0.4)
            for m in node.motors:
                node.set_pose(m, Q_HOME)
            time.sleep(0.4)
            r = call(node, cli["enable"], MotorCommand.Request(motor_id=0))
            if r is None or not r.success:
                failures.append(f"T5 使能失败: {getattr(r, 'message', None)}")
            time.sleep(0.8)
            t_partial = node.t()
            publish(["L1_joint"], [Q_HOME + 0.05])   # 只点名 L1（≈ 夹爪单关节轨迹）
            time.sleep(1.5)
            bad = []
            for m in range(2, 8):
                fr = node.mit_frames(m, t_from=t_partial)
                if fr and abs(fr[-1][3] - Q_HOME) > 0.10:
                    bad.append((m, round(fr[-1][3], 3)))
            dragged = [(m, round(node.pos[m], 3)) for m in range(2, 8)
                       if abs(node.pos[m] - Q_HOME) > 0.10]
            if bad:
                failures.append(
                    f"T5 F51 失败：部分点名轨迹把未点名关节驱动到历史输入 {bad}"
                    f"（实际位 {dragged}）")
            else:
                log(f"T5 只点名 L1 的轨迹未动未点名关节（L2..L7 目标保持 ≈{Q_HOME}）✓")

        # ---- 收尾：失能 ----
        call(node, cli["reset"], MotorCommand.Request(motor_id=0, command=2))

    finally:
        log("→ 收尾")
        kill_exec_node(proc, exec_log)
        ex.shutdown()
        spin_thread.join(timeout=2.0)   # 不等 spin 线程退出，rclpy 会打印 Destroyable 竞态告警
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log("→ 收尾完成")

    print()
    if failures:
        log("✗ F51 事故回归测试失败：")
        for f in failures:
            log("  - " + f)
        return 1
    log("✓ F51 事故回归测试通过（stop 抑制 / 使能重锚 / kp 软起步 / 无甩动）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
