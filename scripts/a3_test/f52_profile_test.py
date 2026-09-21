#!/usr/bin/env python3
"""F52 缺电机降级档回归：执行层「档位」配置（无 CAN 硬件、无 root）。

背景（F52，事故后遗症）：2026-09-14 事故甩断 L6/L7，can1 上只剩 L1–L5 五台电机。
整栈按 7 关节写死时**不是「少两个关节」，而是整臂不可用**：
  · F51 使能门禁逐电机校验反馈新鲜度，L6/L7 永远陈旧 → **整体拒绝使能**，
    连活着的 5 台都动不了（本测试用例 B 复现该现场）；
  · L6/L7 的陈旧缓存值（fresh=false，位置/模式是拽拽脱前最后一帧）会作为
    「看起来正常的假值」参与限位校验与看门狗比对（LL-020 同款陷阱）。

本测试三个用例：
  A. 5J 档（motor_map_5j.yaml + control_gains_5j.yaml，mock 电机 1..5）：
     档位=5、/joint_states 5 名字（无 L6/L7）、MotorStates 5 条、使能 5 帧且成功、
     stop 后仍为零增益保活（F51 语义在降级档下不放松）。
  B. 事故现场复现（7J 默认档 + 总线只有 5 台）：使能**必须被拒**且不发任何帧。
  C. 非法档位（joint_names 5 项 / motor_ids_by_index 7 项）：必须报
     「F52 档位配置非法」并**退回 7 关节默认档**（不静默降级成某个混合档）。

用法（与 incident_regression_test.py 同款隔离，默认 ROS_DOMAIN_ID=57）：
  source scripts/a3_shell_env.sh
  python3 scripts/a3_test/f52_profile_test.py
  python3 scripts/a3_test/f52_profile_test.py --domain 58
"""

import os
import re
import sys
import threading
import time

import rclpy
from a3_can_bridge.msg import MotorStates
from a3_can_bridge.srv import MotorCommand, MotorStop
from builtin_interfaces.msg import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from incident_regression_test import (  # noqa: E402
    GAINS_7J, INSTALL_CFG, MAP_7J, Harness, call, kill_exec_node, log,
    spawn_exec_node, wait_for_publisher,
)

GAINS_5J = f"{INSTALL_CFG}/control_gains_5j.yaml"
MAP_5J = f"{INSTALL_CFG}/motor_map_5j.yaml"
MOTORS_5J = [1, 2, 3, 4, 5]
Q_HOME = 0.03


class Probe(rclpy.node.Node):
    """订阅 /joint_states 与 /a3/motor/states，记录最近一条，供档位断言。

    注意 QoS：执行层这两个话题都是 SensorDataQoS（BEST_EFFORT），
    订阅端必须同为 BEST_EFFORT，否则 QoS 不兼容收不到任何数据（LL-030 同款）。
    """

    def __init__(self):
        super().__init__("f52_profile_probe")
        self.last_js = None
        self.last_states = None
        be = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, "/joint_states", self._on_js, be)
        self.create_subscription(MotorStates, "/a3/motor/states", self._on_states, be)

    def _on_js(self, msg):
        self.last_js = msg

    def _on_states(self, msg):
        self.last_states = msg


def wait_probe(node, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if node.last_js is not None and node.last_states is not None:
            return True
        time.sleep(0.1)
    return False


def parse_array(path, key):
    """从控制参数 YAML 里取逐关节数组（`key: [..]`，与档位等长）。"""
    with open(path) as f:
        m = re.search(rf"^\s*{key}:\s*\[([^\]]*)\]", f.read(), re.M)
    if not m:
        raise KeyError(f"{path} 缺少 {key}")
    return [float(x) for x in m.group(1).split(",")]


def make_traj_targets(gains, n_joints):
    """按档位生成「各关节行程内、互不相同」的 champ 域目标 + 期望的 MIT 域目标。

    MIT 域目标 = joint_signs[i] × champ[i] + joint_offsets_rad[i]（执行层映射契约）。
    目标必须逐关节不同：只有值不同，串关节（route/下标错配）才测得出来；
    也必须落在各自的 joint_cmd_min/max 内，否则被限幅成 0，测试会误报（L3 的正值即如此）。
    """
    signs = parse_array(gains, "joint_signs")
    offsets = parse_array(gains, "joint_offsets_rad")
    lo = parse_array(gains, "joint_cmd_min_rad")
    hi = parse_array(gains, "joint_cmd_max_rad")
    seed = [0.20, 0.25, -0.15, 0.10, 0.30, 0.35, 0.40]
    champ = []
    for i in range(n_joints):
        t = seed[i % len(seed)]
        if lo[i] > hi[i]:
            lo[i], hi[i] = hi[i], lo[i]
        if not (lo[i] < t < hi[i]):
            t = lo[i] + 0.6 * (hi[i] - lo[i])
        champ.append(round(t, 3))
    expect_mit = [round(signs[i] * champ[i] + offsets[i], 4) for i in range(n_joints)]
    return champ, expect_mit


def run_case(name, procs_cfg, motors, expect, failures):
    """拉起执行层 → 断言 → 收尾。procs_cfg: (gains, map, log_path)。"""
    log(f"=== 用例 {name} ===")
    gains, map_file, log_path = procs_cfg
    if os.path.exists(log_path):
        os.remove(log_path)

    rclpy.init()
    harness = Harness(motors=motors)
    probe = Probe()
    ex = MultiThreadedExecutor()
    ex.add_node(harness)
    ex.add_node(probe)
    threading.Thread(target=ex.spin, daemon=True).start()

    proc = fh = None
    try:
        # 上一用例的执行层刚被 killpg，DDS 发现表清理有延迟：给 3 s 宽限再判定残留
        deadline = time.monotonic() + 3.0
        while (time.monotonic() < deadline
               and harness.get_publishers_info_by_topic("/can_tx_frames")):
            time.sleep(0.2)
        if harness.get_publishers_info_by_topic("/can_tx_frames"):
            failures.append(f"{name}: /can_tx_frames 上已有发布者（残留执行层？）")
            return
        proc, fh = spawn_exec_node(gains, map_file, log_path)
        if not wait_for_publisher(harness, "/can_tx_frames", 10.0):
            failures.append(f"{name}: 执行层未就绪")
            return
        for m in motors:
            harness.set_pose(m, Q_HOME)
        time.sleep(1.2)

        if not wait_probe(probe, 8.0):
            failures.append(f"{name}: 未收到 /joint_states 或 /a3/motor/states")
            return

        # ---- 档位断言：js 名字与 MotorStates 条数 ----
        js_names = list(probe.last_js.name)
        n_states = len(probe.last_states.states)
        log(f"{name}: /joint_states {len(js_names)} 名字 {js_names}；MotorStates {n_states} 条")
        if len(js_names) != expect["n_joints"]:
            failures.append(f"{name}: /joint_states 档位不符 {len(js_names)} != {expect['n_joints']}")
        if n_states != expect["n_joints"]:
            failures.append(f"{name}: MotorStates 条数不符 {n_states} != {expect['n_joints']}")
        for absent in expect.get("absent", []):
            if absent in js_names:
                failures.append(f"{name}: 缺电机 {absent} 仍出现在 /joint_states（陈旧缓存值污染）")

        # ---- 使能断言 ----
        cli_enable = harness.create_client(MotorCommand, "/a3/motor/enable")
        cli_stop = harness.create_client(MotorStop, "/a3/motor/stop")
        cli_reset = harness.create_client(MotorCommand, "/a3/motor/reset")
        for c in (cli_enable, cli_stop, cli_reset):
            if not c.wait_for_service(timeout_sec=5.0):
                failures.append(f"{name}: 服务不可用")
                return

        t_en = harness.t()
        r = call(harness, cli_enable, MotorCommand.Request(motor_id=0))
        time.sleep(0.6)
        # Harness 对 /can_tx_frames 挂了 BEST_EFFORT + RELIABLE 两个订阅（LL-030），
        # 每帧会被记两次 → 这里按「去重后的电机」计数（下面保活帧数也说明真实速率：
        # 482 帧/1.2 s ÷ 2 ≈ 200 Hz，正是执行层标称 tx 速率）。
        enabled_motors = sorted({f[1] for f in harness.frames
                                 if f[2] == "enable" and f[0] >= t_en})
        n_enable = len(enabled_motors)

        if expect["enable_ok"]:
            if r is None or not r.success:
                failures.append(f"{name}: 使能应成功但失败: {getattr(r, 'message', None)}")
            elif n_enable != expect["n_joints"]:
                failures.append(
                    f"{name}: 使能电机数 {n_enable} != 档位 {expect['n_joints']}（{enabled_motors}）")
            else:
                log(f"{name}: 使能成功，{n_enable} 台电机逐个使能 {enabled_motors} ✓")

            # ---- 轨迹路径：档位内关节名轨迹必须逐关节落到总线上 ----
            # 只看使能不算「档位可用」：若轨迹处理里还残留 7 关节写死（正是 F52 的病灶），
            # 使能能过、轨迹却丢关节。这里发一条档位内命名轨迹，逐电机核对末次目标。
            traj = expect.get("traj")
            if traj:
                pub = harness.create_publisher(
                    JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10)
                t_sub = time.monotonic()
                while pub.get_subscription_count() < 1 and time.monotonic() - t_sub < 5.0:
                    time.sleep(0.1)
                if pub.get_subscription_count() < 1:
                    failures.append(f"{name}: 轨迹订阅者未上线")
                elif len(motors) != expect["n_joints"]:
                    failures.append(f"{name}: 轨迹用例要求 mock 电机与档位一一对应")
                else:
                    champ, expect_mit = make_traj_targets(gains, expect["n_joints"])
                    msg = JointTrajectory()
                    msg.joint_names = list(js_names)  # 用执行层自己广播的档位名表
                    pt = JointTrajectoryPoint()
                    pt.positions = champ
                    pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
                    msg.points = [pt]
                    t_traj = harness.t()
                    for _ in range(3):
                        pub.publish(msg)
                        time.sleep(0.1)
                    time.sleep(traj["settle_s"])
                    worst = 0.0
                    missing = []
                    last_targets = {}
                    for i, m in enumerate(motors):
                        fr = harness.mit_frames(m, t_from=t_traj)
                        if not fr:
                            missing.append(m)
                            continue
                        last_targets[m] = round(fr[-1][3], 4)
                        worst = max(worst, abs(fr[-1][3] - expect_mit[i]))
                    if missing:
                        failures.append(f"{name}: 轨迹后电机 {missing} 未收到任何 MIT 帧（关节被丢）")
                    elif worst > traj["tol"]:
                        failures.append(
                            f"{name}: 轨迹目标未逐关节到位，最大偏差 {worst:.4f} > {traj['tol']} rad"
                            f"（champ {champ} → 期望 MIT {expect_mit}，实测 {last_targets}）")
                    else:
                        log(f"{name}: {len(motors)} 关节命名轨迹逐关节到位（champ {champ} → "
                            f"MIT {last_targets}，最大偏差 {worst:.4f} rad，"
                            f"实际位 {[round(harness.pos[m], 3) for m in motors]}）✓")
                    pub.destroy()

            # stop 后禁止满增益拉旧目标（F58/LL-053：允许 ≤25 的重力支撑保持，
            # 目标逐帧锚定实测位；降级档下该语义同样不放松）
            t_stop = harness.t()
            r_stop = call(harness, cli_stop, MotorStop.Request(motor_id=0))
            time.sleep(1.2)
            after = [fr for m in motors for fr in harness.mit_frames(m, t_from=t_stop + 0.2)]
            bad_gain = [fr for fr in after if fr[4] > 25.5]
            bad_pull = [fr for fr in after if abs(fr[3] - harness.pos[fr[1]]) > 0.10]
            if r_stop is None or not r_stop.success:
                failures.append(f"{name}: stop 调用失败: {getattr(r_stop, 'message', None)}")
            elif not after:
                failures.append(f"{name}: stop 后完全停帧（保活流断，LL-020）")
            elif bad_gain or bad_pull:
                failures.append(
                    f"{name}: stop 后续发满增益/拉离实测位 帧 {len(bad_gain or bad_pull)} 个")
            else:
                log(f"{name}: stop 后 {len(after)} 帧 kp≤25 重力保持锚定实测位 ✓")
        else:
            if r is not None and r.success:
                failures.append(f"{name}: 使能本应被拒（缺电机）却成功")
            if n_enable:
                failures.append(
                    f"{name}: 拒绝使能却仍对 {enabled_motors} 发了 enable 帧（半档使能 = 半臂僵住）")
            if r is not None and r.success is False and n_enable == 0:
                log(f"{name}: 使能被正确拒绝且未发任何帧 ✓（{r.message}）")

        call(harness, cli_reset, MotorCommand.Request(motor_id=0, command=2))
        time.sleep(0.3)
    finally:
        kill_exec_node(proc, fh)
        # 先停 spin 线程再销毁节点：反序会抛
        # "cannot use Destroyable because destruction was requested"（rclpy 收尾竞态，无害但吵）
        ex.shutdown()
        time.sleep(0.3)
        harness.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
        time.sleep(0.5)


def main():
    domain = "57"
    if "--domain" in sys.argv:
        domain = sys.argv[sys.argv.index("--domain") + 1]
    os.environ["ROS_DOMAIN_ID"] = domain
    log(f"→ 隔离域 ROS_DOMAIN_ID={domain}（真机栈在 domain 0，DDS 互不可见）")

    # 用例 C 的非法档位文件：joint_names 5 项 / motor_ids_by_index 7 项（长度不符）
    bad_map = "/tmp/f52_bad_profile.yaml"
    with open(bad_map, "w") as f:
        f.write(
            "/**:\n  ros__parameters:\n    arm_bus: can1\n"
            "    joint_names: [L1_joint, L2_joint, L3_joint, L4_joint, L5_joint]\n"
            "    motor_ids_by_index: [1, 2, 3, 4, 5, 6, 7]\n"
        )

    failures = []
    all_motors = [1, 2, 3, 4, 5, 6, 7]

    # A. 5J 档可用
    run_case(
        "A/5J档", (GAINS_5J, MAP_5J, "/tmp/f52_exec_5j.log"), MOTORS_5J,
        {"n_joints": 5, "absent": ["L6_joint", "L7_joint"], "enable_ok": True,
         "traj": {"settle_s": 3.0, "tol": 0.02}},
        failures)
    # B. 事故现场复现：7J 默认档 + 只有 5 台电机 → 使能必须被拒
    run_case(
        "B/事故现场", (GAINS_7J, MAP_7J, "/tmp/f52_exec_7j5motor.log"), MOTORS_5J,
        {"n_joints": 7, "enable_ok": False},
        failures)
    # C. 非法档位 → 报错并退回 7J 默认档（不静默降级）
    run_case(
        "C/非法档位", (GAINS_7J, bad_map, "/tmp/f52_exec_bad.log"), all_motors,
        {"n_joints": 7, "enable_ok": True},
        failures)
    with open("/tmp/f52_exec_bad.log") as f:
        bad_log = f.read()
    if "F52 档位配置非法" not in bad_log:
        failures.append("C/非法档位: 日志未见「F52 档位配置非法」")
    else:
        log("C/非法档位: 节点报「F52 档位配置非法」并退回 7 关节默认档 ✓")
    with open("/tmp/f52_exec_5j.log") as f:
        log5 = f.read()
    if "F52 档位：5 关节" not in log5:
        failures.append("A/5J档: 日志未见「F52 档位：5 关节」")
    if "L6_joint→motor6" in log5 or "motor6" in log5.split("F52 档位")[1][:200]:
        failures.append("A/5J档: 档位日志里仍出现 motor6")

    print()
    if failures:
        for f in failures:
            log(f"✗ {f}")
        log(f"F52 档位回归失败（{len(failures)} 项）")
        return 1
    log("✓ F52 档位回归通过（5J 可用 / 事故现场必拒 / 非法档位不静默降级）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
