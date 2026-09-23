#!/usr/bin/env python3
"""
F94 两点轨迹统一速度限幅验收（mock 模式全自动）。

验收步骤（对应 docs/edge/REQUIREMENTS.md F94）：
  1. 启动日志含 7 关节 URDF velocity（L1–L3=33、L4–L7=50 rad/s）
  2. 超限请求：L1 预置 -0.21 后 Δq=3.0 rad、duration=0.05 s
     → 实际下发 duration ≥ 0.20 s；/joint_states 数值微分峰值 ≤ 33 rad/s（容差 5%）
  3. 保守请求 Δq=0.1 rad、duration=2.0 s → duration 不被修改
  4. joint_velocity_scale:=0.5（ros2 param set 运行时生效）
     → 同超限请求地板 0.40 s、实测峰值 ≤ 16.5 rad/s（容差 5%）
  5. 回归：/a3/arm/disable safe-park 正常完成，最终位姿在 home 容差 0.15 rad 内

hardware:=mock 不触碰 CAN，可在机械臂断电下运行。
"""

import os
import re
import signal
import subprocess
import sys
import time

WS = "/home/cat/a3_arm_ws"
LOG = "/tmp/f94_accept_stack.log"

ROS_WRAP = (
    "source /opt/ros/humble/setup.bash && "
    f"source {WS}/install/local_setup.bash && "
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
    "export PYTHONNOUSERSITE=1 && "
)

failures = []


def check(name, ok, detail=""):
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {name}" + (f" -- {detail}" if detail else ""))
    if not ok:
        failures.append(name)


INNER = r'''
import re, sys, time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from a3_msgs.srv import SetJointPositions

ORDER = ["L1_joint","L2_joint","L3_joint","L4_joint","L5_joint","L6_joint","L7_joint"]
results = []

def report(name, ok, detail=""):
    results.append((name, bool(ok), str(detail)))
    print(("PASS" if ok else "FAIL") + "|" + name + "|" + str(detail))

rclpy.init()
n = Node("f94_accept_probe")
latest = {"names": [], "pos": []}
samples = []

def on_js(m):
    latest["names"] = list(m.name)
    latest["pos"] = list(m.position)
    if "L1_joint" in m.name:
        samples.append((time.time(), m.position[m.name.index("L1_joint")]))

n.create_subscription(JointState, "/joint_states", on_js, 10)
ena = n.create_client(Trigger, "/a3/arm/enable")
dis = n.create_client(Trigger, "/a3/arm/disable")
sjp = n.create_client(SetJointPositions, "/a3/arm/set_joint_positions")
setp = n.create_client(SetParameters, "/a3_arm_controller/set_parameters")

def spin(t):
    end = time.time() + t
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

def cur_pose():
    q = dict(zip(latest["names"], latest["pos"]))
    return [q.get(j, 0.0) for j in ORDER]

def jog(target, duration, collect=1.0):
    samples.clear()
    req = SetJointPositions.Request()
    req.positions = target
    req.duration = duration
    t0 = time.time()
    f = sjp.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    r = f.result()
    spin(collect)
    m = re.search(r"\(([0-9.]+)s\)", r.message if r else "")
    got = float(m.group(1)) if m else -1.0
    # 中心差分：DDS 偶发消息成批投递（实测有 dt≈1.8ms 的早到帧），
    # 前向差分把正常一步位移误算成 2 倍速；中心窗口跨 2 步即恢复真实速度。
    peak = 0.0
    for i in range(1, len(samples) - 1):
        ta, pa = samples[i - 1]
        tb, pb = samples[i + 1]
        peak = max(peak, abs((pb - pa) / (tb - ta)))
    return (r.success if r else False), got, peak, time.time() - t0

def set_scale(v):
    req = SetParameters.Request()
    pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
    req.parameters = [Parameter(name="joint_velocity_scale", value=pv)]
    f = setp.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    return bool(f.result() and f.result().results[0].successful)

deadline = time.time() + 60
while time.time() < deadline and not samples:
    rclpy.spin_once(n, timeout_sec=0.2)
if not samples:
    report("joint_states 60s 内到达", False)
    rclpy.shutdown(); sys.exit(1)

assert ena.wait_for_service(timeout_sec=15)
f = ena.call_async(Trigger.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=20)
report("/a3/arm/enable 成功", f.result() is not None and f.result().success,
       f.result().message if f.result() else "no response")
spin(1.5)

# --- 2. 超限（scale=1.0）：预置 L1=-0.21，再 Δq=3.0 / 0.05s ---
q = cur_pose()
q[0] = -0.21
ok, dur, _, _ = jog(q, 0.5, collect=0.8)
spin(0.4)
q = cur_pose()
q[0] = -0.21 + 3.0   # =2.79，在 L1 上限 2.7925 内，不触发 clamp
ok, dur, peak, _ = jog(q, 0.05, collect=0.8)
report("超限请求 duration ≥ 0.20 s", ok and dur >= 0.20, f"duration={dur:.3f}s")
report("实测 L1 峰值速度 ≤ 33 rad/s（+5%）", peak <= 33 * 1.05,
       f"peak={peak:.2f} rad/s")
spin(0.5)

# --- 3. 保守：Δq=0.1 / 2.0 s 不修改 ---
q = cur_pose()
q[0] = q[0] - 0.1
ok, dur, _, _ = jog(q, 2.0, collect=2.4)
report("保守请求 duration 保持 2.00 s", ok and abs(dur - 2.0) < 0.01,
       f"duration={dur:.3f}s")

# --- 4. scale=0.5：地板翻倍 ---
report("ros2 param set joint_velocity_scale=0.5", set_scale(0.5))
q = cur_pose()
q[0] = -0.21                       # 行程 2.69；scale=0.5 地板 0.36s < 请求 1.0s
ok, dur, _, _ = jog(q, 1.0, collect=1.3)
q = cur_pose()
q[0] = -0.21 + 3.0
ok, dur, peak, _ = jog(q, 0.05, collect=1.0)
report("scale=0.5 地板 duration ≥ 0.40 s", ok and dur >= 0.40,
       f"duration={dur:.3f}s")
report("scale=0.5 实测峰值 ≤ 16.5 rad/s（+5%）", peak <= 16.5 * 1.05,
       f"peak={peak:.2f} rad/s")
report("恢复 scale=1.0", set_scale(1.0))
spin(0.3)

# --- 5. 回归：disable safe-park 回 home ---
f = dis.call_async(Trigger.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=30)
dr = f.result()
report("/a3/arm/disable 成功", dr is not None and dr.success,
       dr.message if dr else "no response")

# home：包级 named_poses.yaml + ~/.a3/poses.yaml 覆盖（与 arm_controller 同规则）
import os, yaml
home = [0.0] * 7
try:
    from ament_index_python.packages import get_package_share_directory
    p = os.path.join(get_package_share_directory("a3_description"),
                     "config", "named_poses.yaml")
    data = yaml.safe_load(open(p, encoding="utf-8"))
    h = (data.get("poses") or {}).get("home")
    if h is not None:
        home = list(h["positions"] if isinstance(h, dict) else h)
except Exception as e:
    print("named poses load failed:", e)
up = os.path.expanduser("~/.a3/poses.yaml")
if os.path.exists(up):
    udata = yaml.safe_load(open(up, encoding="utf-8")) or {}
    h = (udata.get("poses") or {}).get("home")
    if h is not None:
        home = list(h["positions"] if isinstance(h, dict) else h)
spin(1.0)
q = cur_pose()
worst = max(abs(a - b) for a, b in zip(q, home))
report("safe-park 最终位姿在 home 0.15 rad 内", worst <= 0.15,
       f"worst={worst:.3f} rad")

rclpy.shutdown()
sys.exit(1 if any(not r[1] for r in results) else 0)
'''


def main():
    stack = subprocess.Popen(
        ["bash", "-c", ROS_WRAP + "exec ros2 launch a3_bringup a3_bringup.launch.py "
         "hardware:=mock use_mqtt:=false use_rviz:=false use_teleop:=false"],
        preexec_fn=os.setsid,
        stdout=open(LOG, "w"),
        stderr=subprocess.STDOUT,
    )

    inner_path = "/tmp/f94_accept_inner.py"
    with open(inner_path, "w") as f:
        f.write(INNER)

    try:
        # --- 1. 启动日志速度限（等 inner 跑完后日志已完整） ---
        time.sleep(20)
        r = subprocess.run(
            ["bash", "-c", ROS_WRAP + f"python3 {inner_path}"],
            capture_output=True, text=True, timeout=180,
        )
        print(r.stdout)
        if r.returncode != 0:
            print("STDERR:", r.stderr[-2000:])

        log = open(LOG, encoding="utf-8", errors="replace").read()
        m = re.search(r"F94 joint velocity limits \(rad/s\): (.+)", log)
        if m:
            got = {}
            for item in m.group(1).split(","):
                k, v = item.strip().split("=")
                got[k] = float(v)
            expect = {"L1_joint": 33, "L2_joint": 33, "L3_joint": 33,
                      "L4_joint": 50, "L5_joint": 50, "L6_joint": 50,
                      "L7_joint": 50}
            check("7 关节 URDF velocity 全部加载", got == expect,
                  m.group(1).strip())
        else:
            check("7 关节 URDF velocity 全部加载", False, "日志未找到 F94 行")

        for line in r.stdout.splitlines():
            parts = line.split("|", 2)
            if len(parts) == 3 and parts[0] in ("PASS", "FAIL"):
                check(parts[1], parts[0] == "PASS", parts[2])
    finally:
        try:
            os.killpg(os.getpgid(stack.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        time.sleep(8)
        try:
            os.killpg(os.getpgid(stack.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

    print()
    if failures:
        print(f"F94 验收失败：{len(failures)} 项 -- {failures}")
        return 1
    print("F94 验收通过：两点轨迹 URDF velocity 地板限幅（超限/保守/scale/回 home 回归）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
