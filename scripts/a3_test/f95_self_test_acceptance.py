#!/usr/bin/env python3
"""
F95 标准自检服务验收（mock 模式全自动，机械臂断电可跑）。

验收步骤（对应 docs/edge/REQUIREMENTS.md F95）：
  A. 节点单独运行（无栈）：SelfTest → passed=false，Connection status level=2(ERROR)
  B. mock 全栈、未 enable：passed=true；≥3 个 status；joint_states rate ≥40 Hz；
     joint_state_broadcaster=active；arm_controller=inactive；id 非空
  C. /a3/arm/enable 后：仍 passed=true，arm_controller=active（状态写入 message/键值）
  D. 回归：/a3/arm/disable safe-park 成功，最终位姿在 home 0.15 rad 内

hardware:=mock 不触碰 CAN。
"""

import os
import signal
import subprocess
import sys
import time

WS = "/home/cat/a3_arm_ws"
LOG = "/tmp/f95_accept_stack.log"

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


# 探针：调用一次 SelfTest，把结果逐行打印；可选先 enable/disable。
PROBE = r'''
import sys, time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.srv import SelfTest
from std_srvs.srv import Trigger

mode = sys.argv[1] if len(sys.argv) > 1 else "call"
rclpy.init()
n = Node("f95_accept_probe")
cli = n.create_client(SelfTest, "/a3_self_test/self_test")
ena = n.create_client(Trigger, "/a3/arm/enable")
dis = n.create_client(Trigger, "/a3/arm/disable")

if not cli.wait_for_service(timeout_sec=60):
    print("ERROR|service not available|")
    rclpy.shutdown(); sys.exit(1)

# 节点内部 1 s rate 窗口 + 500 ms list_controllers 刷新需要时间填满，
# 刚发现服务就调用会读到冷缓存（rate 偏低 / 无 controller 键值）。
end = time.time() + 3.0
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)

if mode == "enable":
    ena.wait_for_service(timeout_sec=15)
    f = ena.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, f, timeout_sec=20)
    r = f.result()
    print("ENABLE|" + ("ok" if r and r.success else "fail") + "|" +
          (r.message if r else "no response"))
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
elif mode == "disable":
    dis.wait_for_service(timeout_sec=15)
    f = dis.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, f, timeout_sec=30)
    r = f.result()
    print("DISABLE|" + ("ok" if r and r.success else "fail") + "|" +
          (r.message if r else "no response"))
    rclpy.shutdown(); sys.exit(0)

f = cli.call_async(SelfTest.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=15)
res = f.result()
if res is None:
    print("ERROR|no self_test response|")
    rclpy.shutdown(); sys.exit(1)

# diagnostic_msgs 4.9.1（板载定制源）把 SelfTest.passed 也定义为 octet，
# rclpy 反序列化为单元素 bytes：b'\x00'=false、b'\x01'=true。
passed = res.passed
if isinstance(passed, (bytes, bytearray)):
    passed = passed[0] != 0
print("RESULT|passed=%s|id=%s" % (str(passed), res.id))
for st in res.status:
    level = st.level[0] if isinstance(st.level, (bytes, bytearray)) else st.level
    print("STATUS|%s|level=%d|%s" % (st.name, level, st.message))
    for kv in st.values:
        print("KV|%s|%s=%s" % (st.name, kv.key, kv.value))
rclpy.shutdown()
'''

# disable 后读取最终位姿（复用 F94 的 home 加载规则）
POSE_PROBE = r'''
import os, time, sys, yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ORDER = ["L1_joint","L2_joint","L3_joint","L4_joint","L5_joint","L6_joint","L7_joint"]
rclpy.init()
n = Node("f95_pose_probe")
latest = {}
def on_js(m):
    latest.update(dict(zip(m.name, m.position)))
n.create_subscription(JointState, "/joint_states", on_js, 10)
end = time.time() + 5
while time.time() < end and len(latest) < 7:
    rclpy.spin_once(n, timeout_sec=0.1)

home = [0.0] * 7
from ament_index_python.packages import get_package_share_directory
p = os.path.join(get_package_share_directory("a3_description"),
                 "config", "named_poses.yaml")
data = yaml.safe_load(open(p, encoding="utf-8"))
h = (data.get("poses") or {}).get("home")
if h is not None:
    home = list(h["positions"] if isinstance(h, dict) else h)
up = os.path.expanduser("~/.a3/poses.yaml")
if os.path.exists(up):
    udata = yaml.safe_load(open(up, encoding="utf-8")) or {}
    h = (udata.get("poses") or {}).get("home")
    if h is not None:
        home = list(h["positions"] if isinstance(h, dict) else h)

q = [latest.get(j, 0.0) for j in ORDER]
worst = max(abs(a - b) for a, b in zip(q, home))
print("POSE|worst=%.4f" % worst)
rclpy.shutdown()
sys.exit(0 if worst <= 0.15 else 1)
'''


def run_probe(path, mode="call", timeout=90):
    r = subprocess.run(
        ["bash", "-c", ROS_WRAP + f"python3 {path} {mode}"],
        capture_output=True, text=True, timeout=timeout,
    )
    if r.returncode != 0:
        print("STDERR:", r.stderr[-1500:])
    return r.stdout


def parse_result(out):
    data = {"passed": None, "id": "", "status": [], "kv": {}}
    for line in out.splitlines():
        if line.startswith("RESULT|"):
            parts = line.split("|")
            for item in parts[1:]:
                if item.startswith("passed="):
                    data["passed"] = item.split("=", 1)[1] == "True"
                elif item.startswith("id="):
                    data["id"] = item.split("=", 1)[1]
        elif line.startswith("STATUS|"):
            _, name, levels, message = line.split("|", 3)
            level = int(levels.split("=")[1])
            data["status"].append((name, level, message))
        elif line.startswith("KV|"):
            _, task, kv = line.split("|", 2)
            k, v = kv.split("=", 1)
            data["kv"][f"{task}:{k}"] = v
    return data


def main():
    probe_path = "/tmp/f95_accept_probe.py"
    pose_path = "/tmp/f95_pose_probe.py"
    with open(probe_path, "w") as f:
        f.write(PROBE)
    with open(pose_path, "w") as f:
        f.write(POSE_PROBE)

    # ---- A. 单独节点（无栈）：passed=false + Connection ERROR ----
    standalone = subprocess.Popen(
        ["bash", "-c", ROS_WRAP + "exec ros2 run a3_self_test a3_self_test"],
        preexec_fn=os.setsid,
        stdout=open("/tmp/f95_standalone.log", "w"),
        stderr=subprocess.STDOUT,
    )
    try:
        time.sleep(3)
        out = run_probe(probe_path)
        data = parse_result(out)
        conn = [s for s in data["status"] if s[0] == "Connection"]
        check("无栈：passed=false", data["passed"] is False,
              f"passed={data['passed']}")
        check("无栈：Connection level=2(ERROR)", bool(conn) and conn[0][1] == 2,
              conn[0][2] if conn else "missing")
    finally:
        try:
            os.killpg(os.getpgid(standalone.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        time.sleep(3)
        try:
            os.killpg(os.getpgid(standalone.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

    # ---- B/C/D. mock 全栈 ----
    stack = subprocess.Popen(
        ["bash", "-c", ROS_WRAP + "exec ros2 launch a3_bringup a3_bringup.launch.py "
         "hardware:=mock use_mqtt:=false use_rviz:=false use_teleop:=false"],
        preexec_fn=os.setsid,
        stdout=open(LOG, "w"),
        stderr=subprocess.STDOUT,
    )
    try:
        # B. 未 enable
        out = run_probe(probe_path)
        data = parse_result(out)
        names = [s[0] for s in data["status"]]
        check("未 enable：passed=true", data["passed"] is True,
              f"passed={data['passed']}")
        check("status ≥ 3 项（Connection/Controllers/Faults）",
              len(data["status"]) >= 3, f"items={names}")
        rate = float(data["kv"].get("Connection:joint_states_rate_hz", "0"))
        check("joint_states rate ≥ 40 Hz", rate >= 40, f"rate={rate:.1f}")
        jsb = data["kv"].get("Controllers:controller:joint_state_broadcaster", "")
        check("joint_state_broadcaster active", jsb == "active", f"state={jsb}")
        arm = data["kv"].get("Controllers:controller:arm_controller", "")
        check("未 enable：arm_controller inactive（已 configure 未 activate）",
              arm == "inactive", f"state={arm}")
        check("id 非空（hostname）", bool(data["id"]), f"id={data['id']}")

        # C. enable 后
        out = run_probe(probe_path, mode="enable")
        en_line = [l for l in out.splitlines() if l.startswith("ENABLE|")]
        check("/a3/arm/enable 成功", bool(en_line) and en_line[0].split("|")[1] == "ok",
              en_line[0].split("|")[2] if en_line else "missing")
        data = parse_result(out)
        arm = data["kv"].get("Controllers:controller:arm_controller", "")
        check("enable 后：passed=true", data["passed"] is True,
              f"passed={data['passed']}")
        check("enable 后：arm_controller active", arm == "active",
              f"state={arm}")

        # D. disable safe-park 回归
        out = run_probe(probe_path, mode="disable")
        dis_line = [l for l in out.splitlines() if l.startswith("DISABLE|")]
        check("/a3/arm/disable 成功",
              bool(dis_line) and dis_line[0].split("|")[1] == "ok",
              dis_line[0].split("|")[2] if dis_line else "missing")
        time.sleep(6)
        r = subprocess.run(
            ["bash", "-c", ROS_WRAP + f"python3 {pose_path}"],
            capture_output=True, text=True, timeout=30,
        )
        pline = [l for l in r.stdout.splitlines() if l.startswith("POSE|")]
        worst = float(pline[0].split("=")[1]) if pline else 9.9
        check("safe-park 最终位姿在 home 0.15 rad 内", worst <= 0.15,
              f"worst={worst:.3f} rad")
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
        print(f"F95 验收失败：{len(failures)} 项 -- {failures}")
        return 1
    n_pass = 13
    print(f"F95 验收通过：标准只读自检服务 {n_pass}/{n_pass}"
          "（无栈 FAIL / 未 enable PASS / enable active / safe-park 回归）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
