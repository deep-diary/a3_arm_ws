#!/usr/bin/env python3
"""
F96 主机资源标准诊断验收（mock 模式全自动，机械臂断电可跑）。

验收步骤（对应 docs/edge/REQUIREMENTS.md F96）：
  A. 默认开启：/diagnostics 出现 cpu_monitor: CPU Information /
     ram_monitor: RAM Information / hd_monitor: <hostname> HD Usage
     （diagnostic_updater 强制节点名前缀），且 level ≤ 1
  B. /diagnostics_agg 出现 /A3/Host/<同名> 三项；
     /diagnostics_toplevel_state 不为 ERROR（level < 2）
  C. use_host_diagnostics:=false：10 s 内 /diagnostics 不出现任何主机状态

hardware:=mock 不触碰 CAN。
"""

import os
import signal
import socket
import subprocess
import sys
import time

WS = "/home/cat/a3_arm_ws"
LOG = "/tmp/f96_accept_stack.log"
LOG_OFF = "/tmp/f96_accept_stack_off.log"

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


# 探针：订阅 /diagnostics、/diagnostics_agg、/diagnostics_toplevel_state，
# 采集固定时长后逐行打印（octet level 归一化为 int）。
PROBE = r'''
import sys, time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

duration = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
rclpy.init()
n = Node("f96_accept_probe")

diag = {}
agg = {}
top_level = None
top_msg = ""

def norm(level):
    return level[0] if isinstance(level, (bytes, bytearray)) else level

def on_diag(m):
    for s in m.status:
        diag[s.name] = norm(s.level)

def on_agg(m):
    for s in m.status:
        agg[s.name] = norm(s.level)

def on_top(s):
    global top_level, top_msg
    top_level = norm(s.level)
    top_msg = s.message

n.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 50)
n.create_subscription(DiagnosticArray, "/diagnostics_agg", on_agg, 50)
# 注意：/diagnostics_toplevel_state 是 DiagnosticStatus（非 Array），
# 状态名固定 "toplevel_state"。
n.create_subscription(DiagnosticStatus, "/diagnostics_toplevel_state", on_top, 10)

end = time.time() + duration
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)

for name, level in sorted(diag.items()):
    print("DIAG|%s|level=%d" % (name, level))
for name, level in sorted(agg.items()):
    print("AGG|%s|level=%d" % (name, level))
print("TOP|level=%s|message=%s" % (
    "-1" if top_level is None else str(top_level), top_msg))
rclpy.shutdown()
'''


def run_probe(path, duration, timeout=60):
    r = subprocess.run(
        ["bash", "-c", ROS_WRAP + f"python3 {path} {duration}"],
        capture_output=True, text=True, timeout=timeout,
    )
    if r.returncode != 0:
        print("STDERR:", r.stderr[-1500:])
    return r.stdout


def parse(out):
    data = {"diag": {}, "agg": {}, "top": None}
    for line in out.splitlines():
        if line.startswith("DIAG|"):
            _, name, levels = line.split("|", 2)
            data["diag"][name] = int(levels.split("=")[1])
        elif line.startswith("AGG|"):
            _, name, levels = line.split("|", 2)
            data["agg"][name] = int(levels.split("=")[1])
        elif line.startswith("TOP|"):
            parts = line.split("|")
            data["top"] = int(parts[1].split("=")[1])
            data["top_msg"] = parts[2].split("=", 1)[1]
    return data


def launch_stack(extra_args="", log=LOG):
    return subprocess.Popen(
        ["bash", "-c", ROS_WRAP + "exec ros2 launch a3_bringup a3_bringup.launch.py "
         "hardware:=mock use_mqtt:=false use_rviz:=false use_teleop:=false "
         + extra_args],
        preexec_fn=os.setsid,
        stdout=open(log, "w"),
        stderr=subprocess.STDOUT,
    )


def teardown(proc, wait=8):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    time.sleep(wait)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    probe_path = "/tmp/f96_accept_probe.py"
    with open(probe_path, "w") as f:
        f.write(PROBE)

    # diagnostic_updater 强制把节点名作为前缀（stat.name = node_name + ': ' + name，
    # 除非 name 以 '/' 开头），launch 中 name= 覆盖后实际名为：
    hostname = socket.gethostname()
    expected = [
        "cpu_monitor: CPU Information",
        "ram_monitor: RAM Information",
        f"hd_monitor: {hostname} HD Usage",
    ]

    # ---- A/B. 默认开启 ----
    stack = launch_stack()
    try:
        time.sleep(12)
        data = parse(run_probe(probe_path, 12.0))

        # A. /diagnostics 三项且 level ≤ 1
        for name in expected:
            level = data["diag"].get(name)
            check(f"/diagnostics 出现 {name} 且非 ERROR",
                  level is not None and level <= 1,
                  "missing" if level is None else f"level={level}")

        # B. /diagnostics_agg /A3/Host 三项
        for name in expected:
            apath = f"/A3/Host/{name}"
            alevel = data["agg"].get(apath)
            check(f"/diagnostics_agg 出现 {apath}",
                  alevel is not None and alevel <= 1,
                  "missing" if alevel is None else f"level={alevel}")
        # mock 用 mock_components/GenericSystem，不发 a3_hardware: 诊断，
        # Hardware/Arm Monitor 分组按 F82 设计为 STALE → toplevel=3 是既有行为。
        # F96 只要求：toplevel 非 ERROR 且 Host 分组未被点名。
        check("/diagnostics_toplevel_state 非 ERROR 且 Host 分组健康",
              data["top"] is not None and data["top"] != 2
              and "Host" not in data["top_msg"],
              f"top={data['top']} msg={data['top_msg']}")
    finally:
        teardown(stack)

    time.sleep(3)

    # ---- C. use_host_diagnostics:=false ----
    stack = launch_stack("use_host_diagnostics:=false", LOG_OFF)
    try:
        time.sleep(12)
        data = parse(run_probe(probe_path, 10.0))
        leaked = [n for n in expected if n in data["diag"]]
        check("use_host_diagnostics:=false 时无主机状态上 /diagnostics",
              not leaked, f"leaked={leaked}")
    finally:
        teardown(stack)

    print()
    if failures:
        print(f"F96 验收失败：{len(failures)} 项 -- {failures}")
        return 1
    n_pass = 8
    print(f"F96 验收通过：主机资源标准诊断 {n_pass}/{n_pass}"
          "（/diagnostics 三项 / /diagnostics_agg /A3/Host 三项 / "
          "toplevel 非 ERROR / 开关关闭可摘除）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
