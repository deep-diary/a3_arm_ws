#!/usr/bin/env python3
"""
F93 systemd 托管验收（mock 模式全自动）。

验收步骤（对应 docs/edge/REQUIREMENTS.md F93）：
  1. systemd-analyze verify 单元文件
  2. 安装单元；确认 is-enabled=disabled、is-active=inactive
  3. mock drop-in 启动：joint_states 频率 + /a3/arm/enable 后三个控制器 active
  4. SIGKILL 整个服务模拟崩溃 → 自动重启、joint_states 恢复、NRestarts 增长
  5. stop 无残留进程；删除 drop-in；最终 disabled/inactive

需要 sudo（安装/启停服务）。hardware:=mock 不触碰 CAN，可在机械臂断电下运行。
"""

import os
import subprocess
import sys
import time

WS = "/home/cat/a3_arm_ws"
SERVICE = "a3-arm"
UNIT_SRC = os.path.join(WS, "systemd", "a3-arm.service")
UNIT_DST = "/etc/systemd/system/a3-arm.service"
DROPIN = "/etc/systemd/system/a3-arm.service.d/zz-f93-mock.conf"
DROPIN_DIR = os.path.dirname(DROPIN)

DROPIN_TEXT = """\
[Service]
ExecStart=
ExecStart=/usr/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/cat/a3_arm_ws/install/local_setup.bash && exec ros2 launch a3_bringup a3_bringup.launch.py hardware:=mock use_mqtt:=false use_rviz:=false use_teleop:=false'
"""

ROS_WRAP = (
    "source /opt/ros/humble/setup.bash && "
    f"source {WS}/install/local_setup.bash && "
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
)

failures = []


def check(name, ok, detail=""):
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {name}" + (f" -- {detail}" if detail else ""))
    if not ok:
        failures.append(name)


_ASKPASS = None


def sudo():
    # askpass 方式：非交互、无 tty 也可用（密码经 A3_SUDO_PASS 环境变量）。
    return ["sudo", "-A"]


def run(cmd, timeout=30):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def systemctl(*args, timeout=30, as_root=False):
    cmd = sudo() + ["systemctl"] if as_root else ["systemctl"]
    return run(cmd + list(args), timeout=timeout)


def active_state():
    r = systemctl("is-active", SERVICE)
    return r.stdout.strip()


def wait_joint_states(timeout=90.0):
    """阻塞等到收到一帧 /joint_states。"""
    # Humble 的 ros2 topic echo 没有 --timeout（Iron 才加入），用 coreutils timeout。
    probe = "timeout 8 ros2 topic echo --once /joint_states"
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = run(
            ["bash", "-c", ROS_WRAP + "exec " + probe],
            timeout=15,
        )
        if r.returncode == 0 and "L1_joint" in r.stdout:
            return True
        state = active_state()
        if state not in ("active", "activating"):
            print(f"  服务状态异常: {state}")
            return False
        time.sleep(2.0)
    return False


def count_joint_states(window=3.0):
    script = (
        "import time, rclpy\n"
        "from rclpy.node import Node\n"
        "from sensor_msgs.msg import JointState\n"
        "rclpy.init()\n"
        "n = Node('f93_probe')\n"
        "c = [0]\n"
        "n.create_subscription(JointState, '/joint_states',\n"
        "                      lambda m: c.__setitem__(0, c[0] + 1), 10)\n"
        f"t = time.time()\n"
        "while time.time() - t < %.1f:\n"
        "    rclpy.spin_once(n, timeout_sec=0.1)\n"
        "print(c[0])\n"
        "rclpy.shutdown()\n" % window
    )
    r = run(["bash", "-c", ROS_WRAP + "python3 - <<'PY'\n" + script + "PY"],
            timeout=20)
    if r.returncode != 0:
        print(r.stderr.strip()[-500:])
        return 0
    return int(r.stdout.strip())


ANSI_RE = __import__("re").compile(r"\x1b\[[0-9;]*m")


def list_controllers():
    r = run(
        ["bash", "-c", ROS_WRAP + "exec ros2 control list_controllers"],
        timeout=20,
    )
    # ros2controlcli 无条件输出 ANSI 颜色，startswith 匹配前先剥掉。
    return ANSI_RE.sub("", r.stdout)


def enable_fsm():
    """调用 /a3/arm/enable（mock 安全）：编排层 switch_controller 激活运动控制器。"""
    script = (
        "import rclpy, sys\n"
        "from std_srvs.srv import Trigger\n"
        "rclpy.init()\n"
        "n = rclpy.create_node('f93_enable')\n"
        "c = n.create_client(Trigger, '/a3/arm/enable')\n"
        "if not c.wait_for_service(timeout_sec=10):\n"
        "    print('enable service unavailable'); sys.exit(2)\n"
        "f = c.call_async(Trigger.Request())\n"
        "rclpy.spin_until_future_complete(n, f, timeout_sec=20)\n"
        "r = f.result()\n"
        "print(r.message if r else 'no response')\n"
        "sys.exit(0 if r and r.success else 1)\n"
    )
    return run(["bash", "-c", ROS_WRAP + "python3 - <<'PY'\n" + script + "PY"],
               timeout=40)


def wait_controllers_active(names, timeout=30.0):
    deadline = time.time() + timeout
    ctl = ""
    while time.time() < deadline:
        ctl = list_controllers()
        if all(
            " active" in next((ln for ln in ctl.splitlines()
                               if ln.startswith(name)), "")
            for name in names
        ):
            return True, ctl
        time.sleep(2.0)
    return False, ctl


def pgrep_stack():
    """当前属于产品栈的残留进程（启动前基线之外的 launch / controller_manager）。"""
    out = set()
    for pat in ("a3_bringup.launch", "controller_manager", "move_group"):
        r = run(["pgrep", "-f", pat], timeout=5)
        if r.returncode == 0:
            out.update(p for p in r.stdout.split())
    return out


def main():
    env = dict(os.environ, PYTHONNOUSERSITE="1")
    os.environ["PYTHONNOUSERSITE"] = "1"

    # 配置 askpass（非交互 sudo；密码只从 A3_SUDO_PASS 读）
    global _ASKPASS
    password = os.environ.get("A3_SUDO_PASS")
    if not password:
        print("需要设置 A3_SUDO_PASS 环境变量后重跑本脚本")
        return 2
    _ASKPASS = f"/tmp/.f93_askpass_{os.getpid()}"
    with open(_ASKPASS, "w") as f:
        f.write("#!/bin/sh\nprintf '%s\\n' \"$A3_SUDO_PASS\"\n")
    os.chmod(_ASKPASS, 0o700)
    os.environ["SUDO_ASKPASS"] = _ASKPASS

    # 校验 sudo 可用
    r = run(sudo() + ["true"], timeout=5)
    if r.returncode != 0:
        print("sudo 校验失败，检查 A3_SUDO_PASS")
        return 2

    # 启动前记录基线进程（避免把别人的栈算进残留）
    baseline = pgrep_stack()

    # --- 1. systemd-analyze verify ---
    r = run(["systemd-analyze", "verify", UNIT_SRC], timeout=20)
    errors = [ln for ln in r.stderr.splitlines()
              if "ERROR" in ln or ln.strip().endswith("failed")]
    check("systemd-analyze verify 无 error", not errors,
          "; ".join(errors) if errors else "exit 0")

    # --- 2. 安装 + 默认禁用 ---
    r = run(sudo() + ["install", "-m", "0644", UNIT_SRC, UNIT_DST], timeout=10)
    check("安装单元到 /etc/systemd/system", r.returncode == 0, r.stderr.strip())
    systemctl("daemon-reload", as_root=True)
    r = systemctl("is-enabled", SERVICE)
    check("默认 is-enabled=disabled", r.stdout.strip() == "disabled",
          r.stdout.strip())
    check("默认 is-active=inactive", active_state() in ("inactive", "deactivating"),
          active_state())

    # --- 3. mock drop-in 启动 ---
    run(sudo() + ["mkdir", "-p", DROPIN_DIR], timeout=5)
    proc = subprocess.Popen(
        sudo() + ["tee", DROPIN], stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL, text=True,
    )
    proc.communicate(DROPIN_TEXT)
    systemctl("daemon-reload", as_root=True)
    r = systemctl("start", SERVICE, timeout=30, as_root=True)
    check("systemctl start（mock drop-in）", r.returncode == 0, r.stderr.strip())

    up = wait_joint_states(90.0)
    check("90 s 内 /joint_states 恢复", up)

    if up:
        cnt = count_joint_states()
        check("3 s 窗口 joint_states ≥100 条", cnt >= 100,
              f"count={cnt}")

        # arm_controller/gripper_controller 按设计 inactive 启动，编排层 enable 才激活。
        er = enable_fsm()
        check("/a3/arm/enable 成功（mock）", er.returncode == 0,
              er.stdout.strip() or er.stderr.strip()[-200:])
        ok_active, ctl = wait_controllers_active(
            ("arm_controller", "gripper_controller",
             "joint_state_broadcaster"))
        for name in ("arm_controller", "gripper_controller",
                     "joint_state_broadcaster"):
            line = next((ln for ln in ctl.splitlines() if ln.startswith(name)), "")
            check(f"控制器 {name} active", " active" in line, line.strip())

    # --- 4. SIGKILL 崩溃 → 自动重启 ---
    n0 = systemctl("show", "-p", "NRestarts", "--value", SERVICE).stdout.strip()
    r = systemctl("kill", "-s", "SIGKILL", SERVICE, timeout=10, as_root=True)
    check("systemctl kill -s SIGKILL", r.returncode == 0, r.stderr.strip())

    restarted = False
    deadline = time.time() + 60
    while time.time() < deadline:
        n1 = systemctl("show", "-p", "NRestarts", "--value",
                       SERVICE).stdout.strip()
        if n1.isdigit() and n0.isdigit() and int(n1) > int(n0):
            restarted = True
            break
        time.sleep(1.0)
    check("systemd 自动重启（NRestarts 增长）", restarted,
          f"{n0} -> {n1 if 'n1' in locals() else '?'}")

    recovered = wait_joint_states(120.0)
    check("重启后 120 s 内 joint_states 恢复", recovered)
    if recovered:
        cnt = count_joint_states()
        check("重启后 3 s 窗口 ≥100 条", cnt >= 100, f"count={cnt}")

    # --- 5. stop + 清理 ---
    systemctl("stop", SERVICE, timeout=30, as_root=True)
    clean = False
    for _ in range(10):
        leftover = pgrep_stack() - baseline
        if not leftover and active_state() == "inactive":
            clean = True
            break
        time.sleep(1.0)
    check("stop 后无残留进程", clean,
          f"leftover={sorted(pgrep_stack() - baseline)}")

    run(sudo() + ["rm", "-rf", DROPIN_DIR], timeout=5)
    systemctl("daemon-reload", as_root=True)
    r = systemctl("is-enabled", SERVICE)
    check("最终 is-enabled=disabled（全程未 enable）",
          r.stdout.strip() == "disabled", r.stdout.strip())
    check("最终 is-active=inactive", active_state() == "inactive",
          active_state())

    if _ASKPASS and os.path.exists(_ASKPASS):
        os.remove(_ASKPASS)

    print()
    if failures:
        print(f"F93 验收失败：{len(failures)} 项 -- {failures}")
        return 1
    print("F93 验收通过：systemd 托管 mock 全链路（安装/禁用/起栈/崩溃重启/清理）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
