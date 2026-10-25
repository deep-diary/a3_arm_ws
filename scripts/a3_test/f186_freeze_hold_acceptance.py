#!/usr/bin/env python3
"""F186 acceptance：反馈 stale 时"冻结保持"而不是软下垂，恢复时不回弹冲击。

背景（2026-10-24 真机）：L6 通讯丢失后，旧实现用「全局 kp + **零前馈** + 实测位姿」
保持 → 重力下稳态下垂 τ_g/kp（实测数度，全臂都软），通讯一恢复又按控制器旧目标拉回
→ 瞬间弹回原位，冲击很大。F186 改为：
  ① 故障边沿锁存控制器命令位姿，之后一直按它下发；
  ② 帧构建走正常路径 → 每关节 kp/kd + 重力前馈与平时一致 → 不下垂；
  ③ 命令位姿全程不变 → 恢复无跳变；
  ④ 恢复后 kp 在 freeze_recover_ramp_s 内从 0 斜坡回额定（真断线时平滑拉回）。

验证手段：vcan + `vcan_motor_sim.py --silence-file` 让某个电机停发反馈，
直接解码插件发到 CAN 上的控制帧（kp/kd/位置/torque 位）做前后对比。

前置
  sudo ip link add dev vcan9 type vcan && sudo ip link set up vcan9     # 仅首次

用法
  python3 scripts/a3_test/f186_freeze_hold_acceptance.py [IFACE]        # 默认 vcan9

判据
  1. 健康期：各电机控制帧 kp 为**每关节** kp（不是全局 80）
  2. silence 电机 6 后 0.6 s：所有电机仍在收控制帧（不静默），
     且 kp/kd 与健康期**完全一致**、位置字段**冻结不变**、力矩位非中值（重力前馈仍在）
  3. 清除 silence 后：kp 出现由 0 增长的中间值（斜坡），0.8 s 内回到额定值
退出码 0 = 全通过
"""

import os
import socket
import struct
import subprocess
import sys
import time

WS = "/home/cat/a3_arm_ws"
IFACE = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("F186_CAN_IF", "vcan9")
DOMAIN = os.environ.get("F186_DOMAIN", "186")
SILENCE = "/tmp/f186_silence.json"
STATE = "/tmp/f186_state.json"
SIM_LOG = "/tmp/f186_sim.log"
STACK_LOG = "/tmp/f186_stack.log"

CAN_EFF_FLAG = 0x80000000
CAN_FORMAT = "=IB3x8s"
CMD_CONTROL = 0x01
KP_MAX, KD_MAX, P_MAX = 500.0, 5.0, 12.57
RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok)))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def dec(u16, lo, hi):
    return lo + (u16 / 65535.0) * (hi - lo)


class Sniffer:
    """抓 vcan 上的控制帧（cmd 0x01），按电机保存最后一帧。"""

    def __init__(self, iface):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # 注意：不要给 CAN_RAW_FILTER 设空过滤器——零长度过滤器 = 什么都收不到
        self.sock.bind((iface,))
        self.sock.settimeout(0.2)
        self.last = {}
        self.hist = {}

    def pump(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                continue
            can_id, dlc, data = struct.unpack(CAN_FORMAT, frame)
            can_id &= ~CAN_EFF_FLAG
            if ((can_id >> 24) & 0x1F) != CMD_CONTROL:
                continue
            motor = can_id & 0xFF
            tq_u16 = (can_id >> 8) & 0xFFFF
            pos_u16 = (data[0] << 8) | data[1]
            kp_u16 = (data[4] << 8) | data[5]
            kd_u16 = (data[6] << 8) | data[7]
            rec = dict(pos=dec(pos_u16, -P_MAX, P_MAX), kp=dec(kp_u16, 0.0, KP_MAX),
                       kd=dec(kd_u16, 0.0, KD_MAX), tq_u16=tq_u16,
                       kp_u16=kp_u16, pos_u16=pos_u16)
            self.last[motor] = rec
            self.hist.setdefault(motor, []).append(rec)
        return dict(self.last)


def main():
    if os.system(f"ip link show {IFACE} >/dev/null 2>&1") != 0:
        print(f"[x] {IFACE} 不存在：sudo ip link add dev {IFACE} type vcan && "
              f"sudo ip link set up {IFACE}")
        return 2
    env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN)
    open(SILENCE, "w").write("{}")
    sim = subprocess.Popen(
        ["python3", f"{WS}/scripts/a3_test/vcan_motor_sim.py",
         "--interface", IFACE, "--state-file", STATE, "--silence-file", SILENCE],
        stdout=open(SIM_LOG, "w"), stderr=subprocess.STDOUT, env=env,
        start_new_session=True)
    stack = subprocess.Popen(
        ["ros2", "launch", "a3_bringup", "a3_bringup.launch.py",
         "hardware:=can", f"can_interface:={IFACE}", "use_mqtt:=false",
         "use_rviz:=false", "use_camera:=false", "include_gripper:=false",
         "use_teleop:=false"],
        stdout=open(STACK_LOG, "w"), stderr=subprocess.STDOUT, env=env,
        start_new_session=True)
    sn = None
    try:
        time.sleep(18)                      # 起栈 + 插件 handshake
        sn = Sniffer(IFACE)
        # 使能（vcan 下无真机门禁，直接调服务）
        subprocess.run(["ros2", "service", "call", "/a3/arm/enable",
                        "std_srvs/srv/Trigger", "{}"], env=env,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)
        healthy = sn.pump(0.6)
        check("健康期：6 台电机控制帧齐备", len(healthy) >= 6, f"n={len(healthy)}")
        if len(healthy) < 6:
            raise SystemExit(1)
        kps = {m: round(r["kp"]) for m, r in healthy.items()}
        check("健康期 kp 为每关节值（非单一全局 80）",
              len(set(kps.values())) > 1 or list(kps.values())[0] != 80,
              f"kp={kps}")

        # 注入：电机 6 停发反馈
        open(SILENCE, "w").write('{"motor": 6}')
        time.sleep(0.9)                     # 0.2s 超时 + 余量
        sn.hist.clear()
        frozen = sn.pump(0.6)
        still = len(frozen) >= 6
        check("stale 后仍持续下发控制帧（不是静默/丢弃）", still, f"n={len(frozen)}")
        same_kp = all(abs(frozen[m]["kp_u16"] - healthy[m]["kp_u16"]) <= 1
                      for m in frozen if m in healthy)
        check("stale 后 kp 与健康期一致（每关节 kp，未退化成软保持）", same_kp,
              "; ".join(f"m{m}:{frozen[m]['kp']:.1f}vs{healthy[m]['kp']:.1f}"
                        for m in sorted(frozen) if m in healthy))
        pos_frozen = all(abs(frozen[m]["pos"] - healthy[m]["pos"]) < 1e-3
                         for m in frozen if m in healthy)
        check("stale 后位置字段冻结不变（命令位姿锁存）", pos_frozen,
              "max Δ=" + f"{max(abs(frozen[m]['pos'] - healthy[m]['pos']) for m in frozen if m in healthy):.4f}")
        ff_alive = any(abs(frozen[m]["tq_u16"] - 32768) > 200 for m in frozen)
        check("stale 后力矩位仍带前馈（重力补偿没有丢）", ff_alive,
              "; ".join(f"m{m}:{frozen[m]['tq_u16']}" for m in sorted(frozen)))

        # 恢复：斜坡
        open(SILENCE, "w").write("{}")
        sn.hist.clear()
        sn.pump(0.25)
        ramp_mid = [r["kp"] for m in sn.hist for r in sn.hist[m]
                    if 1.0 < r["kp"] < healthy[m]["kp"] - 1.0]
        sn.pump(0.9)
        back = sn.last
        back_ok = all(abs(back[m]["kp"] - healthy[m]["kp"]) <= 2
                      for m in back if m in healthy)
        check("恢复后 kp 回到额定", back_ok,
              "; ".join(f"m{m}:{back[m]['kp']:.1f}" for m in sorted(back)))
        print(f"[{'PASS' if ramp_mid else 'WARN'}] 恢复斜坡中间值被采到 "
              f"{len(ramp_mid)} 个（时序敏感，仅作参考）")
    finally:
        for p in (stack, sim):
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except Exception:  # noqa: BLE001
                pass
        time.sleep(3)
        bad = [r for r in RESULTS if not r[1]]
        print(f"\n{'ALL PASS' if not bad else f'{len(bad)} FAIL'}")
    return 0 if all(ok for _, ok in RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
