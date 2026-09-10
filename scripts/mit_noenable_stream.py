#!/usr/bin/env python3
"""A3 电机 CAN 无使能探测与 MIT 零增益流式工具（纯标准库，无 ROS 依赖）。

协议对齐 src/a3_can_bridge/include/a3_can_bridge/protocol_codec.hpp：
  命令帧 ID:   cmd(bit24-28) | 0xFD(bit8-15) | motor_id(bit0-7)   （29bit 扩展帧）
  MIT 控制帧:  0x01<<24 | tau_u16<<8 | motor_id；p/v/kp/kd/tau 各 16bit
  反馈帧:      0x02<<24 | err(bit16-20) | mode(bit22-23) | motor_id<<8 | ...

安全语义：
  - probe   仅发 get_device_id 查询帧（cmd 0，无侵入），再用零增益 MIT 帧验证反馈。
  - stream  持续发 p=0,v=0,kp=0,kd=0,tau=0 的 MIT 帧 —— 零力矩、永不使能、可自由拖动。
  - reset   发停止帧（cmd 4，data 全 0，不触发版本上报），将电机放回失能状态。
全程不发送 cmd 3（使能），不设零点（cmd 6），不写参数。

用法：
  python3 scripts/mit_noenable_stream.py probe  [--iface can1] [--ids 1..127]
  python3 scripts/mit_noenable_stream.py stream [--iface can1] [--ids 1 2 3 4 5 6] [--hz 100]
  python3 scripts/mit_noenable_stream.py reset  [--iface can1] [--ids 1..6]
"""

import argparse
import select
import socket
import struct
import sys
import time

CAN_EFF_FLAG = 0x80000000
CAN_FRAME_FMT = "=IB3x8s"

K_P_MIN, K_P_MAX = -12.57, 12.57
K_V_MIN, K_V_MAX = -50.0, 50.0
K_T_MIN, K_T_MAX = -6.0, 6.0


def u16be(high, low):
    return (high << 8) | low


def uint_to_float(x, x_min, x_max):
    return x * (x_max - x_min) / 65535.0 + x_min


def float_to_uint(x, x_min, x_max):
    x = max(x_min, min(x, x_max))
    return int((x - x_min) / (x_max - x_min) * 65535.0)


def open_socket(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s


def send_frame(s, iface, can_id, data=b"\x00" * 8):
    """发送 29bit 扩展帧（8B）。"""
    if len(data) != 8:
        raise ValueError("frame data must be 8 bytes")
    s.send(struct.pack(CAN_FRAME_FMT, can_id | CAN_EFF_FLAG, 8, data))


def recv_frame(s, timeout=0.0):
    """非阻塞/短超时收一帧，返回 (can_id, data) 或 None。"""
    r, _, _ = select.select([s], [], [], timeout)
    if not r:
        return None
    frame, _ = s.recvfrom(16)
    can_id, _dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
    return can_id & 0x1FFFFFFF, data


def build_cmd_frame(cmd, motor_id):
    return (cmd << 24) | (0xFD << 8) | motor_id


def build_mit_zero_frame(motor_id):
    """p=0,v=0,kp=0,kd=0,tau=0 —— 零力矩 MIT 帧。"""
    tau_u16 = float_to_uint(0.0, K_T_MIN, K_T_MAX)
    can_id = (0x01 << 24) | (tau_u16 << 8) | motor_id
    data = bytes([
        0x7F, 0xFF,  # p = 0
        0x7F, 0xFF,  # v = 0
        0x00, 0x00,  # kp = 0
        0x00, 0x00,  # kd = 0
    ])
    return can_id, data


def decode_device_id_rsp(can_id, data):
    cmd = (can_id >> 24) & 0x1F
    if cmd != 0x00 or (can_id & 0xFF) != 0xFE:
        return None
    motor_id = (can_id >> 8) & 0xFF
    return {"motor_id": motor_id, "uid": data.hex()}


def decode_feedback(can_id, data):
    cmd = (can_id >> 24) & 0x1F
    if cmd not in (0x02, 0x18):
        return None
    fb = {
        "motor_id": (can_id >> 8) & 0xFF,
        "mode": (can_id >> 22) & 0x03,
        "err_voltage": bool((can_id >> 16) & 0x01),
        "err_current": bool((can_id >> 17) & 0x01),
        "err_temp": bool((can_id >> 18) & 0x01),
        "err_magnet": bool((can_id >> 19) & 0x01),
        "err_hall": bool((can_id >> 20) & 0x01),
        "angle": uint_to_float(u16be(data[0], data[1]), K_P_MIN, K_P_MAX),
        "speed": uint_to_float(u16be(data[2], data[3]), K_V_MIN, K_V_MAX),
        "torque": uint_to_float(u16be(data[4], data[5]), K_T_MIN, K_T_MAX),
        "temp": u16be(data[6], data[7]) / 10.0,
    }
    return fb


def parse_ids(spec):
    if ".." in spec:
        lo, hi = spec.split("..", 1)
        return list(range(int(lo), int(hi) + 1))
    return [int(x) for x in spec.split()]


def cmd_probe(args):
    s = open_socket(args.iface)
    print(f"[probe] {args.iface}: 向 ID {args.ids[0]}..{args.ids[-1]} 发 get_device_id 查询帧"
          f"（cmd 0，无侵入，不会使能）...")
    for mid in args.ids:
        send_frame(s, args.iface, build_cmd_frame(0x00, mid))
        time.sleep(0.004)

    deadline = time.monotonic() + 2.0
    found = {}
    while time.monotonic() < deadline:
        fr = recv_frame(s, timeout=0.1)
        if fr is None:
            continue
        rsp = decode_device_id_rsp(*fr)
        if rsp:
            found[rsp["motor_id"]] = rsp["uid"]
    if not found:
        print("[probe] 无任何应答（检查接线 / can1 状态 / 供电）。")
        return 1
    print(f"[probe] 发现 {len(found)} 个电机：")
    for mid in sorted(found):
        print(f"  motor_id={mid:>3}  uid={found[mid]}")

    # 对每个发现的电机发 5 帧零增益 MIT，验证未使能状态下是否有反馈
    print("[probe] 逐个发零增益 MIT 帧（kp=kd=tau=0，零力矩）验证反馈...")
    for mid in sorted(found):
        got = None
        for _ in range(5):
            send_frame(s, args.iface, *build_mit_zero_frame(mid))
            end = time.monotonic() + 0.15
            while time.monotonic() < end:
                fr = recv_frame(s, timeout=0.05)
                if fr is None:
                    continue
                fb = decode_feedback(*fr)
                if fb and fb["motor_id"] == mid:
                    got = fb
                    break
            if got:
                break
        if got:
            errs = "".join(k.split("_")[1][0] for k in
                           ("err_voltage", "err_current", "err_temp", "err_magnet", "err_hall")
                           if got[k]) or "-"
            print(f"  motor_id={mid:>3} 反馈OK  angle={got['angle']:+.4f} rad  "
                  f"speed={got['speed']:+.3f}  torque={got['torque']:+.3f} Nm  "
                  f"temp={got['temp']:.1f}C  mode={got['mode']}  err={errs}")
        else:
            print(f"  motor_id={mid:>3} 无反馈（未使能状态下不响应 MIT 帧）")
    return 0


def cmd_stream(args):
    s_tx = open_socket(args.iface)
    s_rx = open_socket(args.iface)
    ids = args.ids
    period = 1.0 / args.hz
    stats = {mid: {"fb": 0, "angle": None, "last": 0.0} for mid in ids}
    sent = 0
    next_tick = time.monotonic()
    t0 = time.monotonic()
    print(f"[stream] {args.iface}: ids={ids} hz={args.hz} 零增益 MIT 帧（永不使能）")
    print("[stream] Ctrl+C 退出；退出时默认对全部电机发停止帧（--no-reset 可关闭）。")
    try:
        while True:
            now = time.monotonic()
            for mid in ids:
                send_frame(s_tx, args.iface, *build_mit_zero_frame(mid))
            sent += len(ids)
            # 排空 RX：统计各电机反馈
            while True:
                fr = recv_frame(s_rx, timeout=0.0)
                if fr is None:
                    break
                fb = decode_feedback(*fr)
                if fb and fb["motor_id"] in stats:
                    st = stats[fb["motor_id"]]
                    st["fb"] += 1
                    st["angle"] = fb["angle"]
                    st["last"] = now
            if now - t0 >= 10.0:
                parts = [f"tx={sent}"]
                for mid in ids:
                    st = stats[mid]
                    ago = now - st["last"] if st["fb"] else -1.0
                    if st["fb"]:
                        parts.append(f"m{mid}:fb={st['fb']}/10s angle={st['angle']:+.3f}"
                                     f"{' (stale %.0fs)' % ago if ago > 1.0 else ''}")
                    else:
                        parts.append(f"m{mid}:无反馈")
                print("  ".join(parts))
                t0 = now
                sent = 0
                for st in stats.values():
                    st["fb"] = 0
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        if args.reset_on_exit:
            print("[stream] 退出：发停止帧（cmd 4，失能态）...")
            for mid in ids:
                for _ in range(3):
                    send_frame(s_tx, args.iface, build_cmd_frame(0x04, mid))
                    time.sleep(0.02)
        s_tx.close()
        s_rx.close()
    return 0


def cmd_reset(args):
    s = open_socket(args.iface)
    print(f"[reset] {args.iface}: 对 {args.ids} 发停止帧（cmd 4，失能态，不触发版本上报）")
    for _ in range(3):
        for mid in args.ids:
            send_frame(s, args.iface, build_cmd_frame(0x04, mid))
        time.sleep(0.05)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("command", choices=["probe", "stream", "reset"])
    ap.add_argument("--iface", default="can1")
    ap.add_argument("--ids", default="1..127", help="probe 默认 1..127；stream/reset 用如 '1 2 3 4 5 6'")
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--no-reset", action="store_true", help="stream 退出时不发停止帧")
    args = ap.parse_args()

    args.ids = parse_ids(args.ids)
    args.reset_on_exit = not args.no_reset

    try:
        if args.command == "probe":
            return cmd_probe(args)
        if args.command == "stream":
            return cmd_stream(args)
        return cmd_reset(args)
    except OSError as e:
        print(f"[error] CAN 操作失败: {e}（检查 can1 是否 UP：ip link show {args.iface}）",
              file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
