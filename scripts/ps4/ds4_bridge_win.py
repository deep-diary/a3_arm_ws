#!/usr/bin/env python3
"""DS4(蓝牙) → TCP JSON 桥接（Windows 侧发送端）。

配合 WSL 侧 a3_teleop_ps4 的 ds4_tcp_joy_node 使用：
Windows 上 pygame/SDL2 识别到的 DS4（SDL "ds4_sdl" 布局：6 轴 16 键），
归一化为 deep-dog 抽象快照（axes 右/下为正；l2/r2 ∈[0,1]；按钮具名），
以 JSON 行协议发送到 WSL 的 127.0.0.1:<port>（WSL2 自带 localhost 转发）。

布局来源：scripts/ps4/deep_dog_handle_bridge.py 的 ds4_sdl 分支（已在 macOS/板上实测）。
  axes:  0/1 左杆（L→R +1 / U→D 上为+1）；2/3 右杆；4/5 L2/R2（松开 -1 按下 +1）
  buttons: 0✕ 1○ 2□ 3△ 4 Share 5 PS 6 Options 7 L3 8 R3 9 L1 10 R1 11↑ 12↓ 13← 14→ 15 Touch

依赖（Windows python）：
  pip install pygame

用法：
  python scripts\\ps4\\ds4_bridge_win.py                 # 默认 127.0.0.1:8890
  python scripts\\ps4\\ds4_bridge_win.py --port 8890 --hz 40
  python scripts\\ps4\\ds4_bridge_win.py --dump          # 只本地打印，不发送
  python scripts\\ps4\\ds4_bridge_win.py --list          # 列出手柄
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from datetime import datetime
from typing import Any, Optional


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def axis_deadzone(v: float, dz: float = 0.08) -> float:
    return 0.0 if abs(v) < dz else clamp(v, -1.0, 1.0)


def trigger01(v: float) -> float:
    """SDL 扳机（松开 -1 按下 +1）→ 0..1（静息 0）。"""
    return clamp((v + 1.0) * 0.5, 0.0, 1.0)


def require_pygame():
    try:
        import pygame
    except ImportError:
        print("missing dependency: pip3 install pygame", file=sys.stderr)
        sys.exit(2)
    return pygame


def list_joysticks(pygame) -> int:
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    print(f"joysticks: {count}")
    for i in range(count):
        j = pygame.joystick.Joystick(i)
        j.init()
        print(
            f"  [{i}] {j.get_name()!r} axes={j.get_numaxes()} "
            f"buttons={j.get_numbuttons()} hats={j.get_numhats()}"
        )
    return 0 if count > 0 else 1


def read_snapshot(joy, pygame) -> Optional[dict]:
    """pygame joystick → deep-dog 抽象快照（ds4_sdl 布局）。"""
    pygame.event.pump()
    n_axes = joy.get_numaxes()
    n_buttons = joy.get_numbuttons()
    n_hats = joy.get_numhats()

    def raw_ax(i: int) -> float:
        return joy.get_axis(i) if i < n_axes else 0.0

    def btn(i: int) -> bool:
        return bool(joy.get_button(i)) if i < n_buttons else False

    # ds4_sdl（Windows SDL2 实测）：LX/RX 右=+1；LY/RY 上=-1（与 macOS SDL 相反，
    # macOS 上=+1 需取反，Windows 无需）→ 抽象统一为右/下为正（down=+1）
    lx = axis_deadzone(raw_ax(0))
    ly = axis_deadzone(raw_ax(1))
    rx = axis_deadzone(raw_ax(2))
    ry = axis_deadzone(raw_ax(3))
    l2 = trigger01(raw_ax(4))
    r2 = trigger01(raw_ax(5))

    a, b = btn(0), btn(1)
    x, y = btn(2), btn(3)  # □=x, △=y
    select, ps, start = btn(4), btn(5), btn(6)
    l3, r3 = btn(7), btn(8)
    l1, r1 = btn(9), btn(10)
    dpad_up, dpad_down = btn(11), btn(12)
    dpad_left, dpad_right = btn(13), btn(14)
    touch = btn(15)

    dx = dy = 0.0
    if n_hats > 0:
        hx, hy = joy.get_hat(0)
        dx, dy = float(hx), float(hy)
    else:
        # 无 hat（Windows DS4 v1 蓝牙实测 hats=0）：十字键在按钮 11-14
        dx = (1.0 if dpad_right else 0.0) - (1.0 if dpad_left else 0.0)
        dy = (1.0 if dpad_down else 0.0) - (1.0 if dpad_up else 0.0)

    return {
        "connected": True,
        "source": "wsl-bridge",
        "axes": {"lx": lx, "ly": ly, "rx": rx, "ry": ry},
        "buttons": {
            "a": a,
            "b": b,
            "x": x,
            "y": y,
            "l1": l1,
            "r1": r1,
            "l2": l2,
            "r2": r2,
            "select": select,
            "start": start,
            "ps": ps,
            "l3": l3,
            "r3": r3,
            "touch": touch,
            "dpad_up": dpad_up,
            "dpad_down": dpad_down,
            "dpad_left": dpad_left,
            "dpad_right": dpad_right,
        },
        "dpad": {"dx": dx, "dy": dy},
        "ts": int(time.time()),
    }


def open_joystick(pygame, index: int, hard: bool = False):
    if hard:
        pygame.joystick.quit()
        pygame.joystick.init()
        pygame.event.clear()
    elif not pygame.joystick.get_init():
        pygame.joystick.init()
    if pygame.joystick.get_count() <= 0:
        return None
    idx = index if 0 <= index < pygame.joystick.get_count() else 0
    joy = pygame.joystick.Joystick(idx)
    joy.init()
    pygame.event.clear()
    return joy


def joystick_alive(joy, pygame) -> bool:
    if joy is None:
        return False
    try:
        if not pygame.joystick.get_init() or pygame.joystick.get_count() <= 0:
            return False
        _ = joy.get_numaxes()
        return True
    except Exception:
        return False


def offline_snapshot() -> dict:
    empty = {k: False for k in (
        "a", "b", "x", "y", "l1", "r1", "select", "start", "ps", "l3", "r3", "touch",
        "dpad_up", "dpad_down", "dpad_left", "dpad_right",
    )}
    empty.update({"l2": 0.0, "r2": 0.0})
    return {
        "connected": False,
        "source": "wsl-bridge",
        "axes": {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0},
        "buttons": empty,
        "dpad": {"dx": 0.0, "dy": 0.0},
        "ts": int(time.time()),
    }


def snapshot_equal(a: Optional[dict], b: Optional[dict], eps: float = 0.02) -> bool:
    if a is None or b is None:
        return False
    if a.get("connected") != b.get("connected"):
        return False
    aa, ba = a.get("axes", {}), b.get("axes", {})
    for k in ("lx", "ly", "rx", "ry"):
        if abs(float(aa.get(k, 0)) - float(ba.get(k, 0))) > eps:
            return False
    ab, bb = a.get("buttons", {}), b.get("buttons", {})
    for k in ab:
        if abs(float(ab.get(k, 0)) - float(bb.get(k, 0))) > eps:
            return False
    ad, bd = a.get("dpad", {}), b.get("dpad", {})
    if abs(float(ad.get("dx", 0)) - float(bd.get("dx", 0))) > eps:
        return False
    if abs(float(ad.get("dy", 0)) - float(bd.get("dy", 0))) > eps:
        return False
    return True


def connect_sock(host: str, port: int, timeout_s: float = 3.0) -> Optional[socket.socket]:
    try:
        s = socket.create_connection((host, port), timeout=timeout_s)
        s.settimeout(5.0)
        return s
    except OSError as e:
        print(f"[{ts()}] connect {host}:{port} failed: {e}", file=sys.stderr)
        return None


def send_snap(s: socket.socket, snap: dict) -> bool:
    try:
        s.sendall((json.dumps(snap, separators=(",", ":")) + "\n").encode("utf-8"))
        return True
    except OSError as e:
        print(f"[{ts()}] send failed: {e}", file=sys.stderr)
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description="DS4(蓝牙) → TCP JSON 桥接（Windows 侧）")
    ap.add_argument("--host", default="127.0.0.1", help="WSL 侧 ds4_tcp_joy_node 监听地址（WSL2 localhost 转发）")
    ap.add_argument("--port", type=int, default=8890)
    ap.add_argument("--joystick", type=int, default=0)
    ap.add_argument("--hz", type=float, default=40.0, help="变化时最大发送频率")
    ap.add_argument("--heartbeat-ms", type=int, default=200, help="空闲心跳间隔")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--dump", action="store_true", help="只本地打印，不发送")
    args = ap.parse_args()

    pygame = require_pygame()
    if args.list:
        return list_joysticks(pygame)

    pygame.init()
    pygame.joystick.init()

    joy = open_joystick(pygame, args.joystick)
    if joy is None:
        print(f"[{ts()}] no joystick; waiting… (plug/connect DS4, --list to see devices)")
    if not args.dump:
        sock = connect_sock(args.host, args.port)
        if sock is None:
            print(f"[{ts()}] cannot connect to WSL bridge at {args.host}:{args.port}; retrying…")

    interval = 1.0 / max(args.hz, 1.0)
    heartbeat_s = max(args.heartbeat_ms, 50) / 1000.0
    reconnect_s = 1.0
    last_pub = 0.0
    last_snap: Optional[dict] = None
    last_rescan = 0.0
    pad_online = joy is not None

    print(f"[{ts()}] DS4 bridge running → {args.host}:{args.port} "
          f"(change@{args.hz:.0f}Hz, heartbeat {args.heartbeat_ms}ms)")

    try:
        while True:
            now = time.time()

            if not joystick_alive(joy, pygame):
                if pad_online:
                    print(f"[{ts()}] pad lost — waiting…")
                    pad_online = False
                    last_snap = None
                    if not args.dump and sock is not None:
                        send_snap(sock, offline_snapshot())
                if now - last_rescan >= reconnect_s:
                    last_rescan = now
                    joy = open_joystick(pygame, args.joystick, hard=True)
                    if joy is not None:
                        print(f"[{ts()}] pad reconnected: {joy.get_name()!r}")
                        pad_online = True
                if not args.dump and sock is None and now - last_rescan >= reconnect_s:
                    sock = connect_sock(args.host, args.port)
                time.sleep(0.05)
                continue

            snap = read_snapshot(joy, pygame)
            if snap is None:
                time.sleep(0.01)
                continue

            if args.dump:
                b = snap["buttons"]
                a = snap["axes"]
                if not snapshot_equal(snap, last_snap):
                    print(
                        f"[{ts()}] lx={a['lx']:+.2f} ly={a['ly']:+.2f} "
                        f"rx={a['rx']:+.2f} ry={a['ry']:+.2f} "
                        f"l2={b['l2']:.2f} r2={b['r2']:.2f} "
                        f"a={int(b['a'])} b={int(b['b'])} x={int(b['x'])} y={int(b['y'])} "
                        f"dpad={snap['dpad']}"
                    )
                    last_snap = snap
                time.sleep(0.02)
                continue

            if sock is None:
                if now - last_rescan >= reconnect_s:
                    last_rescan = now
                    sock = connect_sock(args.host, args.port)
                time.sleep(0.05)
                continue

            pad_online = True
            changed = not snapshot_equal(snap, last_snap)
            due_change = changed and (now - last_pub >= interval)
            due_heartbeat = (now - last_pub) >= heartbeat_s
            if due_change or due_heartbeat:
                if not send_snap(sock, snap):
                    try:
                        sock.close()
                    except OSError:
                        pass
                    sock = None
                    last_pub = 0.0
                else:
                    last_pub = now
                    last_snap = snap
            time.sleep(0.01)
    except KeyboardInterrupt:
        print(f"\n[{ts()}] stopping")
        if not args.dump and sock is not None:
            try:
                send_snap(sock, offline_snapshot())
                sock.close()
            except OSError:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
