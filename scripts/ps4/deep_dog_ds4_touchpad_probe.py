#!/usr/bin/env python3
"""DS4 touchpad + buttons concurrent parse probe (I06 stage-1).

Reads DualShock 4 HID reports via hidapi and prints normalized axes/buttons
plus touchpad XY. Does NOT publish MQTT.

Modes:
  hid     — full snapshot from one USB report (default; proves concurrent parse)
  hybrid  — pygame for keys/axes + hidapi for touchpad only (contention check)

Dependencies:
  pip3 install hidapi
  # hybrid also needs: pip3 install pygame

Examples (prefer the Python that has hidapi, e.g. Frameworks 3.11):
  python3 scripts/deep_dog/deep_dog_ds4_touchpad_probe.py --list
  python3 scripts/deep_dog/deep_dog_ds4_touchpad_probe.py --mode hid
  python3 scripts/deep_dog/deep_dog_ds4_touchpad_probe.py --mode hybrid --hz 20
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime

# Allow `python scripts/...` from repo root
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from deep_dog_ds4_hid import (  # noqa: E402
    axis_deadzone,
    clamp,
    is_ds4,
    list_sony_devices,
    open_ds4_device,
    parse_ds4_usb_report,
    read_ds4_report,
)


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def import_hid():
    try:
        import hid  # type: ignore
    except ImportError:
        print("missing dependency: pip3 install hidapi", file=sys.stderr)
        print(f"(current interpreter: {sys.executable})", file=sys.stderr)
        sys.exit(2)
    return hid


def fmt_buttons(b: dict) -> str:
    keys = (
        "a", "b", "x", "y", "l1", "r1", "select", "start", "ps", "l3", "r3", "touch",
        "dpad_up", "dpad_down", "dpad_left", "dpad_right",
    )
    parts = [f"{k}:{1 if b.get(k) else 0}" for k in keys]
    parts.append(f"l2:{float(b.get('l2', 0)):.2f}")
    parts.append(f"r2:{float(b.get('r2', 0)):.2f}")
    return "{" + " ".join(parts) + "}"


def fmt_axes(a: dict) -> str:
    return (
        "{"
        f"lx:{float(a.get('lx', 0)):.2f} "
        f"ly:{float(a.get('ly', 0)):.2f} "
        f"rx:{float(a.get('rx', 0)):.2f} "
        f"ry:{float(a.get('ry', 0)):.2f}"
        "}"
    )


def fmt_touchpad(t: dict | None) -> str:
    if not t:
        return "{none}"
    return (
        "{"
        f"active:{1 if t.get('active') else 0} "
        f"x:{float(t.get('x', 0)):.3f} "
        f"y:{float(t.get('y', 0)):.3f} "
        f"fingers:{int(t.get('fingers', 0))}"
        "}"
    )


def snap_sig(snap: dict) -> tuple:
    b = snap.get("buttons", {})
    a = snap.get("axes", {})
    t = snap.get("touchpad") or {}
    return (
        tuple(
            bool(b.get(k))
            for k in (
                "a", "b", "x", "y", "l1", "r1", "select", "start", "ps", "l3", "r3", "touch",
                "dpad_up", "dpad_down", "dpad_left", "dpad_right",
            )
        ),
        round(float(b.get("l2", 0)), 2),
        round(float(b.get("r2", 0)), 2),
        round(float(a.get("lx", 0)), 2),
        round(float(a.get("ly", 0)), 2),
        round(float(a.get("rx", 0)), 2),
        round(float(a.get("ry", 0)), 2),
        bool(t.get("active")),
        round(float(t.get("x", 0)), 3),
        round(float(t.get("y", 0)), 3),
        int(t.get("fingers", 0)),
    )


def print_snap(mode: str, snap: dict) -> None:
    print(
        f"[{ts()}] mode={mode} "
        f"buttons={fmt_buttons(snap.get('buttons', {}))} "
        f"axes={fmt_axes(snap.get('axes', {}))} "
        f"touchpad={fmt_touchpad(snap.get('touchpad'))}"
    )


def cmd_list(hid) -> int:
    devices = list_sony_devices(hid)
    if not devices:
        print("No Sony (VID 054C) HID devices found.")
        return 1
    print(f"Sony HID devices ({len(devices)}):")
    for i, d in enumerate(devices):
        mark = "DS4" if is_ds4(d) else "other"
        path = d.get("path")
        path_s = (
            path.decode("utf-8", errors="replace")
            if isinstance(path, (bytes, bytearray))
            else str(path)
        )
        print(
            f"  [{i}] {mark} pid=0x{int(d.get('product_id') or 0):04X} "
            f"usage_page=0x{int(d.get('usage_page') or 0):04X} "
            f"usage=0x{int(d.get('usage') or 0):04X} "
            f"iface={d.get('interface_number')} "
            f"product={d.get('product_string')!r} "
            f"path={path_s}"
        )
    return 0


def open_logged(hid, path: bytes | None = None):
    dev, info = open_ds4_device(hid, path=path)
    if not info or not dev:
        print(
            "No DS4 found (PID 05C4/09CC/0BA0). Plug USB DualShock 4.",
            file=sys.stderr,
        )
        return None, None
    path_s = info["path"]
    if isinstance(path_s, (bytes, bytearray)):
        path_s = path_s.decode("utf-8", errors="replace")
    print(
        f"[{ts()}] opened DS4 pid=0x{int(info.get('product_id') or 0):04X} "
        f"product={info.get('product_string')!r} path={path_s}"
    )
    print(
        f"[{ts()}] touchpad XY: x left→right [0,1], y top→bottom [0,1]; "
        f"buttons.touch = click (separate from finger contact)"
    )
    return dev, info


def run_hid_mode(hid, args) -> int:
    path = args.path.encode("utf-8") if args.path else None
    dev, _info = open_logged(hid, path=path)
    if not dev:
        return 1

    interval = 1.0 / max(1.0, float(args.hz))
    last_sig = None
    last_print = 0.0
    raw_dump_left = 3
    meta_printed = False
    try:
        while True:
            data = read_ds4_report(dev)
            if not data:
                time.sleep(0.001)
                continue
            if raw_dump_left > 0:
                hex_part = " ".join(f"{b:02x}" for b in data[:48])
                print(f"[{ts()}] raw[{len(data)}]: {hex_part}...")
                raw_dump_left -= 1
            snap = parse_ds4_usb_report(data)
            if not snap:
                continue
            if not meta_printed:
                print(
                    f"[{ts()}] touch_off={snap.get('_touch_off')} "
                    f"pay_len={snap.get('_pay_len')}"
                )
                meta_printed = True
            now = time.time()
            sig = snap_sig(snap)
            if sig != last_sig or (now - last_print) >= max(interval, 0.5):
                print_snap("hid", snap)
                last_sig = sig
                last_print = now
            time.sleep(0.001)
    except KeyboardInterrupt:
        print(f"[{ts()}] stop")
    finally:
        try:
            dev.close()
        except Exception:
            pass
    return 0


def run_hybrid_mode(hid, args) -> int:
    try:
        import pygame
    except ImportError:
        print("hybrid mode needs pygame: pip3 install pygame", file=sys.stderr)
        return 2

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= 0:
        print("hybrid: no pygame joystick", file=sys.stderr)
        return 1
    joy = pygame.joystick.Joystick(args.joystick if args.joystick >= 0 else 0)
    joy.init()
    print(
        f"[{ts()}] pygame joystick: {joy.get_name()!r} "
        f"axes={joy.get_numaxes()} buttons={joy.get_numbuttons()}"
    )

    path = args.path.encode("utf-8") if args.path else None
    try:
        dev, _info = open_logged(hid, path=path)
    except Exception as e:
        print(f"[{ts()}] hybrid: hid open raised: {e}", file=sys.stderr)
        dev = None
    if not dev:
        print(
            f"[{ts()}] hybrid FAIL: cannot open DS4 with hidapi while pygame holds "
            f"device (contention). Prefer --mode hid for production --touchpad-xy.",
            file=sys.stderr,
        )
        pygame.quit()
        return 1

    last_sig = None
    last_print = 0.0
    interval = 1.0 / max(1.0, float(args.hz))
    touchpad: dict = {"active": False, "x": 0.0, "y": 0.0, "fingers": 0}

    try:
        while True:
            pygame.event.pump()
            n_axes = joy.get_numaxes()
            n_buttons = joy.get_numbuttons()

            def raw_ax(i: int) -> float:
                return joy.get_axis(i) if i < n_axes else 0.0

            def btn(i: int) -> bool:
                return bool(joy.get_button(i)) if i < n_buttons else False

            def trigger01(axis_i: int) -> float:
                if axis_i >= n_axes:
                    return 0.0
                return clamp((raw_ax(axis_i) + 1.0) * 0.5, 0.0, 1.0)

            snap = {
                "connected": True,
                "source": "wifi",
                "axes": {
                    "lx": axis_deadzone(raw_ax(0)),
                    "ly": axis_deadzone(-raw_ax(1)),
                    "rx": axis_deadzone(raw_ax(2)),
                    "ry": axis_deadzone(-raw_ax(3)),
                },
                "buttons": {
                    "a": btn(0),
                    "b": btn(1),
                    "x": btn(2),
                    "y": btn(3),
                    "select": btn(4),
                    "ps": btn(5),
                    "start": btn(6),
                    "l3": btn(7),
                    "r3": btn(8),
                    "l1": btn(9),
                    "r1": btn(10),
                    "dpad_up": btn(11),
                    "dpad_down": btn(12),
                    "dpad_left": btn(13),
                    "dpad_right": btn(14),
                    "touch": btn(15),
                    "l2": trigger01(4),
                    "r2": trigger01(5),
                },
                "touchpad": touchpad,
                "ts": int(time.time()),
            }

            data = read_ds4_report(dev)
            if data:
                parsed = parse_ds4_usb_report(data)
                if parsed and parsed.get("touchpad"):
                    touchpad = parsed["touchpad"]
                    snap["touchpad"] = touchpad

            now = time.time()
            sig = snap_sig(snap)
            if sig != last_sig or (now - last_print) >= max(interval, 0.5):
                print_snap("hybrid", snap)
                last_sig = sig
                last_print = now
            time.sleep(0.001)
    except KeyboardInterrupt:
        print(f"[{ts()}] stop")
    finally:
        try:
            if dev:
                dev.close()
        except Exception:
            pass
        pygame.quit()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="DS4 touchpad+buttons concurrent probe (I06)")
    ap.add_argument("--list", action="store_true", help="list Sony HID devices and exit")
    ap.add_argument("--mode", choices=("hid", "hybrid"), default="hid")
    ap.add_argument("--hz", type=float, default=20.0, help="heartbeat print Hz when idle")
    ap.add_argument("--path", default=None, help="hidapi device path (from --list)")
    ap.add_argument("--joystick", type=int, default=0, help="pygame joystick index (hybrid)")
    args = ap.parse_args()

    hid = import_hid()
    if args.list:
        return cmd_list(hid)
    if args.mode == "hybrid":
        return run_hybrid_mode(hid, args)
    return run_hid_mode(hid, args)


if __name__ == "__main__":
    sys.exit(main())
