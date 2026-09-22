#!/usr/bin/env python3
"""vcan0 simulator for the 7 RobStride MIT motors of the A3 arm.

Speaks the project's verified CAN-id convention (see a3_hardware_interface
protocol_codec.hpp):
  RX cmd frames: id = (cmd_type<<24) | ... | motor_id(low byte)
                 control (0x01): bits 8-23 carry torque_ff u16
  TX feedback:   id = (0x02<<24) | (motor_id<<8) | 0xFD
Behaviour: first-order position follow on control frames; type-2 feedback
on reset/enable/control. Initial angles match the URDF home pose through
the joint direction mapping (motor 2=+0.785, motor 3=+0.785).
"""

import argparse
import json
import os
import socket
import struct
import time

CAN_EFF_FLAG = 0x80000000
CAN_FORMAT = "=IB3x8s"

CMD_CONTROL = 0x01
CMD_FEEDBACK = 0x02
CMD_ENABLE = 0x03
CMD_RESET = 0x04
CMD_GET_PARAM = 0x11
CMD_SET_PARAM = 0x12
MASTER_ID = 0xFD

PARAM_CAN_TIMEOUT = 0x7028
# 0x7028 is uint32 with ~50 us per count (20000 ≈ 1 s)
TIMEOUT_COUNTS_PER_SEC = 20000.0

P_RANGE = 12.57
# motor_id -> (torque_max, speed_max); RS00 1-3, EL05 4-7
TORQUE_MAX = {i: (14.0 if i <= 3 else 6.0) for i in range(1, 8)}
SPEED_MAX = {i: (33.0 if i <= 3 else 50.0) for i in range(1, 8)}
# Initial motor angles at joint home [0, .785, -.785, 0,0,0,0]
INIT_ANGLE = {1: 0.0, 2: 0.785, 3: 0.785, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0}


def float_to_u16(value, lo, hi):
    norm = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    return int(norm * 65535)


def u16_to_float(raw, lo, hi):
    return lo + (raw / 65535.0) * (hi - lo)


class MotorState:
    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.angle = INIT_ANGLE[motor_id]
        self.speed = 0.0
        self.torque = 0.0
        self.last_t = time.monotonic()
        # F86: last Type-18 0x7028 value (uint32 counts; 0 = disarmed)
        self.timeout_counts = 0
        self.tripped = False
        self.trip_delay = None


def write_state_file(path, motors):
    payload = {
        "motors": {
            str(mid): {
                "armed": m.timeout_counts > 0,
                "counts": m.timeout_counts,
                "tripped": m.tripped,
                "trip_delay": m.trip_delay,
            }
            for mid, m in sorted(motors.items())
        }
    }
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, path)


def send_param_reply(sock, m, param_id, value):
    can_id = (CMD_GET_PARAM << 24) | (m.motor_id << 8) | MASTER_ID
    data = [
        param_id & 0xFF, (param_id >> 8) & 0xFF, 0x00, 0x00,
        value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ]
    sock.send(pack_frame(can_id, data))


def pack_frame(can_id, data):
    return struct.pack(CAN_FORMAT, can_id | CAN_EFF_FLAG, 8, bytes(data))


def send_feedback(sock, m, health):
    can_id = (CMD_FEEDBACK << 24) | (m.motor_id << 8) | MASTER_ID
    if health.motor == m.motor_id:
        can_id |= (int(health.fault) & 0x3F) << 16
        can_id |= (int(health.mode) & 0x3) << 22
    tmax = TORQUE_MAX[m.motor_id]
    vmax = SPEED_MAX[m.motor_id]
    data = [0] * 8
    p = float_to_u16(m.angle, -P_RANGE, P_RANGE)
    v = float_to_u16(m.speed, -vmax, vmax)
    t = float_to_u16(m.torque, -tmax, tmax)
    temp_c = health.temp if health.motor == m.motor_id else 30.0
    temp = max(0, min(65535, int(temp_c * 10)))
    raw = ((p & 0xFFFF).to_bytes(2, "big") + (v & 0xFFFF).to_bytes(2, "big") +
           (t & 0xFFFF).to_bytes(2, "big") + (temp & 0xFFFF).to_bytes(2, "big"))
    sock.send(pack_frame(can_id, raw))


class ExternalForce:
    """Operator push in motor coordinates, read from a JSON file.

    Format: {"motor": 3, "torque": 0.6}; {} clears. Only applied in effort
    mode (control frames with kp~0), where the controller's t_ff already
    cancels gravity, so the environment load is -t_ff + ext.
    """

    def __init__(self, path):
        self.path = path
        self.mtime = 0.0
        self.last_check = 0.0
        self.motor = None
        self.torque = 0.0

    def poll(self):
        now = time.monotonic()
        if now - self.last_check < 0.05:
            return
        self.last_check = now
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return
        if mtime == self.mtime:
            return
        self.mtime = mtime
        try:
            with open(self.path) as f:
                data = json.load(f)
            self.motor = int(data["motor"])
            self.torque = float(data["torque"])
        except (OSError, ValueError, KeyError, TypeError):
            self.motor = None
            self.torque = 0.0


class SilenceCtl:
    """Per-motor feedback TX kill switch, read from a JSON file.

    Format: {"motor": 4}; {} clears. The motor still receives and executes
    control frames, only type-2 feedback TX is suppressed — emulating a dead
    feedback channel (F81).
    """

    def __init__(self, path):
        self.path = path
        self.mtime = 0.0
        self.last_check = 0.0
        self.motor = None

    def poll(self):
        now = time.monotonic()
        if now - self.last_check < 0.05:
            return
        self.last_check = now
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return
        if mtime == self.mtime:
            return
        self.mtime = mtime
        try:
            with open(self.path) as f:
                data = json.load(f)
            self.motor = int(data["motor"]) if data else None
        except (OSError, ValueError, KeyError, TypeError):
            self.motor = None


class HealthCtl:
    """Per-motor temperature/mode/fault-word injection, read from JSON file.

    Format: {"motor": 3, "temp_c": 92.0, "fault": 4, "mode": 1}; {} clears.
    Temp goes into feedback data[6:8] (x10), fault into CAN-id bits 16-21,
    mode into bits 22-23 (F84).
    """

    def __init__(self, path):
        self.path = path
        self.mtime = 0.0
        self.last_check = 0.0
        self.motor = None
        self.temp = 30.0
        self.fault = 0
        self.mode = 0

    def poll(self):
        now = time.monotonic()
        if now - self.last_check < 0.05:
            return
        self.last_check = now
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return
        if mtime == self.mtime:
            return
        self.mtime = mtime
        try:
            with open(self.path) as f:
                data = json.load(f)
            if not data:
                self.motor = None
                return
            self.motor = int(data["motor"])
            self.temp = float(data.get("temp_c", 30.0))
            self.fault = int(data.get("fault", 0))
            self.mode = int(data.get("mode", 0))
        except (OSError, ValueError, KeyError, TypeError):
            self.motor = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="vcan0")
    parser.add_argument("--alpha", type=float, default=0.2,
                        help="first-order follow gain per control frame")
    parser.add_argument("--ext-file", default="/tmp/f73_ext.json",
                        help="external-force injection file (effort mode only)")
    parser.add_argument("--inertia", type=float, default=0.08,
                        help="assumed rotor-side inertia for effort dynamics")
    parser.add_argument("--viscous", type=float, default=0.1,
                        help="assumed rotor-side viscous damping")
    parser.add_argument("--silence-file", default="/tmp/f81_silence.json",
                        help="per-motor feedback kill file: {\"motor\": 4}, {} clears")
    parser.add_argument("--health-file", default="/tmp/f84_health.json",
                        help="temp/mode/fault injection: "
                             "{\"motor\": 3, \"temp_c\": 92, \"fault\": 4}, {} clears")
    parser.add_argument("--state-file", default="/tmp/f86_timeout_state.json",
                        help="F86 watchdog state (armed/tripped per motor)")
    args = parser.parse_args()

    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.interface,))
    # Tick the F86 watchdog even when no frames arrive (e.g. after host death).
    sock.settimeout(0.02)

    motors = {i: MotorState(i) for i in range(1, 8)}
    ext = ExternalForce(args.ext_file)
    silence = SilenceCtl(args.silence_file)
    health = HealthCtl(args.health_file)
    # CAN is a shared bus: every motor observes every frame. The firmware
    # watchdog counts bus activity, not frames addressed to one motor.
    bus_last_t = time.monotonic()
    last_state_write = 0.0
    print(f"vcan motor sim on {args.interface}, alpha={args.alpha}", flush=True)

    def tick(now):
        nonlocal last_state_write
        changed = False
        for mt in motors.values():
            if mt.timeout_counts > 0 and not mt.tripped:
                window = mt.timeout_counts / TIMEOUT_COUNTS_PER_SEC
                if now - bus_last_t > window:
                    mt.tripped = True
                    mt.speed = 0.0
                    mt.torque = 0.0
                    mt.trip_delay = now - bus_last_t
                    changed = True
        if changed or now - last_state_write > 0.1:
            write_state_file(args.state_file, motors)
            last_state_write = now

    while True:
        try:
            frame = sock.recv(72)
        except socket.timeout:
            tick(time.monotonic())
            continue
        can_id, _dlc, data = struct.unpack(CAN_FORMAT, frame)
        can_id &= 0x1FFFFFFF
        cmd_type = (can_id >> 24) & 0x1F
        motor_id = can_id & 0xFF
        m = motors.get(motor_id)
        now = time.monotonic()
        bus_last_t = now
        if m is None:
            continue

        silence.poll()
        health.poll()

        def reply():
            if silence.motor != motor_id:
                send_feedback(sock, m, health)

        if cmd_type == CMD_RESET:
            m.speed = 0.0
            m.torque = 0.0
            m.tripped = False
            m.trip_delay = None
            reply()
        elif cmd_type == CMD_ENABLE:
            reply()
        elif cmd_type == CMD_SET_PARAM:
            param_id = data[0] | (data[1] << 8)
            if param_id == PARAM_CAN_TIMEOUT:
                m.timeout_counts = (
                    data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24))
        elif cmd_type == CMD_GET_PARAM:
            param_id = data[0] | (data[1] << 8)
            value = m.timeout_counts if param_id == PARAM_CAN_TIMEOUT else 0
            send_param_reply(sock, m, param_id, value)
        elif cmd_type == CMD_CONTROL and not m.tripped:
            vmax = SPEED_MAX[motor_id]
            target = u16_to_float((data[0] << 8) | data[1], -P_RANGE, P_RANGE)
            target_v = u16_to_float((data[2] << 8) | data[3], -vmax, vmax)
            kp = u16_to_float((data[4] << 8) | data[5], 0.0, 500.0)
            kd = u16_to_float((data[6] << 8) | data[7], 0.0, 5.0)
            t_ff_raw = (can_id >> 8) & 0xFFFF
            t_ff = u16_to_float(t_ff_raw, -TORQUE_MAX[motor_id], TORQUE_MAX[motor_id])

            dt = max(1e-3, now - m.last_t)
            m.last_t = now
            if kp < 1.0:
                # Effort mode: t_ff cancels gravity; the environment adds
                # -t_ff plus any operator push from the ext file.
                ext.poll()
                push = ext.torque if ext.motor == motor_id else 0.0
                m.torque = max(-TORQUE_MAX[motor_id],
                               min(TORQUE_MAX[motor_id],
                                   t_ff + kp * (target - m.angle) + kd * (target_v - m.speed)))
                net = m.torque - t_ff + push
                accel = (net - args.viscous * m.speed) / args.inertia
                m.speed = max(-vmax, min(vmax, m.speed + accel * dt))
                m.angle += m.speed * dt
            else:
                prev = m.angle
                m.angle += args.alpha * (target - m.angle)
                m.speed = max(-vmax, min(vmax, (m.angle - prev) / dt))
                m.torque = max(-TORQUE_MAX[motor_id],
                               min(TORQUE_MAX[motor_id],
                                   t_ff + kp * (target - m.angle) + kd * (target_v - m.speed)))
            reply()
        tick(now)


if __name__ == "__main__":
    main()
