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
MASTER_ID = 0xFD

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


def pack_frame(can_id, data):
    return struct.pack(CAN_FORMAT, can_id | CAN_EFF_FLAG, 8, bytes(data))


def send_feedback(sock, m):
    can_id = (CMD_FEEDBACK << 24) | (m.motor_id << 8) | MASTER_ID
    tmax = TORQUE_MAX[m.motor_id]
    vmax = SPEED_MAX[m.motor_id]
    data = [0] * 8
    p = float_to_u16(m.angle, -P_RANGE, P_RANGE)
    v = float_to_u16(m.speed, -vmax, vmax)
    t = float_to_u16(m.torque, -tmax, tmax)
    temp = max(0, min(65535, int(30.0 * 10)))
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
    args = parser.parse_args()

    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.interface,))

    motors = {i: MotorState(i) for i in range(1, 8)}
    ext = ExternalForce(args.ext_file)
    silence = SilenceCtl(args.silence_file)
    print(f"vcan motor sim on {args.interface}, alpha={args.alpha}", flush=True)

    while True:
        frame = sock.recv(72)
        can_id, _dlc, data = struct.unpack(CAN_FORMAT, frame)
        can_id &= 0x1FFFFFFF
        cmd_type = (can_id >> 24) & 0x1F
        motor_id = can_id & 0xFF
        m = motors.get(motor_id)
        if m is None:
            continue

        silence.poll()

        def reply():
            if silence.motor != motor_id:
                send_feedback(sock, m)

        if cmd_type == CMD_RESET:
            m.speed = 0.0
            m.torque = 0.0
            reply()
        elif cmd_type == CMD_ENABLE:
            reply()
        elif cmd_type == CMD_CONTROL:
            vmax = SPEED_MAX[motor_id]
            target = u16_to_float((data[0] << 8) | data[1], -P_RANGE, P_RANGE)
            target_v = u16_to_float((data[2] << 8) | data[3], -vmax, vmax)
            kp = u16_to_float((data[4] << 8) | data[5], 0.0, 500.0)
            kd = u16_to_float((data[6] << 8) | data[7], 0.0, 5.0)
            t_ff_raw = (can_id >> 8) & 0xFFFF
            t_ff = u16_to_float(t_ff_raw, -TORQUE_MAX[motor_id], TORQUE_MAX[motor_id])

            now = time.monotonic()
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


if __name__ == "__main__":
    main()
