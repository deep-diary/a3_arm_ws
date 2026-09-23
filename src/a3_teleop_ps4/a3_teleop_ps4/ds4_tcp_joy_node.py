#!/usr/bin/env python3
"""
TCP JSON → /joy（WSL 侧接收端，配合 Windows scripts/ps4/ds4_bridge_win.py）.

Windows 桥接脚本把蓝牙 DS4 归一化为 deep-dog 抽象快照（JSON 行协议）经
TCP 发来（WSL2 localhost 转发，Windows 连 127.0.0.1 即可）。本节点将其转换为
**Linux joydev（hid-sony 蓝牙）空间**的 sensor_msgs/Joy 发布到 /joy，
使现有 ps4_mapper / ps4_arm_teleop（ds4_linux.yaml 布局）原样工作，与真机一致。

抽象 → Linux /joy 映射：
  axes[0..7] = LX LY L2 RX RY R2 DPAD_X DPAD_Y
    抽象 lx/ly/rx/ry 右/下为正 → Linux 左/上为正 → 取反
    抽象 l2/r2 ∈[0,1]（静息 0）→ Linux 静息 +1、按满 -1 → 1-2v
    dpad: 右=+1、下=+1（与 Linux dpad_x/dpad_y 一致）
  buttons[0..13] = cross circle triangle square L1 R1 L2 R2 share options PS L3 R3 touchpad

用法：
  ros2 run a3_teleop_ps4 ds4_tcp_joy_node        # 监听 0.0.0.0:8890
  ros2 run a3_teleop_ps4 ds4_tcp_joy_node --ros-args -p port:=8890 -p joy_topic:=/joy
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class Ds4TcpJoyNode(Node):
    def __init__(self) -> None:
        super().__init__("ds4_tcp_joy_node")
        self.declare_parameter("port", 8890)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("l2_digital_threshold", 0.5)
        self.declare_parameter("r2_digital_threshold", 0.5)

        self._port = int(self.get_parameter("port").value)
        self._pub = self.create_publisher(
            Joy, str(self.get_parameter("joy_topic").value), 10
        )
        self._l2_th = float(self.get_parameter("l2_digital_threshold").value)
        self._r2_th = float(self.get_parameter("r2_digital_threshold").value)

        self._server: Optional[socket.socket] = None
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        self.get_logger().info(f"ds4 tcp joy bridge listening on 0.0.0.0:{self._port}")

    # ------------------------------------------------------------------ TCP

    def _serve(self) -> None:
        try:
            self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.bind(("0.0.0.0", self._port))
            self._server.listen(1)
        except OSError as e:
            self.get_logger().error(f"bind 0.0.0.0:{self._port} failed: {e}")
            return
        while rclpy.ok():
            try:
                conn, addr = self._server.accept()
            except OSError:
                break
            self.get_logger().info(f"bridge connected: {addr[0]}")
            self._handle_conn(conn)
            self.get_logger().info("bridge disconnected; zeroing /joy")
            self._publish_zero()
            conn.close()

    def _handle_conn(self, conn: socket.socket) -> None:
        conn.settimeout(1.0)
        buf = b""
        while rclpy.ok():
            try:
                chunk = conn.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    snap = json.loads(line.decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    continue
                self._on_snapshot(snap)

    # ------------------------------------------------------------------ 转换

    def _on_snapshot(self, snap: dict) -> None:
        axes = snap.get("axes") or {}
        btns = snap.get("buttons") or {}
        dpad = snap.get("dpad") or {}

        lx = float(axes.get("lx", 0.0))
        ly = float(axes.get("ly", 0.0))
        rx = float(axes.get("rx", 0.0))
        ry = float(axes.get("ry", 0.0))
        l2 = float(btns.get("l2", 0.0))
        r2 = float(btns.get("r2", 0.0))
        # 抽象 dpad（有则用，无则从具名按钮推导）
        dx = float(dpad.get("dx", 0.0))
        dy = float(dpad.get("dy", 0.0))
        if not dpad:
            dx = (1.0 if btns.get("dpad_right") else 0.0) - (
                1.0 if btns.get("dpad_left") else 0.0
            )
            dy = (1.0 if btns.get("dpad_down") else 0.0) - (
                1.0 if btns.get("dpad_up") else 0.0
            )

        def b(name: str) -> bool:
            return bool(btns.get(name))

        msg = Joy()
        msg.axes = [
            -lx,                 # 0 LX：抽象右=+1 → Linux 左=+1
            -ly,                 # 1 LY：抽象下=+1 → Linux 上=+1
            1.0 - 2.0 * l2,      # 2 L2：静息 +1、按满 -1
            -rx,                 # 3 RX
            -ry,                 # 4 RY
            1.0 - 2.0 * r2,      # 5 R2
            dx,                  # 6 DPAD_X：右=+1
            dy,                  # 7 DPAD_Y：下=+1
        ]
        msg.buttons = [
            int(b("a")),                          # 0 cross
            int(b("b")),                          # 1 circle
            int(b("y")),                          # 2 triangle
            int(b("x")),                          # 3 square
            int(b("l1")),                         # 4 L1
            int(b("r1")),                         # 5 R1
            int(l2 > self._l2_th),                # 6 L2（数字）
            int(r2 > self._r2_th),                # 7 R2（数字）
            int(b("select")),                     # 8 Share
            int(b("start")),                      # 9 Options
            int(b("ps")),                         # 10 PS
            int(b("l3")),                         # 11 L3
            int(b("r3")),                         # 12 R3
            int(b("touch")),                      # 13 Touchpad
        ]
        self._pub.publish(msg)

    def _publish_zero(self) -> None:
        msg = Joy()
        msg.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        msg.buttons = [0] * 14
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Ds4TcpJoyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
