#!/usr/bin/env python3
"""F103: SocketCAN physical-layer health on the standard diagnostics channel.

Samples `ip -s -d -j link show <iface>` every 2 s and reports diagnostic task
"a3_can_bus: CAN link <iface>": UP + ERROR-ACTIVE = OK, ERROR-WARN/ERROR-PASSIVE
= WARN, missing/down/BUS-OFF = ERROR. CAN error counters (re-started,
bus-errors, arbit-lost, error-warn, error-pass, bus-off) are attached when the
kernel reports them.
"""

import json
import socket
import subprocess

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from rclpy.node import Node


class CanBusMonitor(Node):
    def __init__(self):
        super().__init__("a3_can_bus")
        self.declare_parameter("interface", "can1")
        self.interface = self.get_parameter("interface").value

        self.updater = Updater(node=self)
        self.updater.setHardwareID(f"{socket.gethostname()}:{self.interface}")
        self.updater.add(f"CAN link {self.interface}", self.produce_diagnostics)
        self.create_timer(2.0, self.updater.force_update)

    def _read_link(self):
        proc = subprocess.run(
            ["ip", "-s", "-d", "-j", "link", "show", self.interface],
            capture_output=True, text=True, check=False)
        if proc.returncode != 0:
            return None
        try:
            data = json.loads(proc.stdout or "[]")
        except json.JSONDecodeError:
            return None
        return data[0] if data else None

    def produce_diagnostics(self, stat: DiagnosticStatusWrapper):
        link = self._read_link()
        if link is None:
            stat.summary(
                DiagnosticStatus.ERROR,
                f"interface {self.interface} not found")
            return stat

        flags = link.get("flags", [])
        is_up = "UP" in flags
        linkinfo = link.get("linkinfo", {})
        info = linkinfo.get("info_data", {})
        state = info.get("state", "")
        restart_ms = info.get("restart_ms")

        stat.add("operstate", link.get("operstate", "unknown"))
        if restart_ms is not None:
            stat.add("restart_ms", str(restart_ms))
        berr = info.get("berr_counter", {})
        if berr:
            stat.add("berr_rx", str(berr.get("rx", "")))
            stat.add("berr_tx", str(berr.get("tx", "")))

        if state:
            stat.add("can_state", state)
        if not is_up or state == "BUS-OFF":
            level = DiagnosticStatus.ERROR
        elif state.startswith("ERROR-WARN") or state.startswith("ERROR-PASS"):
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.OK

        stats = link.get("stats64", {})
        for direction in ("rx", "tx"):
            d = stats.get(direction, {})
            packets = d.get("packets")
            if packets is not None:
                stat.add(f"{direction}_packets", str(packets))

        counters = linkinfo.get("info_xstats", {})
        for key in ("restarts", "bus_error", "arbitration_lost",
                    "error_warning", "error_passive", "bus_off"):
            if key in counters:
                stat.add(key, str(counters[key]))

        stat.summary(
            level,
            f"{self.interface}: {'UP' if is_up else 'DOWN'}"
            f"{f' {state}' if state else ''}")
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = CanBusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
