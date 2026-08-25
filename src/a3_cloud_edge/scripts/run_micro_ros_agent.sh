#!/usr/bin/env bash
# Resolve micro_ros_agent binary: apt package or third_party build.
set -eo pipefail

PORT="${1:-8888}"
VERBOSE="${2:-4}"

if command -v ros2 >/dev/null 2>&1; then
  if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "${PORT}" -v "${VERBOSE}"
  fi
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${A3_ARM_WS:-}" && -d "${A3_ARM_WS}/src/third_party/micro_ros_host" ]]; then
  WS_ROOT="${A3_ARM_WS}"
elif [[ -d "${SCRIPT_DIR}/../../../src/third_party" ]]; then
  WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
elif [[ -d "${SCRIPT_DIR}/../../../../src/third_party" ]]; then
  WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
else
  echo "Cannot locate workspace root. Set A3_ARM_WS or run setup_microros_host.sh" >&2
  exit 1
fi

HOST_DIR="${WS_ROOT}/src/third_party/micro_ros_host"
AGENT_BIN="${HOST_DIR}/agent_build/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"

if [[ -x "${AGENT_BIN}" ]]; then
  exec "${AGENT_BIN}" udp4 --port "${PORT}" -v "${VERBOSE}"
fi

echo "micro_ros_agent not found. Install ros-humble-micro-ros-agent or run:" >&2
echo "  ${WS_ROOT}/src/a3_cloud_edge/scripts/setup_microros_host.sh" >&2
exit 1
