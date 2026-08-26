#!/usr/bin/env bash
# Dual-domain Wave A sim: Edge (DOMAIN 10) + CloudEdge-style executor (DOMAIN 20).
# Same zero→work trajectory fan-out (independent publishers, identical params).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"
set -u

EDGE_DOMAIN="${EDGE_DOMAIN:-10}"
CE_DOMAIN="${CE_DOMAIN:-20}"
DURATION="${DURATION_S:-3.0}"
LOG_DIR="${ROOT}/docs/dev/_wave_a_sim_logs"
mkdir -p "${LOG_DIR}"

PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do
    kill "${p}" 2>/dev/null || true
  done
}
trap cleanup EXIT

echo "[dual] Edge DOMAIN=${EDGE_DOMAIN}, CloudEdge DOMAIN=${CE_DOMAIN}, duration=${DURATION}s"

export ROS_DOMAIN_ID="${EDGE_DOMAIN}"
ros2 launch a3_bringup edge_sim_wave_a.launch.py duration_s:="${DURATION}" use_rviz:=false \
  >"${LOG_DIR}/edge.log" 2>&1 &
PIDS+=($!)

export ROS_DOMAIN_ID="${CE_DOMAIN}"
ros2 run a3_bringup sim_executor --ros-args \
  -r __node:=ce_sim_executor -p rate_hz:=50.0 \
  >"${LOG_DIR}/cloud_edge_executor.log" 2>&1 &
PIDS+=($!)
sleep 2
ros2 run a3_bringup zero_to_work_publisher --ros-args \
  -r __node:=ce_zero_to_work \
  -p duration_s:="${DURATION}" -p delay_s:=0.5 -p num_waypoints:=11 \
  >"${LOG_DIR}/cloud_edge_traj.log" 2>&1 &
PIDS+=($!)

# Wait for motion to finish (traj delay + duration + margin)
sleep "$(python3 -c "print(${DURATION} + 4.0)")"

export ROS_DOMAIN_ID="${EDGE_DOMAIN}"
timeout 4 ros2 topic echo /joint_states --once >"${LOG_DIR}/edge_joint_states.txt" 2>&1 || true
timeout 4 ros2 topic echo /a3/gravity_torque --once >"${LOG_DIR}/edge_gravity.txt" 2>&1 || true

export ROS_DOMAIN_ID="${CE_DOMAIN}"
timeout 4 ros2 topic echo /joint_states --once >"${LOG_DIR}/cloud_edge_joint_states.txt" 2>&1 || true

echo "[dual] logs in ${LOG_DIR}"
echo "[dual] done"
