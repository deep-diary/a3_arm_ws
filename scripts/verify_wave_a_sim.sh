#!/usr/bin/env bash
# Wave A full simulation verification (Pinocchio gravity + zero→work + dual domain).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"
set -u

LOG_DIR="${ROOT}/docs/dev/_wave_a_sim_logs"
mkdir -p "${LOG_DIR}"
REPORT_SNIP="${LOG_DIR}/verify_summary.txt"
: >"${REPORT_SNIP}"

pass() { echo "PASS: $*" | tee -a "${REPORT_SNIP}"; }
fail() { echo "FAIL: $*" | tee -a "${REPORT_SNIP}"; exit 1; }

echo "=== 0. Pinocchio import ==="
python3 - <<'PY' | tee -a "${REPORT_SNIP}"
import pinocchio as pin
print(f"pinocchio {pin.__version__} OK")
PY

DOMAIN=55
export ROS_DOMAIN_ID="${DOMAIN}"
PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do kill "${p}" 2>/dev/null || true; done
}
trap cleanup EXIT

echo "=== 1. Edge sim launch (DOMAIN=${DOMAIN}) ==="
ros2 launch a3_bringup edge_sim_wave_a.launch.py duration_s:=3.0 \
  >"${LOG_DIR}/verify_edge.log" 2>&1 &
PIDS+=($!)

# wait for topics
for i in $(seq 1 20); do
  if ros2 topic list 2>/dev/null | grep -q '/a3/gravity_torque'; then
    break
  fi
  sleep 0.5
done

sleep 1
# Check backend from log
if grep -q 'backend=pinocchio' "${LOG_DIR}/verify_edge.log"; then
  pass "gravity backend=pinocchio"
else
  grep -E 'gravity_torque backend|Pinocchio' "${LOG_DIR}/verify_edge.log" | tee -a "${REPORT_SNIP}" || true
  fail "expected pinocchio backend"
fi

# Sample at start (near zero) then after motion (work)
sleep 1
timeout 3 ros2 topic echo /a3/gravity_torque --once >"${LOG_DIR}/verify_grav_early.txt" 2>&1 || true
sleep 5
timeout 3 ros2 topic echo /joint_states --once >"${LOG_DIR}/verify_js_final.txt" 2>&1 || true
timeout 3 ros2 topic echo /a3/gravity_torque --once >"${LOG_DIR}/verify_grav_work.txt" 2>&1 || true
timeout 3 ros2 topic echo /a3/control_mode --once >"${LOG_DIR}/verify_mode.txt" 2>&1 || true

python3 - <<'PY' | tee -a "${REPORT_SNIP}"
import re, pathlib, sys
log = pathlib.Path("docs/dev/_wave_a_sim_logs")
js = (log / "verify_js_final.txt").read_text()
grav = (log / "verify_grav_work.txt").read_text()

def positions(text):
    m = re.search(r"position:\n((?:- .*\n)+)", text)
    if not m:
        print("FAIL: no positions"); sys.exit(1)
    return [float(x[2:]) for x in m.group(1).strip().splitlines()]

def efforts(text):
    m = re.search(r"effort:\n((?:- .*\n)+)", text)
    if not m:
        print("FAIL: no efforts"); sys.exit(1)
    return [float(x[2:]) for x in m.group(1).strip().splitlines()]

q = positions(js)
work = [0.0, 0.8901179, -0.9948377, 0.0, 0.0, 0.0, 0.0]
err = max(abs(a-b) for a,b in zip(q, work))
print(f"final q={q}")
print(f"max |q-work|={err:.6f}")
if err > 0.02:
    print("FAIL: did not reach work"); sys.exit(1)
print("PASS: zero→work tracking")

tau = efforts(grav)
print(f"tau@work={tau}")
if len(tau) < 7:
    print("FAIL: expected 7 efforts"); sys.exit(1)
if any(abs(t) > 50 for t in tau):
    print("FAIL: torque magnitude unreasonable"); sys.exit(1)
# L3 typically largest magnitude for this arm at work; L4 should be non-zero with Pinocchio
if abs(tau[3]) < 1e-6 and abs(tau[2]) < 0.01:
    print("FAIL: L3/L4 look like approx stub"); sys.exit(1)
print(f"PASS: Pinocchio gravity all joints (L2={tau[1]:.4f} L3={tau[2]:.4f} L4={tau[3]:.4f} Nm)")
PY

# Mode interlocking: start should work in IDLE
echo "=== 2. Gravity start/stop services ==="
ros2 service call /a3/gravity_compensation/stop std_srvs/srv/Trigger '{}' >"${LOG_DIR}/verify_stop.txt" 2>&1 || true
sleep 0.5
# After traj finished mode should be IDLE; start OK
ros2 service call /a3/gravity_compensation/start std_srvs/srv/Trigger '{}' >"${LOG_DIR}/verify_start.txt" 2>&1 || true
if grep -q 'success=True' "${LOG_DIR}/verify_start.txt"; then
  pass "gravity start service"
else
  cat "${LOG_DIR}/verify_start.txt" | tee -a "${REPORT_SNIP}"
  fail "gravity start"
fi
ros2 service call /a3/gravity_compensation/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || true

cleanup
trap - EXIT
PIDS=()

echo "=== 3. Dual domain ==="
EDGE_DOMAIN=10 CE_DOMAIN=20 DURATION_S=2.5 bash "${ROOT}/scripts/dual_domain_zero_to_work.sh" \
  | tee -a "${REPORT_SNIP}"

python3 - <<'PY' | tee -a "${REPORT_SNIP}"
import re, pathlib, sys
log = pathlib.Path("docs/dev/_wave_a_sim_logs")
def pos(path):
    t = path.read_text()
    m = re.search(r"position:\n((?:- .*\n)+)", t)
    assert m, path
    return [float(x[2:]) for x in m.group(1).strip().splitlines()]
qe = pos(log / "edge_joint_states.txt")
qc = pos(log / "cloud_edge_joint_states.txt")
work = [0.0, 0.8901179, -0.9948377, 0.0, 0.0, 0.0, 0.0]
ee = max(abs(a-b) for a,b in zip(qe, work))
ec = max(abs(a-b) for a,b in zip(qc, work))
print(f"Edge err={ee:.6f} CloudEdge err={ec:.6f}")
if ee > 0.02 or ec > 0.02:
    print("FAIL: dual domain tracking"); sys.exit(1)
print("PASS: dual-domain zero→work")
PY

pass "all Wave A simulation checks"
echo "Summary written to ${REPORT_SNIP}"
