#!/usr/bin/env bash
# Smoke-test helper for EL05/RS00 family motors on SocketCAN (A3 arm IDs 1..7).
# Usage examples:
#   IFACE=can0 MOTOR_IDS="1 2 3 4 5 6 7" bash a3_motor_cansend.sh ping
#   IFACE=can0 MOTOR_IDS="1" bash a3_motor_cansend.sh check
set -euo pipefail

IFACE="${IFACE:-can0}"
MOTOR_IDS="${MOTOR_IDS:-1 2 3 4 5 6 7}"
MASTER_ID="${MASTER_ID:-253}"   # 0xFD
SLEEP_MS="${SLEEP_MS:-20}"
CMD="${1:-ping}"

hex2() { printf "%02X" "$1"; }

build_id() {
  local cmd="$1" mid="$2"
  # CMD<<24 | MASTER<<8 | motor_id  (same as trotbot protocol_codec / EL05)
  local id=$(( (cmd << 24) | (MASTER_ID << 8) | mid ))
  printf "%08X" "$id"
}

send_frame() {
  local can_id="$1" data="$2"
  cansend "$IFACE" "${can_id}#${data}"
  sleep "$(echo "scale=3; ${SLEEP_MS}/1000" | bc)"
}

echo "IFACE=$IFACE MASTER=0x$(hex2 $MASTER_ID) IDS=[$MOTOR_IDS] CMD=$CMD"

case "$CMD" in
  ping)
    for mid in $MOTOR_IDS; do
      echo "would use $IFACE motor_id=$mid"
    done
    ;;
  zero)
    for mid in $MOTOR_IDS; do
      id=$(build_id 6 "$mid")
      echo "set_zero motor $mid -> $IFACE $id"
      send_frame "$id" "0100000000000000"
    done
    ;;
  enable|init)
    for mid in $MOTOR_IDS; do
      id=$(build_id 3 "$mid")
      echo "enable motor $mid"
      send_frame "$id" "0000000000000000"
    done
    ;;
  disable|reset)
    for mid in $MOTOR_IDS; do
      id=$(build_id 4 "$mid")
      echo "disable motor $mid"
      send_frame "$id" "0000000000000000"
    done
    ;;
  mit)
    # zero MIT frame kp=20 kd=1.5
    for mid in $MOTOR_IDS; do
      id=$(build_id 1 "$mid")
      echo "mit motor $mid"
      send_frame "$id" "7FFF7FFF147AE666"
    done
    ;;
  check)
    for mid in $MOTOR_IDS; do
      id=$(build_id 6 "$mid")
      echo "check/set_zero probe motor $mid"
      send_frame "$id" "0100000000000000"
    done
    echo "Watch feedback with: candump -tz $IFACE"
    ;;
  *)
    echo "Usage: $0 {ping|zero|enable|disable|mit|check}"
    exit 1
    ;;
esac
