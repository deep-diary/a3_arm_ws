#!/usr/bin/env bash
# 12 电机 MIT 显式循环发送（默认 200Hz）
# - 默认帧：p=0, v=0, kp=20, kd=1.5, tau=0
# - 用途：验证实际发送频率与反馈频率是否接近 200Hz
#
# 依赖：
#   sudo apt-get install -y can-utils bc
#
# 示例：
#   bash src/a3_can_bridge/scripts/el05_mit_stream_200hz_12_direct.sh
#   DURATION_S=8 bash src/a3_can_bridge/scripts/el05_mit_stream_200hz_12_direct.sh
#   LOOPS=1000 bash src/a3_can_bridge/scripts/el05_mit_stream_200hz_12_direct.sh
set -euo pipefail

IFACE_FRONT="${IFACE_FRONT:-can0}"
IFACE_REAR="${IFACE_REAR:-can1}"
TARGET_HZ="${TARGET_HZ:-200}"
PERIOD_S="$(awk -v hz="$TARGET_HZ" 'BEGIN{printf "%.6f", 1.0/hz}')"

# 二选一：优先按固定循环数；否则按持续时间
LOOPS="${LOOPS:-0}"           # >0 时生效
DURATION_S="${DURATION_S:-5}" # LOOPS=0 时生效

# MIT 默认：p=0,v=0,kp=20,kd=1.5,tau=0
MIT_DATA="800080000A3D4CCD"

# tau=0 时 CAN ID 前缀固定 0x018000xx
FRONT_IDS=(0B 0C 0D 15 16 17)
REAR_IDS=(33 34 35 3D 3E 3F)

send_once_all_12() {
  local id
  for id in "${FRONT_IDS[@]}"; do
    cansend "$IFACE_FRONT" "018000${id}#${MIT_DATA}"
  done
  for id in "${REAR_IDS[@]}"; do
    cansend "$IFACE_REAR" "018000${id}#${MIT_DATA}"
  done
}

echo "MIT stream start: target=${TARGET_HZ}Hz period=${PERIOD_S}s data=${MIT_DATA}"
echo "front=${IFACE_FRONT} rear=${IFACE_REAR}"

sent_loops=0
start_ts="$(date +%s.%N)"

if [[ "$LOOPS" -gt 0 ]]; then
  while [[ "$sent_loops" -lt "$LOOPS" ]]; do
    send_once_all_12
    sent_loops=$((sent_loops + 1))
    sleep "$PERIOD_S"
  done
else
  while true; do
    now_ts="$(date +%s.%N)"
    elapsed="$(awk -v n="$now_ts" -v s="$start_ts" 'BEGIN{printf "%.6f", n-s}')"
    if awk -v e="$elapsed" -v d="$DURATION_S" 'BEGIN{exit !(e>=d)}'; then
      break
    fi
    send_once_all_12
    sent_loops=$((sent_loops + 1))
    sleep "$PERIOD_S"
  done
fi

end_ts="$(date +%s.%N)"
actual_elapsed="$(awk -v e="$end_ts" -v s="$start_ts" 'BEGIN{printf "%.6f", e-s}')"
actual_hz="$(awk -v n="$sent_loops" -v t="$actual_elapsed" 'BEGIN{if(t>0) printf "%.2f", n/t; else print "0"}')"

echo "done: loops=${sent_loops}, elapsed=${actual_elapsed}s, actual_loop_hz=${actual_hz}"
echo "tip: 每个 loop 发 12 帧，理论总发送帧率约 = loop_hz * 12"
