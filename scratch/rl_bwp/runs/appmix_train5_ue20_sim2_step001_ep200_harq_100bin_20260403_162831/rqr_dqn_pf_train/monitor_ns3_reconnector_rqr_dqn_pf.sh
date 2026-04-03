#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 4 ]]; then
  cat <<'EOF' >&2
Usage:
  monitor_ns3_reconnector.sh <train_pid> <port> <check_interval_s> <max_missing_checks> [ns3_args...]

Example:
  monitor_ns3_reconnector.sh 1971161 64001 10 5 \
    aoi-prb-urban-appmix --numUes=20 --simTime=2.0 --envStepTime=0.01 \
    --enableRlMcsControl=true --enableOpenGym=true --trafficModel=mixed --schedulerPolicy=pf
EOF
  exit 2
fi

TRAIN_PID="$1"
PORT="$2"
CHECK_INTERVAL="$3"
MAX_MISSING="$4"
shift 4

if [[ $# -lt 1 ]]; then
  echo "ns3 script arguments are required" >&2
  exit 2
fi

NS3_ARGS=("$@")
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="$ROOT_DIR/scratch/rl_bwp/runs/monitor_logs"
mkdir -p "$LOG_DIR"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/monitor_port${PORT}_${STAMP}.log"
RUN_LOG="$LOG_DIR/ns3_run_port${PORT}_${STAMP}.log"

missing_count=0
launch_count=0

log() {
  printf '%s %s\n' "$(date '+%F %T%z')" "$*" | tee -a "$LOG_FILE"
}

has_established_connection() {
  ss -tnp 2>/dev/null | grep -F "127.0.0.1:${PORT}" | grep -F "ESTAB" >/dev/null 2>&1
}

has_ns3_process() {
  pgrep -af "openGymPort=${PORT}" | grep -F "ns3.46-" >/dev/null 2>&1
}

start_ns3() {
  launch_count=$((launch_count + 1))
  log "relaunch #${launch_count}: ./ns3 run ${NS3_ARGS[*]} --openGymPort=${PORT}"
  (
    cd "$ROOT_DIR"
    ./ns3 run "${NS3_ARGS[*]} --openGymPort=${PORT}"
  ) >>"$RUN_LOG" 2>&1 &
}

log "monitor start: train_pid=${TRAIN_PID} port=${PORT} interval=${CHECK_INTERVAL}s max_missing=${MAX_MISSING}"
log "run log: $RUN_LOG"

while true; do
  if ! kill -0 "$TRAIN_PID" 2>/dev/null; then
    log "train pid ${TRAIN_PID} is gone; exiting monitor"
    exit 0
  fi

  if has_established_connection; then
    if [[ "$missing_count" -ne 0 ]]; then
      log "connection restored on port ${PORT}"
    fi
    missing_count=0
  else
    missing_count=$((missing_count + 1))
    if has_ns3_process; then
      log "missing connection (${missing_count}/${MAX_MISSING}), ns-3 process exists"
    else
      log "missing connection (${missing_count}/${MAX_MISSING}), ns-3 process absent"
    fi

    if [[ "$missing_count" -ge "$MAX_MISSING" ]]; then
      if has_ns3_process; then
        log "threshold reached but ns-3 process still exists; skipping relaunch"
      else
        start_ns3
      fi
      missing_count=0
      sleep "$CHECK_INTERVAL"
      continue
    fi
  fi

  sleep "$CHECK_INTERVAL"
done
