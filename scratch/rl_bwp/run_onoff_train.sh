#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "usage: $0 <drqn|rqrdqn> <run_name> <train_port> [num_ues] [traffic_model]"
  exit 2
fi

ALG="$1"
RUN_NAME="$2"
TRAIN_PORT="$3"
NUM_UES="${4:-20}"
TRAFFIC_MODEL="${5:-legacy}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

STEP_TIME="0.05"
TRAIN_EPISODE_S="30"
MIN_COMPLETED_EPISODES="5"
FINAL_TRAIN_UPDATES="256"
DEVICE="cpu"
BURST_RATE_MBPS="100"
BACKGROUND_RATE_KBPS="500"

BASE_STEPS_PER_EPISODE="$(NUM_UES="$NUM_UES" python3 - <<'PY'
import os
episode_time_s = 30.0
step_time_s = 0.05
num_ues = int(os.environ["NUM_UES"])
print(int(round(episode_time_s / step_time_s)) * num_ues)
PY
)"
TOTAL_ENV_STEPS="$(( BASE_STEPS_PER_EPISODE * MIN_COMPLETED_EPISODES ))"

OUT_DIR="scratch/rl_bwp/runs/${RUN_NAME}"
mkdir -p "$OUT_DIR"

COMMON_ARGS=(
  --backend ns3
  --run-name "$RUN_NAME"
  --ns3-script aoi-prb-urban-onoff
  --start-sim
  --num-ues "$NUM_UES"
  --step-time-s "$STEP_TIME"
  --episode-time-s "$TRAIN_EPISODE_S"
  --total-env-steps "$TOTAL_ENV_STEPS"
  --min-completed-episodes "$MIN_COMPLETED_EPISODES"
  --final-train-updates "$FINAL_TRAIN_UPDATES"
  --device "$DEVICE"
  --shared-per-ue
  --ns3-arg trafficModel="$TRAFFIC_MODEL"
  --ns3-arg burstRateMbps="$BURST_RATE_MBPS"
  --ns3-arg backgroundRateKbps="$BACKGROUND_RATE_KBPS"
  --ns3-arg schedulerPolicy=pf
  --ns3-arg aequitasEnableMcsSelection=false
  --ns3-arg envStepTime="$STEP_TIME"
)

case "$ALG" in
  drqn)
    ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
      "${COMMON_ARGS[@]}" \
      --port "$TRAIN_PORT" \
      >"${OUT_DIR}/train.log" 2>&1
    ;;
  rqrdqn)
    ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_rqrdqn.py \
      "${COMMON_ARGS[@]}" \
      --port "$TRAIN_PORT" \
      >"${OUT_DIR}/train.log" 2>&1
    ;;
  *)
    echo "unknown algorithm: $ALG" >&2
    exit 2
    ;;
esac
