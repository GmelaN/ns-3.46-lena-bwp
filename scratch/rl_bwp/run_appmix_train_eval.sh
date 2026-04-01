#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 4 ]; then
  echo "usage: $0 <drqn|rqrdqn> <run_name> <train_port> <eval_port> [num_ues]"
  exit 2
fi

ALG="$1"
RUN_NAME="$2"
TRAIN_PORT="$3"
EVAL_PORT="$4"
NUM_UES="${5:-20}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

STEP_TIME="0.05"
TRAIN_EPISODE_S="30"
EVAL_EPISODE_S="60"
MIN_COMPLETED_EPISODES="5"
FINAL_TRAIN_UPDATES="256"
DEVICE="cpu"

BASE_STEPS_PER_EPISODE="$(python3 - <<'PY'
episode_time_s = 30.0
step_time_s = 0.05
num_ues = 20
print(int(round(episode_time_s / step_time_s)) * num_ues)
PY
)"
TOTAL_ENV_STEPS="$(( BASE_STEPS_PER_EPISODE * MIN_COMPLETED_EPISODES ))"

OUT_DIR="scratch/rl_bwp/runs/${RUN_NAME}"
mkdir -p "$OUT_DIR"

COMMON_ARGS=(
  --backend ns3
  --run-name "$RUN_NAME"
  --ns3-script aoi-prb-urban-appmix
  --start-sim
  --num-ues "$NUM_UES"
  --step-time-s "$STEP_TIME"
  --total-env-steps "$TOTAL_ENV_STEPS"
  --min-completed-episodes "$MIN_COMPLETED_EPISODES"
  --final-train-updates "$FINAL_TRAIN_UPDATES"
  --device "$DEVICE"
  --shared-per-ue
  --ns3-arg trafficModel=mixed
  --ns3-arg schedulerPolicy=pf
  --ns3-arg aequitasEnableMcsSelection=false
  --ns3-arg envStepTime="$STEP_TIME"
)

EVAL_COMMON_ARGS=(
  --backend ns3
  --ns3-script aoi-prb-urban-appmix
  --start-sim
  --num-ues "$NUM_UES"
  --step-time-s "$STEP_TIME"
  --device "$DEVICE"
  --output-json "${OUT_DIR}/eval.json"
  --ns3-arg trafficModel=mixed
  --ns3-arg schedulerPolicy=pf
  --ns3-arg aequitasEnableMcsSelection=false
  --ns3-arg envStepTime="$STEP_TIME"
)

case "$ALG" in
  drqn)
    ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
      "${COMMON_ARGS[@]}" \
      --port "$TRAIN_PORT" \
      --episode-time-s "$TRAIN_EPISODE_S" \
      >"${OUT_DIR}/train.log" 2>&1

    ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
      --model-path "${OUT_DIR}/final_model.pt" \
      "${EVAL_COMMON_ARGS[@]}" \
      --port "$EVAL_PORT" \
      --episode-time-s "$EVAL_EPISODE_S" \
      >"${OUT_DIR}/eval.log" 2>&1
    ;;
  rqrdqn)
    ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_rqrdqn.py \
      "${COMMON_ARGS[@]}" \
      --port "$TRAIN_PORT" \
      --episode-time-s "$TRAIN_EPISODE_S" \
      >"${OUT_DIR}/train.log" 2>&1

    ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_rqrdqn.py \
      --model-path "${OUT_DIR}/final_model.pt" \
      "${EVAL_COMMON_ARGS[@]}" \
      --port "$EVAL_PORT" \
      --episode-time-s "$EVAL_EPISODE_S" \
      >"${OUT_DIR}/eval.log" 2>&1
    ;;
  *)
    echo "unknown algorithm: $ALG" >&2
    exit 2
    ;;
esac

