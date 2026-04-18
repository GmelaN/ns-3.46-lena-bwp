#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

RUN_NAME_BASE="rqrdqn_rqr_ue20_ep20_tti1ms_$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${REPO_ROOT}/scratch/rl_bwp/runs/${RUN_NAME_BASE}"

NUM_UES=20
EPISODE_TIME_S=10.0
STEP_TIME_S=0.001
MIN_COMPLETED_EPISODES=20

VALUE_LOSS_N_STEP=500
CHECKPOINT_EVERY_EPISODES=1
RQR_PORT=6555

PYTHON_BIN="${REPO_ROOT}/ns3gym-venv/bin/python"
MONITOR="${REPO_ROOT}/scratch/rl_bwp/monitor_ns3_reconnector.sh"

usage() {
  cat <<USAGE
Usage: $0 [options]

Options:
  --run-name-base NAME             Base run name under scratch/rl_bwp/runs (default: ${RUN_NAME_BASE})
  --value-loss-n-step N            Value loss CSV bin size in steps (default: ${VALUE_LOSS_N_STEP})
  --checkpoint-every-episodes N    Save checkpoint .pt every N episodes (default: ${CHECKPOINT_EVERY_EPISODES})
  --num-ues N                      Number of UEs (default: ${NUM_UES})
  --episode-time-s FLOAT           Episode time in seconds (default: ${EPISODE_TIME_S})
  --step-time-s FLOAT              Env step time in seconds (default: ${STEP_TIME_S})
  --min-completed-episodes N       Minimum completed episodes (default: ${MIN_COMPLETED_EPISODES})
  --port N                         OpenGym port (default: ${RQR_PORT})
  -h, --help                       Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-name-base)
      RUN_NAME_BASE="$2"
      shift 2
      ;;
    --value-loss-n-step)
      VALUE_LOSS_N_STEP="$2"
      shift 2
      ;;
    --checkpoint-every-episodes)
      CHECKPOINT_EVERY_EPISODES="$2"
      shift 2
      ;;
    --num-ues)
      NUM_UES="$2"
      shift 2
      ;;
    --episode-time-s)
      EPISODE_TIME_S="$2"
      shift 2
      ;;
    --step-time-s)
      STEP_TIME_S="$2"
      shift 2
      ;;
    --min-completed-episodes)
      MIN_COMPLETED_EPISODES="$2"
      shift 2
      ;;
    --port)
      RQR_PORT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

RUN_DIR="${REPO_ROOT}/scratch/rl_bwp/runs/${RUN_NAME_BASE}"
mkdir -p "$RUN_DIR"

echo "All runs will be saved in: $RUN_DIR"
echo "Options: value-loss-n-step=${VALUE_LOSS_N_STEP}, checkpoint-every-episodes=${CHECKPOINT_EVERY_EPISODES}, port=${RQR_PORT}"

# Cleanup previous runs just in case.
pkill -f "train_rqrdqn.py" || true
pkill -f "monitor_ns3_reconnector.sh" || true
pkill -f "aoi-prb-urban-appmix" || true
sleep 2

# Single run: RQR-DQN with requested setup.
# NOTE: train_rqrdqn currently expects shared-per-UE adapter path.
echo "--- Launching RQR-DQN training (UE=20, simTime=10s, step=1ms, episode=20) ---"
RQR_LOG="${RUN_DIR}/rqrdqn_train.log"
RQR_MONITOR_LOG="${RUN_DIR}/rqrdqn_monitor.log"

RQR_CMD=(
  "$PYTHON_BIN" -u "${REPO_ROOT}/scratch/rl_bwp/train_rqrdqn.py"
  --backend "ns3"
  --ns3-script "aoi-prb-urban-appmix"
  --run-name "${RUN_NAME_BASE}"
  --num-ues "$NUM_UES" --episode-time-s "$EPISODE_TIME_S" --step-time-s "$STEP_TIME_S"
  --min-completed-episodes "$MIN_COMPLETED_EPISODES"
  --total-env-steps 1
  --loss-bin-size "$VALUE_LOSS_N_STEP"
)
"${RQR_CMD[@]}" >"$RQR_LOG" 2>&1 &
RQR_PID=$!
echo "single BWP agent process started with PID: $RQR_PID"

nohup "$MONITOR" "$RQR_PID" "$RQR_PORT" 10 5 \
  "aoi-prb-urban-appmix" \
  --numUes="$NUM_UES" --simTime="$EPISODE_TIME_S" --envStepTime="$STEP_TIME_S" \
  --enableOpenGym=false --initialBwpId=0 --trafficModel=mixed \
  >"$RQR_MONITOR_LOG" 2>&1 &
RQR_MONITOR_PID=$!
echo "RQR-DQN monitor process started with PID: $RQR_MONITOR_PID"

wait "$RQR_PID"
echo "--- RQR-DQN training finished. Killing monitor. ---"
kill "$RQR_MONITOR_PID" 2>/dev/null || true
pkill -f "openGymPort=$RQR_PORT" || true

echo "--- Training complete. ---"
