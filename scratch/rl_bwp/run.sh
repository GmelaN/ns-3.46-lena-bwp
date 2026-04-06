#!/usr/bin/env bash
set -euo pipefail

# Sequential training runner with configurable logging/checkpoint options.
# Defaults keep current behavior unless overridden via CLI options.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

RUN_NAME_BASE="appmix_ue20_rr_20260406"
RUN_DIR="${REPO_ROOT}/scratch/rl_bwp/runs/${RUN_NAME_BASE}"

NUM_UES=20
EPISODE_TIME_S=5.0
STEP_TIME_S=0.01
MIN_COMPLETED_EPISODES=50

# Requested options
PER_N_STEP_CSV=200             # -> --reward-bin-size (reward bin CSV every N steps)
VALUE_GRU_LOSS_N_STEP=200      # -> --loss-bin-size (value/gru loss CSV every N steps)
CHECKPOINT_EVERY_EPISODES=1    # -> --save-every-episodes

SCHEDULER_POLICY="rr"
ENABLE_HARQ_RETX="true"

PYTHON_BIN="${REPO_ROOT}/ns3gym-venv/bin/python"
MONITOR="${REPO_ROOT}/scratch/rl_bwp/monitor_ns3_reconnector.sh"

usage() {
  cat <<USAGE
Usage: $0 [options]

Options:
  --run-name-base NAME             Base run name under scratch/rl_bwp/runs (default: ${RUN_NAME_BASE})
  --per-n-step-csv N               Reward CSV bin size in steps (default: ${PER_N_STEP_CSV})
  --value-gru-loss-n-step N        Value/GRU loss CSV bin size in steps (default: ${VALUE_GRU_LOSS_N_STEP})
  --checkpoint-every-episodes N    Save checkpoint .pt every N episodes (default: ${CHECKPOINT_EVERY_EPISODES})
  --num-ues N                      Number of UEs (default: ${NUM_UES})
  --episode-time-s FLOAT           Episode time in seconds (default: ${EPISODE_TIME_S})
  --step-time-s FLOAT              Env step time in seconds (default: ${STEP_TIME_S})
  --min-completed-episodes N       Minimum completed episodes (default: ${MIN_COMPLETED_EPISODES})
  --scheduler-policy NAME          ns-3 scheduler policy (default: ${SCHEDULER_POLICY})
  --enable-harq-retx true|false    ns-3 HARQ reTx toggle (default: ${ENABLE_HARQ_RETX})
  -h, --help                       Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-name-base)
      RUN_NAME_BASE="$2"
      shift 2
      ;;
    --per-n-step-csv)
      PER_N_STEP_CSV="$2"
      shift 2
      ;;
    --value-gru-loss-n-step)
      VALUE_GRU_LOSS_N_STEP="$2"
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
    --scheduler-policy)
      SCHEDULER_POLICY="$2"
      shift 2
      ;;
    --enable-harq-retx)
      ENABLE_HARQ_RETX="$2"
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
echo "Options: per-n-step-csv=${PER_N_STEP_CSV}, value-gru-loss-n-step=${VALUE_GRU_LOSS_N_STEP}, checkpoint-every-episodes=${CHECKPOINT_EVERY_EPISODES}"

# Cleanup previous runs just in case
pkill -f "train_rqrdqn.py" || true
pkill -f "train_drqn.py" || true
pkill -f "monitor_ns3_reconnector.sh" || true
pkill -f "aoi-prb-urban-appmix" || true
sleep 2

# --- 1. RQR-DQN (Proposed: BWP + MCS) ---
echo "--- (1/2) Launching RQR-DQN training ---"
RQR_LOG="${RUN_DIR}/rqrdqn_train.log"
RQR_MONITOR_LOG="${RUN_DIR}/rqrdqn_monitor.log"
RQR_PORT=5555

RQR_CMD=(
  "$PYTHON_BIN" -u "${REPO_ROOT}/scratch/rl_bwp/train_rqrdqn.py"
  --run-name "${RUN_NAME_BASE}"
  --num-ues "$NUM_UES" --episode-time-s "$EPISODE_TIME_S" --step-time-s "$STEP_TIME_S"
  --min-completed-episodes "$MIN_COMPLETED_EPISODES"
  --reward-bin-size "$PER_N_STEP_CSV"
  --loss-bin-size "$VALUE_GRU_LOSS_N_STEP"
  --save-every-episodes "$CHECKPOINT_EVERY_EPISODES"
  --port "$RQR_PORT"
  --ns3-script "aoi-prb-urban-appmix"
  --ns3-arg "schedulerPolicy=${SCHEDULER_POLICY}"
  --ns3-arg "enableHarqReTx=${ENABLE_HARQ_RETX}"
  --ns3-arg "enableRlMcsControl=true"
  --backend "ns3"
)
"${RQR_CMD[@]}" >"$RQR_LOG" 2>&1 &
RQR_PID=$!
echo "RQR-DQN agent process started with PID: $RQR_PID"

nohup "$MONITOR" "$RQR_PID" "$RQR_PORT" 10 5 \
  "aoi-prb-urban-appmix" \
  --numUes="$NUM_UES" --simTime="$EPISODE_TIME_S" --envStepTime="$STEP_TIME_S" \
  --enableOpenGym=true --trafficModel=mixed --schedulerPolicy="$SCHEDULER_POLICY" \
  --enableRlMcsControl=true --enableHarqReTx="$ENABLE_HARQ_RETX" \
  >"$RQR_MONITOR_LOG" 2>&1 &
RQR_MONITOR_PID=$!
echo "RQR-DQN monitor process started with PID: $RQR_MONITOR_PID"

# --- 2. DRQN (Baseline: BWP only) ---
echo "--- (2/2) Launching DRQN training ---"
DRQN_LOG="${RUN_DIR}/drqn_train.log"
DRQN_MONITOR_LOG="${RUN_DIR}/drqn_monitor.log"
DRQN_PORT=5556

DRQN_CMD=(
  "$PYTHON_BIN" -u "${REPO_ROOT}/scratch/rl_bwp/train_drqn.py"
  --run-name "${RUN_NAME_BASE}"
  --num-ues "$NUM_UES" --episode-time-s "$EPISODE_TIME_S" --step-time-s "$STEP_TIME_S"
  --min-completed-episodes "$MIN_COMPLETED_EPISODES"
  --reward-bin-size "$PER_N_STEP_CSV"
  --loss-bin-size "$VALUE_GRU_LOSS_N_STEP"
  --save-every-episodes "$CHECKPOINT_EVERY_EPISODES"
  --port "$DRQN_PORT"
  --ns3-script "aoi-prb-urban-appmix"
  --ns3-arg "schedulerPolicy=${SCHEDULER_POLICY}"
  --ns3-arg "enableHarqReTx=${ENABLE_HARQ_RETX}"
  --backend "ns3"
)
"${DRQN_CMD[@]}" >"$DRQN_LOG" 2>&1 &
DRQN_PID=$!
echo "DRQN agent process started with PID: $DRQN_PID"

nohup "$MONITOR" "$DRQN_PID" "$DRQN_PORT" 10 5 \
  "aoi-prb-urban-appmix" \
  --numUes="$NUM_UES" --simTime="$EPISODE_TIME_S" --envStepTime="$STEP_TIME_S" \
  --enableOpenGym=true --trafficModel=mixed --schedulerPolicy="$SCHEDULER_POLICY" \
  --rlDrqnProfile=true --enableRlMcsControl=false --enableHarqReTx="$ENABLE_HARQ_RETX" \
  >"$DRQN_MONITOR_LOG" 2>&1 &
DRQN_MONITOR_PID=$!
echo "DRQN monitor process started with PID: $DRQN_MONITOR_PID"


wait "$RQR_PID"
echo "--- RQR-DQN training finished. Killing monitor... ---"
kill "$RQR_MONITOR_PID" 2>/dev/null || true
pkill -f "openGymPort=$RQR_PORT" || true
sleep 5

wait "$DRQN_PID"
echo "--- DRQN training finished. Killing monitor... ---"
kill "$DRQN_MONITOR_PID" 2>/dev/null || true
pkill -f "openGymPort=$DRQN_PORT" || true

echo "--- All training complete. ---"
