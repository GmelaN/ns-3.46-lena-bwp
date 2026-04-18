#!/usr/bin/env bash
set -euo pipefail

# Sequential training runner with configurable logging/checkpoint options.
# Defaults keep current behavior unless overridden via CLI options.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

RUN_NAME_BASE="dqn_appmix_ue20_20260407"
NUM_UES=20
EPISODE_TIME_S=5.0
STEP_TIME_S=0.01
MIN_COMPLETED_EPISODES=50

# Requested options
PER_N_STEP_CSV=200             # -> --reward-bin-size (reward bin CSV every N steps)
VALUE_LOSS_N_STEP=200          # -> --loss-bin-size (value loss CSV every N steps)
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
  --value-loss-n-step N            Value loss CSV bin size in steps (default: ${VALUE_LOSS_N_STEP})
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
echo "Options: per-n-step-csv=${PER_N_STEP_CSV}, value-loss-n-step=${VALUE_LOSS_N_STEP}, checkpoint-every-episodes=${CHECKPOINT_EVERY_EPISODES}"

# Cleanup previous runs just in case
pkill -f "train_dqn.py" || true
pkill -f "monitor_ns3_reconnector.sh" || true
pkill -f "aoi-prb-urban-appmix" || true
sleep 2

# --- 1. DQN (Proposed: BWP + MCS) ---
echo "--- (1/2) Launching Proposed DQN training ---"
PROP_RUN_NAME="${RUN_NAME_BASE}/proposed_dqn"
PROP_LOG="${RUN_DIR}/proposed_dqn_train.log"
PROP_MONITOR_LOG="${RUN_DIR}/proposed_dqn_monitor.log"
PROP_PORT=5555

PROP_CMD=(
  "$PYTHON_BIN" -u "${REPO_ROOT}/scratch/rl_bwp/train_dqn.py"
  --run-name "${PROP_RUN_NAME}"
  --num-ues "$NUM_UES" --episode-time-s "$EPISODE_TIME_S" --step-time-s "$STEP_TIME_S"
  --min-completed-episodes "$MIN_COMPLETED_EPISODES"
  --reward-bin-size "$PER_N_STEP_CSV"
  --loss-bin-size "$VALUE_LOSS_N_STEP"
  --save-every-episodes "$CHECKPOINT_EVERY_EPISODES"
  --port "$PROP_PORT"
  --ns3-script "aoi-prb-urban-appmix"
  --ns3-arg "schedulerPolicy=${SCHEDULER_POLICY}"
  --ns3-arg "enableHarqReTx=${ENABLE_HARQ_RETX}"
  --ns3-arg "enableRlMcsControl=true"
  --backend "ns3"
)
"${PROP_CMD[@]}" >"$PROP_LOG" 2>&1 &
PROP_PID=$!
echo "Proposed DQN agent process started with PID: $PROP_PID"

nohup "$MONITOR" "$PROP_PID" "$PROP_PORT" 10 5 \
  "aoi-prb-urban-appmix" \
  --numUes="$NUM_UES" --simTime="$EPISODE_TIME_S" --envStepTime="$STEP_TIME_S" \
  --enableOpenGym=true --trafficModel=mixed --schedulerPolicy="$SCHEDULER_POLICY" \
  --enableRlMcsControl=true --enableHarqReTx="$ENABLE_HARQ_RETX" \
  >"$PROP_MONITOR_LOG" 2>&1 &
PROP_MONITOR_PID=$!
echo "Proposed DQN monitor process started with PID: $PROP_MONITOR_PID"

# --- 2. DQN-Baseline (Baseline: BWP only) ---
echo "--- (2/2) Launching Baseline DQN training ---"
BASE_RUN_NAME="${RUN_NAME_BASE}/baseline_drqn"
BASE_LOG="${RUN_DIR}/baseline_dqn_train.log"
BASE_MONITOR_LOG="${RUN_DIR}/baseline_dqn_monitor.log"
BASE_PORT=5556

BASE_CMD=(
  "$PYTHON_BIN" -u "${REPO_ROOT}/scratch/rl_bwp/train_dqn.py"
  --run-name "${BASE_RUN_NAME}"
  --num-ues "$NUM_UES" --episode-time-s "$EPISODE_TIME_S" --step-time-s "$STEP_TIME_S"
  --min-completed-episodes "$MIN_COMPLETED_EPISODES"
  --reward-bin-size "$PER_N_STEP_CSV"
  --loss-bin-size "$VALUE_LOSS_N_STEP"
  --save-every-episodes "$CHECKPOINT_EVERY_EPISODES"
  --port "$BASE_PORT"
  --ns3-script "aoi-prb-urban-appmix"
  --ns3-arg "schedulerPolicy=${SCHEDULER_POLICY}"
  --ns3-arg "enableHarqReTx=${ENABLE_HARQ_RETX}"
  --drqn-profile
  --backend "ns3"
)
"${BASE_CMD[@]}" >"$BASE_LOG" 2>&1 &
BASE_PID=$!
echo "Baseline DQN agent process started with PID: $BASE_PID"

nohup "$MONITOR" "$BASE_PID" "$BASE_PORT" 10 5 \
  "aoi-prb-urban-appmix" \
  --numUes="$NUM_UES" --simTime="$EPISODE_TIME_S" --envStepTime="$STEP_TIME_S" \
  --enableOpenGym=true --trafficModel=mixed --schedulerPolicy="$SCHEDULER_POLICY" \
  --rlDrqnProfile=true --enableRlMcsControl=false --enableHarqReTx="$ENABLE_HARQ_RETX" \
  >"$BASE_MONITOR_LOG" 2>&1 &
BASE_MONITOR_PID=$!
echo "Baseline DQN monitor process started with PID: $BASE_MONITOR_PID"


wait "$PROP_PID"
echo "--- Proposed DQN training finished. Killing monitor... ---"
kill "$PROP_MONITOR_PID" 2>/dev/null || true
pkill -f "openGymPort=$PROP_PORT" || true
sleep 5

wait "$BASE_PID"
echo "--- Baseline DQN training finished. Killing monitor... ---"
kill "$BASE_MONITOR_PID" 2>/dev/null || true
pkill -f "openGymPort=$BASE_PORT" || true

echo "--- All training complete. ---"
