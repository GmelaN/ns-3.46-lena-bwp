#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PYTHON_BIN="${REPO_ROOT}/ns3gym-venv/bin/python"
TRAIN_PY="${REPO_ROOT}/scratch/rl_bwp/train_drqn.py"
MONITOR_SH="${REPO_ROOT}/scratch/rl_bwp/monitor_ns3_reconnector.sh"

RUN_NAME="drqn"
SEED=1
NUM_UES=20
STEP_TIME_S=0.01
EPISODE_TIME_S=10.0
TOTAL_ENV_STEPS=100000
MIN_COMPLETED_EPISODES=10
PORT=55555
START_SIM=true
NS3_SCRIPT="aoi-prb-urban-appmix"
CHECK_INTERVAL_S=10
MAX_MISSING_CHECKS=5

EXTRA_NS3_ARGS=()

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --run-name NAME
  --seed N
  --num-ues N
  --step-time-s FLOAT
  --episode-time-s FLOAT
  --total-env-steps N
  --min-completed-episodes N
  --port N
  --ns3-script NAME
  --ns3-arg key=value              (repeatable)
  --check-interval-s N
  --max-missing-checks N
  --no-start-sim
  -h, --help

Example:
  $0 --run-name drqn_appmix --seed 7 --port 55555 \\
     --num-ues 20 --step-time-s 0.01 --episode-time-s 10.0 \\
     --total-env-steps 50000 --min-completed-episodes 50 \\
     --ns3-arg appStart=0.1
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-name) RUN_NAME="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --num-ues) NUM_UES="$2"; shift 2 ;;
    --step-time-s) STEP_TIME_S="$2"; shift 2 ;;
    --episode-time-s) EPISODE_TIME_S="$2"; shift 2 ;;
    --total-env-steps) TOTAL_ENV_STEPS="$2"; shift 2 ;;
    --min-completed-episodes) MIN_COMPLETED_EPISODES="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --ns3-script) NS3_SCRIPT="$2"; shift 2 ;;
    --ns3-arg) EXTRA_NS3_ARGS+=("$2"); shift 2 ;;
    --check-interval-s) CHECK_INTERVAL_S="$2"; shift 2 ;;
    --max-missing-checks) MAX_MISSING_CHECKS="$2"; shift 2 ;;
    --no-start-sim) START_SIM=false; shift 1 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python not found: ${PYTHON_BIN}" >&2
  exit 1
fi

if [[ ! -f "${TRAIN_PY}" ]]; then
  echo "Train script not found: ${TRAIN_PY}" >&2
  exit 1
fi

if [[ ! -x "${MONITOR_SH}" ]]; then
  echo "Monitor script not executable: ${MONITOR_SH}" >&2
  exit 1
fi

LOG_DIR="${REPO_ROOT}/scratch/rl_bwp/runs/${RUN_NAME}/drqn"
mkdir -p "${LOG_DIR}"

TRAIN_LOG="${LOG_DIR}/train_stdout.log"
MONITOR_LOG="${LOG_DIR}/monitor_stdout.log"

pkill -f "openGymPort=${PORT}" || true
sleep 1

TRAIN_CMD=(
  "${PYTHON_BIN}" -u "${TRAIN_PY}"
  --backend ns3
  --run-name "${RUN_NAME}"
  --seed "${SEED}"
  --num-ues "${NUM_UES}"
  --step-time-s "${STEP_TIME_S}"
  --episode-time-s "${EPISODE_TIME_S}"
  --total-env-steps "${TOTAL_ENV_STEPS}"
  --min-completed-episodes "${MIN_COMPLETED_EPISODES}"
  --port "${PORT}"
  --ns3-script "${NS3_SCRIPT}"
)

if [[ "${START_SIM}" == "true" ]]; then
  TRAIN_CMD+=(--start-sim)
fi

for arg in "${EXTRA_NS3_ARGS[@]}"; do
  TRAIN_CMD+=(--ns3-arg "${arg}")
done

echo "Launching train_drqn.py on port ${PORT}"
"${TRAIN_CMD[@]}" >"${TRAIN_LOG}" 2>&1 &
TRAIN_PID=$!
echo "train pid=${TRAIN_PID}, log=${TRAIN_LOG}"

MON_NS3_ARGS=(
  "${NS3_SCRIPT}"
  "--numUes=${NUM_UES}"
  "--simTime=${EPISODE_TIME_S}"
  "--envStepTime=${STEP_TIME_S}"
)

for arg in "${EXTRA_NS3_ARGS[@]}"; do
  MON_NS3_ARGS+=("--${arg}")
done

MON_NS3_ARGS+=(
  "--enableOpenGym=true"
  "--rlDrqnProfile=true"
  "--enableRlMcsControl=false"
)

MONITOR_TAG="${RUN_NAME}" nohup "${MONITOR_SH}" \
  "${TRAIN_PID}" "${PORT}" "${CHECK_INTERVAL_S}" "${MAX_MISSING_CHECKS}" \
  "${MON_NS3_ARGS[@]}" >"${MONITOR_LOG}" 2>&1 &
MONITOR_PID=$!
echo "monitor pid=${MONITOR_PID}, log=${MONITOR_LOG}"

wait "${TRAIN_PID}"
TRAIN_RC=$?

kill "${MONITOR_PID}" 2>/dev/null || true
pkill -f "openGymPort=${PORT}" || true

echo "train exited with code ${TRAIN_RC}"
exit "${TRAIN_RC}"
