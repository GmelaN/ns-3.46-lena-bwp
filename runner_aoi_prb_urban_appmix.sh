#!/usr/bin/env bash
set -euo pipefail

# Shell runner for scratch/aoi-prb-urban-appmix
# Scope: execution orchestration only (no CSV parsing/stat summary)

RUNS=1
NUM_UES=20
SIM_TIME=5.0
APP_START=0.1
ENV_STEP_TIME=0.02
BWP_BASELINE="dpp"
BWP_QUEUE_THRESHOLD=800
SCHEDULER_POLICY="rr"
AGE_OPTIMAL_GAMMA_PENALTY=20.0
TPS_DEADLINE_MS=100.0
DGS_DELAY_TARGET_MS=100.0
DPP_V=0.5
DPP_LAMBDA_SWITCH=0.005
DPP_LAMBDA_BLER=1.0
DPP_EPOCH_MIN_INTERVAL_S=0.01
RANDOM_SEED=1
RANDOM_RUN=1
RUN_16_COMBOS=0
INCLUDE_DPP=0
MAX_WORKERS=4
DRQN_MODEL_PATH="scratch/rl_bwp/runs/drqn/final_model.pt"
PYTHON_BIN="ns3gym-venv/bin/python"
OPEN_GYM_PORT_BASE=5600
OUT_DIR=""
ENABLE_MCS_SWITCH=0
INITIAL_BWP_ID=""

EXTRA_ARGS=()

usage() {
  cat << USAGE
Usage: $0 [options] [--extra ...]

Core options:
  --runs N
  --num-ues N
  --sim-time S
  --app-start S
  --env-step-time S
  --bwp-baseline STR
  --bwp-queue-threshold N
  --scheduler-policy STR
  --age-optimal-gamma-penalty X
  --tps-deadline-ms X
  --dgs-delay-target-ms X
  --dpp-v X
  --dpp-lambda-switch X
  --dpp-lambda-bler X
  --dpp-epoch-min-interval-s X
  --random-seed N
  --random-run N

Batch modes:
  --run-16-combos
  --include-dpp
  --max-workers N
  --drqn-model-path PATH
  --python-bin PATH
  --open-gym-port-base N
  --out-dir PATH

Single-run helpers:
  --enable-mcs-switch 0|1
  --initial-bwp-id 0|1

Pass-through:
  --extra [args forwarded to ns3 script]
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runs) RUNS="$2"; shift 2 ;;
    --num-ues) NUM_UES="$2"; shift 2 ;;
    --sim-time) SIM_TIME="$2"; shift 2 ;;
    --app-start) APP_START="$2"; shift 2 ;;
    --env-step-time) ENV_STEP_TIME="$2"; shift 2 ;;
    --bwp-baseline) BWP_BASELINE="$2"; shift 2 ;;
    --bwp-queue-threshold) BWP_QUEUE_THRESHOLD="$2"; shift 2 ;;
    --scheduler-policy) SCHEDULER_POLICY="$2"; shift 2 ;;
    --age-optimal-gamma-penalty) AGE_OPTIMAL_GAMMA_PENALTY="$2"; shift 2 ;;
    --tps-deadline-ms) TPS_DEADLINE_MS="$2"; shift 2 ;;
    --dgs-delay-target-ms) DGS_DELAY_TARGET_MS="$2"; shift 2 ;;
    --dpp-v) DPP_V="$2"; shift 2 ;;
    --dpp-lambda-switch) DPP_LAMBDA_SWITCH="$2"; shift 2 ;;
    --dpp-lambda-bler) DPP_LAMBDA_BLER="$2"; shift 2 ;;
    --dpp-epoch-min-interval-s) DPP_EPOCH_MIN_INTERVAL_S="$2"; shift 2 ;;
    --random-seed) RANDOM_SEED="$2"; shift 2 ;;
    --random-run) RANDOM_RUN="$2"; shift 2 ;;
    --run-16-combos) RUN_16_COMBOS=1; shift ;;
    --include-dpp) INCLUDE_DPP=1; shift ;;
    --max-workers) MAX_WORKERS="$2"; shift 2 ;;
    --drqn-model-path) DRQN_MODEL_PATH="$2"; shift 2 ;;
    --python-bin) PYTHON_BIN="$2"; shift 2 ;;
    --open-gym-port-base) OPEN_GYM_PORT_BASE="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --enable-mcs-switch) ENABLE_MCS_SWITCH="$2"; shift 2 ;;
    --initial-bwp-id) INITIAL_BWP_ID="$2"; shift 2 ;;
    --extra)
      shift
      EXTRA_ARGS=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$OUT_DIR" ]]; then
  TS="$(date +%Y%m%d_%H%M%S)"
  OUT_DIR="scratch/rl_bwp/runs/runner_${TS}"
fi
mkdir -p "$OUT_DIR"

throttle_jobs() {
  while true; do
    local running
    running=$(jobs -rp | wc -l | tr -d ' ')
    if (( running < MAX_WORKERS )); then
      break
    fi
    wait -n || true
  done
}

run_one() {
  local run_idx="$1"
  local case_name="$2"
  local driver="$3"
  local bwp_baseline="$4"
  local scheduler="$5"
  local initial_bwp="$6"
  local enable_mcs_switch="$7"
  local random_run="$8"
  local open_gym_port="$9"

  local summary_path="${OUT_DIR}/summary_${case_name}_run${run_idx}.csv"
  local log_path="${OUT_DIR}/log_${case_name}_run${run_idx}.txt"

  local cmd_args=(
    "--numUes=${NUM_UES}"
    "--simTime=${SIM_TIME}"
    "--appStart=${APP_START}"
    "--bwpBaseline=${bwp_baseline}"
    "--bwpQueueThreshold=${BWP_QUEUE_THRESHOLD}"
    "--envStepTime=${ENV_STEP_TIME}"
    "--schedulerPolicy=${scheduler}"
    "--ageOptimalGammaPenalty=${AGE_OPTIMAL_GAMMA_PENALTY}"
    "--tpsDeadlineMs=${TPS_DEADLINE_MS}"
    "--dgsDelayTargetMs=${DGS_DELAY_TARGET_MS}"
    "--summaryFile=${summary_path}"
    "--dppV=${DPP_V}"
    "--dppLambdaSwitch=${DPP_LAMBDA_SWITCH}"
    "--dppLambdaBler=${DPP_LAMBDA_BLER}"
    "--dppEpochMinIntervalS=${DPP_EPOCH_MIN_INTERVAL_S}"
    "--randomSeed=${RANDOM_SEED}"
    "--randomRun=${random_run}"
  )

  if [[ -n "$initial_bwp" ]]; then
    cmd_args+=("--initialBwpId=${initial_bwp}")
  fi
  cmd_args+=("--enableMcsSwitch=${enable_mcs_switch}")

  if (( ${#EXTRA_ARGS[@]} > 0 )); then
    cmd_args+=("${EXTRA_ARGS[@]}")
  fi

  if [[ "$driver" == "drqn" ]]; then
    local full_cmd=(
      "$PYTHON_BIN"
      "scratch/rl_bwp/eval_drqn.py"
      "--backend" "ns3"
      "--model-path" "$DRQN_MODEL_PATH"
      "--episodes" "1"
      "--seed" "$RANDOM_SEED"
      "--num-ues" "$NUM_UES"
      "--step-time-s" "$ENV_STEP_TIME"
      "--episode-time-s" "$SIM_TIME"
      "--port" "$open_gym_port"
      "--start-sim"
      "--ns3-script" "aoi-prb-urban-appmix"
    )
    local kv
    for kv in "${cmd_args[@]}"; do
      full_cmd+=("--ns3-arg" "${kv#--}")
    done

    echo "[$case_name run ${run_idx}] ${full_cmd[*]}"
    "${full_cmd[@]}" >"$log_path" 2>&1
  else
    local run_str="scratch/aoi-prb-urban-appmix ${cmd_args[*]}"
    local full_cmd=("./ns3" "run" "$run_str")
    echo "[$case_name run ${run_idx}] ${full_cmd[*]}"
    "${full_cmd[@]}" >"$log_path" 2>&1
  fi

  echo "[$case_name run ${run_idx}] done: ${summary_path}"
}

if (( RUN_16_COMBOS == 1 )); then
  bwp_modes=("BWP0ONLY" "BWP1ONLY" "threshold" "DRQN")
  if (( INCLUDE_DPP == 1 )); then
    bwp_modes+=("DPP")
  fi
  schedulers=("rr" "aequitas" "tps" "dgs")

  case_idx=0
  for ((i=1; i<=RUNS; i++)); do
    rr=$((RANDOM_RUN + i - 1))
    for bwp in "${bwp_modes[@]}"; do
      for sched in "${schedulers[@]}"; do
        case_name="$(echo "${bwp}_${sched}" | tr '[:upper:]' '[:lower:]')"

        driver="ns3"
        bwp_baseline="none"
        initial_bwp=""
        enable_mcs_switch=0

        case "$bwp" in
          BWP0ONLY)
            bwp_baseline="none"
            initial_bwp="0"
            ;;
          BWP1ONLY)
            bwp_baseline="none"
            initial_bwp="1"
            ;;
          threshold)
            bwp_baseline="queue"
            ;;
          DRQN)
            driver="drqn"
            bwp_baseline="none"
            ;;
          DPP)
            bwp_baseline="dpp"
            ;;
        esac

        open_gym_port=$((OPEN_GYM_PORT_BASE + (i - 1) * ${#bwp_modes[@]} * ${#schedulers[@]} + case_idx))
        case_idx=$((case_idx + 1))

        throttle_jobs
        run_one "$i" "$case_name" "$driver" "$bwp_baseline" "$sched" "$initial_bwp" "$enable_mcs_switch" "$rr" "$open_gym_port" &
      done
    done
  done
  wait
else
  for ((i=1; i<=RUNS; i++)); do
    rr=$((RANDOM_RUN + i - 1))
    throttle_jobs
    run_one "$i" "default" "ns3" "$BWP_BASELINE" "$SCHEDULER_POLICY" "$INITIAL_BWP_ID" "$ENABLE_MCS_SWITCH" "$rr" "$OPEN_GYM_PORT_BASE" &
  done
  wait
fi

echo "All runs completed. OUT_DIR=${OUT_DIR}"
