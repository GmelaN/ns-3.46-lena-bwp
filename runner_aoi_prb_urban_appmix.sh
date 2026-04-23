#!/usr/bin/env bash
set -euo pipefail

# Shell runner for scratch/aoi-prb-urban-appmix
# Scope: execution orchestration only (no CSV parsing/stat summary)

/home/jshyeon/src/ns-3.46-bwp-temp/ns3 build

RUNS=1
NUM_UES=20
SIM_TIME=30.0
APP_START=0.1
ENV_STEP_TIME=0.02
BWP_BASELINE="dpp"
BWP_QUEUE_THRESHOLD=800
SCHEDULER_POLICY="rr"
AGE_OPTIMAL_GAMMA_PENALTY=20.0
TPS_DEADLINE_MS=100.0
DGS_DELAY_TARGET_MS=100.0
DPP_V=10
DPP_LAMBDA_SWITCH=1.0
DPP_LAMBDA_BLER=1.0
DPP_EPOCH_MIN_INTERVAL_S=0.01
APP_LOAD_SCALE=1.0
ENQUEUE_RELAX_FACTOR=1.0
RANDOM_SEED=1
RANDOM_RUN=1
RUN_16_COMBOS=0
INCLUDE_DPP=0
SWEEP_DPP=0
SWEEP_DPP_V_LIST="0.5"
SWEEP_DPP_LAMBDA_SWITCH_LIST="0.005"
SWEEP_DPP_LAMBDA_BLER_LIST="1.0"
MAX_WORKERS=4
DRQN_MODEL_PATH="scratch/rl_bwp/runs/drqn/final_model.pt"
PYTHON_BIN="ns3gym-venv/bin/python"
OPEN_GYM_PORT_BASE=5600
OUT_DIR=""
ENABLE_MCS_SWITCH=0
INITIAL_BWP_ID=""
CASE_TIMEOUT_S=0
TIMEOUT_RETRIES=1

EXTRA_ARGS=()
FAILED_JOBS=0

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
  --app-load-scale X
  --enqueue-relax-factor X
  --random-seed N
  --random-run N

Batch modes:
  --run-16-combos
  --include-dpp   (alone: run DPP with 4 schedulers; with --run-16-combos: add DPP mode)
  --sweep-dpp
  --sweep-dpp-v-list CSV
  --sweep-dpp-lambda-switch-list CSV
  --sweep-dpp-lambda-bler-list CSV
  --max-workers N
  --drqn-model-path PATH
  --python-bin PATH
  --open-gym-port-base N
  --out-dir PATH
  --case-timeout-s N    (0 disables timeout; recommended for long batch runs)
  --timeout-retries N   (retry count for timeout-killed jobs)

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
    --app-load-scale) APP_LOAD_SCALE="$2"; shift 2 ;;
    --enqueue-relax-factor) ENQUEUE_RELAX_FACTOR="$2"; shift 2 ;;
    --random-seed) RANDOM_SEED="$2"; shift 2 ;;
    --random-run) RANDOM_RUN="$2"; shift 2 ;;
    --run-16-combos) RUN_16_COMBOS=1; shift ;;
    --include-dpp) INCLUDE_DPP=1; shift ;;
    --sweep-dpp) SWEEP_DPP=1; shift ;;
    --sweep-dpp-v-list) SWEEP_DPP_V_LIST="$2"; shift 2 ;;
    --sweep-dpp-lambda-switch-list) SWEEP_DPP_LAMBDA_SWITCH_LIST="$2"; shift 2 ;;
    --sweep-dpp-lambda-bler-list) SWEEP_DPP_LAMBDA_BLER_LIST="$2"; shift 2 ;;
    --max-workers) MAX_WORKERS="$2"; shift 2 ;;
    --drqn-model-path) DRQN_MODEL_PATH="$2"; shift 2 ;;
    --python-bin) PYTHON_BIN="$2"; shift 2 ;;
    --open-gym-port-base) OPEN_GYM_PORT_BASE="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --case-timeout-s) CASE_TIMEOUT_S="$2"; shift 2 ;;
    --timeout-retries) TIMEOUT_RETRIES="$2"; shift 2 ;;
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
NEXT_OPEN_GYM_PORT="$OPEN_GYM_PORT_BASE"
ALLOCATED_OPEN_GYM_PORT="$OPEN_GYM_PORT_BASE"

alloc_open_gym_port() {
  ALLOCATED_OPEN_GYM_PORT="$NEXT_OPEN_GYM_PORT"
  NEXT_OPEN_GYM_PORT=$((NEXT_OPEN_GYM_PORT + 1))
}

throttle_jobs() {
  while true; do
    local running
    running=$(jobs -rp | wc -l | tr -d ' ')
    if (( running < MAX_WORKERS )); then
      break
    fi
    if ! wait -n; then
      FAILED_JOBS=$((FAILED_JOBS + 1))
    fi
  done
}

wait_all_jobs() {
  while true; do
    local running
    running=$(jobs -rp | wc -l | tr -d ' ')
    if (( running == 0 )); then
      break
    fi
    if ! wait -n; then
      FAILED_JOBS=$((FAILED_JOBS + 1))
    fi
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
  local attempt=0
  local rc=0

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
    "--appLoadScale=${APP_LOAD_SCALE}"
    "--enqueueRelaxFactor=${ENQUEUE_RELAX_FACTOR}"
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

    : > "$log_path"
    echo "[$case_name run ${run_idx}] ${full_cmd[*]}"
    while true; do
      attempt=$((attempt + 1))
      {
        echo "[runner] attempt=${attempt} driver=${driver} case=${case_name} run=${run_idx} port=${open_gym_port}"
        if [[ "${CASE_TIMEOUT_S}" != "0" ]]; then
          timeout --signal=TERM --kill-after=30s "${CASE_TIMEOUT_S}s" "${full_cmd[@]}"
        else
          "${full_cmd[@]}"
        fi
      } >>"$log_path" 2>&1
      rc=$?
      if (( rc == 0 )); then
        break
      fi
      if [[ "${CASE_TIMEOUT_S}" != "0" ]] && (( (rc == 124 || rc == 137) && attempt <= TIMEOUT_RETRIES )); then
        echo "[$case_name run ${run_idx}] timeout(rc=${rc}) retry ${attempt}/${TIMEOUT_RETRIES}" >&2
        sleep 1
        continue
      fi
      return "$rc"
    done
  else
    local bin_path="build/scratch/ns3.46-aoi-prb-urban-appmix-optimized"
    if [[ ! -x "$bin_path" ]]; then
      echo "Binary not found or not executable: $bin_path" >&2
      echo "Build first: ./ns3 build --build-profile=optimized" >&2
      exit 1
    fi
    local full_cmd=("$bin_path" "${cmd_args[@]}")
    : > "$log_path"
    echo "[$case_name run ${run_idx}] ${full_cmd[*]}"
    while true; do
      attempt=$((attempt + 1))
      {
        echo "[runner] attempt=${attempt} driver=${driver} case=${case_name} run=${run_idx}"
        if [[ "${CASE_TIMEOUT_S}" != "0" ]]; then
          timeout --signal=TERM --kill-after=30s "${CASE_TIMEOUT_S}s" "${full_cmd[@]}"
        else
          "${full_cmd[@]}"
        fi
      } >>"$log_path" 2>&1
      rc=$?
      if (( rc == 0 )); then
        break
      fi
      if [[ "${CASE_TIMEOUT_S}" != "0" ]] && (( (rc == 124 || rc == 137) && attempt <= TIMEOUT_RETRIES )); then
        echo "[$case_name run ${run_idx}] timeout(rc=${rc}) retry ${attempt}/${TIMEOUT_RETRIES}" >&2
        sleep 1
        continue
      fi
      return "$rc"
    done
  fi

  echo "[$case_name run ${run_idx}] done: ${summary_path}"
}

if (( SWEEP_DPP == 1 )); then
  IFS=',' read -r -a v_list <<< "$SWEEP_DPP_V_LIST"
  IFS=',' read -r -a lsw_list <<< "$SWEEP_DPP_LAMBDA_SWITCH_LIST"
  IFS=',' read -r -a lbler_list <<< "$SWEEP_DPP_LAMBDA_BLER_LIST"
  for ((i=1; i<=RUNS; i++)); do
    rr=$((RANDOM_RUN + i - 1))
    for v in "${v_list[@]}"; do
      for lsw in "${lsw_list[@]}"; do
        for lbler in "${lbler_list[@]}"; do
          case_name="dpp_v${v}_lsw${lsw}_lbler${lbler}"
          alloc_open_gym_port
          open_gym_port="$ALLOCATED_OPEN_GYM_PORT"
          throttle_jobs
          DPP_V="$v" DPP_LAMBDA_SWITCH="$lsw" DPP_LAMBDA_BLER="$lbler" \
            run_one "$i" "$case_name" "ns3" "dpp" "$SCHEDULER_POLICY" "$INITIAL_BWP_ID" 0 "$rr" "$open_gym_port" &
        done
      done
    done
  done
  wait_all_jobs
elif (( RUN_16_COMBOS == 1 )); then
  bwp_modes=("BWP0ONLY" "BWP1ONLY" "threshold" "DRQN")
  if (( INCLUDE_DPP == 1 )); then
    bwp_modes+=("DPP")
  fi
  schedulers=("rr" "aequitas" "tps" "age_optimal")

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

        alloc_open_gym_port
        open_gym_port="$ALLOCATED_OPEN_GYM_PORT"

        throttle_jobs
        run_one "$i" "$case_name" "$driver" "$bwp_baseline" "$sched" "$initial_bwp" "$enable_mcs_switch" "$rr" "$open_gym_port" &
      done
    done
  done
  wait_all_jobs
elif (( INCLUDE_DPP == 1 )); then
  schedulers=("rr" "aequitas" "tps" "age_optimal")
  for ((i=1; i<=RUNS; i++)); do
    rr=$((RANDOM_RUN + i - 1))
    for sched in "${schedulers[@]}"; do
      case_name="dpp_${sched}"
      alloc_open_gym_port
      open_gym_port="$ALLOCATED_OPEN_GYM_PORT"
      throttle_jobs
      run_one "$i" "$case_name" "ns3" "dpp" "$sched" "$INITIAL_BWP_ID" 0 "$rr" "$open_gym_port" &
    done
  done
  wait_all_jobs
else
  for ((i=1; i<=RUNS; i++)); do
    rr=$((RANDOM_RUN + i - 1))
    alloc_open_gym_port
    open_gym_port="$ALLOCATED_OPEN_GYM_PORT"
    throttle_jobs
    run_one "$i" "default" "ns3" "$BWP_BASELINE" "$SCHEDULER_POLICY" "$INITIAL_BWP_ID" "$ENABLE_MCS_SWITCH" "$rr" "$open_gym_port" &
  done
  wait_all_jobs
fi

echo "All runs completed. OUT_DIR=${OUT_DIR}"
if (( FAILED_JOBS > 0 )); then
  echo "Completed with ${FAILED_JOBS} failed run(s). Check per-run logs under ${OUT_DIR}." >&2
  exit 1
fi
