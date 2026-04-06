#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${1:-scratch/rl_bwp/runs/perf_extract_appmix_$(date -u +%Y%m%d_%H%M%S)}"
NUM_UES="${2:-20}"
SIM_TIME_S="${3:-5}"
TRAIN_STEP_TIME_S="${4:-0.05}"
TRAIN_EPISODES="${5:-1000}"
EVAL_EPISODES="${6:-5}"
SEED="${7:-1}"
BATCH_SIZE="${8:-32}"
EVAL_STEP_TIME_S="${9:-0.01}"
REWARD_BIN_STEPS="${10:-250}"
SCENARIO="${11:-appmix}"
PORT_BASE="${12:-58011}"

if [[ "$SCENARIO" == "onoff" ]]; then
  NS3_SCRIPT="aoi-prb-urban-onoff"
elif [[ "$SCENARIO" == "appmix" ]]; then
  NS3_SCRIPT="aoi-prb-urban-appmix"
else
  echo "Unsupported scenario: $SCENARIO (expected: appmix|onoff)"
  exit 2
fi

mkdir -p "$ROOT_DIR"

DRQN_PF_RUN="drqn_pf_${SCENARIO}_ue${NUM_UES}_sim${SIM_TIME_S}_trstep${TRAIN_STEP_TIME_S}_ep${TRAIN_EPISODES}_seed${SEED}"
RQR_PF_RUN="rqrdqn_pf_${SCENARIO}_ue${NUM_UES}_sim${SIM_TIME_S}_trstep${TRAIN_STEP_TIME_S}_ep${TRAIN_EPISODES}_seed${SEED}"
DRQN_AEQ_RUN="drqn_aequitas_${SCENARIO}_ue${NUM_UES}_sim${SIM_TIME_S}_trstep${TRAIN_STEP_TIME_S}_ep${TRAIN_EPISODES}_seed${SEED}"

DRQN_PF_PORT_TRAIN="${PORT_BASE}"
DRQN_PF_PORT_EVAL="$((PORT_BASE + 1))"
RQR_PF_PORT_TRAIN="$((PORT_BASE + 1000))"
RQR_PF_PORT_EVAL="$((PORT_BASE + 1001))"
DRQN_AEQ_PORT_TRAIN="$((PORT_BASE + 2000))"
DRQN_AEQ_PORT_EVAL="$((PORT_BASE + 2001))"

echo "[phase] train_parallel_start"

declare -a PIDS=()
declare -a LABELS=()

launch_bg() {
  local label="$1"
  shift
  echo "[start] $label"
  "$@" &
  local pid=$!
  PIDS+=("$pid")
  LABELS+=("$label")
}

wait_all_or_fail() {
  local rc=0
  local i
  for i in "${!PIDS[@]}"; do
    local pid="${PIDS[$i]}"
    local label="${LABELS[$i]}"
    if wait "$pid"; then
      echo "[done] $label"
    else
      echo "[fail] $label (pid=$pid)"
      rc=1
    fi
  done
  if [[ "$rc" -ne 0 ]]; then
    echo "[error] one or more background jobs failed"
    exit 1
  fi
  PIDS=()
  LABELS=()
}

# 1) Train DRQN + PF (parallel)
launch_bg "train_drqn_pf" \
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
    --backend ns3 \
    --run-name "$DRQN_PF_RUN" \
    --seed "$SEED" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$SIM_TIME_S" \
    --step-time-s "$TRAIN_STEP_TIME_S" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --batch-size "$BATCH_SIZE" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --port "$DRQN_PF_PORT_TRAIN"

# 2) Train RQR-DQN + PF (parallel)
launch_bg "train_rqrdqn_pf" \
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_rqrdqn.py \
    --backend ns3 \
    --run-name "$RQR_PF_RUN" \
    --seed "$SEED" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$SIM_TIME_S" \
    --step-time-s "$TRAIN_STEP_TIME_S" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --batch-size "$BATCH_SIZE" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --port "$RQR_PF_PORT_TRAIN"

# 3) Train DRQN + Aequitas scheduler (parallel)
launch_bg "train_drqn_aequitas" \
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
    --backend ns3 \
    --run-name "$DRQN_AEQ_RUN" \
    --seed "$SEED" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$SIM_TIME_S" \
    --step-time-s "$TRAIN_STEP_TIME_S" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --batch-size "$BATCH_SIZE" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=aequitas \
    --port "$DRQN_AEQ_PORT_TRAIN"

wait_all_or_fail
echo "[phase] train_parallel_done"

echo "[phase] eval_baseline_parallel_start"

# 4) Eval DRQN + PF (parallel)
launch_bg "eval_drqn_pf" \
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
    --backend ns3 \
    --model-path "scratch/rl_bwp/runs/$DRQN_PF_RUN/final_model.pt" \
    --episodes "$EVAL_EPISODES" \
    --seed "$SEED" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$SIM_TIME_S" \
    --step-time-s "$EVAL_STEP_TIME_S" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --port "$DRQN_PF_PORT_EVAL" \
    --output-json "$ROOT_DIR/eval_drqn_pf.json" \
    --step-log-csv "$ROOT_DIR/eval_drqn_pf_step_log.csv"

# 5) Eval RQR-DQN + PF (parallel)
launch_bg "eval_rqrdqn_pf" \
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_rqrdqn.py \
    --backend ns3 \
    --model-path "scratch/rl_bwp/runs/$RQR_PF_RUN/final_model.pt" \
    --episodes "$EVAL_EPISODES" \
    --seed "$SEED" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$SIM_TIME_S" \
    --step-time-s "$EVAL_STEP_TIME_S" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --port "$RQR_PF_PORT_EVAL" \
    --output-json "$ROOT_DIR/eval_rqrdqn_pf.json" \
    --step-log-csv "$ROOT_DIR/eval_rqrdqn_pf_step_log.csv"

# 6) Eval DRQN + Aequitas (parallel)
launch_bg "eval_drqn_aequitas" \
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
    --backend ns3 \
    --model-path "scratch/rl_bwp/runs/$DRQN_AEQ_RUN/final_model.pt" \
    --episodes "$EVAL_EPISODES" \
    --seed "$SEED" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$SIM_TIME_S" \
    --step-time-s "$EVAL_STEP_TIME_S" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=aequitas \
    --port "$DRQN_AEQ_PORT_EVAL" \
    --output-json "$ROOT_DIR/eval_drqn_aequitas.json" \
    --step-log-csv "$ROOT_DIR/eval_drqn_aequitas_step_log.csv"

# 7) Baseline BWP0 + Aequitas (parallel)
launch_bg "baseline_bwp0_aequitas" \
  ./ns3 run "scratch/${NS3_SCRIPT} --numUes=${NUM_UES} --simTime=${SIM_TIME_S} --envStepTime=${EVAL_STEP_TIME_S} --enableOpenGym=false --enableRlBwpControl=false --schedulerPolicy=aequitas --trafficModel=mixed --initialBwp=0 --summaryFile=${ROOT_DIR}/baseline_bwp0_aequitas_summary.txt"

# 8) Baseline BWP1 + Aequitas (parallel)
launch_bg "baseline_bwp1_aequitas" \
  ./ns3 run "scratch/${NS3_SCRIPT} --numUes=${NUM_UES} --simTime=${SIM_TIME_S} --envStepTime=${EVAL_STEP_TIME_S} --enableOpenGym=false --enableRlBwpControl=false --schedulerPolicy=aequitas --trafficModel=mixed --initialBwp=1 --summaryFile=${ROOT_DIR}/baseline_bwp1_aequitas_summary.txt"

wait_all_or_fail
echo "[phase] eval_baseline_parallel_done"

./ns3gym-venv/bin/python3 scratch/rl_bwp/postprocess_perf_extract.py \
  --root-dir "$ROOT_DIR" \
  --drqn-pf-run "$DRQN_PF_RUN" \
  --rqrdqn-pf-run "$RQR_PF_RUN" \
  --drqn-aequitas-run "$DRQN_AEQ_RUN" \
  --reward-bin-size "$REWARD_BIN_STEPS"

echo "DONE_ROOT=$ROOT_DIR"
