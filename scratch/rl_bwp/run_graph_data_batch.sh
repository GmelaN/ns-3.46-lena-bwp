#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: $0 <scenario:onoff_mixed|appmix> <root_run_dir>" >&2
  exit 1
fi

SCENARIO="$1"
ROOT="$2"
mkdir -p "$ROOT"

NUM_UES=20
TRAIN_SIM=10
EVAL_SIM=20
STEP=0.05
TRAIN_EPISODES=5
TOTAL_ENV_STEPS=20000

case "$SCENARIO" in
  onoff_mixed)
    NS3_SCRIPT="aoi-prb-urban-onoff"
    COMMON_ARGS=(--ns3-arg trafficModel=mixed --ns3-arg burstRateMbps=100 --ns3-arg backgroundRateKbps=500)
    BASELINE_EXTRA="--trafficModel=mixed --burstRateMbps=100 --backgroundRateKbps=500"
    PORT_BASE=59000
    ;;
  appmix)
    NS3_SCRIPT="aoi-prb-urban-appmix"
    COMMON_ARGS=(--ns3-arg trafficModel=mixed)
    BASELINE_EXTRA="--trafficModel=mixed"
    PORT_BASE=59200
    ;;
  *)
    echo "unknown scenario: $SCENARIO" >&2
    exit 1
    ;;
esac

run_train_drqn() {
  local run_name="$1"
  local scheduler="$2"
  local aeq_mcs="$3"
  local port="$4"
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
    --backend ns3 \
    --run-name "$run_name" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    "${COMMON_ARGS[@]}" \
    --ns3-arg schedulerPolicy="$scheduler" \
    --ns3-arg aequitasEnableMcsSelection="$aeq_mcs" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$TRAIN_SIM" \
    --step-time-s "$STEP" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --total-env-steps "$TOTAL_ENV_STEPS" \
    --port "$port"
}

run_train_rqrdqn() {
  local run_name="$1"
  local port="$2"
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_rqrdqn.py \
    --backend ns3 \
    --run-name "$run_name" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    "${COMMON_ARGS[@]}" \
    --ns3-arg schedulerPolicy=pf \
    --num-ues "$NUM_UES" \
    --episode-time-s "$TRAIN_SIM" \
    --step-time-s "$STEP" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --total-env-steps "$TOTAL_ENV_STEPS" \
    --port "$port"
}

run_eval_drqn() {
  local model_path="$1"
  local output_json="$2"
  local scheduler="$3"
  local aeq_mcs="$4"
  local port="$5"
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
    --backend ns3 \
    --model-path "$model_path" \
    --seed 1 \
    --episodes 1 \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EVAL_SIM" \
    --step-time-s "$STEP" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    "${COMMON_ARGS[@]}" \
    --ns3-arg schedulerPolicy="$scheduler" \
    --ns3-arg aequitasEnableMcsSelection="$aeq_mcs" \
    --port "$port" \
    --output-json "$output_json"
}

run_eval_rqrdqn() {
  local model_path="$1"
  local output_json="$2"
  local port="$3"
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_rqrdqn.py \
    --backend ns3 \
    --model-path "$model_path" \
    --seed 1 \
    --episodes 1 \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EVAL_SIM" \
    --step-time-s "$STEP" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    "${COMMON_ARGS[@]}" \
    --ns3-arg schedulerPolicy=pf \
    --port "$port" \
    --output-json "$output_json"
}

run_fixed_baseline() {
  local initial_bwp="$1"
  local summary_file="$2"
  local json_file="$3"
  ./ns3 run "$NS3_SCRIPT --numUes=$NUM_UES --simTime=$EVAL_SIM --envStepTime=$STEP --enableOpenGym=false --schedulerPolicy=aequitas --aequitasEnableMcsSelection=true --enableMcsSwitch=false --initialBwpId=$initial_bwp --summaryFile=$summary_file $BASELINE_EXTRA"
  python3 scratch/rl_bwp/parse_baseline_summary.py --summary-file "$summary_file" --output-json "$json_file"
}

DRQN_PF_RUN="${ROOT}/drqn_pf_train"
DRQN_AEQ_RUN="${ROOT}/drqn_aeq_train"
RQR_PF_RUN="${ROOT}/rqrdqn_pf_train"
mkdir -p "${ROOT}/eval" "${ROOT}/baselines"

run_train_drqn "${DRQN_PF_RUN#scratch/rl_bwp/runs/}" pf false $((PORT_BASE + 21))
run_train_drqn "${DRQN_AEQ_RUN#scratch/rl_bwp/runs/}" aequitas true $((PORT_BASE + 31))
run_train_rqrdqn "${RQR_PF_RUN#scratch/rl_bwp/runs/}" $((PORT_BASE + 41))

run_eval_drqn "${DRQN_PF_RUN}/final_model.pt" "${ROOT}/eval/drqn_pf_eval.json" pf false $((PORT_BASE + 121))
run_eval_drqn "${DRQN_AEQ_RUN}/final_model.pt" "${ROOT}/eval/drqn_aeq_eval.json" aequitas true $((PORT_BASE + 131))
run_eval_rqrdqn "${RQR_PF_RUN}/final_model.pt" "${ROOT}/eval/rqrdqn_pf_eval.json" $((PORT_BASE + 141))

run_fixed_baseline 0 "${ROOT}/baselines/bwp0_aeq_summary.txt" "${ROOT}/baselines/bwp0_aeq_eval.json"
run_fixed_baseline 1 "${ROOT}/baselines/bwp1_aeq_summary.txt" "${ROOT}/baselines/bwp1_aeq_eval.json"
