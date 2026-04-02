#!/usr/bin/env bash
set -euo pipefail

ROOT_REL="${1:-appmix_graph_data_priority_20260401_ue20_sim2_eval5_ep500_step001_opt}"
RUN_ROOT="scratch/rl_bwp/runs/$ROOT_REL"
mkdir -p "$RUN_ROOT"

NUM_UES=20
TRAIN_SIM=2
EVAL_SIM=5
STEP=0.01
TRAIN_EPISODES=500
EVAL_EPISODES=1
TOTAL_ENV_STEPS=$(( NUM_UES * 200 * TRAIN_EPISODES ))
NS3_SCRIPT="aoi-prb-urban-appmix"
PORT_BASE=64000
SCEN_ROOT="$RUN_ROOT/appmix"
RUN_PREFIX="$ROOT_REL/appmix"

mkdir -p "$SCEN_ROOT/logs" "$SCEN_ROOT/eval" "$SCEN_ROOT/baselines"

if ! ./ns3 show profile | grep -q "Build profile: optimized"; then
  ./ns3 configure --build-profile=optimized
fi
./ns3 build

run_train_drqn() {
  local run_name="$1"
  local port="$2"
  shift 2
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
    --backend ns3 \
    --run-name "$run_name" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$TRAIN_SIM" \
    --step-time-s "$STEP" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --total-env-steps "$TOTAL_ENV_STEPS" \
    --reward-bin-size 300 \
    --disable-step-reward-log \
    --save-every-episodes 50 \
    --port "$port" \
    --ns3-arg trafficModel=mixed \
    "$@"
}

run_train_rqrdqn() {
  local run_name="$1"
  local port="$2"
  shift 2
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_rqrdqn.py \
    --backend ns3 \
    --run-name "$run_name" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$TRAIN_SIM" \
    --step-time-s "$STEP" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --total-env-steps "$TOTAL_ENV_STEPS" \
    --reward-bin-size 300 \
    --disable-step-reward-log \
    --save-every-episodes 50 \
    --port "$port" \
    --ns3-arg trafficModel=mixed \
    "$@"
}

run_eval_drqn() {
  local model_path="$1"
  local output_json="$2"
  local step_csv="$3"
  local port="$4"
  shift 4
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
    --backend ns3 \
    --model-path "$model_path" \
    --seed 1 \
    --episodes "$EVAL_EPISODES" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EVAL_SIM" \
    --step-time-s "$STEP" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --port "$port" \
    --output-json "$output_json" \
    --step-log-csv "$step_csv" \
    --ns3-arg trafficModel=mixed \
    "$@"
}

run_eval_rqrdqn() {
  local model_path="$1"
  local output_json="$2"
  local step_csv="$3"
  local port="$4"
  shift 4
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_rqrdqn.py \
    --backend ns3 \
    --model-path "$model_path" \
    --seed 1 \
    --episodes "$EVAL_EPISODES" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EVAL_SIM" \
    --step-time-s "$STEP" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --port "$port" \
    --output-json "$output_json" \
    --step-log-csv "$step_csv" \
    --ns3-arg trafficModel=mixed \
    "$@"
}

run_fixed_baseline() {
  local initial_bwp="$1"
  local summary_file="$2"
  local json_file="$3"
  ./ns3 run "$NS3_SCRIPT --numUes=$NUM_UES --simTime=$EVAL_SIM --envStepTime=$STEP --enableOpenGym=false --trafficModel=mixed --schedulerPolicy=aequitas --aequitasEnableMcsSelection=true --enableMcsSwitch=false --initialBwpId=$initial_bwp --summaryFile=$summary_file"
  python3 scratch/rl_bwp/parse_baseline_summary.py --summary-file "$summary_file" --output-json "$json_file"
}

(
  run_train_drqn "$RUN_PREFIX/drqn_aeq_train" "$((PORT_BASE + 21))" \
    --ns3-arg schedulerPolicy=aequitas \
    --ns3-arg aequitasEnableMcsSelection=true \
    >"$SCEN_ROOT/logs/drqn_aeq_train.log" 2>&1

  run_eval_drqn "$SCEN_ROOT/drqn_aeq_train/final_model.pt" \
    "$SCEN_ROOT/eval/drqn_aeq_eval.json" "$SCEN_ROOT/eval/drqn_aeq_eval_steps.csv" "$((PORT_BASE + 22))" \
    --ns3-arg schedulerPolicy=aequitas \
    --ns3-arg aequitasEnableMcsSelection=true \
    >"$SCEN_ROOT/logs/drqn_aeq_eval.log" 2>&1
) &
DRQN_AEQ_PID=$!

(
  run_train_rqrdqn "$RUN_PREFIX/rqrdqn_pf_train" "$((PORT_BASE + 1))" \
    --ns3-arg schedulerPolicy=pf \
    >"$SCEN_ROOT/logs/rqrdqn_pf_train.log" 2>&1

  run_eval_rqrdqn "$SCEN_ROOT/rqrdqn_pf_train/final_model.pt" \
    "$SCEN_ROOT/eval/rqrdqn_pf_eval.json" "$SCEN_ROOT/eval/rqrdqn_pf_eval_steps.csv" "$((PORT_BASE + 2))" \
    --ns3-arg schedulerPolicy=pf \
    >"$SCEN_ROOT/logs/rqrdqn_pf_eval.log" 2>&1
) &
RQR_PID=$!

wait "$DRQN_AEQ_PID"
wait "$RQR_PID"

run_train_drqn "$RUN_PREFIX/drqn_pf_train" "$((PORT_BASE + 11))" \
  --ns3-arg schedulerPolicy=pf \
  --ns3-arg aequitasEnableMcsSelection=false \
  >"$SCEN_ROOT/logs/drqn_pf_train.log" 2>&1

run_eval_drqn "$SCEN_ROOT/drqn_pf_train/final_model.pt" \
  "$SCEN_ROOT/eval/drqn_pf_eval.json" "$SCEN_ROOT/eval/drqn_pf_eval_steps.csv" "$((PORT_BASE + 12))" \
  --ns3-arg schedulerPolicy=pf \
  --ns3-arg aequitasEnableMcsSelection=false \
  >"$SCEN_ROOT/logs/drqn_pf_eval.log" 2>&1

run_fixed_baseline 0 "$SCEN_ROOT/baselines/bwp0_aeq_summary.txt" "$SCEN_ROOT/baselines/bwp0_aeq_eval.json"
run_fixed_baseline 1 "$SCEN_ROOT/baselines/bwp1_aeq_summary.txt" "$SCEN_ROOT/baselines/bwp1_aeq_eval.json"

python3 scratch/rl_bwp/summarize_graph_data_request_20260401.py --root "$RUN_ROOT"
