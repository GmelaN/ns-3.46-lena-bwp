#!/usr/bin/env bash
set -euo pipefail

ROOT_REL="${1:-graph_data_20260401_ue20_sim2_eval5_ep500_step001}"
RUN_ROOT="scratch/rl_bwp/runs/$ROOT_REL"
mkdir -p "$RUN_ROOT"

NUM_UES=20
TRAIN_SIM=2
EVAL_SIM=5
STEP=0.01
TRAIN_EPISODES=500
EVAL_EPISODES=1
TOTAL_ENV_STEPS=$(( NUM_UES * 200 * TRAIN_EPISODES ))

run_train_drqn() {
  local run_name="$1"
  local ns3_script="$2"
  local port="$3"
  shift 3
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
    --backend ns3 \
    --run-name "$run_name" \
    --start-sim \
    --ns3-script "$ns3_script" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$TRAIN_SIM" \
    --step-time-s "$STEP" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --total-env-steps "$TOTAL_ENV_STEPS" \
    --reward-bin-size 300 \
    --disable-step-reward-log \
    --port "$port" \
    "$@"
}

run_train_rqrdqn() {
  local run_name="$1"
  local ns3_script="$2"
  local port="$3"
  shift 3
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_rqrdqn.py \
    --backend ns3 \
    --run-name "$run_name" \
    --start-sim \
    --ns3-script "$ns3_script" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$TRAIN_SIM" \
    --step-time-s "$STEP" \
    --min-completed-episodes "$TRAIN_EPISODES" \
    --total-env-steps "$TOTAL_ENV_STEPS" \
    --reward-bin-size 300 \
    --disable-step-reward-log \
    --port "$port" \
    "$@"
}

run_eval_drqn() {
  local model_path="$1"
  local ns3_script="$2"
  local output_json="$3"
  local step_csv="$4"
  local port="$5"
  shift 5
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
    --backend ns3 \
    --model-path "$model_path" \
    --seed 1 \
    --episodes "$EVAL_EPISODES" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EVAL_SIM" \
    --step-time-s "$STEP" \
    --start-sim \
    --ns3-script "$ns3_script" \
    --port "$port" \
    --output-json "$output_json" \
    --step-log-csv "$step_csv" \
    "$@"
}

run_eval_rqrdqn() {
  local model_path="$1"
  local ns3_script="$2"
  local output_json="$3"
  local step_csv="$4"
  local port="$5"
  shift 5
  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_rqrdqn.py \
    --backend ns3 \
    --model-path "$model_path" \
    --seed 1 \
    --episodes "$EVAL_EPISODES" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EVAL_SIM" \
    --step-time-s "$STEP" \
    --start-sim \
    --ns3-script "$ns3_script" \
    --port "$port" \
    --output-json "$output_json" \
    --step-log-csv "$step_csv" \
    "$@"
}

run_fixed_baseline() {
  local ns3_script="$1"
  local initial_bwp="$2"
  local summary_file="$3"
  local json_file="$4"
  shift 4
  ./ns3 run "$ns3_script --numUes=$NUM_UES --simTime=$EVAL_SIM --envStepTime=$STEP --enableOpenGym=false --schedulerPolicy=aequitas --aequitasEnableMcsSelection=true --enableMcsSwitch=false --initialBwpId=$initial_bwp --summaryFile=$summary_file $*"
  python3 scratch/rl_bwp/parse_baseline_summary.py --summary-file "$summary_file" --output-json "$json_file"
}

run_scenario() {
  local scenario="$1"
  local ns3_script="$2"
  local port_base="$3"
  local baseline_extra="$4"
  shift 4
  local extra_args=("$@")
  local scen_root="$RUN_ROOT/$scenario"
  local run_prefix="$ROOT_REL/$scenario"
  mkdir -p "$scen_root/logs" "$scen_root/eval" "$scen_root/baselines"

  (
    run_train_rqrdqn "$run_prefix/rqrdqn_pf_train" "$ns3_script" "$((port_base + 1))" \
      "${extra_args[@]}" \
      --ns3-arg schedulerPolicy=pf \
      >"$scen_root/logs/rqrdqn_pf_train.log" 2>&1

    run_eval_rqrdqn "$scen_root/rqrdqn_pf_train/final_model.pt" "$ns3_script" \
      "$scen_root/eval/rqrdqn_pf_eval.json" "$scen_root/eval/rqrdqn_pf_eval_steps.csv" "$((port_base + 2))" \
      "${extra_args[@]}" \
      --ns3-arg schedulerPolicy=pf \
      >"$scen_root/logs/rqrdqn_pf_eval.log" 2>&1
  ) &
  local rqrdqn_pid=$!

  (
    run_train_drqn "$run_prefix/drqn_pf_train" "$ns3_script" "$((port_base + 11))" \
      "${extra_args[@]}" \
      --ns3-arg schedulerPolicy=pf \
      --ns3-arg aequitasEnableMcsSelection=false \
      >"$scen_root/logs/drqn_pf_train.log" 2>&1

    run_eval_drqn "$scen_root/drqn_pf_train/final_model.pt" "$ns3_script" \
      "$scen_root/eval/drqn_pf_eval.json" "$scen_root/eval/drqn_pf_eval_steps.csv" "$((port_base + 12))" \
      "${extra_args[@]}" \
      --ns3-arg schedulerPolicy=pf \
      --ns3-arg aequitasEnableMcsSelection=false \
      >"$scen_root/logs/drqn_pf_eval.log" 2>&1
  ) &
  local drqn_pf_pid=$!

  wait "$rqrdqn_pid"
  wait "$drqn_pf_pid"

  run_train_drqn "$run_prefix/drqn_aeq_train" "$ns3_script" "$((port_base + 21))" \
    "${extra_args[@]}" \
    --ns3-arg schedulerPolicy=aequitas \
    --ns3-arg aequitasEnableMcsSelection=true \
    >"$scen_root/logs/drqn_aeq_train.log" 2>&1

  run_eval_drqn "$scen_root/drqn_aeq_train/final_model.pt" "$ns3_script" \
    "$scen_root/eval/drqn_aeq_eval.json" "$scen_root/eval/drqn_aeq_eval_steps.csv" "$((port_base + 22))" \
    "${extra_args[@]}" \
    --ns3-arg schedulerPolicy=aequitas \
    --ns3-arg aequitasEnableMcsSelection=true \
    >"$scen_root/logs/drqn_aeq_eval.log" 2>&1

  run_fixed_baseline "$ns3_script" 0 \
    "$scen_root/baselines/bwp0_aeq_summary.txt" "$scen_root/baselines/bwp0_aeq_eval.json" \
    $baseline_extra

  run_fixed_baseline "$ns3_script" 1 \
    "$scen_root/baselines/bwp1_aeq_summary.txt" "$scen_root/baselines/bwp1_aeq_eval.json" \
    $baseline_extra
}

run_scenario "onoff_mixed" "aoi-prb-urban-onoff" 61000 \
  "--trafficModel=mixed --burstRateMbps=100 --backgroundRateKbps=500" \
  --ns3-arg trafficModel=mixed \
  --ns3-arg burstRateMbps=100 \
  --ns3-arg backgroundRateKbps=500

run_scenario "appmix" "aoi-prb-urban-appmix" 62000 \
  "--trafficModel=mixed" \
  --ns3-arg trafficModel=mixed

python3 scratch/rl_bwp/summarize_graph_data_request_20260401.py --root "$RUN_ROOT"
