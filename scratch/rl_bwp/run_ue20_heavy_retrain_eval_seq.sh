#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT_DIR"

PYTHON_BIN="ns3gym-venv/bin/python3"
NS3_BIN="./ns3"
RUN_ROOT="scratch/rl_bwp/runs/ue20_heavy_retrain_20260327"

mkdir -p "$RUN_ROOT"

run_direct() {
  local summary_txt="$1"
  local log_txt="$2"
  shift 2
  if [[ -f "$summary_txt" ]]; then
    echo "[skip] direct exists: $summary_txt"
    return
  fi
  mkdir -p "$(dirname "$summary_txt")" "$(dirname "$log_txt")"
  $NS3_BIN run "aoi-prb-urban-onoff $*" >"$log_txt" 2>&1
}

run_train_drqn() {
  local run_name="$1"
  local port="$2"
  shift 2
  local out_dir="scratch/rl_bwp/runs/$run_name"
  if [[ -f "$out_dir/final_model.pt" ]]; then
    echo "[skip] train exists: $run_name"
    return
  fi
  MPLCONFIGDIR=/tmp/mpl "$PYTHON_BIN" scratch/rl_bwp/train_drqn.py \
    --backend ns3 \
    --start-sim \
    --run-name "$run_name" \
    --num-ues 20 \
    --episode-time-s 5.0 \
    --step-time-s 0.02 \
    --switch-delay-ms 0.1 \
    --port "$port" \
    --device cpu \
    "$@"
}

run_eval_drqn() {
  local model_path="$1"
  local output_json="$2"
  local step_log_csv="$3"
  local port="$4"
  shift 4
  if [[ -f "$output_json" ]]; then
    echo "[skip] eval exists: $output_json"
    return
  fi
  mkdir -p "$(dirname "$output_json")"
  MPLCONFIGDIR=/tmp/mpl "$PYTHON_BIN" scratch/rl_bwp/eval_drqn.py \
    --backend ns3 \
    --start-sim \
    --model-path "$model_path" \
    --episodes 5 \
    --num-ues 20 \
    --episode-time-s 5.0 \
    --step-time-s 0.02 \
    --switch-delay-ms 0.1 \
    --port "$port" \
    --device cpu \
    --output-json "$output_json" \
    --step-log-csv "$step_log_csv" \
    "$@"
}

run_train_rqrdqn() {
  local run_name="$1"
  local port="$2"
  shift 2
  local out_dir="scratch/rl_bwp/runs/$run_name"
  if [[ -f "$out_dir/final_model.pt" ]]; then
    echo "[skip] train exists: $run_name"
    return
  fi
  MPLCONFIGDIR=/tmp/mpl "$PYTHON_BIN" scratch/rl_bwp/train_rqrdqn.py \
    --backend ns3 \
    --start-sim \
    --run-name "$run_name" \
    --num-ues 20 \
    --episode-time-s 5.0 \
    --step-time-s 0.02 \
    --switch-delay-ms 0.1 \
    --port "$port" \
    --device cpu \
    "$@"
}

run_eval_rqrdqn() {
  local model_path="$1"
  local output_json="$2"
  local step_log_csv="$3"
  local port="$4"
  shift 4
  if [[ -f "$output_json" ]]; then
    echo "[skip] eval exists: $output_json"
    return
  fi
  mkdir -p "$(dirname "$output_json")"
  MPLCONFIGDIR=/tmp/mpl "$PYTHON_BIN" scratch/rl_bwp/eval_rqrdqn.py \
    --backend ns3 \
    --start-sim \
    --model-path "$model_path" \
    --episodes 5 \
    --num-ues 20 \
    --episode-time-s 5.0 \
    --step-time-s 0.02 \
    --switch-delay-ms 0.1 \
    --port "$port" \
    --device cpu \
    --output-json "$output_json" \
    --step-log-csv "$step_log_csv" \
    "$@"
}

write_summary_from_eval() {
  local eval_json="$1"
  local summary_txt="$2"
  local label="$3"
  mkdir -p "$(dirname "$summary_txt")"
  "$PYTHON_BIN" - <<'PY' "$eval_json" "$summary_txt" "$label"
import json
import sys
from pathlib import Path

eval_json, summary_txt, label = sys.argv[1:4]
with open(eval_json, "r", encoding="utf-8") as fh:
    data = json.load(fh)

keys = [
    "mean_thr_mbps",
    "mean_aoi_ms",
    "requested_bwp0_count",
    "requested_bwp1_count",
    "switch_count",
    "reward_total",
]

out = [f"label={label}"]
for key in keys:
    block = data.get("episode_metric_means", {}).get(key)
    if block:
        out.append(
            f"{key}.mean={block.get('mean', 0.0):.6f},"
            f"{key}.p25={block.get('p25', 0.0):.6f},"
            f"{key}.p75={block.get('p75', 0.0):.6f}"
        )

Path(summary_txt).write_text("\n".join(out) + "\n", encoding="utf-8")
PY
}

run_scenario() {
  local scenario="$1"
  local traffic_model="$2"
  local burst_rate="$3"
  local bg_rate="$4"
  local mobility="$5"
  local port_base="$6"
  local mixed_light="$7"
  local mixed_moderate="$8"
  local mixed_heavy="$9"

  local scen_dir="$RUN_ROOT/$scenario"
  mkdir -p "$scen_dir/logs" "$scen_dir/evals" "$scen_dir/summaries"

  local common_cli=(
    --numUes=20
    --simTime=5.0
    --appStart=0.3
    --trafficModel="$traffic_model"
    --burstRateMbps="$burst_rate"
    --backgroundRateKbps="$bg_rate"
    --enableMobility="$mobility"
    --mixedLightRatio="$mixed_light"
    --mixedModerateRatio="$mixed_moderate"
    --mixedHeavyRatio="$mixed_heavy"
  )

  local common_args=(
    --ns3-arg simTime=5.0
    --ns3-arg appStart=0.3
    --ns3-arg trafficModel="$traffic_model"
    --ns3-arg burstRateMbps="$burst_rate"
    --ns3-arg backgroundRateKbps="$bg_rate"
    --ns3-arg enableMobility="$mobility"
    --ns3-arg mixedLightRatio="$mixed_light"
    --ns3-arg mixedModerateRatio="$mixed_moderate"
    --ns3-arg mixedHeavyRatio="$mixed_heavy"
  )

  echo "[scenario] $scenario"

  run_direct \
    "$scen_dir/summaries/bwp0_pf.txt" \
    "$scen_dir/logs/bwp0_pf.log" \
    "${common_cli[*]} --enableOpenGym=false --initialBwpId=0 --schedulerPolicy=pf --summaryFile=$scen_dir/summaries/bwp0_pf.txt"
  run_direct \
    "$scen_dir/summaries/bwp1_pf.txt" \
    "$scen_dir/logs/bwp1_pf.log" \
    "${common_cli[*]} --enableOpenGym=false --initialBwpId=1 --schedulerPolicy=pf --summaryFile=$scen_dir/summaries/bwp1_pf.txt"
  run_direct \
    "$scen_dir/summaries/aequitas_bwp0.txt" \
    "$scen_dir/logs/aequitas_bwp0.log" \
    "${common_cli[*]} --enableOpenGym=false --initialBwpId=0 --schedulerPolicy=aequitas --summaryFile=$scen_dir/summaries/aequitas_bwp0.txt"
  run_direct \
    "$scen_dir/summaries/aequitas_bwp1.txt" \
    "$scen_dir/logs/aequitas_bwp1.log" \
    "${common_cli[*]} --enableOpenGym=false --initialBwpId=1 --schedulerPolicy=aequitas --summaryFile=$scen_dir/summaries/aequitas_bwp1.txt"

  run_train_drqn "ue20_heavy_retrain_20260327/$scenario/drqn_pf_train" "$((port_base + 1))" \
    "${common_args[@]}" \
    --ns3-arg schedulerPolicy=pf \
    >"$scen_dir/logs/drqn_pf_train.log" 2>&1
  run_eval_drqn \
    "$scen_dir/drqn_pf_train/final_model.pt" \
    "$scen_dir/evals/drqn_pf.json" \
    "$scen_dir/evals/drqn_pf_steps.csv" \
    "$((port_base + 2))" \
    "${common_args[@]}" \
    --ns3-arg schedulerPolicy=pf \
    >"$scen_dir/logs/drqn_pf_eval.log" 2>&1
  write_summary_from_eval "$scen_dir/evals/drqn_pf.json" "$scen_dir/summaries/drqn_pf.txt" "drqn_pf"

  run_train_rqrdqn "ue20_heavy_retrain_20260327/$scenario/rqrdqn_bwponly_train" "$((port_base + 3))" \
    --drqn-profile \
    "${common_args[@]}" \
    --ns3-arg schedulerPolicy=pf \
    >"$scen_dir/logs/rqrdqn_bwponly_train.log" 2>&1
  run_eval_rqrdqn \
    "$scen_dir/rqrdqn_bwponly_train/final_model.pt" \
    "$scen_dir/evals/rqrdqn_bwponly.json" \
    "$scen_dir/evals/rqrdqn_bwponly_steps.csv" \
    "$((port_base + 4))" \
    --drqn-profile \
    "${common_args[@]}" \
    --ns3-arg schedulerPolicy=pf \
    >"$scen_dir/logs/rqrdqn_bwponly_eval.log" 2>&1
  write_summary_from_eval "$scen_dir/evals/rqrdqn_bwponly.json" "$scen_dir/summaries/rqrdqn_bwponly.txt" "rqrdqn_bwponly"

  run_train_rqrdqn "ue20_heavy_retrain_20260327/$scenario/rqrdqn_full_train" "$((port_base + 5))" \
    "${common_args[@]}" \
    --ns3-arg schedulerPolicy=pf \
    >"$scen_dir/logs/rqrdqn_full_train.log" 2>&1
  run_eval_rqrdqn \
    "$scen_dir/rqrdqn_full_train/final_model.pt" \
    "$scen_dir/evals/rqrdqn_full.json" \
    "$scen_dir/evals/rqrdqn_full_steps.csv" \
    "$((port_base + 6))" \
    "${common_args[@]}" \
    --ns3-arg schedulerPolicy=pf \
    >"$scen_dir/logs/rqrdqn_full_eval.log" 2>&1
  write_summary_from_eval "$scen_dir/evals/rqrdqn_full.json" "$scen_dir/summaries/rqrdqn_full.txt" "rqrdqn_full"

  run_train_drqn "ue20_heavy_retrain_20260327/$scenario/drqn_aeq_train" "$((port_base + 7))" \
    "${common_args[@]}" \
    --ns3-arg schedulerPolicy=aequitas \
    >"$scen_dir/logs/drqn_aeq_train.log" 2>&1
  run_eval_drqn \
    "$scen_dir/drqn_aeq_train/final_model.pt" \
    "$scen_dir/evals/drqn_aeq.json" \
    "$scen_dir/evals/drqn_aeq_steps.csv" \
    "$((port_base + 8))" \
    "${common_args[@]}" \
    --ns3-arg schedulerPolicy=aequitas \
    >"$scen_dir/logs/drqn_aeq_eval.log" 2>&1
  write_summary_from_eval "$scen_dir/evals/drqn_aeq.json" "$scen_dir/summaries/drqn_aeq.txt" "drqn_aeq"
}

# heavy_traffic: mixed traffic with only the heavy class enabled.
# Base rates are chosen so the heavy-class effective load is roughly 160 Mbps / 1000 Kbps.
run_scenario heavy_traffic mixed 100 625 false 63230 0.0 0.0 1.0

# legacy_heavy: legacy bursty heavy load.
run_scenario legacy_heavy legacy 160 1000 false 63330 0.4 0.4 0.2

echo "[done] ue20 heavy retrain/eval sequence finished"
