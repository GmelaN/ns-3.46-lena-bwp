#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 7 ]]; then
  echo "usage: $0 <scenario_name> <traffic_model> <burst_mbps> <bg_kbps> <enable_mobility> <port_base> <run_root>" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT_DIR"

SCENARIO="$1"
TRAFFIC_MODEL="$2"
BURST_MBPS="$3"
BG_KBPS="$4"
ENABLE_MOBILITY="$5"
PORT_BASE="$6"
RUN_ROOT="$7"

PYTHON_BIN="ns3gym-venv/bin/python3"
NS3_BIN="./ns3"
SCEN_DIR="$RUN_ROOT/$SCENARIO"

mkdir -p "$SCEN_DIR/logs" "$SCEN_DIR/evals" "$SCEN_DIR/summaries"

run_direct() {
  local summary_txt="$1"
  local log_txt="$2"
  shift 2
  if [[ -f "$summary_txt" ]]; then
    echo "[skip] direct exists: $summary_txt"
    return
  fi
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
    --episode-time-s 10.0 \
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
  MPLCONFIGDIR=/tmp/mpl "$PYTHON_BIN" scratch/rl_bwp/eval_drqn.py \
    --backend ns3 \
    --start-sim \
    --model-path "$model_path" \
    --episodes 5 \
    --num-ues 20 \
    --episode-time-s 10.0 \
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
    --episode-time-s 10.0 \
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
  MPLCONFIGDIR=/tmp/mpl "$PYTHON_BIN" scratch/rl_bwp/eval_rqrdqn.py \
    --backend ns3 \
    --start-sim \
    --model-path "$model_path" \
    --episodes 5 \
    --num-ues 20 \
    --episode-time-s 10.0 \
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
for key, block in data.get("per_ue_last_percentiles", {}).items():
    out.append(
        f"{key}.min={block.get('min', 0.0):.6f},"
        f"{key}.p25={block.get('p25', 0.0):.6f},"
        f"{key}.p50={block.get('p50', 0.0):.6f},"
        f"{key}.p95={block.get('p95', 0.0):.6f},"
        f"{key}.max={block.get('max', 0.0):.6f}"
    )

Path(summary_txt).write_text("\n".join(out) + "\n", encoding="utf-8")
PY
}

COMMON_CLI=(
  --numUes=20
  --simTime=10.0
  --appStart=0.3
  --trafficModel="$TRAFFIC_MODEL"
  --burstRateMbps="$BURST_MBPS"
  --backgroundRateKbps="$BG_KBPS"
  --enableMobility="$ENABLE_MOBILITY"
)

COMMON_ARGS=(
  --ns3-arg simTime=10.0
  --ns3-arg appStart=0.3
  --ns3-arg trafficModel="$TRAFFIC_MODEL"
  --ns3-arg burstRateMbps="$BURST_MBPS"
  --ns3-arg backgroundRateKbps="$BG_KBPS"
  --ns3-arg enableMobility="$ENABLE_MOBILITY"
)

run_direct \
  "$SCEN_DIR/summaries/bwp0_pf.txt" \
  "$SCEN_DIR/logs/bwp0_pf.log" \
  "${COMMON_CLI[*]} --enableOpenGym=false --initialBwpId=0 --schedulerPolicy=pf --summaryFile=$SCEN_DIR/summaries/bwp0_pf.txt"
run_direct \
  "$SCEN_DIR/summaries/bwp1_pf.txt" \
  "$SCEN_DIR/logs/bwp1_pf.log" \
  "${COMMON_CLI[*]} --enableOpenGym=false --initialBwpId=1 --schedulerPolicy=pf --summaryFile=$SCEN_DIR/summaries/bwp1_pf.txt"
run_direct \
  "$SCEN_DIR/summaries/aequitas_bwp0.txt" \
  "$SCEN_DIR/logs/aequitas_bwp0.log" \
  "${COMMON_CLI[*]} --enableOpenGym=false --initialBwpId=0 --schedulerPolicy=aequitas --summaryFile=$SCEN_DIR/summaries/aequitas_bwp0.txt"
run_direct \
  "$SCEN_DIR/summaries/aequitas_bwp1.txt" \
  "$SCEN_DIR/logs/aequitas_bwp1.log" \
  "${COMMON_CLI[*]} --enableOpenGym=false --initialBwpId=1 --schedulerPolicy=aequitas --summaryFile=$SCEN_DIR/summaries/aequitas_bwp1.txt"

run_train_drqn "$RUN_ROOT/$SCENARIO/drqn_pf_train" "$((PORT_BASE + 1))" \
  "${COMMON_ARGS[@]}" \
  --ns3-arg schedulerPolicy=pf \
  >"$SCEN_DIR/logs/drqn_pf_train.log" 2>&1
run_eval_drqn \
  "$SCEN_DIR/drqn_pf_train/final_model.pt" \
  "$SCEN_DIR/evals/drqn_pf.json" \
  "$SCEN_DIR/evals/drqn_pf_steps.csv" \
  "$((PORT_BASE + 2))" \
  "${COMMON_ARGS[@]}" \
  --ns3-arg schedulerPolicy=pf \
  >"$SCEN_DIR/logs/drqn_pf_eval.log" 2>&1
write_summary_from_eval "$SCEN_DIR/evals/drqn_pf.json" "$SCEN_DIR/summaries/drqn_pf.txt" "drqn_pf"

run_train_rqrdqn "$RUN_ROOT/$SCENARIO/rqrdqn_bwponly_train" "$((PORT_BASE + 3))" \
  --drqn-profile \
  "${COMMON_ARGS[@]}" \
  --ns3-arg schedulerPolicy=pf \
  >"$SCEN_DIR/logs/rqrdqn_bwponly_train.log" 2>&1
run_eval_rqrdqn \
  "$SCEN_DIR/rqrdqn_bwponly_train/final_model.pt" \
  "$SCEN_DIR/evals/rqrdqn_bwponly.json" \
  "$SCEN_DIR/evals/rqrdqn_bwponly_steps.csv" \
  "$((PORT_BASE + 4))" \
  --drqn-profile \
  "${COMMON_ARGS[@]}" \
  --ns3-arg schedulerPolicy=pf \
  >"$SCEN_DIR/logs/rqrdqn_bwponly_eval.log" 2>&1
write_summary_from_eval "$SCEN_DIR/evals/rqrdqn_bwponly.json" "$SCEN_DIR/summaries/rqrdqn_bwponly.txt" "rqrdqn_bwponly"

run_train_rqrdqn "$RUN_ROOT/$SCENARIO/rqrdqn_full_train" "$((PORT_BASE + 5))" \
  "${COMMON_ARGS[@]}" \
  --ns3-arg schedulerPolicy=pf \
  >"$SCEN_DIR/logs/rqrdqn_full_train.log" 2>&1
run_eval_rqrdqn \
  "$SCEN_DIR/rqrdqn_full_train/final_model.pt" \
  "$SCEN_DIR/evals/rqrdqn_full.json" \
  "$SCEN_DIR/evals/rqrdqn_full_steps.csv" \
  "$((PORT_BASE + 6))" \
  "${COMMON_ARGS[@]}" \
  --ns3-arg schedulerPolicy=pf \
  >"$SCEN_DIR/logs/rqrdqn_full_eval.log" 2>&1
write_summary_from_eval "$SCEN_DIR/evals/rqrdqn_full.json" "$SCEN_DIR/summaries/rqrdqn_full.txt" "rqrdqn_full"

run_train_drqn "$RUN_ROOT/$SCENARIO/drqn_aeq_train" "$((PORT_BASE + 7))" \
  "${COMMON_ARGS[@]}" \
  --ns3-arg schedulerPolicy=aequitas \
  >"$SCEN_DIR/logs/drqn_aeq_train.log" 2>&1
run_eval_drqn \
  "$SCEN_DIR/drqn_aeq_train/final_model.pt" \
  "$SCEN_DIR/evals/drqn_aeq.json" \
  "$SCEN_DIR/evals/drqn_aeq_steps.csv" \
  "$((PORT_BASE + 8))" \
  "${COMMON_ARGS[@]}" \
  --ns3-arg schedulerPolicy=aequitas \
  >"$SCEN_DIR/logs/drqn_aeq_eval.log" 2>&1
write_summary_from_eval "$SCEN_DIR/evals/drqn_aeq.json" "$SCEN_DIR/summaries/drqn_aeq.txt" "drqn_aeq"

echo "[done] $SCENARIO"
