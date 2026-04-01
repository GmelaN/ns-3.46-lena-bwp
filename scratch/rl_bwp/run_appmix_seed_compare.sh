#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${1:-scratch/rl_bwp/runs/appmix_seed_compare_20260331}"
SEED_COUNT="${2:-10}"
NUM_UES="${3:-20}"
EPISODE_TIME_S="${4:-1}"
STEP_TIME_S="${5:-0.01}"
EPISODES="${6:-50}"
EVAL_EPISODES="${7:-1}"
NS3_SCRIPT="aoi-prb-urban-appmix"

mkdir -p "$ROOT_DIR"
SUMMARY_CSV="$ROOT_DIR/summary.csv"
printf "model,seed,mean_thr_mbps,mean_aoi_ms,switch_count,requested_bwp0_count,requested_bwp1_count,ue_thr_p25,ue_aoi_p75,run_dir\n" > "$SUMMARY_CSV"

for seed in $(seq 1 "$SEED_COUNT"); do
  drqn_run="drqn_appmix_seed${seed}_train1s_ep${EPISODES}_eval1s_step001_20260331"
  rqrdqn_run="rqrdqn_appmix_seed${seed}_train1s_ep${EPISODES}_eval1s_step001_20260331"

  drqn_port_train=$((58000 + seed * 10 + 1))
  drqn_port_eval=$((58000 + seed * 10 + 2))
  rqrdqn_port_train=$((59000 + seed * 10 + 1))
  rqrdqn_port_eval=$((59000 + seed * 10 + 2))

  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
    --backend ns3 \
    --run-name "$drqn_run" \
    --seed "$seed" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EPISODE_TIME_S" \
    --step-time-s "$STEP_TIME_S" \
    --min-completed-episodes "$EPISODES" \
    --total-env-steps 100000 \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --port "$drqn_port_train"

  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
    --backend ns3 \
    --model-path "scratch/rl_bwp/runs/$drqn_run/final_model.pt" \
    --seed "$seed" \
    --episodes "$EVAL_EPISODES" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EPISODE_TIME_S" \
    --step-time-s "$STEP_TIME_S" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --port "$drqn_port_eval" \
    --output-json "scratch/rl_bwp/runs/$drqn_run/eval.json"

  ./ns3gym-venv/bin/python3 scratch/rl_bwp/train_rqrdqn.py \
    --backend ns3 \
    --run-name "$rqrdqn_run" \
    --seed "$seed" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EPISODE_TIME_S" \
    --step-time-s "$STEP_TIME_S" \
    --min-completed-episodes "$EPISODES" \
    --total-env-steps 100000 \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --port "$rqrdqn_port_train"

  ./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_rqrdqn.py \
    --backend ns3 \
    --model-path "scratch/rl_bwp/runs/$rqrdqn_run/final_model.pt" \
    --seed "$seed" \
    --episodes "$EVAL_EPISODES" \
    --num-ues "$NUM_UES" \
    --episode-time-s "$EPISODE_TIME_S" \
    --step-time-s "$STEP_TIME_S" \
    --start-sim \
    --ns3-script "$NS3_SCRIPT" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --port "$rqrdqn_port_eval" \
    --output-json "scratch/rl_bwp/runs/$rqrdqn_run/eval.json"

  python3 - "$seed" "$drqn_run" "$rqrdqn_run" "$SUMMARY_CSV" <<'PY'
import csv, json, sys
seed = int(sys.argv[1])
drqn_run = sys.argv[2]
rqrdqn_run = sys.argv[3]
summary_csv = sys.argv[4]

def load(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def row(model, seed, run, data):
    means = data.get("episode_metric_means", {})
    return [
        model,
        seed,
        means.get("mean_thr_mbps", 0.0),
        means.get("mean_aoi_ms", 0.0),
        means.get("switch_count", 0.0),
        means.get("requested_bwp0_count", 0.0),
        means.get("requested_bwp1_count", 0.0),
        data.get("ue_thr_p25", 0.0),
        data.get("ue_aoi_p75", 0.0),
        run,
    ]

drqn = load(f"scratch/rl_bwp/runs/{drqn_run}/eval.json")
rqrdqn = load(f"scratch/rl_bwp/runs/{rqrdqn_run}/eval.json")
with open(summary_csv, "a", encoding="utf-8", newline="") as f:
    w = csv.writer(f)
    w.writerow(row("drqn", seed, drqn_run, drqn))
    w.writerow(row("rqrdqn", seed, rqrdqn_run, rqrdqn))
PY
done

echo "done: $SUMMARY_CSV"
