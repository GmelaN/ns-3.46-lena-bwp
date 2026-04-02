#!/usr/bin/env bash
set -euo pipefail

BASE_REL="drqn_aeq_appmix_ue20_sim2_eval5_ep10_step001_goodput_harq_opt_20260402"
RUN_REL="${1:-${BASE_REL}/drqn_aeq_train_continue_ep40}"
RUN_ROOT="scratch/rl_bwp/runs/${RUN_REL}"
BASE_ROOT="scratch/rl_bwp/runs/${BASE_REL}"
INIT_MODEL="${BASE_ROOT}/drqn_aeq_train/final_model.pt"

mkdir -p "${RUN_ROOT}" "${BASE_ROOT}/logs" "${RUN_ROOT}/eval"

NUM_UES=20
TRAIN_SIM=2
EVAL_SIM=5
STEP=0.01
TRAIN_EPISODES=40
EVAL_EPISODES=1
TOTAL_ENV_STEPS=$(( NUM_UES * 200 * TRAIN_EPISODES ))
NS3_SCRIPT="aoi-prb-urban-appmix"
PORT_BASE=65800

if ! ./ns3 show profile | grep -q "Build profile: optimized"; then
  ./ns3 configure -d optimized
fi
./ns3 build

./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
  --backend ns3 \
  --run-name "${RUN_REL}" \
  --init-model-path "${INIT_MODEL}" \
  --start-sim \
  --ns3-script "${NS3_SCRIPT}" \
  --num-ues "${NUM_UES}" \
  --episode-time-s "${TRAIN_SIM}" \
  --step-time-s "${STEP}" \
  --min-completed-episodes "${TRAIN_EPISODES}" \
  --total-env-steps "${TOTAL_ENV_STEPS}" \
  --reward-bin-size 300 \
  --disable-step-reward-log \
  --port "$((PORT_BASE + 21))" \
  --ns3-arg trafficModel=mixed \
  --ns3-arg schedulerPolicy=aequitas \
  --ns3-arg aequitasEnableMcsSelection=true \
  --ns3-arg enableHarqReTx=true \
  >"${BASE_ROOT}/logs/drqn_aeq_train_continue_ep40.log" 2>&1

./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
  --backend ns3 \
  --model-path "${RUN_ROOT}/final_model.pt" \
  --seed 1 \
  --episodes "${EVAL_EPISODES}" \
  --num-ues "${NUM_UES}" \
  --episode-time-s "${EVAL_SIM}" \
  --step-time-s "${STEP}" \
  --start-sim \
  --ns3-script "${NS3_SCRIPT}" \
  --port "$((PORT_BASE + 22))" \
  --output-json "${RUN_ROOT}/eval/drqn_aeq_eval.json" \
  --step-log-csv "${RUN_ROOT}/eval/drqn_aeq_eval_steps.csv" \
  --ns3-arg trafficModel=mixed \
  --ns3-arg schedulerPolicy=aequitas \
  --ns3-arg aequitasEnableMcsSelection=true \
  --ns3-arg enableHarqReTx=true \
  >"${BASE_ROOT}/logs/drqn_aeq_eval_continue_ep40.log" 2>&1

python3 - <<'PY' "${RUN_ROOT}/eval/drqn_aeq_eval.json" "${RUN_ROOT}/eval/eval_percentiles.csv"
import csv
import json
import re
import sys
from pathlib import Path

json_path = Path(sys.argv[1])
csv_path = Path(sys.argv[2])
data = json.loads(json_path.read_text())
per = data.get("per_ue_last_percentiles", {})
means = data.get("episode_metric_means", {})

def metric_mean(label: str, pattern: str) -> float:
    vals = [float(v.get("mean", 0.0)) for k, v in means.items() if re.match(pattern, k)]
    return (sum(vals) / len(vals)) if vals else 0.0

rows = []
for metric, pattern in (
    ("thr_mbps", r"^ue\d+_thr_mbps$"),
    ("aoi_ms", r"^ue\d+_aoi_ms$"),
    ("goodput_mbps", r"^ue\d+_goodput_mbps$"),
):
    if metric not in per:
        continue
    item = per[metric]
    rows.append(
        {
            "metric": metric,
            "mean": metric_mean(metric, pattern),
            "min": float(item.get("min", 0.0)),
            "p25": float(item.get("p25", 0.0)),
            "p50": float(item.get("p50", 0.0)),
            "p75": float(item.get("p75", 0.0)),
            "max": float(item.get("max", 0.0)),
        }
    )

with csv_path.open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["metric", "mean", "min", "p25", "p50", "p75", "max"])
    writer.writeheader()
    writer.writerows(rows)
PY

echo "Run root: ${RUN_ROOT}"
