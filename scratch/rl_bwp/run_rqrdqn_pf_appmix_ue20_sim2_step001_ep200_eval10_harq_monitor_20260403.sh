#!/usr/bin/env bash
set -euo pipefail

ROOT_REL="${1:-rqrdqn_pf_appmix_ue20_sim2_eval5x10_ep200_step001_harq_monitor_20260403}"
RUN_ROOT="scratch/rl_bwp/runs/${ROOT_REL}"
TRAIN_ROOT="${RUN_ROOT}/rqrdqn_pf_train"
NS3_SCRIPT="aoi-prb-urban-appmix"
NUM_UES=20
TRAIN_SIM=2
EVAL_SIM=5
STEP=0.01
TRAIN_EPISODES=200
TOTAL_ENV_STEPS=$(( NUM_UES * 200 * TRAIN_EPISODES ))
TRAIN_PORT=67111
EVAL_PORT_BASE=67120
EVAL_SEEDS=(1001 1002 1003 1004 1005 1006 1007 1008 1009 1010)

mkdir -p "${RUN_ROOT}/logs" "${RUN_ROOT}/eval" "${RUN_ROOT}/report"

if ! ./ns3 show profile | grep -q "Build profile: optimized"; then
  ./ns3 configure -d optimized
fi
./ns3 build scratch/aoi-prb-urban-appmix

TRAIN_LOG="${RUN_ROOT}/logs/train.log"
TRAIN_MONITOR_LOG="${RUN_ROOT}/logs/train_monitor.log"

./ns3gym-venv/bin/python3 -u scratch/rl_bwp/train_rqrdqn.py \
  --backend ns3 \
  --run-name "${ROOT_REL}/rqrdqn_pf_train" \
  --start-sim \
  --ns3-script "${NS3_SCRIPT}" \
  --num-ues "${NUM_UES}" \
  --episode-time-s "${TRAIN_SIM}" \
  --step-time-s "${STEP}" \
  --min-completed-episodes "${TRAIN_EPISODES}" \
  --total-env-steps "${TOTAL_ENV_STEPS}" \
  --reward-bin-size 300 \
  --disable-step-reward-log \
  --drqn-profile \
  --port "${TRAIN_PORT}" \
  --ns3-arg trafficModel=mixed \
  --ns3-arg schedulerPolicy=pf \
  --ns3-arg enableHarqReTx=true \
  >"${TRAIN_LOG}" 2>&1 &
TRAIN_PID=$!
echo "${TRAIN_PID}" > "${RUN_ROOT}/train.pid"

nohup ./scratch/rl_bwp/monitor_ns3_reconnector.sh \
  "${TRAIN_PID}" \
  "${TRAIN_PORT}" \
  10 \
  5 \
  "${NS3_SCRIPT}" \
  --numUes="${NUM_UES}" \
  --simTime="${TRAIN_SIM}.0" \
  --envStepTime="${STEP}" \
  --enableOpenGym=true \
  --trafficModel=mixed \
  --schedulerPolicy=pf \
  --enableHarqReTx=true \
  --rlDrqnProfile=true \
  --enableRlMcsControl=false \
  >"${TRAIN_MONITOR_LOG}" 2>&1 &
TRAIN_MONITOR_PID=$!
echo "${TRAIN_MONITOR_PID}" > "${RUN_ROOT}/train_monitor.pid"

train_status=0
wait "${TRAIN_PID}" || train_status=$?
kill "${TRAIN_MONITOR_PID}" 2>/dev/null || true
wait "${TRAIN_MONITOR_PID}" 2>/dev/null || true
if [[ "${train_status}" -ne 0 ]]; then
  echo "training failed: status=${train_status}" >&2
  exit "${train_status}"
fi

cp "${TRAIN_ROOT}/train_episode_metrics.csv" "${RUN_ROOT}/report/train_episode_metrics.csv"
cp "${TRAIN_ROOT}/train_reward_bins_300.csv" "${RUN_ROOT}/report/train_reward_bins_300.csv"
cp "${TRAIN_ROOT}/run_config.json" "${RUN_ROOT}/report/train_run_config.json"

seed_index=0
for seed in "${EVAL_SEEDS[@]}"; do
  port=$(( EVAL_PORT_BASE + seed_index ))
  eval_json="${RUN_ROOT}/eval/seed_${seed}.json"
  eval_steps="${RUN_ROOT}/eval/seed_${seed}_steps.csv"
  eval_log="${RUN_ROOT}/logs/eval_seed_${seed}.log"
  eval_monitor_log="${RUN_ROOT}/logs/eval_seed_${seed}_monitor.log"

  ./ns3gym-venv/bin/python3 -u scratch/rl_bwp/eval_rqrdqn.py \
    --backend ns3 \
    --model-path "${TRAIN_ROOT}/final_model.pt" \
    --episodes 1 \
    --seed "${seed}" \
    --num-ues "${NUM_UES}" \
    --episode-time-s "${EVAL_SIM}" \
    --step-time-s "${STEP}" \
    --start-sim \
    --ns3-script "${NS3_SCRIPT}" \
    --drqn-profile \
    --port "${port}" \
    --output-json "${eval_json}" \
    --step-log-csv "${eval_steps}" \
    --ns3-arg trafficModel=mixed \
    --ns3-arg schedulerPolicy=pf \
    --ns3-arg enableHarqReTx=true \
    >"${eval_log}" 2>&1 &
  eval_pid=$!
  echo "${eval_pid}" > "${RUN_ROOT}/eval_seed_${seed}.pid"

  nohup ./scratch/rl_bwp/monitor_ns3_reconnector.sh \
    "${eval_pid}" \
    "${port}" \
    5 \
    3 \
    "${NS3_SCRIPT}" \
    --numUes="${NUM_UES}" \
    --simTime="${EVAL_SIM}.0" \
    --envStepTime="${STEP}" \
    --enableOpenGym=true \
    --trafficModel=mixed \
    --schedulerPolicy=pf \
    --enableHarqReTx=true \
    --rlDrqnProfile=true \
    --enableRlMcsControl=false \
    >"${eval_monitor_log}" 2>&1 &
  eval_monitor_pid=$!
  echo "${eval_monitor_pid}" > "${RUN_ROOT}/eval_seed_${seed}_monitor.pid"

  eval_status=0
  wait "${eval_pid}" || eval_status=$?
  kill "${eval_monitor_pid}" 2>/dev/null || true
  wait "${eval_monitor_pid}" 2>/dev/null || true
  if [[ "${eval_status}" -ne 0 ]]; then
    echo "evaluation failed: seed=${seed} status=${eval_status}" >&2
    exit "${eval_status}"
  fi

  seed_index=$(( seed_index + 1 ))
done

python3 - <<'PY' "${RUN_ROOT}"
import csv
import json
import re
import sys
from pathlib import Path

run_root = Path(sys.argv[1])
eval_dir = run_root / "eval"
report_dir = run_root / "report"
report_dir.mkdir(parents=True, exist_ok=True)

metric_labels = {
    "thr_mbps": r"^ue\d+_thr_mbps$",
    "goodput_mbps": r"^ue\d+_goodput_mbps$",
    "aoi_ms": r"^ue\d+_aoi_ms$",
    "bler": r"^ue\d+_bler$",
}
ue_re = re.compile(r"^ue(\d+)_")

all_rows = []
seed_rows = []

for json_path in sorted(eval_dir.glob("seed_*.json")):
    seed = int(json_path.stem.split("_")[1])
    data = json.loads(json_path.read_text())
    means = data.get("episode_metric_means", {})
    reward_mean = float(data.get("reward_mean", 0.0))
    metric_to_values = {metric: [] for metric in metric_labels}
    for key, stats in means.items():
        for metric, pattern in metric_labels.items():
            if re.match(pattern, key):
                m = ue_re.match(key)
                if not m:
                    continue
                ue = int(m.group(1))
                value = float(stats.get("mean", 0.0))
                all_rows.append(
                    {
                        "seed": seed,
                        "ue": ue,
                        "metric": metric,
                        "value": value,
                    }
                )
                metric_to_values[metric].append(value)
                break
    for metric, values in metric_to_values.items():
        if not values:
            continue
        seed_rows.append(
            {
                "seed": seed,
                "metric": metric,
                "mean": sum(values) / len(values),
                "reward_mean": reward_mean,
            }
        )

with (report_dir / "eval_seed_ue_metrics.csv").open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["seed", "ue", "metric", "value"])
    writer.writeheader()
    writer.writerows(all_rows)

with (report_dir / "eval_seed_metric_means.csv").open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["seed", "metric", "mean", "reward_mean"])
    writer.writeheader()
    writer.writerows(seed_rows)

summary_rows = []
for metric in metric_labels:
    values = [row["value"] for row in all_rows if row["metric"] == metric]
    if not values:
        continue
    values.sort()
    def pct(p: float) -> float:
        if len(values) == 1:
            return float(values[0])
        rank = (len(values) - 1) * p / 100.0
        lo = int(rank)
        hi = min(lo + 1, len(values) - 1)
        frac = rank - lo
        return float(values[lo] * (1.0 - frac) + values[hi] * frac)
    summary_rows.append(
        {
            "metric": metric,
            "min": float(values[0]),
            "p25": pct(25),
            "p50": pct(50),
            "p75": pct(75),
            "max": float(values[-1]),
        }
    )

with (report_dir / "eval_percentiles.csv").open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["metric", "min", "p25", "p50", "p75", "max"])
    writer.writeheader()
    writer.writerows(summary_rows)

with (report_dir / "summary.txt").open("w", encoding="utf-8") as f:
    f.write(f"run_root={run_root}\n")
    f.write(f"train_episode_metrics={report_dir / 'train_episode_metrics.csv'}\n")
    f.write(f"train_reward_bins_300={report_dir / 'train_reward_bins_300.csv'}\n")
    f.write(f"eval_seed_ue_metrics={report_dir / 'eval_seed_ue_metrics.csv'}\n")
    f.write(f"eval_seed_metric_means={report_dir / 'eval_seed_metric_means.csv'}\n")
    f.write(f"eval_percentiles_csv={report_dir / 'eval_percentiles.csv'}\n")
    for row in summary_rows:
        f.write(
            f"{row['metric']}: min={row['min']:.6f}, p25={row['p25']:.6f}, "
            f"p50={row['p50']:.6f}, p75={row['p75']:.6f}, max={row['max']:.6f}\n"
        )
PY

echo "Run root: ${RUN_ROOT}"
