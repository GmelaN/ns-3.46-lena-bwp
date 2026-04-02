#!/usr/bin/env bash
set -euo pipefail

ROOT_REL="${1:-drqn_pf_appmix_ue20_sim2_eval5_ep10_step001_harq_opt_20260402}"
RUN_ROOT="scratch/rl_bwp/runs/$ROOT_REL"
mkdir -p "$RUN_ROOT"

NUM_UES=20
TRAIN_SIM=2
EVAL_SIM=5
STEP=0.01
TRAIN_EPISODES=10
EVAL_EPISODES=1
TOTAL_ENV_STEPS=$(( NUM_UES * 200 * TRAIN_EPISODES ))
NS3_SCRIPT="aoi-prb-urban-appmix"
PORT_BASE=65420

mkdir -p "$RUN_ROOT/logs" "$RUN_ROOT/eval" "$RUN_ROOT/report"

if ! ./ns3 show profile | grep -q "Build profile: optimized"; then
  ./ns3 configure -d optimized
fi
./ns3 build scratch/aoi-prb-urban-appmix

./ns3gym-venv/bin/python3 scratch/rl_bwp/train_drqn.py \
  --backend ns3 \
  --run-name "$ROOT_REL/drqn_pf_train" \
  --start-sim \
  --ns3-script "$NS3_SCRIPT" \
  --num-ues "$NUM_UES" \
  --episode-time-s "$TRAIN_SIM" \
  --step-time-s "$STEP" \
  --min-completed-episodes "$TRAIN_EPISODES" \
  --total-env-steps "$TOTAL_ENV_STEPS" \
  --reward-bin-size 300 \
  --disable-step-reward-log \
  --port "$((PORT_BASE + 1))" \
  --ns3-arg trafficModel=mixed \
  --ns3-arg schedulerPolicy=pf \
  --ns3-arg aequitasEnableMcsSelection=false \
  --ns3-arg enableHarqReTx=true \
  >"$RUN_ROOT/logs/drqn_pf_train.log" 2>&1

cp "$RUN_ROOT/drqn_pf_train/train_reward_bins_300.csv" \
   "$RUN_ROOT/report/train_reward_bins_300.csv"
cp "$RUN_ROOT/drqn_pf_train/train_episode_metrics.csv" \
   "$RUN_ROOT/report/train_episode_metrics.csv"
cp "$RUN_ROOT/drqn_pf_train/run_config.json" \
   "$RUN_ROOT/report/train_run_config.json"

./ns3gym-venv/bin/python3 scratch/rl_bwp/eval_drqn.py \
  --backend ns3 \
  --model-path "$RUN_ROOT/drqn_pf_train/final_model.pt" \
  --seed 1001 \
  --episodes "$EVAL_EPISODES" \
  --num-ues "$NUM_UES" \
  --episode-time-s "$EVAL_SIM" \
  --step-time-s "$STEP" \
  --start-sim \
  --ns3-script "$NS3_SCRIPT" \
  --port "$((PORT_BASE + 2))" \
  --output-json "$RUN_ROOT/eval/drqn_pf_eval.json" \
  --step-log-csv "$RUN_ROOT/eval/drqn_pf_eval_steps.csv" \
  --ns3-arg trafficModel=mixed \
  --ns3-arg schedulerPolicy=pf \
  --ns3-arg aequitasEnableMcsSelection=false \
  --ns3-arg enableHarqReTx=true \
  >"$RUN_ROOT/logs/drqn_pf_eval.log" 2>&1

python3 - <<'PY' "$RUN_ROOT/drqn_pf_train/train_reward_bins_300.csv" "$RUN_ROOT/eval/drqn_pf_eval.json" "$RUN_ROOT/report"
import csv
import json
import sys
from pathlib import Path

train_bins = Path(sys.argv[1])
eval_json = Path(sys.argv[2])
report_dir = Path(sys.argv[3])
report_dir.mkdir(parents=True, exist_ok=True)

data = json.loads(eval_json.read_text())
per = data.get("per_ue_last_percentiles", {})

rows = []
for metric in ("thr_mbps", "aoi_ms", "goodput_mbps"):
    item = per.get(metric)
    if not item:
        continue
    rows.append(
        {
            "metric": metric,
            "min": float(item.get("min", 0.0)),
            "p25": float(item.get("p25", 0.0)),
            "p50": float(item.get("p50", 0.0)),
            "p75": float(item.get("p75", 0.0)),
            "max": float(item.get("max", 0.0)),
        }
    )

with (report_dir / "eval_percentiles.csv").open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["metric", "min", "p25", "p50", "p75", "max"])
    writer.writeheader()
    writer.writerows(rows)

train_rows = []
with train_bins.open("r", encoding="utf-8", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        train_rows.append(
            {
                "episode": int(row["episode"]),
                "bin_index": int(row["bin_index"]),
                "bin_size": int(row["bin_size"]),
                "global_step_end": int(row["global_step_end"]),
                "avg_reward": float(row["avg_reward"]),
            }
        )

summary = {
    "train_reward_bin_csv": str((report_dir / "train_reward_bins_300.csv").resolve()),
    "eval_json": str(eval_json.resolve()),
    "eval_percentiles_csv": str((report_dir / "eval_percentiles.csv").resolve()),
    "train_bins_recorded": len(train_rows),
    "evaluation_percentiles": rows,
    "reward_mean": float(data.get("reward_mean", 0.0)),
    "reward_std": float(data.get("reward_std", 0.0)),
}
with (report_dir / "summary.json").open("w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2)

with (report_dir / "summary.txt").open("w", encoding="utf-8") as f:
    f.write(f"run_root={report_dir.parent}\n")
    f.write(f"train_reward_bins_300={report_dir / 'train_reward_bins_300.csv'}\n")
    f.write(f"train_episode_metrics={report_dir / 'train_episode_metrics.csv'}\n")
    f.write(f"eval_json={eval_json}\n")
    f.write(f"eval_percentiles_csv={report_dir / 'eval_percentiles.csv'}\n")
    for row in rows:
        f.write(
            f"{row['metric']}: min={row['min']:.6f}, p25={row['p25']:.6f}, "
            f"p50={row['p50']:.6f}, p75={row['p75']:.6f}, max={row['max']:.6f}\n"
        )
PY

echo "Run root: $RUN_ROOT"
