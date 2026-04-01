#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT_DIR"

RUN_ROOT="scratch/rl_bwp/runs/ue20_sim10_retrain_20260327"
mkdir -p "$RUN_ROOT"

bash scratch/rl_bwp/run_ue20_sim10_scenario.sh mixed mixed 100 500 true 63430 "$RUN_ROOT" \
  >"$RUN_ROOT/mixed_runner.log" 2>&1 &
pid_mixed=$!

bash scratch/rl_bwp/run_ue20_sim10_scenario.sh legacy_heavy legacy 150 750 false 63530 "$RUN_ROOT" \
  >"$RUN_ROOT/legacy_heavy_runner.log" 2>&1 &
pid_legacy=$!

wait "$pid_mixed"
wait "$pid_legacy"

echo "[done] ue20 sim10 parallel batch finished"
