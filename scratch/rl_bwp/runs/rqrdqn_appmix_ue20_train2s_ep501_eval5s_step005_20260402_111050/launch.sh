#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/jshyeon/ns-3.46-bwp"
RUN_DIR="$ROOT/scratch/rl_bwp/runs/rqrdqn_appmix_ue20_train2s_ep501_eval5s_step005_20260402_111050"
TRAIN_PORT=62021
EVAL_PORT=62022
PYTHON_BIN="$ROOT/ns3gym-venv/bin/python"

cd "$ROOT"

./ns3 show profile > "$RUN_DIR/build_profile.txt"

"$PYTHON_BIN" -u scratch/rl_bwp/train_rqrdqn.py \
  --backend ns3 \
  --run-name rqrdqn_appmix_ue20_train2s_ep501_eval5s_step005_20260402_111050 \
  --seed 1 \
  --num-ues 20 \
  --step-time-s 0.05 \
  --episode-time-s 2.0 \
  --shared-per-ue \
  --total-env-steps 400800 \
  --min-completed-episodes 501 \
  --port "$TRAIN_PORT" \
  --start-sim \
  --ns3-script aoi-prb-urban-appmix \
  --ns3-arg trafficModel=mixed \
  --final-train-updates 256 \
  > "$RUN_DIR/train.log" 2>&1

"$PYTHON_BIN" -u scratch/rl_bwp/eval_rqrdqn.py \
  --backend ns3 \
  --model-path "$RUN_DIR/final_model.pt" \
  --episodes 1 \
  --seed 1 \
  --num-ues 20 \
  --step-time-s 0.05 \
  --episode-time-s 5.0 \
  --shared-per-ue \
  --port "$EVAL_PORT" \
  --start-sim \
  --ns3-script aoi-prb-urban-appmix \
  --ns3-arg trafficModel=mixed \
  --output-json "$RUN_DIR/eval_sim5.json" \
  > "$RUN_DIR/eval.log" 2>&1
