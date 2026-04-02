# RL BWP Switching (Scratch)

This folder contains a training-loop-first scaffold for recurrent PPO based:
- UE-wise BWP switching (narrow/wide)
- UE-wise MCS delta control (`-1/0/+1`)

It follows your current assumptions:
- Single gNB, downlink only
- Fixed UE count (`num_ues`, currently intended as `1`)
- Step time: `20 ms`
- BWP switch delay: `5 ms`
- Goal: minimize AoI increase while penalizing actual BWP switches
  while preserving throughput
- Include PRB saturation proxy in state:
  `prb_utility = prb_demand / total_prb_current_bwp`,
  where `prb_demand = dl_rlc_queue_bytes / (c0 + c1 * mcs)`

## Files

- `envs.py`
  - `MockBwpEnv`: lightweight POMDP sandbox for loop validation.
  - `Ns3BwpEnv`: wrapper on top of `ns3gym`.
- `train_rppo.py`
  - RecurrentPPO (`MlpLstmPolicy`) training entrypoint.
- `eval_rppo.py`
  - deterministic evaluation loop for trained checkpoint.

## Action and Observation

Observation (per UE, flattened):
- `mcs`
- `bwp_mode` (`0` narrow, `1` wide)
- `prb_utility`
- `sinr_db`
- `log(1 + aoi_ms)`
- `log(1 + throughput_mbps)`
- `log(1 + dl_queue_bytes)` from the downlink RLC queue

Action (per UE):
- `bwp_cmd in {0: hold, 1: switch}`
- `mcs_delta_idx in {0,1,2}` -> `{-1,0,+1}`

`Ns3BwpEnv` encodes per-UE action as:
`code = bwp_cmd * 3 + mcs_delta_idx`

## Reward

Default shaped reward:
- `-(log(1 + AoI_t) - log(1 + AoI_{t-1}))`
- `+ log(1 + throughput_mbps_t)`
- actual BWP switch penalty

## Quick start (mock backend)

```bash
python3 scratch/rl_bwp/train_rppo.py \
  --backend mock \
  --run-name rppo_mock_ue1 \
  --num-ues 1 \
  --total-timesteps 100000
```

Evaluate:

```bash
python3 scratch/rl_bwp/eval_rppo.py \
  --backend mock \
  --model-path scratch/rl_bwp/runs/rppo_mock_ue1/final_model.zip \
  --vecnorm-path scratch/rl_bwp/runs/rppo_mock_ue1/vecnormalize.pkl \
  --num-ues 1
```

## ns-3 backend notes

Use `--backend ns3` when your ns-3 side emits/accepts matching state/action schema.

For now the wrapper expects:
- ns-3 observation to include/pad the fields above
- ns-3 action to accept one discrete code per UE (`0..5`)

Current integration target:
- `scratch/aoi-prb-urban-onoff.cc`
- multi-UE per-UE actions are supported (`BWP switch + MCS delta` per UE)
- per-UE DL MCS is applied through scheduler `RNTI -> MCS override` (not UE0-only)

Auto-start support:
- pass `--start-sim` to let Python launch ns-3 automatically.
- default script is `aoi-prb-urban-onoff` (override with `--ns3-script`).
- `simTime`, `envStepTime`, `numUes` are passed automatically from RL args.
- extra ns-3 CLI args can be appended with repeated `--ns3-arg key=value`.

Example (ns-3 eval with auto-start):

```bash
python3 scratch/rl_bwp/eval_rppo.py \
  --backend ns3 \
  --start-sim \
  --ns3-script aoi-prb-urban-onoff \
  --model-path scratch/rl_bwp/runs/rppo_bwp_ue1/final_model.zip \
  --vecnorm-path scratch/rl_bwp/runs/rppo_bwp_ue1/vecnormalize.pkl \
  --num-ues 1 \
  --episode-time-s 10 \
  --episodes 3
```

python3 scratch/rl_bwp/eval_rppo.py \
  --backend ns3 \
  --start-sim \
  --ns3-script aoi-prb-urban-onoff \
  --model-path scratch/rl_bwp/runs/rppo_bwp_ue1/final_model.zip \
  --vecnorm-path scratch/rl_bwp/runs/rppo_bwp_ue1/vecnormalize.pkl \
  --num-ues 1 \
  --episode-time-s 1800 \
  --episodes 3

python3 scratch/rl_bwp/train_rppo.py --backend ns3 --start-sim --ns3-script aoi-prb-urban-onoff --num-ues 5 --episode-time-s 1 --run-name 5ues --n-epochs 1000 --switch-delay-ms 10 --ns3-arg "numUes=5" --ns3-arg "simTime=3600" --n-steps 200 --total-timesteps 200
