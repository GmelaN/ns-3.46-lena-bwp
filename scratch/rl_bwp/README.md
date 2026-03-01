# RL BWP Switching (Scratch)

This folder contains a training-loop-first scaffold for recurrent PPO based:
- UE-wise BWP switching (narrow/wide)
- UE-wise MCS delta control (`-1/0/+1`)

It follows your current assumptions:
- Single gNB, downlink only
- Fixed UE count (`num_ues`, currently intended as `1`)
- Step time: `20 ms`
- BWP switch delay: `5 ms`
- Goal: minimize system mean AoI (mean over UEs)
- Include PRB saturation proxy in state:
  `prb_utility = allocated_prb / total_prb_current_bwp`

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
- `queue_norm`
- `cqi_norm`
- `sinr_norm`
- `mcs_norm`
- `bwp_mode` (`0` narrow, `1` wide)
- `prb_utility`
- `cooldown_norm`

Action (per UE):
- `bwp_cmd in {0: hold, 1: switch}`
- `mcs_delta_idx in {0,1,2}` -> `{-1,0,+1}`

`Ns3BwpEnv` encodes per-UE action as:
`code = bwp_cmd * 3 + mcs_delta_idx`

## Reward

Default shaped reward:
- primary: `-mean_aoi_ms`
- penalties:
  - switch count penalty
  - queue overflow ratio penalty
  - switch delay burden (`delay_ratio = 5/20 = 0.25` with current setup)

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

If you run ns-3 manually, keep `--start-sim` disabled (default) and start ns-3 first.
