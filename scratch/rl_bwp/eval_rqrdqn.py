#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import socket
from collections import defaultdict
from numbers import Real
from dataclasses import asdict

import numpy as np
import torch

from envs import DiscretePerUeActionAdapter, EnvConfig, MockBwpEnv, Ns3BwpEnv, SharedPerUeAdapter
from rqrdqn_torchrl import RqrdqnConfig, RqrdqnNet, select_action


STALE_GLOBAL_REWARD_KEYS = {
    "reward_aoi_term",
    "reward_aoi_penalty",
    "reward_thr_term",
    "reward_service_ratio_term",
    "reward_service_term",
    "reward_aux_term",
    "reward_se_term",
    "reward_switch_penalty",
    "reward_total",
}


def parse_extra_args(items: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"Invalid --ns3-arg '{item}', expected key=value.")
        k, v = item.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def _numeric_metrics(info: dict) -> dict[str, float]:
    metrics: dict[str, float] = {}
    for k, v in info.items():
        if isinstance(v, (bool, np.bool_)):
            continue
        if isinstance(v, Real):
            metrics[str(k)] = float(v)
    return metrics


def _per_ue_tail(metrics: dict[str, list[float]]) -> dict[str, float]:
    out: dict[str, float] = {}
    patterns = [
        ("ue_thr_p25", r"^ue\d+_thr_mbps$", 25),
        ("ue_aoi_p75", r"^ue\d+_aoi_ms$", 75),
    ]
    for out_key, pattern, pct in patterns:
        vals = [v[-1] for k, v in metrics.items() if v and re.match(pattern, k)]
        if vals:
            out[out_key] = float(np.percentile(np.asarray(vals, dtype=np.float64), pct))
    return out


def _per_ue_distribution_summary(metric_values: dict[str, list[float]]) -> dict[str, dict[str, float]]:
    patterns = {
        "thr_mbps": r"^ue\d+_thr_mbps$",
        "goodput_mbps": r"^ue\d+_goodput_mbps$",
        "aoi_ms": r"^ue\d+_aoi_ms$",
        "bler": r"^ue\d+_bler$",
        "aoi_burst_ms": r"^ue\d+_aoi_burst_ms$",
        "aoi_bg_ms": r"^ue\d+_aoi_bg_ms$",
    }
    out: dict[str, dict[str, float]] = {}
    for label, pattern in patterns.items():
        vals = [
            float(np.mean(np.asarray(v, dtype=np.float64)))
            for k, v in metric_values.items()
            if v and re.match(pattern, k)
        ]
        if not vals:
            continue
        arr = np.asarray(vals, dtype=np.float64)
        out[label] = {
            "min": float(np.min(arr)),
            "p25": float(np.percentile(arr, 25)),
            "p50": float(np.percentile(arr, 50)),
            "p75": float(np.percentile(arr, 75)),
            "max": float(np.max(arr)),
        }
    return out


def build_env(args, *, seed: int):
    ns3_args = parse_extra_args(args.ns3_arg)
    enable_rl_mcs = bool(args.enable_rl_mcs_control)

    # train.py와 맞춤
    ns3_args["rlRqrProfile"] = "true"
    ns3_args["enableRlMcsControl"] = "true" if enable_rl_mcs else "false"
    ns3_args["enableRlBwpControl"] = "true"
    ns3_args["useSymbolicSwitchDelay"] = "false"
    ns3_args["switchDelayMs"] = f"{float(args.switch_delay_ms):.6f}"

    cfg = EnvConfig(
        num_ues=args.num_ues,
        step_time_s=args.step_time_s,
        episode_time_s=args.episode_time_s,
        switch_delay_ms=float(args.switch_delay_ms),
        queue_max_bytes=args.queue_max_bytes,
        reward_lambda_switch=args.reward_lambda_switch,
        reward_lambda_queue=args.reward_lambda_queue,
        reward_lambda_delay=args.reward_lambda_delay,
        seed=seed,
        bwp_only_actions=not enable_rl_mcs,
        drqn_profile=False,
        rqr_profile=True,
    )

    if args.backend == "mock":
        env = MockBwpEnv(cfg)
    else:
        base_sim_args = {
            "numUes": cfg.num_ues,
            "simTime": cfg.episode_time_s,
            "envStepTime": cfg.step_time_s,
            "enableRlMcsControl": "true" if enable_rl_mcs else "false",
            "enableOpenGym": "true",
            "rlRqrProfile": "true",
        }
        base_sim_args.update(ns3_args)
        port = int(args.port)
        if port == 0:
            # Pick an ephemeral TCP port to avoid collisions across concurrent eval runs.
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("127.0.0.1", 0))
                port = int(s.getsockname()[1])
        env = Ns3BwpEnv(
            cfg=cfg,
            port=port,
            start_sim=args.start_sim,
            sim_seed=seed,
            sim_args=base_sim_args,
            sim_script=args.ns3_script,
            debug=args.debug_ns3,
        )

    if args.shared_per_ue:
        env = SharedPerUeAdapter(env)
    env = DiscretePerUeActionAdapter(env)
    return cfg, env


def main():
    parser = argparse.ArgumentParser(description="Evaluate recurrent quantile DQN over OpenGym/ns3 env.")
    parser.add_argument("--backend", choices=["mock", "ns3"], default="mock")
    parser.add_argument("--model-path", type=str, required=True)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--seed", type=int, default=1)

    # train.py 기본값에 맞춤
    parser.add_argument("--num-ues", type=int, default=20)
    parser.add_argument("--step-time-s", type=float, default=0.01)
    parser.add_argument("--episode-time-s", type=float, default=5.0)
    parser.add_argument("--switch-delay-ms", type=float, default=5.0)
    parser.add_argument("--queue-max-bytes", type=float, default=200000.0)
    parser.add_argument("--reward-lambda-switch", type=float, default=0.0001)
    parser.add_argument("--reward-lambda-queue", type=float, default=0.20)
    parser.add_argument("--reward-lambda-delay", type=float, default=0.10)
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--start-sim", action="store_true", default=True)
    parser.add_argument("--debug-ns3", action="store_true", default=False)
    parser.add_argument("--ns3-script", type=str, default="aoi-prb-urban-appmix")
    parser.add_argument("--ns3-arg", action="append", default=[])

    parser.add_argument("--output-json", type=str, default="")
    parser.add_argument("--step-log-csv", type=str, default="")

    parser.add_argument("--shared-per-ue", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-rl-mcs-control", action=argparse.BooleanOptionalAction, default=True)

    # 평가 시 CLI 기본값보다 checkpoint config를 우선 사용
    parser.add_argument("--action-selection-quantile", type=float, default=None)
    parser.add_argument("--conditional-risk-selection", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--risk-low-cqi-threshold", type=float, default=None)
    parser.add_argument("--risk-mid-cqi-threshold", type=float, default=None)
    parser.add_argument("--risk-quantile-low", type=float, default=None)
    parser.add_argument("--risk-quantile-mid", type=float, default=None)
    parser.add_argument("--risk-quantile-high", type=float, default=None)
    parser.add_argument("--device", type=str, default="cpu")
    args = parser.parse_args()

    ckpt = torch.load(args.model_path, map_location=args.device)
    rq_cfg_dict = ckpt.get("rqrdqn_config", {}) or {}

    # checkpoint의 학습 당시 설정으로 보정
    if args.action_selection_quantile is None:
        args.action_selection_quantile = float(rq_cfg_dict.get("action_selection_quantile", 0.2))
    if args.conditional_risk_selection is None:
        args.conditional_risk_selection = bool(rq_cfg_dict.get("conditional_risk_selection", False))
    if args.risk_low_cqi_threshold is None:
        args.risk_low_cqi_threshold = float(rq_cfg_dict.get("risk_low_cqi_threshold", 5.0))
    if args.risk_mid_cqi_threshold is None:
        args.risk_mid_cqi_threshold = float(rq_cfg_dict.get("risk_mid_cqi_threshold", 10.0))
    if args.risk_quantile_low is None:
        args.risk_quantile_low = float(rq_cfg_dict.get("risk_quantile_low", 0.2))
    if args.risk_quantile_mid is None:
        args.risk_quantile_mid = float(rq_cfg_dict.get("risk_quantile_mid", 0.5))
    if args.risk_quantile_high is None:
        args.risk_quantile_high = float(rq_cfg_dict.get("risk_quantile_high", 0.8))

    net = RqrdqnNet(
        int(ckpt["obs_dim"]),
        int(ckpt["hidden_dim"]),
        int(ckpt["action_dim"]),
        int(ckpt["num_quantiles"]),
    )
    net.load_state_dict(ckpt["state_dict"])
    net.to(args.device)
    net.eval()

    q_cfg = RqrdqnConfig(
        obs_dim=int(ckpt["obs_dim"]),
        action_dim=int(ckpt["action_dim"]),
        hidden_dim=int(ckpt["hidden_dim"]),
        num_quantiles=int(ckpt["num_quantiles"]),
        seq_len=int(rq_cfg_dict.get("seq_len", 8)),
        burn_in=int(rq_cfg_dict.get("burn_in", 4)),
        gamma=float(rq_cfg_dict.get("gamma", 0.98)),
        lr=float(rq_cfg_dict.get("lr", 1e-4)),
        batch_size=int(rq_cfg_dict.get("batch_size", 32)),
        replay_capacity=int(rq_cfg_dict.get("replay_capacity", 20000)),
        warmup_sequences=int(rq_cfg_dict.get("warmup_sequences", 300)),
        target_sync=int(rq_cfg_dict.get("target_sync", 1000)),
        eps_start=float(rq_cfg_dict.get("eps_start", 1.0)),
        eps_end=float(rq_cfg_dict.get("eps_end", 0.05)),
        eps_decay=float(rq_cfg_dict.get("eps_decay", 0.999)),
        train_updates_per_step=int(rq_cfg_dict.get("train_updates_per_step", 1)),
        huber_kappa=float(rq_cfg_dict.get("huber_kappa", 1.0)),
        action_selection_quantile=float(args.action_selection_quantile),
        conditional_risk_selection=bool(args.conditional_risk_selection),
        risk_low_cqi_threshold=float(args.risk_low_cqi_threshold),
        risk_mid_cqi_threshold=float(args.risk_mid_cqi_threshold),
        risk_quantile_low=float(args.risk_quantile_low),
        risk_quantile_mid=float(args.risk_quantile_mid),
        risk_quantile_high=float(args.risk_quantile_high),
        drqn_profile=False,
    )

    _, env = build_env(args, seed=args.seed)
    obs, _ = env.reset(seed=args.seed)
    obs = np.asarray(obs, dtype=np.float32)

    if args.shared_per_ue:
        hidden_bank = net.zero_hidden(args.num_ues, torch.device(args.device))
        current_ue_idx = 0
    else:
        hidden = net.zero_hidden(1, torch.device(args.device))

    episode_rewards = []
    episode_metric_means: dict[str, list[float]] = defaultdict(list)
    episode_metric_last: dict[str, list[float]] = defaultdict(list)
    current_metrics: dict[str, list[float]] = defaultdict(list)
    running_reward = 0.0
    finished = 0
    step_index = 0

    step_log_fh = None
    step_log_writer = None
    if args.step_log_csv:
        out_dir = os.path.dirname(args.step_log_csv)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        obs_dim = int(np.asarray(obs).reshape(-1).shape[0])
        step_log_fh = open(args.step_log_csv, "w", encoding="utf-8", newline="")
        step_log_writer = csv.writer(step_log_fh)
        step_log_writer.writerow(
            [
                "episode",
                "step",
                "ue_index",
                "action_code",
                "action_bwp",
                "action_mcs_idx",
                "reward",
                "reward_aoi_penalty_local",
                "reward_service_ratio_local",
                "reward_aux_term_local",
                "reward_switch_penalty_local",
                "done",
                "metric_valid",
                "mean_aoi_ms",
                "mean_thr_mbps",
                "requested_bwp0_count",
                "requested_bwp1_count",
                "switch_count",
                *[f"obs_{i}" for i in range(obs_dim)],
            ]
        )

    with torch.no_grad():
        while finished < args.episodes:
            if args.shared_per_ue:
                hidden_in = hidden_bank[current_ue_idx: current_ue_idx + 1]
            else:
                hidden_in = hidden

            # train.py의 selection 로직과 동일하게 사용
            action, next_hidden = select_action(
                net,
                obs,
                hidden_in,
                epsilon=0.0,  # evaluation
                device=torch.device(args.device),
                cfg=q_cfg,
            )

            obs_used = np.asarray(obs).reshape(-1).copy()
            next_obs, reward, terminated, truncated, info = env.step(action)
            done = bool(terminated or truncated)
            running_reward += float(reward)

            step_metrics = _numeric_metrics(info)
            metric_valid = bool(step_metrics.get("metric_valid", 1.0))

            if metric_valid:
                for k, v in step_metrics.items():
                    if args.shared_per_ue and k in STALE_GLOBAL_REWARD_KEYS:
                        continue
                    current_metrics[k].append(v)

            if step_log_writer is not None:
                ue_index = int(step_metrics.get("ue_index", -1))
                if args.enable_rl_mcs_control:
                    action_bwp = action // 5
                    action_mcs_idx = action % 5
                else:
                    action_bwp = action
                    action_mcs_idx = 0

                step_log_writer.writerow(
                    [
                        finished + 1,
                        step_index,
                        ue_index,
                        action,
                        action_bwp,
                        action_mcs_idx,
                        float(reward),
                        step_metrics.get("reward_aoi_penalty_local", 0.0),
                        step_metrics.get("reward_service_ratio_local", 0.0),
                        step_metrics.get("reward_aux_term_local", 0.0),
                        step_metrics.get("reward_switch_penalty_local", 0.0),
                        done,
                        metric_valid,
                        step_metrics.get("mean_aoi_ms", 0.0),
                        step_metrics.get("mean_thr_mbps", 0.0),
                        step_metrics.get("requested_bwp0_count", 0.0),
                        step_metrics.get("requested_bwp1_count", 0.0),
                        step_metrics.get("switch_count", 0.0),
                        *obs_used.tolist(),
                    ]
                )

            step_index += 1
            obs = np.asarray(next_obs, dtype=np.float32)

            if args.shared_per_ue:
                acted_ue_idx = int(step_metrics.get("ue_index", current_ue_idx))
                if acted_ue_idx < 0 or acted_ue_idx >= args.num_ues:
                    acted_ue_idx = current_ue_idx
                hidden_bank[acted_ue_idx] = next_hidden.detach().squeeze(0)
                current_ue_idx = 0 if done else (acted_ue_idx + 1) % args.num_ues
                if done:
                    hidden_bank = net.zero_hidden(args.num_ues, torch.device(args.device))
            else:
                hidden = next_hidden.detach() if not done else net.zero_hidden(1, torch.device(args.device))

            if done:
                episode_rewards.append(running_reward)
                running_reward = 0.0

                for k, values in current_metrics.items():
                    if not values:
                        continue
                    arr = np.asarray(values, dtype=np.float64)
                    episode_metric_means[k].append(float(np.mean(arr)))
                    episode_metric_last[k].append(float(arr[-1]))

                for k, v in _per_ue_tail(current_metrics).items():
                    episode_metric_means[k].append(v)
                    episode_metric_last[k].append(v)

                current_metrics.clear()
                finished += 1

                reset_seed = args.seed + finished
                obs, _ = env.reset(seed=reset_seed)
                obs = np.asarray(obs, dtype=np.float32)

    if step_log_fh is not None:
        step_log_fh.close()
    env.close()

    summary = {
        "model_path": os.path.abspath(args.model_path),
        "episodes": int(args.episodes),
        "env": {
            "backend": args.backend,
            "num_ues": args.num_ues,
            "step_time_s": args.step_time_s,
            "episode_time_s": args.episode_time_s,
            "switch_delay_ms": args.switch_delay_ms,
            "queue_max_bytes": args.queue_max_bytes,
            "reward_lambda_switch": args.reward_lambda_switch,
            "reward_lambda_queue": args.reward_lambda_queue,
            "reward_lambda_delay": args.reward_lambda_delay,
            "shared_per_ue": bool(args.shared_per_ue),
            "enable_rl_mcs_control": bool(args.enable_rl_mcs_control),
            "ns3_script": args.ns3_script,
            "ns3_args": parse_extra_args(args.ns3_arg),
        },
        "policy": {
            "action_selection_quantile": args.action_selection_quantile,
            "conditional_risk_selection": args.conditional_risk_selection,
            "risk_low_cqi_threshold": args.risk_low_cqi_threshold,
            "risk_mid_cqi_threshold": args.risk_mid_cqi_threshold,
            "risk_quantile_low": args.risk_quantile_low,
            "risk_quantile_mid": args.risk_quantile_mid,
            "risk_quantile_high": args.risk_quantile_high,
        },
        "episode_rewards": [float(x) for x in episode_rewards],
        "reward_mean": float(np.mean(episode_rewards)) if episode_rewards else 0.0,
        "reward_std": float(np.std(episode_rewards)) if episode_rewards else 0.0,
        "reward_p25": float(np.percentile(episode_rewards, 25)) if episode_rewards else 0.0,
        "reward_p75": float(np.percentile(episode_rewards, 75)) if episode_rewards else 0.0,
        "episode_metric_means": {},
        "episode_metric_last": {},
        "per_ue_last_percentiles": _per_ue_distribution_summary(episode_metric_last),
    }

    for k, values in episode_metric_means.items():
        arr = np.asarray(values, dtype=np.float64)
        summary["episode_metric_means"][k] = {
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr)),
            "p25": float(np.percentile(arr, 25)),
            "p75": float(np.percentile(arr, 75)),
        }

    for k, values in episode_metric_last.items():
        arr = np.asarray(values, dtype=np.float64)
        summary["episode_metric_last"][k] = {
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr)),
            "p25": float(np.percentile(arr, 25)),
            "p75": float(np.percentile(arr, 75)),
        }

    if args.output_json:
        out_dir = os.path.dirname(args.output_json)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.output_json, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
