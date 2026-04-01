#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
from collections import defaultdict
from numbers import Real

import numpy as np
import torch

from drqn_torchrl import DrqnNet
from envs import EnvConfig, MockBwpEnv, Ns3BwpEnv, SharedPerUeAdapter


STALE_GLOBAL_REWARD_KEYS = {
    "reward_aoi_term",
    "reward_aoi_penalty",
    "reward_thr_term",
    "reward_goodput_term",
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
        "aoi_ms": r"^ue\d+_aoi_ms$",
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
            "p95": float(np.percentile(arr, 95)),
            "max": float(np.max(arr)),
        }
    return out


def build_env(args):
    ns3_args = parse_extra_args(args.ns3_arg)
    ns3_args["rlDrqnProfile"] = "true"
    ns3_args["enableRlMcsControl"] = "false"
    cfg = EnvConfig(
        num_ues=args.num_ues,
        step_time_s=args.step_time_s,
        episode_time_s=args.episode_time_s,
        switch_delay_ms=args.switch_delay_ms,
        queue_max_bytes=args.queue_max_bytes,
        reward_lambda_switch=args.reward_lambda_switch,
        reward_lambda_queue=args.reward_lambda_queue,
        reward_lambda_delay=args.reward_lambda_delay,
        seed=args.seed,
        bwp_only_actions=True,
        drqn_profile=True,
    )
    if args.backend == "mock":
        env = MockBwpEnv(cfg)
    else:
        base_sim_args = {
            "numUes": cfg.num_ues,
            "simTime": cfg.episode_time_s,
            "envStepTime": cfg.step_time_s,
        }
        base_sim_args.update(ns3_args)
        env = Ns3BwpEnv(
            cfg=cfg,
            port=args.port,
            start_sim=args.start_sim,
            sim_seed=args.seed,
            sim_args=base_sim_args,
            sim_script=args.ns3_script,
            debug=args.debug_ns3,
        )
    if args.shared_per_ue:
        env = SharedPerUeAdapter(env)
    return env


def main():
    parser = argparse.ArgumentParser(description="Evaluate TorchRL-style DRQN over OpenGym/ns3 env.")
    parser.add_argument("--backend", choices=["mock", "ns3"], default="mock")
    parser.add_argument("--model-path", type=str, required=True)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--num-ues", type=int, default=1)
    parser.add_argument("--step-time-s", type=float, default=0.02)
    parser.add_argument("--episode-time-s", type=float, default=10.0)
    parser.add_argument("--switch-delay-ms", type=float, default=5.0)
    parser.add_argument("--queue-max-bytes", type=float, default=200000.0)
    parser.add_argument("--reward-lambda-switch", type=float, default=0.01)
    parser.add_argument("--reward-lambda-queue", type=float, default=0.20)
    parser.add_argument("--reward-lambda-delay", type=float, default=0.10)
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--start-sim", action="store_true", default=False)
    parser.add_argument("--debug-ns3", action="store_true", default=False)
    parser.add_argument("--ns3-script", type=str, default="aoi-prb-urban-onoff")
    parser.add_argument("--ns3-arg", action="append", default=[])
    parser.add_argument("--output-json", type=str, default="")
    parser.add_argument("--step-log-csv", type=str, default="")
    parser.add_argument("--shared-per-ue", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--device", type=str, default="cpu")
    args = parser.parse_args()

    ckpt = torch.load(args.model_path, map_location=args.device)
    net = DrqnNet(int(ckpt["obs_dim"]), int(ckpt["hidden_dim"]), int(ckpt["action_dim"]))
    net.load_state_dict(ckpt["state_dict"])
    net.to(args.device)
    net.eval()

    env = build_env(args)
    obs, _ = env.reset(seed=args.seed)
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
                "action_bwp",
                "reward",
                "reward_aoi_penalty_local",
                "reward_goodput_term_local",
                "reward_aux_term_local",
                "reward_se_term_local",
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
            obs_used = np.asarray(obs).reshape(-1).copy()
            obs_t = torch.as_tensor(obs_used, dtype=torch.float32, device=args.device).unsqueeze(0)
            q, next_hidden = net.forward_step(obs_t, hidden)
            action = int(torch.argmax(q, dim=-1).item())
            obs, reward, terminated, truncated, info = env.step([action])
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
                step_log_writer.writerow(
                    [
                        finished,
                        step_index,
                        ue_index,
                        action,
                        float(reward),
                        step_metrics.get("reward_aoi_penalty_local", 0.0),
                        step_metrics.get("reward_goodput_term_local", 0.0),
                        step_metrics.get("reward_aux_term_local", 0.0),
                        step_metrics.get("reward_se_term_local", 0.0),
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
            obs = np.asarray(obs, dtype=np.float32)
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
                obs, _ = env.reset()
                obs = np.asarray(obs, dtype=np.float32)

    if step_log_fh is not None:
        step_log_fh.close()
    env.close()

    summary = {
        "model_path": os.path.abspath(args.model_path),
        "episodes": int(args.episodes),
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
