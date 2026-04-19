#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import random
from dataclasses import asdict

import numpy as np
import torch

from drqn_torchrl import (
    DrqnConfig,
    DrqnNet,
    episode_to_sequences,
    epsilon_by_step,
    make_replay_buffer,
    select_action,
    train_batch,
)
from envs import EnvConfig, MockBwpEnv, Ns3BwpEnv, SharedPerUeAdapter
from tensordict import TensorDict


def parse_extra_args(items: list[str]) -> dict[str, str]:
    out: dict[str, str] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"Invalid --ns3-arg '{item}', expected key=value.")
        k, v = item.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def build_env(args, *, seed: int):
    ns3_args = parse_extra_args(args.ns3_arg)
    ns3_args["enableOpenGym"] = "true"
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
        seed=seed,
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
            sim_seed=seed,
            sim_args=base_sim_args,
            sim_script=args.ns3_script,
            debug=args.debug_ns3,
        )
    if args.shared_per_ue:
        env = SharedPerUeAdapter(env)
    return cfg, env


def steps_per_episode(cfg: EnvConfig, *, shared_per_ue: bool) -> int:
    base_steps = max(1, int(round(cfg.episode_time_s / cfg.step_time_s)))
    if shared_per_ue:
        return base_steps * cfg.num_ues
    return base_steps


def save_checkpoint(
    out_dir: str,
    episode_count: int,
    online_net: DrqnNet,
    obs_dim: int,
    action_dim: int,
    hidden_dim: int,
    drqn_cfg: DrqnConfig,
) -> str:
    ckpt_path = os.path.join(out_dir, f"checkpoint_ep{episode_count:04d}.pt")
    ckpt = {
        "state_dict": online_net.state_dict(),
        "obs_dim": obs_dim,
        "action_dim": action_dim,
        "hidden_dim": hidden_dim,
        "drqn_config": asdict(drqn_cfg),
        "episode": episode_count,
    }
    torch.save(ckpt, ckpt_path)
    return ckpt_path


def main():
    parser = argparse.ArgumentParser(description="Train TorchRL-style DRQN over OpenGym/ns3 env.")
    parser.add_argument("--backend", choices=["mock", "ns3"], default="mock")
    parser.add_argument("--run-name", type=str, default="drqn_bwp_ue1")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--num-ues", type=int, default=20)
    parser.add_argument("--step-time-s", type=float, default=0.01)
    parser.add_argument("--episode-time-s", type=float, default=5.0)
    parser.add_argument("--switch-delay-ms", type=float, default=5.0)
    parser.add_argument("--queue-max-bytes", type=float, default=200000.0)
    parser.add_argument("--reward-lambda-switch", type=float, default=0.01)
    parser.add_argument("--reward-lambda-queue", type=float, default=0.20)
    parser.add_argument("--reward-lambda-delay", type=float, default=0.10)
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--start-sim", action="store_true", default=True)
    parser.add_argument("--debug-ns3", action="store_true", default=False)
    parser.add_argument("--ns3-script", type=str, default="aoi-prb-urban-appmix")
    parser.add_argument("--ns3-arg", action="append", default=[])
    parser.add_argument("--shared-per-ue", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--total-env-steps", type=int, default=20000)
    parser.add_argument("--hidden-dim", type=int, default=128)
    parser.add_argument("--seq-len", type=int, default=8)
    parser.add_argument("--burn-in", type=int, default=4)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--replay-capacity", type=int, default=2000)
    parser.add_argument("--warmup-sequences", type=int, default=64)
    parser.add_argument("--target-sync", type=int, default=100)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--gamma", type=float, default=0.98)
    parser.add_argument("--eps-start", type=float, default=1.0)
    parser.add_argument("--eps-end", type=float, default=0.05)
    parser.add_argument("--eps-decay", type=float, default=0.999)
    parser.add_argument("--train-updates-per-step", type=int, default=1)
    parser.add_argument("--final-train-updates", type=int, default=256)
    parser.add_argument("--min-completed-episodes", type=int, default=1)
    parser.add_argument("--reward-bin-size", type=int, default=300)
    parser.add_argument("--loss-bin-size", type=int, default=300)
    parser.add_argument("--disable-step-reward-log", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--save-every-episodes", type=int, default=0)
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument("--init-model-path", type=str, default="")
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)

    out_dir = os.path.join("/home/jshyeon/ns-3.46-bwp/scratch/rl_bwp/runs", args.run_name, "drqn")
    print(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    episode_metrics_path = os.path.join(out_dir, "train_episode_metrics.csv")
    step_reward_log_path = os.path.join(out_dir, "step_reward_log.csv")
    reward_bin_log_path = os.path.join(out_dir, f"train_reward_bins_{args.reward_bin_size}.csv")
    value_loss_log_path = os.path.join(out_dir, f"value_loss_{args.loss_bin_size}.csv")
    gru_loss_log_path = os.path.join(out_dir, f"gru_loss_{args.loss_bin_size}.csv")
    with open(episode_metrics_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["episode", "global_step", "mean_reward", "mean_thr_mbps", "mean_aoi_ms"],
        )
        writer.writeheader()
    with open(reward_bin_log_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["episode", "bin_index", "bin_size", "global_step_end", "avg_reward"],
        )
        writer.writeheader()
    with open(value_loss_log_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["episode", "global_step_end", "window_steps", "num_updates", "avg_value_loss"],
        )
        writer.writeheader()
    with open(gru_loss_log_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["episode", "global_step_end", "window_steps", "num_updates", "avg_gru_loss"],
        )
        writer.writeheader()
    if not args.disable_step_reward_log:
        with open(step_reward_log_path, "w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "episode",
                    "episode_step",
                    "global_step",
                    "ue_index",
                    "reward",
                    "mean_thr_mbps",
                    "mean_aoi_ms",
                    "reward_aoi_penalty_local",
                    "reward_service_ratio_local",
                    "reward_aux_term_local",
                    "reward_switch_penalty_local",
                ],
            )
            writer.writeheader()

    cfg, env = build_env(args, seed=args.seed)
    episode_step_budget = steps_per_episode(cfg, shared_per_ue=bool(args.shared_per_ue))
    effective_total_env_steps = args.total_env_steps
    if args.min_completed_episodes > 0:
        effective_total_env_steps = max(
            effective_total_env_steps, episode_step_budget * args.min_completed_episodes
        )
    obs, _ = env.reset(seed=args.seed)
    obs = obs.astype("float32")
    obs_dim = int(obs.shape[0])
    action_dim = int(env.action_space.nvec[0]) if hasattr(env.action_space, "nvec") else int(env.action_space.n)

    drqn_cfg = DrqnConfig(
        obs_dim=obs_dim,
        action_dim=action_dim,
        hidden_dim=args.hidden_dim,
        seq_len=args.seq_len,
        burn_in=args.burn_in,
        gamma=args.gamma,
        lr=args.learning_rate,
        batch_size=args.batch_size,
        replay_capacity=args.replay_capacity,
        warmup_sequences=args.warmup_sequences,
        target_sync=args.target_sync,
        eps_start=args.eps_start,
        eps_end=args.eps_end,
        eps_decay=args.eps_decay,
        train_updates_per_step=args.train_updates_per_step,
    )

    device = torch.device(args.device)
    online_net = DrqnNet(obs_dim, args.hidden_dim, action_dim).to(device)
    target_net = DrqnNet(obs_dim, args.hidden_dim, action_dim).to(device)
    if args.init_model_path:
        ckpt = torch.load(args.init_model_path, map_location=device)
        online_net.load_state_dict(ckpt["state_dict"])
    target_net.load_state_dict(online_net.state_dict())
    optimizer = torch.optim.Adam(online_net.parameters(), lr=args.learning_rate)
    replay = make_replay_buffer(args.replay_capacity)

    hidden = online_net.zero_hidden(1, device)
    episode_transitions: list[dict] = []
    global_step = 0
    episode_count = 0
    episode_step = 0
    episode_bin_index = 0
    reward_bin_values: list[float] = []
    value_loss_bin_values: list[float] = []
    gru_loss_bin_values: list[float] = []
    recent_rewards: list[float] = []
    loss_history: list[float] = []
    last_step_info: dict[str, float] = {}
    episode_reward_values: list[float] = []
    episode_thr_values: list[float] = []
    episode_aoi_values: list[float] = []

    while global_step < effective_total_env_steps:
        epsilon = epsilon_by_step(drqn_cfg, global_step)
        action, next_hidden = select_action(online_net, obs, hidden, epsilon, device)
        next_obs, reward, terminated, truncated, step_info = env.step([action])
        last_step_info = dict(step_info)
        done = bool(terminated or truncated)
        forced_episode_end = bool(episode_step_budget > 0 and episode_step + 1 >= episode_step_budget)
        episode_end = bool(done or forced_episode_end)
        next_obs = next_obs.astype("float32")
        episode_transitions.append(
            {
                "obs": obs.copy(),
                "action": int(action),
                "reward": float(reward),
                "next_obs": next_obs.copy(),
                "done": episode_end,
            }
        )
        recent_rewards.append(float(reward))
        episode_reward_values.append(float(reward))
        metric_valid = float(step_info.get("metric_valid", 1.0)) > 0.5
        if metric_valid:
            episode_thr_values.append(float(step_info.get("mean_thr_mbps", 0.0)))
            episode_aoi_values.append(float(step_info.get("mean_aoi_ms", 0.0)))
        episode_step += 1
        step_row = {
            "episode": episode_count + 1,
            "episode_step": episode_step,
            "global_step": global_step + 1,
            "ue_index": int(step_info.get("ue_index", -1)),
            "reward": float(reward),
            "mean_thr_mbps": float(step_info.get("mean_thr_mbps", 0.0)),
            "mean_aoi_ms": float(step_info.get("mean_aoi_ms", 0.0)),
            "reward_aoi_penalty_local": float(step_info.get("reward_aoi_penalty_local", 0.0)),
            "reward_service_ratio_local": float(step_info.get("reward_service_ratio_local", 0.0)),
            "reward_aux_term_local": float(step_info.get("reward_aux_term_local", 0.0)),
            "reward_switch_penalty_local": float(step_info.get("reward_switch_penalty_local", 0.0)),
        }
        reward_bin_values.append(float(reward))
        if not args.disable_step_reward_log:
            with open(step_reward_log_path, "a", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=[
                        "episode",
                        "episode_step",
                        "global_step",
                        "ue_index",
                        "reward",
                        "mean_thr_mbps",
                        "mean_aoi_ms",
                        "reward_aoi_penalty_local",
                        "reward_service_ratio_local",
                        "reward_aux_term_local",
                        "reward_switch_penalty_local",
                    ],
                )
                writer.writerow(step_row)
        if len(reward_bin_values) == args.reward_bin_size:
            with open(reward_bin_log_path, "a", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=["episode", "bin_index", "bin_size", "global_step_end", "avg_reward"],
                )
                writer.writerow(
                    {
                        "episode": episode_count + 1,
                        "bin_index": episode_bin_index,
                        "bin_size": len(reward_bin_values),
                        "global_step_end": global_step + 1,
                        "avg_reward": float(sum(reward_bin_values) / len(reward_bin_values)),
                    }
                )
            reward_bin_values.clear()
            episode_bin_index += 1

        if len(replay) >= args.warmup_sequences:
            for _ in range(args.train_updates_per_step):
                batch = replay.sample(args.batch_size)
                loss = train_batch(online_net, target_net, optimizer, batch, drqn_cfg, device)
                loss_history.append(loss)
                value_loss_bin_values.append(float(loss))
                # DRQN has a single TD objective; log the same scalar for GRU loss trend tracking.
                gru_loss_bin_values.append(float(loss))

        if global_step > 0 and global_step % args.target_sync == 0:
            target_net.load_state_dict(online_net.state_dict())

        obs = next_obs
        hidden = next_hidden.detach()
        global_step += 1

        if global_step % args.loss_bin_size == 0:
            with open(value_loss_log_path, "a", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=["episode", "global_step_end", "window_steps", "num_updates", "avg_value_loss"],
                )
                writer.writerow(
                    {
                        "episode": episode_count + 1,
                        "global_step_end": global_step,
                        "window_steps": args.loss_bin_size,
                        "num_updates": len(value_loss_bin_values),
                        "avg_value_loss": float(sum(value_loss_bin_values) / len(value_loss_bin_values))
                        if value_loss_bin_values
                        else 0.0,
                    }
                )
            with open(gru_loss_log_path, "a", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=["episode", "global_step_end", "window_steps", "num_updates", "avg_gru_loss"],
                )
                writer.writerow(
                    {
                        "episode": episode_count + 1,
                        "global_step_end": global_step,
                        "window_steps": args.loss_bin_size,
                        "num_updates": len(gru_loss_bin_values),
                        "avg_gru_loss": float(sum(gru_loss_bin_values) / len(gru_loss_bin_values))
                        if gru_loss_bin_values
                        else 0.0,
                    }
                )
            value_loss_bin_values.clear()
            gru_loss_bin_values.clear()

        if episode_end:
            for seq_td in episode_to_sequences(episode_transitions, args.seq_len):
                replay.add(seq_td)
            episode_transitions.clear()
            episode_count += 1
            episode_row = {
                "episode": episode_count,
                "global_step": global_step,
                "mean_reward": float(sum(episode_reward_values) / max(1, len(episode_reward_values))),
                "mean_thr_mbps": float(sum(episode_thr_values) / max(1, len(episode_thr_values))),
                "mean_aoi_ms": float(sum(episode_aoi_values) / max(1, len(episode_aoi_values))),
            }
            with open(episode_metrics_path, "a", encoding="utf-8", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=["episode", "global_step", "mean_reward", "mean_thr_mbps", "mean_aoi_ms"],
                )
                writer.writerow(episode_row)
            if reward_bin_values:
                with open(reward_bin_log_path, "a", encoding="utf-8", newline="") as f:
                    writer = csv.DictWriter(
                        f,
                        fieldnames=["episode", "bin_index", "bin_size", "global_step_end", "avg_reward"],
                    )
                    writer.writerow(
                        {
                            "episode": episode_count,
                            "bin_index": episode_bin_index,
                            "bin_size": len(reward_bin_values),
                            "global_step_end": global_step,
                            "avg_reward": float(sum(reward_bin_values) / len(reward_bin_values)),
                        }
                    )
                reward_bin_values.clear()
            next_episode_seed = args.seed + episode_count
            obs, _ = env.reset(seed=next_episode_seed)
            obs = obs.astype("float32")
            hidden = online_net.zero_hidden(1, device)
            episode_step = 0
            episode_bin_index = 0
            episode_reward_values.clear()
            episode_thr_values.clear()
            episode_aoi_values.clear()
            if args.save_every_episodes > 0 and episode_count % args.save_every_episodes == 0:
                save_checkpoint(out_dir, episode_count, online_net, obs_dim, action_dim, args.hidden_dim, drqn_cfg)

    if episode_transitions:
        for seq_td in episode_to_sequences(episode_transitions, args.seq_len):
            replay.add(seq_td)

    if len(replay) >= args.warmup_sequences:
        for _ in range(args.final_train_updates):
            batch = replay.sample(args.batch_size)
            loss = train_batch(online_net, target_net, optimizer, batch, drqn_cfg, device)
            loss_history.append(loss)
            value_loss_bin_values.append(float(loss))
            gru_loss_bin_values.append(float(loss))

    if value_loss_bin_values:
        with open(value_loss_log_path, "a", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["episode", "global_step_end", "window_steps", "num_updates", "avg_value_loss"],
            )
            writer.writerow(
                {
                    "episode": episode_count if episode_count > 0 else 0,
                    "global_step_end": global_step,
                    "window_steps": global_step % args.loss_bin_size if global_step % args.loss_bin_size != 0 else args.loss_bin_size,
                    "num_updates": len(value_loss_bin_values),
                    "avg_value_loss": float(sum(value_loss_bin_values) / len(value_loss_bin_values)),
                }
            )
    if gru_loss_bin_values:
        with open(gru_loss_log_path, "a", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["episode", "global_step_end", "window_steps", "num_updates", "avg_gru_loss"],
            )
            writer.writerow(
                {
                    "episode": episode_count if episode_count > 0 else 0,
                    "global_step_end": global_step,
                    "window_steps": global_step % args.loss_bin_size if global_step % args.loss_bin_size != 0 else args.loss_bin_size,
                    "num_updates": len(gru_loss_bin_values),
                    "avg_gru_loss": float(sum(gru_loss_bin_values) / len(gru_loss_bin_values)),
                }
            )

    model_path = save_checkpoint(out_dir, episode_count if episode_count > 0 else 0, online_net, obs_dim, action_dim, args.hidden_dim, drqn_cfg)
    final_model_path = os.path.join(out_dir, "final_model.pt")
    torch.save(torch.load(model_path, map_location="cpu"), final_model_path)

    run_cfg = {
        "backend": args.backend,
        "env": asdict(cfg),
        "ns3_script": args.ns3_script,
        "ns3_args": parse_extra_args(args.ns3_arg)
        | {"enableOpenGym": "true", "rlDrqnProfile": "true", "enableRlMcsControl": "false"},
        "model": asdict(drqn_cfg),
        "shared_per_ue": bool(args.shared_per_ue),
        "total_env_steps": args.total_env_steps,
        "effective_total_env_steps": effective_total_env_steps,
        "episode_step_budget": episode_step_budget,
        "final_train_updates": args.final_train_updates,
        "min_completed_episodes": args.min_completed_episodes,
        "episode_metrics_csv": episode_metrics_path,
        "step_reward_log_csv": step_reward_log_path if not args.disable_step_reward_log else "",
        "reward_bin_csv": reward_bin_log_path,
        "value_loss_csv": value_loss_log_path,
        "gru_loss_csv": gru_loss_log_path,
        "init_model_path": args.init_model_path,
        "save_every_episodes": args.save_every_episodes,
        "episodes_finished": episode_count,
        "mean_recent_reward": float(sum(recent_rewards[-200:]) / max(1, len(recent_rewards[-200:]))),
        "mean_recent_loss": float(sum(loss_history[-200:]) / max(1, len(loss_history[-200:]))) if loss_history else 0.0,
    }
    with open(os.path.join(out_dir, "run_config.json"), "w", encoding="utf-8") as f:
        json.dump(run_cfg, f, indent=2)
    print(f"Saved model: {final_model_path}")
    env.close()


if __name__ == "__main__":
    main()
