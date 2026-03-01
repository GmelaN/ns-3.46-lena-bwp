#!/usr/bin/env python3
"""
Evaluate a trained Recurrent PPO model.
"""

from __future__ import annotations

import argparse
import os

import numpy as np
from sb3_contrib import RecurrentPPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs import EnvConfig, MockBwpEnv, Ns3BwpEnv


def build_env(args):
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
    )

    def _factory():
        if args.backend == "mock":
            env = MockBwpEnv(cfg)
        else:
            env = Ns3BwpEnv(
                cfg=cfg,
                port=args.port,
                start_sim=args.start_sim,
                sim_seed=args.seed,
                sim_args={},
                debug=args.debug_ns3,
            )
        return Monitor(env)

    return DummyVecEnv([_factory])


def main():
    parser = argparse.ArgumentParser(description="Evaluate Recurrent PPO for BWP switching.")
    parser.add_argument("--backend", choices=["mock", "ns3"], default="mock")
    parser.add_argument("--model-path", type=str, required=True)
    parser.add_argument("--vecnorm-path", type=str, default="")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--num-ues", type=int, default=1)
    parser.add_argument("--step-time-s", type=float, default=0.02)
    parser.add_argument("--episode-time-s", type=float, default=10.0)
    parser.add_argument("--switch-delay-ms", type=float, default=5.0)
    parser.add_argument("--queue-max-bytes", type=float, default=200000.0)
    parser.add_argument("--reward-lambda-switch", type=float, default=0.05)
    parser.add_argument("--reward-lambda-queue", type=float, default=0.20)
    parser.add_argument("--reward-lambda-delay", type=float, default=0.10)
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--start-sim", action="store_true", default=False)
    parser.add_argument("--debug-ns3", action="store_true", default=False)
    parser.add_argument("--device", type=str, default="auto")
    args = parser.parse_args()

    env = build_env(args)
    if args.vecnorm_path:
        if not os.path.exists(args.vecnorm_path):
            raise FileNotFoundError(f"VecNormalize file not found: {args.vecnorm_path}")
        env = VecNormalize.load(args.vecnorm_path, env)
        env.training = False
        env.norm_reward = False

    model = RecurrentPPO.load(args.model_path, env=env, device=args.device)

    episode_rewards = []
    lstm_states = None
    episode_starts = np.ones((env.num_envs,), dtype=bool)

    obs = env.reset()
    running_reward = 0.0
    finished = 0

    while finished < args.episodes:
        action, lstm_states = model.predict(
            obs,
            state=lstm_states,
            episode_start=episode_starts,
            deterministic=True,
        )
        obs, rewards, dones, infos = env.step(action)
        running_reward += float(rewards[0])
        episode_starts = dones
        if dones[0]:
            episode_rewards.append(running_reward)
            running_reward = 0.0
            finished += 1
            lstm_states = None

    print(f"Episodes: {args.episodes}")
    print(f"Mean reward: {np.mean(episode_rewards):.4f}")
    print(f"Std reward:  {np.std(episode_rewards):.4f}")
    print(f"Min reward:  {np.min(episode_rewards):.4f}")
    print(f"Max reward:  {np.max(episode_rewards):.4f}")
    env.close()


if __name__ == "__main__":
    main()

