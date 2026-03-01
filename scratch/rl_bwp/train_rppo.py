#!/usr/bin/env python3
"""
Train Recurrent PPO for UE-wise BWP switching + MCS delta control.
"""

from __future__ import annotations

import argparse
import os
from dataclasses import asdict

from sb3_contrib import RecurrentPPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs import EnvConfig, MockBwpEnv, Ns3BwpEnv


def build_env(args, eval_mode: bool = False, port: int | None = None, sim_seed: int | None = None):
    cfg = EnvConfig(
        num_ues=args.num_ues,
        step_time_s=args.step_time_s,
        episode_time_s=args.episode_time_s,
        switch_delay_ms=args.switch_delay_ms,
        queue_max_bytes=args.queue_max_bytes,
        reward_lambda_switch=args.reward_lambda_switch,
        reward_lambda_queue=args.reward_lambda_queue,
        reward_lambda_delay=args.reward_lambda_delay,
        seed=args.seed + (10000 if eval_mode else 0),
    )

    def _factory():
        if args.backend == "mock":
            env = MockBwpEnv(cfg)
        else:
            env = Ns3BwpEnv(
                cfg=cfg,
                port=args.port if port is None else port,
                start_sim=args.start_sim,
                sim_seed=args.seed if sim_seed is None else sim_seed,
                sim_args={},
                debug=args.debug_ns3,
            )
        return Monitor(env)

    return cfg, _factory


def main():
    parser = argparse.ArgumentParser(description="Train Recurrent PPO for BWP switching.")
    parser.add_argument("--backend", choices=["mock", "ns3"], default="mock")
    parser.add_argument("--run-name", type=str, default="rppo_bwp_ue1")
    parser.add_argument("--total-timesteps", type=int, default=300000)
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
    parser.add_argument("--n-steps", type=int, default=512)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--n-epochs", type=int, default=10)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae-lambda", type=float, default=0.95)
    parser.add_argument("--clip-range", type=float, default=0.2)
    parser.add_argument("--ent-coef", type=float, default=0.01)
    parser.add_argument("--vf-coef", type=float, default=0.5)
    parser.add_argument("--max-grad-norm", type=float, default=0.5)
    parser.add_argument("--eval-freq", type=int, default=10000)
    parser.add_argument("--checkpoint-freq", type=int, default=25000)
    parser.add_argument("--enable-eval", action="store_true", default=False)
    parser.add_argument("--eval-port", type=int, default=5556)
    parser.add_argument("--device", type=str, default="auto")
    args = parser.parse_args()

    out_dir = os.path.join("scratch", "rl_bwp", "runs", args.run_name)
    os.makedirs(out_dir, exist_ok=True)

    print("Creating training env...", flush=True)
    cfg, train_env_factory = build_env(args, eval_mode=False, port=args.port, sim_seed=args.seed)
    train_env = DummyVecEnv([train_env_factory])
    train_env = VecNormalize(
        train_env,
        norm_obs=True,
        norm_reward=True,
        clip_obs=10.0,
        gamma=args.gamma,
    )

    eval_env = None
    if args.enable_eval:
        eval_port = args.eval_port if args.backend == "ns3" else args.port
        if args.backend == "ns3" and eval_port == args.port:
            raise ValueError("For ns3 backend, --eval-port must differ from --port.")
        print("Creating eval env...", flush=True)
        _, eval_env_factory = build_env(
            args,
            eval_mode=True,
            port=eval_port,
            sim_seed=args.seed + 10000,
        )
        eval_env = DummyVecEnv([eval_env_factory])
        eval_env = VecNormalize(
            eval_env,
            norm_obs=True,
            norm_reward=False,
            clip_obs=10.0,
            gamma=args.gamma,
            training=False,
        )
        eval_env.obs_rms = train_env.obs_rms

    callbacks = [
        CheckpointCallback(
            save_freq=max(1, args.checkpoint_freq),
            save_path=out_dir,
            name_prefix="checkpoint",
            save_replay_buffer=False,
            save_vecnormalize=True,
        )
    ]
    if eval_env is not None:
        callbacks.append(
            EvalCallback(
                eval_env,
                best_model_save_path=out_dir,
                log_path=out_dir,
                eval_freq=max(1, args.eval_freq),
                n_eval_episodes=5,
                deterministic=True,
                render=False,
            )
        )

    model = RecurrentPPO(
        policy="MlpLstmPolicy",
        env=train_env,
        learning_rate=args.learning_rate,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_range=args.clip_range,
        ent_coef=args.ent_coef,
        vf_coef=args.vf_coef,
        max_grad_norm=args.max_grad_norm,
        policy_kwargs={
            "lstm_hidden_size": 128,
            "n_lstm_layers": 1,
            "shared_lstm": False,
            "enable_critic_lstm": True,
            "net_arch": [128, 128],
        },
        seed=args.seed,
        verbose=1,
        tensorboard_log=out_dir,
        device=args.device,
    )

    print("Training config:")
    print({**asdict(cfg), "backend": args.backend, "total_timesteps": args.total_timesteps})
    if args.backend == "ns3" and not args.enable_eval:
        print("ns3 backend: eval env disabled by default to avoid port/session deadlocks.", flush=True)
    model.learn(total_timesteps=args.total_timesteps, callback=callbacks, progress_bar=True)

    model_path = os.path.join(out_dir, "final_model.zip")
    vn_path = os.path.join(out_dir, "vecnormalize.pkl")
    model.save(model_path)
    train_env.save(vn_path)
    print(f"Saved model: {model_path}")
    print(f"Saved VecNormalize stats: {vn_path}")

    train_env.close()
    if eval_env is not None:
        eval_env.close()


if __name__ == "__main__":
    main()
