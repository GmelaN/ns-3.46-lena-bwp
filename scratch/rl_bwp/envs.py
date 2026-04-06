#!/usr/bin/env python3
"""
Environment definitions for Recurrent PPO based BWP/MCS control.

Two backends are provided:
1) MockBwpEnv: lightweight POMDP-style sandbox for training-loop validation.
2) Ns3BwpEnv: wrapper over ns3gym for live integration.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Any

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover
    import gym  # type: ignore
    from gym import spaces  # type: ignore


MCS_MIN = 0
MCS_MAX = 27
@dataclass
class EnvConfig:
    num_ues: int = 1
    step_time_s: float = 0.02  # 20 ms
    episode_time_s: float = 10.0
    switch_delay_ms: float = 5.0
    queue_max_bytes: float = 200000.0
    reward_lambda_switch: float = 0.0001
    reward_lambda_queue: float = 0.20
    reward_lambda_delay: float = 0.10
    seed: int = 1
    bwp_only_actions: bool = False
    drqn_profile: bool = False
    dqn_delay_target_ms: float = 80.0
    dqn_thr_target_mbps: float = 2.0
    dqn_latency_ue_ratio: float = 0.5
    dqn_alpha: float = 0.7
    dqn_beta: float = 0.3


def _encode_per_ue_action(bwp_cmd: int, mcs_delta_idx: int) -> int:
    """
    Encodes per-UE action into one discrete code:
    bwp_cmd in {0: force BWP 0, 1: force BWP 1}
    mcs_delta_idx in {0,1,2,3,4} -> delta in {-2,-1,0,+1,+2}
    """
    return int(bwp_cmd) * 5 + int(mcs_delta_idx)


def _decode_mcs_delta(mcs_delta_idx: int) -> int:
    return (-2, -1, 0, +1, +2)[int(mcs_delta_idx)]


def _bwp_se_penalty(ue_se: float, bwp_se: float) -> float:
    if ue_se <= 0.0:
        return 0.0
    return float(np.log1p(max(0.0, bwp_se) / max(1e-6, ue_se)))


def _signed_log1p(x: np.ndarray | float) -> np.ndarray | float:
    return np.sign(x) * np.log1p(np.abs(x))


class BaseBwpEnv(gym.Env):
    """
    Shared action/observation conventions.
    Observation is a flat vector containing per-UE features.
    """

    metadata = {"render_modes": []}

    def __init__(self, cfg: EnvConfig):
        super().__init__()
        self.cfg = cfg
        self.num_ues = cfg.num_ues
        self.features_per_ue = 13 if cfg.drqn_profile else 9
        self.obs_size = self.features_per_ue * self.num_ues

        if cfg.bwp_only_actions:
            action_nvec = np.array(([2] * self.num_ues), dtype=np.int64)
        else:
            action_nvec = np.array(([2, 5] * self.num_ues), dtype=np.int64)
        obs_low = -max(float(cfg.queue_max_bytes) * 2.0, 1.0e6)
        obs_high = max(float(cfg.queue_max_bytes) * 2.0, 1.0e6)
        self.action_space = spaces.MultiDiscrete(action_nvec)
        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            shape=(self.obs_size,),
            dtype=np.float32,
        )

    def _shape_reward(
        self,
        aoi_ms: float,
        prev_aoi_ms: float,
        throughput_mbps: float,
        switch_count: int,
    ) -> float:
        reward = -float(np.log1p(max(0.0, aoi_ms)) - np.log1p(max(0.0, prev_aoi_ms)))
        reward += float(np.log1p(max(0.0, throughput_mbps)))
        reward -= self.cfg.reward_lambda_switch * float(switch_count)
        return float(reward)


class SharedPerUeAdapter(gym.Env):
    """
    Keeps a shared policy but feeds one UE sample at a time.
    A full global action set is accumulated across UE samples, then dispatched once.
    Rewards are returned per UE on the next cycle after the joint action is applied.
    """

    metadata = {"render_modes": []}

    def __init__(self, env: BaseBwpEnv):
        super().__init__()
        self._env = env
        self.cfg = env.cfg
        self.num_ues = env.num_ues
        self.features_per_ue = env.features_per_ue
        if self.cfg.bwp_only_actions:
            self.action_space = spaces.MultiDiscrete(np.array([2], dtype=np.int64))
        else:
            self.action_space = spaces.MultiDiscrete(np.array([2, 5], dtype=np.int64))
        self.observation_space = spaces.Box(
            low=-100.0,
            high=100.0,
            shape=(self.features_per_ue,),
            dtype=np.float32,
        )

        self._global_obs = np.zeros((self.num_ues, self.features_per_ue), dtype=np.float32)
        self._pending_actions = np.zeros((self.num_ues, 2), dtype=np.int64)
        self._reward_buffer = np.zeros(self.num_ues, dtype=np.float32)
        self._info_buffer: list[dict[str, Any]] = [dict() for _ in range(self.num_ues)]
        self._current_ue = 0
        self._flushing = False

    def _split_obs(self, obs: np.ndarray) -> np.ndarray:
        arr = np.asarray(obs, dtype=np.float32).reshape(self.num_ues, self.features_per_ue)
        return arr

    def _maybe_resize_from_obs(self, obs: np.ndarray) -> None:
        arr = np.asarray(obs, dtype=np.float32).reshape(-1)
        if arr.size == 0 or arr.size % self.num_ues != 0:
            return
        new_features = int(arr.size // self.num_ues)
        if new_features == self.features_per_ue:
            return
        self.features_per_ue = new_features
        self.observation_space = spaces.Box(
            low=-100.0,
            high=100.0,
            shape=(self.features_per_ue,),
            dtype=np.float32,
        )
        self._global_obs = np.zeros((self.num_ues, self.features_per_ue), dtype=np.float32)

    def _build_default_actions(self, obs: np.ndarray) -> np.ndarray:
        defaults = np.zeros((self.num_ues, 2), dtype=np.int64)
        bwp_col = 1 if self.cfg.drqn_profile else 0
        defaults[:, 0] = np.rint(obs[:, bwp_col]).astype(np.int64)
        defaults[:, 1] = 2
        return defaults

    def _compute_per_ue_rewards(
        self,
        prev_obs: np.ndarray,
        next_obs: np.ndarray,
        info: dict[str, Any],
    ) -> tuple[np.ndarray, list[dict[str, Any]]]:
        rewards = np.zeros(self.num_ues, dtype=np.float32)
        info_list: list[dict[str, Any]] = []
        for ue in range(self.num_ues):
            switch_penalty = 0.0
            if self.cfg.drqn_profile:
                aoi_ms = float(info.get(f"ue{ue}_aoi_ms", 0.0))
                thr_mbps = float(info.get(f"ue{ue}_thr_mbps", 0.0))
                delay_norm = float(np.clip(aoi_ms / max(1.0, self.cfg.dqn_delay_target_ms), 0.0, 1.0))
                thr_norm = float(np.clip(thr_mbps / max(1e-6, self.cfg.dqn_thr_target_mbps), 0.0, 1.0))
                latency_ues = int(round(self.cfg.dqn_latency_ue_ratio * self.num_ues))
                is_latency = ue < latency_ues
                lam = self.cfg.dqn_alpha if is_latency else (1.0 - self.cfg.dqn_alpha)
                mu = self.cfg.dqn_beta if is_latency else (1.0 - self.cfg.dqn_beta)
                aux_term = 0.0
                goodput_term = 0.0
                se_term = 0.0
                aoi_penalty = -(lam * delay_norm)
                reward = -(lam * delay_norm + mu * (1.0 - thr_norm))
            else:
                current_aoi_ms = float(info.get(f"ue{ue}_aoi_ms", 0.0))
                aoi_penalty = -float(np.log1p(max(0.0, current_aoi_ms)))
                delivered_bytes = float(info.get(f"ue{ue}_delivered_bytes", 0.0))
                goodput_term = float(np.log1p(max(0.0, delivered_bytes)))
                aux_term = 0.0
                se_term = 0.0
                switch_penalty = 0.0
                reward = aoi_penalty + goodput_term
            rewards[ue] = reward
            ue_info = dict(info)
            ue_info["ue_index"] = ue
            ue_info["metric_valid"] = 1.0
            ue_info["reward_aoi_penalty_local"] = aoi_penalty
            ue_info["reward_goodput_term_local"] = goodput_term
            ue_info["reward_aux_term_local"] = aux_term
            ue_info["reward_se_term_local"] = se_term
            ue_info["reward_bwp_se_local"] = 0.0
            ue_info["reward_switch_penalty_local"] = switch_penalty
            ue_info["reward_drop_penalty_local"] = 0.0
            info_list.append(ue_info)
        return rewards, info_list

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None):
        obs, info = self._env.reset(seed=seed, options=options)
        self._maybe_resize_from_obs(obs)
        self._global_obs = self._split_obs(obs)
        self._pending_actions = self._build_default_actions(self._global_obs)
        self._reward_buffer.fill(0.0)
        self._info_buffer = [dict(info, ue_index=ue, metric_valid=0.0) for ue in range(self.num_ues)]
        self._current_ue = 0
        self._flushing = False
        return self._global_obs[self._current_ue].copy(), self._info_buffer[self._current_ue]

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.int64).reshape(-1)
        expected_action_size = 1 if self.cfg.bwp_only_actions else 2
        if action.size != expected_action_size:
            raise ValueError(f"Expected per-UE action size {expected_action_size}, got {action.size}")

        ue = self._current_ue
        reward = float(self._reward_buffer[ue])
        info = dict(self._info_buffer[ue])

        if self._flushing:
            terminated = ue == self.num_ues - 1
            truncated = False
            if not terminated:
                self._current_ue += 1
            return self._global_obs[ue].copy(), reward, terminated, truncated, info

        self._pending_actions[ue, 0] = int(action[0])
        self._pending_actions[ue, 1] = 2 if self.cfg.bwp_only_actions else int(action[1])

        if ue < self.num_ues - 1:
            self._current_ue += 1
            return self._global_obs[self._current_ue].copy(), reward, False, False, info

        prev_obs = self._global_obs.copy()
        if self.cfg.bwp_only_actions:
            dispatch_action = self._pending_actions[:, 0].reshape(-1)
        else:
            dispatch_action = self._pending_actions.reshape(-1)
        next_obs_flat, _, terminated, truncated, next_info = self._env.step(dispatch_action)
        self._global_obs = self._split_obs(next_obs_flat)
        self._pending_actions = self._build_default_actions(self._global_obs)
        self._reward_buffer, self._info_buffer = self._compute_per_ue_rewards(prev_obs, self._global_obs, next_info)
        self._current_ue = 0
        self._flushing = bool(terminated or truncated)
        return self._global_obs[self._current_ue].copy(), reward, False, False, info

    def close(self):
        self._env.close()


class DiscretePerUeActionAdapter(gym.Env):
    """
    Wraps a per-UE MultiDiscrete action env into a single Discrete action env.
    For BWP-only envs: action in {0,1}.
    For BWP+MCS envs: action in {0..9}, decoded as bwp*5 + mcs_idx.
    """

    metadata = {"render_modes": []}

    def __init__(self, env: gym.Env):
        super().__init__()
        self._env = env
        self.observation_space = env.observation_space
        nvec = getattr(env.action_space, "nvec", None)
        if nvec is None:
            raise TypeError("DiscretePerUeActionAdapter expects a MultiDiscrete action space")
        nvec = np.asarray(nvec, dtype=np.int64).reshape(-1)
        if nvec.size == 1:
            self._bwp_only = True
            self.action_space = spaces.Discrete(int(nvec[0]))
        elif nvec.size == 2:
            self._bwp_only = False
            self.action_space = spaces.Discrete(int(nvec[0] * nvec[1]))
        else:
            raise ValueError(f"Unsupported per-UE action shape: {nvec}")

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None):
        return self._env.reset(seed=seed, options=options)

    def step(self, action: int):
        code = int(action)
        if self._bwp_only:
            decoded = np.asarray([code], dtype=np.int64)
        else:
            decoded = np.asarray([code // 5, code % 5], dtype=np.int64)
        return self._env.step(decoded)

    def close(self):
        self._env.close()


class MockBwpEnv(BaseBwpEnv):
    """
    Small POMDP-like environment to validate Recurrent PPO training loop quickly.
    """

    def __init__(self, cfg: EnvConfig):
        super().__init__(cfg)
        self.rng = np.random.default_rng(cfg.seed)
        self.max_steps = max(1, int(round(cfg.episode_time_s / cfg.step_time_s)))
        self.step_count = 0

        self.queue_bytes = np.zeros(self.num_ues, dtype=np.float32)
        self.sinr_db = np.zeros(self.num_ues, dtype=np.float32)
        self.cqi = np.zeros(self.num_ues, dtype=np.float32)
        self.mcs = np.full(self.num_ues, 10, dtype=np.int32)
        self.last_requested_mcs_offset = np.zeros(self.num_ues, dtype=np.float32)
        self.bwp_mode = np.zeros(self.num_ues, dtype=np.int32)  # 0=narrow, 1=wide
        self.cooldown_ms = np.zeros(self.num_ues, dtype=np.float32)
        self.aoi_ms = np.zeros(self.num_ues, dtype=np.float32)
        self.prev_aoi_ms = np.zeros(self.num_ues, dtype=np.float32)
        self.burst_on = np.zeros(self.num_ues, dtype=np.bool_)
        self.prb_utility = np.zeros(self.num_ues, dtype=np.float32)
        self.throughput_mbps = np.zeros(self.num_ues, dtype=np.float32)
        self.last_arrival_bytes = np.zeros(self.num_ues, dtype=np.float32)
        self.last_served_bytes = np.zeros(self.num_ues, dtype=np.float32)
        self.last_prev_queue_bytes = np.zeros(self.num_ues, dtype=np.float32)
        self.time_since_last_switch_ms = np.zeros(self.num_ues, dtype=np.float32)
        self.recent_bler = np.zeros(self.num_ues, dtype=np.float32)
        self.hol_age_ms = np.zeros(self.num_ues, dtype=np.float32)
        self.last_switch_action = np.zeros(self.num_ues, dtype=np.int32)
        self.pending_queue_chunks: list[list[list[float]]] = [[] for _ in range(self.num_ues)]

    def _sinr_to_cqi(self, sinr_db: np.ndarray) -> np.ndarray:
        # Approximate mapping for scaffolding only.
        cqi = np.clip(np.floor((sinr_db + 6.0) / 2.0), 0, 15)
        return cqi.astype(np.float32)

    def _build_obs(self) -> np.ndarray:
        if self.cfg.drqn_profile:
            queue_norm = np.clip(self.queue_bytes / max(1.0, self.cfg.queue_max_bytes), 0.0, 1.0)
            channel_norm = np.clip(self.cqi / 15.0, 0.0, 1.0)
            delay_norm = np.clip(self.aoi_ms / max(1.0, self.cfg.dqn_delay_target_ms), 0.0, 1.0)
            tx_norm = np.clip(self.last_served_bytes / max(1.0, self.cfg.queue_max_bytes), 0.0, 1.0)
            in_norm = np.clip(self.last_arrival_bytes / max(1.0, self.cfg.queue_max_bytes), 0.0, 1.0)
            bwp0 = (self.bwp_mode == 0).astype(np.float32)
            bwp1 = (self.bwp_mode == 1).astype(np.float32)
            bler_norm = np.zeros(self.num_ues, dtype=np.float32)
            drop_rate = np.zeros(self.num_ues, dtype=np.float32)
            active_mask = self.last_arrival_bytes > 0.0
            drop_rate[active_mask] = np.clip(
                (self.last_arrival_bytes[active_mask] - self.last_served_bytes[active_mask])
                / np.maximum(self.last_arrival_bytes[active_mask], 1e-6),
                0.0,
                1.0,
            ).astype(np.float32)
            bwp_active0 = np.full(self.num_ues, np.mean(bwp0, dtype=np.float32), dtype=np.float32)
            bwp_active1 = np.full(self.num_ues, np.mean(bwp1, dtype=np.float32), dtype=np.float32)
            bwp_bler0 = np.zeros(self.num_ues, dtype=np.float32)
            bwp_bler1 = np.zeros(self.num_ues, dtype=np.float32)
            stacked = np.stack(
                [
                    bler_norm,
                    drop_rate,
                    queue_norm.astype(np.float32),
                    channel_norm.astype(np.float32),
                    delay_norm.astype(np.float32),
                    tx_norm.astype(np.float32),
                    in_norm.astype(np.float32),
                    bwp0,
                    bwp1,
                    bwp_active0,
                    bwp_bler0,
                    bwp_active1,
                    bwp_bler1,
                ],
                axis=1,
            )
            return stacked.reshape(-1).astype(np.float32)
        total_prb = np.where(self.bwp_mode == 1, 100.0, 40.0).astype(np.float32)
        step_se = (self.last_served_bytes * 8.0) / np.maximum(total_prb, 1e-6)
        bwp_avg_se = np.zeros(2, dtype=np.float32)
        for bwp in (0, 1):
            mask = self.bwp_mode == bwp
            if np.any(mask):
                bwp_avg_se[bwp] = float(np.mean(step_se[mask]))
        rel_se = np.zeros(self.num_ues, dtype=np.float32)
        for ue in range(self.num_ues):
            rel_se[ue] = float(step_se[ue] / max(1e-6, bwp_avg_se[int(self.bwp_mode[ue])]))
        log_rel_se = np.log1p(np.maximum(rel_se, 0.0))
        current_bwp = self.bwp_mode.astype(np.float32)
        signed_log_mcs_offset = np.sign(self.last_requested_mcs_offset) * np.log1p(
            np.abs(self.last_requested_mcs_offset).astype(np.float32)
        )
        log_cqi = np.log1p(np.maximum(self.cqi.astype(np.float32), 0.0))
        log_queue_backlog = np.log1p(np.maximum(self.queue_bytes.astype(np.float32), 0.0))
        queue_delta = self.queue_bytes - self.last_prev_queue_bytes
        signed_log_queue_delta = np.sign(queue_delta) * np.log1p(
            np.abs(queue_delta).astype(np.float32)
        )
        log_recent_goodput = np.log1p(np.maximum(self.throughput_mbps.astype(np.float32), 0.0))
        log_drop_rate = np.log1p(
            np.clip(
                np.maximum(self.last_arrival_bytes - self.last_served_bytes, 0.0)
                / np.maximum(self.last_arrival_bytes, 1e-6),
                0.0,
                1.0,
            ).astype(np.float32)
        )
        log_time_since_last_switch = np.log1p(np.maximum(self.time_since_last_switch_ms.astype(np.float32), 0.0))
        stacked = np.stack(
            [
                current_bwp,
                signed_log_mcs_offset,
                log_cqi,
                log_queue_backlog,
                signed_log_queue_delta,
                log_recent_goodput,
                log_rel_se,
                log_drop_rate,
                log_time_since_last_switch,
            ],
            axis=1,
        )
        return stacked.reshape(-1).astype(np.float32)

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.step_count = 0
        self.queue_bytes.fill(0.0)
        self.sinr_db[:] = self.rng.normal(loc=8.0, scale=2.0, size=self.num_ues).astype(np.float32)
        self.cqi = self._sinr_to_cqi(self.sinr_db)
        self.mcs[:] = 10
        self.last_requested_mcs_offset.fill(0.0)
        self.bwp_mode[:] = 0
        self.cooldown_ms.fill(0.0)
        self.aoi_ms.fill(10.0)
        self.prev_aoi_ms.fill(10.0)
        self.burst_on[:] = False
        self.prb_utility.fill(0.0)
        self.throughput_mbps.fill(0.0)
        self.last_arrival_bytes.fill(0.0)
        self.last_served_bytes.fill(0.0)
        self.last_prev_queue_bytes.fill(0.0)
        self.time_since_last_switch_ms.fill(0.0)
        self.recent_bler.fill(0.0)
        self.hol_age_ms.fill(0.0)
        self.last_switch_action.fill(0)
        self.pending_queue_chunks = [[] for _ in range(self.num_ues)]
        return self._build_obs(), {}

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.int64).reshape(-1)
        expected = self.num_ues if self.cfg.bwp_only_actions else 2 * self.num_ues
        if action.size != expected:
            raise ValueError(f"Expected action size {expected}, got {action.size}")

        switch_count = 0
        self.last_switch_action.fill(0)

        # Apply actions.
        for ue in range(self.num_ues):
            if self.cfg.bwp_only_actions:
                bwp_target = int(action[ue])
                delta_idx = 2
            else:
                bwp_target = int(action[2 * ue + 0])
                delta_idx = int(action[2 * ue + 1])
            mcs_delta = _decode_mcs_delta(delta_idx)

            self.mcs[ue] = int(np.clip(self.mcs[ue] + mcs_delta, MCS_MIN, MCS_MAX))
            self.last_requested_mcs_offset[ue] = float(mcs_delta)

            if bwp_target != int(self.bwp_mode[ue]) and self.cooldown_ms[ue] <= 0.0:
                self.bwp_mode[ue] = bwp_target
                self.cooldown_ms[ue] = self.cfg.switch_delay_ms
                self.time_since_last_switch_ms[ue] = 0.0
                self.last_switch_action[ue] = 1
                switch_count += 1

        # Channel evolution (simple correlated fading + shadowing perturbation).
        noise = self.rng.normal(loc=0.0, scale=1.2, size=self.num_ues).astype(np.float32)
        self.sinr_db = (0.90 * self.sinr_db + 0.10 * 8.0 + noise).astype(np.float32)
        self.cqi = self._sinr_to_cqi(self.sinr_db)

        # Bursty arrivals.
        arrivals = np.zeros(self.num_ues, dtype=np.float32)
        prev_queue = self.queue_bytes.copy()
        current_time_s = float(self.step_count) * float(self.cfg.step_time_s)
        for ue in range(self.num_ues):
            if self.rng.random() < 0.05:
                self.burst_on[ue] = not self.burst_on[ue]
            if self.burst_on[ue]:
                arrival = self.rng.uniform(8000.0, 22000.0)  # bytes/step
            else:
                arrival = self.rng.uniform(200.0, 1800.0)
            arrivals[ue] = float(arrival)
            self.queue_bytes[ue] += float(arrival)
            if arrival > 0.0:
                self.pending_queue_chunks[ue].append([current_time_s, float(arrival)])

        # Service model.
        # Wide BWP has more PRBs; narrow BWP has fewer PRBs.
        bwp_factor = np.where(self.bwp_mode == 1, 1.0, 0.55).astype(np.float32)
        mcs_eff = np.clip((self.mcs + 1) / 28.0, 0.05, 1.0).astype(np.float32)
        sinr_eff = np.clip((self.sinr_db + 5.0) / 20.0, 0.1, 1.2).astype(np.float32)

        # Switch delay reduces effective service during the same step.
        delay_ratio = self.cfg.switch_delay_ms / max(self.cfg.step_time_s * 1000.0, 1e-6)
        cooldown_hit = (self.cooldown_ms > 0.0).astype(np.float32)
        switch_eff = np.clip(1.0 - delay_ratio * cooldown_hit, 0.0, 1.0).astype(np.float32)

        base_capacity = 18000.0  # bytes/step at ideal wide + high MCS/SINR
        served_capacity = base_capacity * bwp_factor * mcs_eff * sinr_eff * switch_eff
        served = np.minimum(self.queue_bytes, served_capacity)
        self.queue_bytes -= served
        self.last_arrival_bytes = arrivals
        self.last_served_bytes = served.astype(np.float32)
        self.last_prev_queue_bytes = prev_queue.astype(np.float32)

        bytes_per_prb = np.maximum(1.0, 5.0 + 2.0 * self.mcs.astype(np.float32))
        demanded_prb = self.queue_bytes / bytes_per_prb
        total_prb = np.where(self.bwp_mode == 1, 100.0, 40.0).astype(np.float32)
        self.prb_utility = (demanded_prb / total_prb).astype(np.float32)

        # AoI dynamics.
        # If any payload served in this step, AoI decreases; otherwise increases by step.
        step_ms = self.cfg.step_time_s * 1000.0
        self.prev_aoi_ms = self.aoi_ms.copy()
        delivered = served > 0.0
        self.aoi_ms = np.where(delivered, np.maximum(self.aoi_ms * 0.7, 1.0), self.aoi_ms + step_ms)
        self.throughput_mbps = (served * 8.0 / max(self.cfg.step_time_s, 1e-6) / 1e6).astype(np.float32)
        self.recent_bler = np.clip((1.0 - sinr_eff) * (0.25 + 0.75 * mcs_eff), 0.0, 1.0).astype(np.float32)

        step_end_time_s = current_time_s + float(self.cfg.step_time_s)
        for ue in range(self.num_ues):
            remaining = float(served[ue])
            chunks = self.pending_queue_chunks[ue]
            while remaining > 1e-6 and chunks:
                if chunks[0][1] <= remaining + 1e-6:
                    remaining -= chunks[0][1]
                    chunks.pop(0)
                else:
                    chunks[0][1] -= remaining
                    remaining = 0.0
            self.hol_age_ms[ue] = 0.0 if not chunks else max(0.0, (step_end_time_s - chunks[0][0]) * 1000.0)

        # Cooldown timer countdown.
        self.cooldown_ms = np.maximum(0.0, self.cooldown_ms - step_ms)
        self.time_since_last_switch_ms += step_ms

        self.step_count += 1
        terminated = self.step_count >= self.max_steps
        truncated = False

        mean_aoi_ms = float(np.mean(self.aoi_ms))
        mean_delta_aoi_ms = float(np.mean(self.aoi_ms - self.prev_aoi_ms))
        mean_throughput_mbps = float(np.mean(self.throughput_mbps))
        if self.cfg.drqn_profile:
            reward = self._shape_reward(
                mean_aoi_ms,
                float(np.mean(self.prev_aoi_ms)),
                mean_throughput_mbps,
                switch_count,
            )
        else:
            queue_ratio = self.queue_bytes / np.maximum(self.last_prev_queue_bytes, 1e-6)
            reward = (
                -float(np.mean(np.log1p(np.maximum(self.aoi_ms, 0.0))))
                + float(np.mean(np.log1p(np.maximum(self.throughput_mbps, 0.0))))
                - float(self.cfg.reward_lambda_queue * np.mean(np.log1p(queue_ratio)))
                - float(self.cfg.reward_lambda_switch * switch_count / max(1, self.num_ues))
            )
        info = {
            "mean_aoi_ms": mean_aoi_ms,
            "mean_delta_aoi_ms": mean_delta_aoi_ms,
            "mean_thr_mbps": mean_throughput_mbps,
            "switch_count": int(switch_count),
            "mean_prb_utility": float(np.mean(self.prb_utility)),
            "backend": "mock",
        }
        step_se = (served * 8.0) / np.maximum(total_prb, 1e-6)
        bwp0_mask = self.bwp_mode == 0
        bwp1_mask = self.bwp_mode == 1
        bwp0_avg_mcs = float(np.mean(self.mcs[bwp0_mask])) if np.any(bwp0_mask) else 0.0
        bwp1_avg_mcs = float(np.mean(self.mcs[bwp1_mask])) if np.any(bwp1_mask) else 0.0
        for ue in range(self.num_ues):
            info[f"ue{ue}_se"] = float(step_se[ue])
            info[f"ue{ue}_assigned_prb"] = float(total_prb[ue])
            info[f"ue{ue}_arrived_bytes"] = float(self.last_arrival_bytes[ue])
            info[f"ue{ue}_delivered_bytes"] = float(self.last_served_bytes[ue])
            info[f"ue{ue}_prev_queue_bytes"] = float(self.last_prev_queue_bytes[ue])
            info[f"ue{ue}_queue_bytes"] = float(self.queue_bytes[ue])
            info[f"ue{ue}_actual_switch"] = float(self.last_switch_action[ue] > 0)
            info[f"ue{ue}_thr_mbps"] = float(self.throughput_mbps[ue])
            info[f"ue{ue}_goodput_mbps"] = float(self.throughput_mbps[ue])
            info[f"ue{ue}_aoi_ms"] = float(self.aoi_ms[ue])
            info[f"ue{ue}_prev_aoi_ms"] = float(self.prev_aoi_ms[ue])
            info[f"ue{ue}_bler"] = float(self.recent_bler[ue])
            info[f"ue{ue}_hol_age_ms"] = float(self.hol_age_ms[ue])
        info["bwp0_avg_mcs"] = bwp0_avg_mcs
        info["bwp1_avg_mcs"] = bwp1_avg_mcs
        info["bwp0_total_prb"] = 40.0
        info["bwp1_total_prb"] = 100.0
        return self._build_obs(), reward, terminated, truncated, info


class Ns3BwpEnv(BaseBwpEnv):
    """
    ns3gym wrapper.
    This wrapper assumes ns-3 side action space accepts one discrete action per UE.
    """

    def __init__(
        self,
        cfg: EnvConfig,
        port: int = 5555,
        start_sim: bool = False,
        sim_seed: int = 1,
        sim_args: dict[str, Any] | None = None,
        sim_script: str = "aoi-prb-urban-onoff",
        ns3_path: str = "./ns3",
        debug: bool = False,
    ):
        super().__init__(cfg)
        self._port = int(port)
        self._start_sim = bool(start_sim)
        self._sim_seed = int(sim_seed)
        self._sim_args = dict(sim_args or {})
        self._sim_script = str(sim_script)
        self._ns3_path = str(ns3_path)
        self._debug = bool(debug)
        self._ns3_proc: subprocess.Popen | None = None

        # Backward compatibility for older ns3gym that still references np.float.
        if not hasattr(np, "float"):
            np.float = float  # type: ignore[attr-defined]
        from ns3gym import ns3env  # Imported lazily for users without ns3gym.

        # Use explicit script launching to avoid ns3gym cwd-based script detection.
        if self._start_sim:
            self._launch_ns3(self._sim_seed)
        self._env = ns3env.Ns3Env(
            port=self._port,
            stepTime=cfg.step_time_s,
            startSim=False,
            simSeed=self._sim_seed,
            simArgs={},
            debug=self._debug,
        )
        self._log_connected("initial")

    def _log_connected(self, phase: str):
        print(
            f"[Ns3BwpEnv] connected ({phase}) port={self._port} num_ues={self.num_ues} seed={self._sim_seed}",
            flush=True,
        )

    def _build_run_arg(self, sim_seed: int) -> str:
        cli_args = [
            self._sim_script,
            f"--openGymPort={self._port}",
        ]
        for k, v in self._sim_args.items():
            key = str(k)
            if not key.startswith("--"):
                key = f"--{key}"
            cli_args.append(f"{key}={v}")
        return " ".join(cli_args)

    def _launch_ns3(self, sim_seed: int):
        repo_root = Path(__file__).resolve().parents[2]
        run_arg = self._build_run_arg(sim_seed)
        cmd = [self._ns3_path, "run", run_arg]
        stdio = None if self._debug else subprocess.DEVNULL
        self._ns3_proc = subprocess.Popen(
            cmd,
            cwd=repo_root,
            stdout=stdio,
            stderr=stdio,
        )

    def _close_ns3_proc(self):
        if self._ns3_proc is None:
            return
        if self._ns3_proc.poll() is None:
            self._ns3_proc.terminate()
            try:
                self._ns3_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._ns3_proc.kill()
                self._ns3_proc.wait(timeout=2.0)
        self._ns3_proc = None

    def _maybe_resize_from_obs(self, raw_obs: Any) -> None:
        arr = np.asarray(raw_obs, dtype=np.float32).reshape(-1)
        if arr.size == 0 or arr.size % self.num_ues != 0:
            return
        if arr.size == self.obs_size:
            return
        self.features_per_ue = int(arr.size // self.num_ues)
        self.obs_size = int(arr.size)
        obs_low = -max(float(self.cfg.queue_max_bytes) * 2.0, 1.0e6)
        obs_high = max(float(self.cfg.queue_max_bytes) * 2.0, 1.0e6)
        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            shape=(self.obs_size,),
            dtype=np.float32,
        )

    def _obs_to_fixed(self, raw_obs: Any) -> np.ndarray:
        arr = np.asarray(raw_obs, dtype=np.float32).reshape(-1)
        if arr.size >= self.obs_size:
            out = arr[: self.obs_size]
            out = np.nan_to_num(out, nan=0.0, posinf=100.0, neginf=-100.0)
            return np.clip(out, -100.0, 100.0)
        out = np.zeros(self.obs_size, dtype=np.float32)
        out[: arr.size] = arr
        out = np.nan_to_num(out, nan=0.0, posinf=100.0, neginf=-100.0)
        return np.clip(out, -100.0, 100.0)

    def _parse_info(self, info: Any) -> dict[str, Any]:
        if isinstance(info, dict):
            return dict(info)
        if isinstance(info, str):
            # Best-effort parse for strings like "k1=v1|k2=v2" or opaque messages.
            parsed: dict[str, Any] = {}
            chunks = info.replace(",", "|").replace(";", "|").split("|")
            for chunk in chunks:
                chunk = chunk.strip()
                if not chunk:
                    continue
                if "=" in chunk:
                    k, v = chunk.split("=", 1)
                    k = k.strip()
                    v = v.strip()
                    try:
                        parsed[k] = float(v)
                    except ValueError:
                        parsed[k] = v
                else:
                    parsed["raw_info"] = info
            if not parsed:
                parsed["raw_info"] = info
            return parsed
        return {"raw_info": info}

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None):
        if seed is not None:
            np.random.seed(seed)
        # ns3gym marks envDirty after first step. Recreate bridge and restart ns-3 per episode.
        if self._start_sim and getattr(self._env, "envDirty", False):
            self._env.close()
            self._close_ns3_proc()
            self._sim_seed += 1
            self._launch_ns3(self._sim_seed)
            from ns3gym import ns3env

            self._env = ns3env.Ns3Env(
                port=self._port,
                stepTime=self.cfg.step_time_s,
                startSim=False,
                simSeed=self._sim_seed,
                simArgs={},
                debug=self._debug,
            )
            self._log_connected("reset")
        raw_obs = self._env.reset()
        self._maybe_resize_from_obs(raw_obs)
        return self._obs_to_fixed(raw_obs), {}

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.int64).reshape(-1)
        expected = self.num_ues if self.cfg.bwp_only_actions else 2 * self.num_ues
        if action.size != expected:
            raise ValueError(f"Expected action size {expected}, got {action.size}")

        encoded = []
        for ue in range(self.num_ues):
            if self.cfg.bwp_only_actions:
                bwp_target = int(action[ue])
                delta_idx = 2
            else:
                bwp_target = int(action[2 * ue + 0])
                delta_idx = int(action[2 * ue + 1])
            encoded.append(_encode_per_ue_action(bwp_target, delta_idx))

        payload = np.asarray(encoded, dtype=np.uint32)
        if self.num_ues == 1:
            payload = int(payload.item())

        raw_obs, base_reward, done, raw_info = self._env.step(payload)
        info = self._parse_info(raw_info)
        reward = float(base_reward)
        terminated = bool(done)
        truncated = False
        info["backend"] = "ns3"
        return self._obs_to_fixed(raw_obs), reward, terminated, truncated, info

    def close(self):
        self._env.close()
        self._close_ns3_proc()
