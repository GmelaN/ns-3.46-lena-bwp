#!/usr/bin/env python3
"""
Environment definitions for Recurrent PPO based BWP/MCS control.

Two backends are provided:
1) MockBwpEnv: lightweight POMDP-style sandbox for training-loop validation.
2) Ns3BwpEnv: wrapper over ns3gym for live integration.
"""

from __future__ import annotations

from dataclasses import dataclass
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
    reward_lambda_switch: float = 0.05
    reward_lambda_queue: float = 0.20
    reward_lambda_delay: float = 0.10
    seed: int = 1


def _encode_per_ue_action(bwp_cmd: int, mcs_delta_idx: int) -> int:
    """
    Encodes per-UE action into one discrete code:
    bwp_cmd in {0: hold, 1: switch}
    mcs_delta_idx in {0,1,2} -> delta in {-1,0,+1}
    """
    return int(bwp_cmd) * 3 + int(mcs_delta_idx)


def _decode_mcs_delta(mcs_delta_idx: int) -> int:
    return (-1, 0, +1)[int(mcs_delta_idx)]


class BaseBwpEnv(gym.Env):
    """
    Shared action/observation conventions.
    Observation is a flat vector containing per-UE features:
    [queue_norm, cqi_norm, sinr_norm, mcs_norm, bwp_mode, prb_utility, cooldown_norm] * num_ues
    """

    metadata = {"render_modes": []}

    def __init__(self, cfg: EnvConfig):
        super().__init__()
        self.cfg = cfg
        self.num_ues = cfg.num_ues
        self.features_per_ue = 7
        self.obs_size = self.features_per_ue * self.num_ues

        # Per UE action: [bwp_cmd(0/1), mcs_delta_idx(0/1/2)].
        action_nvec = np.array(([2, 3] * self.num_ues), dtype=np.int64)
        self.action_space = spaces.MultiDiscrete(action_nvec)
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(self.obs_size,),
            dtype=np.float32,
        )

    def _shape_reward(
        self,
        base_reward: float,
        mean_aoi_ms: float | None,
        switch_count: int,
        queue_overflow_ratio: float,
    ) -> float:
        """
        Reward design:
        - primary: minimize mean AoI -> reward = -mean_aoi_ms
        - penalties: switch count, queue overflow pressure, switch delay burden
        """
        if mean_aoi_ms is None:
            reward = float(base_reward)
        else:
            reward = -float(mean_aoi_ms)

        delay_ratio = self.cfg.switch_delay_ms / max(self.cfg.step_time_s * 1000.0, 1e-6)
        reward -= self.cfg.reward_lambda_switch * float(switch_count)
        reward -= self.cfg.reward_lambda_queue * float(queue_overflow_ratio)
        reward -= self.cfg.reward_lambda_delay * delay_ratio * float(switch_count)
        return float(reward)


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
        self.bwp_mode = np.zeros(self.num_ues, dtype=np.int32)  # 0=narrow, 1=wide
        self.cooldown_ms = np.zeros(self.num_ues, dtype=np.float32)
        self.aoi_ms = np.zeros(self.num_ues, dtype=np.float32)
        self.burst_on = np.zeros(self.num_ues, dtype=np.bool_)
        self.prb_utility = np.zeros(self.num_ues, dtype=np.float32)

    def _sinr_to_cqi(self, sinr_db: np.ndarray) -> np.ndarray:
        # Approximate mapping for scaffolding only.
        cqi = np.clip(np.floor((sinr_db + 6.0) / 2.0), 0, 15)
        return cqi.astype(np.float32)

    def _build_obs(self) -> np.ndarray:
        queue_norm = np.clip(self.queue_bytes / self.cfg.queue_max_bytes, 0.0, 1.0)
        cqi_norm = np.clip(self.cqi / 15.0, 0.0, 1.0)
        sinr_norm = np.clip((self.sinr_db + 10.0) / 40.0, 0.0, 1.0)
        mcs_norm = np.clip((self.mcs - MCS_MIN) / float(MCS_MAX - MCS_MIN), 0.0, 1.0)
        bwp_norm = self.bwp_mode.astype(np.float32)
        cooldown_norm = np.clip(
            self.cooldown_ms / max(self.cfg.switch_delay_ms, 1e-6),
            0.0,
            1.0,
        )

        stacked = np.stack(
            [queue_norm, cqi_norm, sinr_norm, mcs_norm, bwp_norm, self.prb_utility, cooldown_norm],
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
        self.bwp_mode[:] = 0
        self.cooldown_ms.fill(0.0)
        self.aoi_ms.fill(10.0)
        self.burst_on[:] = False
        self.prb_utility.fill(0.0)
        return self._build_obs(), {}

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.int64).reshape(-1)
        expected = 2 * self.num_ues
        if action.size != expected:
            raise ValueError(f"Expected action size {expected}, got {action.size}")

        switch_count = 0

        # Apply actions.
        for ue in range(self.num_ues):
            bwp_cmd = int(action[2 * ue + 0])
            delta_idx = int(action[2 * ue + 1])
            mcs_delta = _decode_mcs_delta(delta_idx)

            self.mcs[ue] = int(np.clip(self.mcs[ue] + mcs_delta, MCS_MIN, MCS_MAX))

            if bwp_cmd == 1 and self.cooldown_ms[ue] <= 0.0:
                self.bwp_mode[ue] = 1 - self.bwp_mode[ue]
                self.cooldown_ms[ue] = self.cfg.switch_delay_ms
                switch_count += 1

        # Channel evolution (simple correlated fading + shadowing perturbation).
        noise = self.rng.normal(loc=0.0, scale=1.2, size=self.num_ues).astype(np.float32)
        self.sinr_db = (0.90 * self.sinr_db + 0.10 * 8.0 + noise).astype(np.float32)
        self.cqi = self._sinr_to_cqi(self.sinr_db)

        # Bursty arrivals.
        for ue in range(self.num_ues):
            if self.rng.random() < 0.05:
                self.burst_on[ue] = not self.burst_on[ue]
            if self.burst_on[ue]:
                arrival = self.rng.uniform(8000.0, 22000.0)  # bytes/step
            else:
                arrival = self.rng.uniform(200.0, 1800.0)
            self.queue_bytes[ue] += float(arrival)

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

        # Per-UE PRB utility proxy.
        # utility = allocated_prb / total_prb_of_current_bwp
        # In this simplified backend we proxy it by service usage ratio.
        with np.errstate(divide="ignore", invalid="ignore"):
            util = np.where(served_capacity > 0.0, served / served_capacity, 0.0)
        self.prb_utility = np.clip(util.astype(np.float32), 0.0, 1.0)

        # AoI dynamics.
        # If any payload served in this step, AoI decreases; otherwise increases by step.
        step_ms = self.cfg.step_time_s * 1000.0
        delivered = served > 0.0
        self.aoi_ms = np.where(delivered, np.maximum(self.aoi_ms * 0.7, 1.0), self.aoi_ms + step_ms)

        # Cooldown timer countdown.
        self.cooldown_ms = np.maximum(0.0, self.cooldown_ms - step_ms)

        self.step_count += 1
        terminated = self.step_count >= self.max_steps
        truncated = False

        mean_aoi_ms = float(np.mean(self.aoi_ms))
        queue_overflow_ratio = float(np.mean(self.queue_bytes > self.cfg.queue_max_bytes))
        reward = self._shape_reward(
            base_reward=0.0,
            mean_aoi_ms=mean_aoi_ms,
            switch_count=switch_count,
            queue_overflow_ratio=queue_overflow_ratio,
        )

        info = {
            "mean_aoi_ms": mean_aoi_ms,
            "switch_count": int(switch_count),
            "queue_overflow_ratio": queue_overflow_ratio,
            "mean_prb_utility": float(np.mean(self.prb_utility)),
            "backend": "mock",
        }
        return self._build_obs(), reward, terminated, truncated, info


class Ns3BwpEnv(BaseBwpEnv):
    """
    ns3gym wrapper.
    This wrapper assumes ns-3 side action space accepts one discrete action per UE:
      code = bwp_cmd * 3 + mcs_delta_idx
    where bwp_cmd in {0,1}, mcs_delta_idx in {0,1,2}.
    """

    def __init__(
        self,
        cfg: EnvConfig,
        port: int = 5555,
        start_sim: bool = False,
        sim_seed: int = 1,
        sim_args: dict[str, Any] | None = None,
        debug: bool = False,
    ):
        super().__init__(cfg)
        # Backward compatibility for older ns3gym that still references np.float.
        if not hasattr(np, "float"):
            np.float = float  # type: ignore[attr-defined]
        from ns3gym import ns3env  # Imported lazily for users without ns3gym.

        self._env = ns3env.Ns3Env(
            port=port,
            stepTime=cfg.step_time_s,
            startSim=start_sim,
            simSeed=sim_seed,
            simArgs=sim_args or {},
            debug=debug,
        )

    def _obs_to_fixed(self, raw_obs: Any) -> np.ndarray:
        arr = np.asarray(raw_obs, dtype=np.float32).reshape(-1)
        if arr.size >= self.obs_size:
            return arr[: self.obs_size]
        out = np.zeros(self.obs_size, dtype=np.float32)
        out[: arr.size] = arr
        return out

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
        raw_obs = self._env.reset()
        return self._obs_to_fixed(raw_obs), {}

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.int64).reshape(-1)
        expected = 2 * self.num_ues
        if action.size != expected:
            raise ValueError(f"Expected action size {expected}, got {action.size}")

        encoded = []
        switch_count = 0
        for ue in range(self.num_ues):
            bwp_cmd = int(action[2 * ue + 0])
            delta_idx = int(action[2 * ue + 1])
            encoded.append(_encode_per_ue_action(bwp_cmd, delta_idx))
            switch_count += bwp_cmd

        payload = np.asarray(encoded, dtype=np.uint32)
        if self.num_ues == 1:
            payload = int(payload.item())

        raw_obs, base_reward, done, raw_info = self._env.step(payload)
        info = self._parse_info(raw_info)
        mean_aoi_ms = info.get("mean_aoi_ms")
        queue_overflow_ratio = float(info.get("queue_overflow_ratio", 0.0))
        reward = self._shape_reward(
            base_reward=float(base_reward),
            mean_aoi_ms=float(mean_aoi_ms) if mean_aoi_ms is not None else None,
            switch_count=int(switch_count),
            queue_overflow_ratio=queue_overflow_ratio,
        )
        terminated = bool(done)
        truncated = False
        info["backend"] = "ns3"
        return self._obs_to_fixed(raw_obs), reward, terminated, truncated, info

    def close(self):
        self._env.close()
