#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch
from tensordict import TensorDict
from torch import nn
from torchrl.data import ListStorage, TensorDictReplayBuffer


@dataclass
class DqnConfig:
    obs_dim: int
    action_dim: int
    hidden_dim: int = 128
    num_quantiles: int = 51
    gamma: float = 0.98
    lr: float = 1e-3
    batch_size: int = 32
    replay_capacity: int = 2000
    warmup_steps: int = 64
    target_sync: int = 100
    eps_start: float = 1.0
    eps_end: float = 0.05
    eps_decay: float = 0.999
    train_updates_per_step: int = 1
    huber_kappa: float = 1.0
    action_selection_quantile: float = 0.1
    conditional_risk_selection: bool = False
    risk_low_cqi_threshold: float = 5.0
    risk_mid_cqi_threshold: float = 10.0
    risk_quantile_low: float = 0.2
    risk_quantile_mid: float = 0.5
    risk_quantile_high: float = 0.8
    drqn_profile: bool = False


class DqnNet(nn.Module):
    def __init__(self, obs_dim: int, hidden_dim: int, action_dim: int, num_quantiles: int):
        super().__init__()
        self.obs_dim = obs_dim
        self.hidden_dim = hidden_dim
        self.action_dim = action_dim
        self.num_quantiles = num_quantiles
        self.in_proj = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.head = nn.Linear(hidden_dim, action_dim * num_quantiles)
        taus = (torch.arange(num_quantiles, dtype=torch.float32) + 0.5) / float(num_quantiles)
        self.register_buffer("taus", taus)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        x = self.in_proj(obs)
        quantiles = self.head(x).view(-1, self.action_dim, self.num_quantiles)
        return quantiles

    def q_values(self, quantiles: torch.Tensor) -> torch.Tensor:
        return quantiles.mean(dim=-1)

    def lower_quantile_values(
        self,
        quantiles: torch.Tensor,
        selection_quantile: float | torch.Tensor,
    ) -> torch.Tensor:
        max_idx = max(1, self.num_quantiles - 1)
        if isinstance(selection_quantile, torch.Tensor):
            q = torch.clamp(selection_quantile.to(quantiles.device, dtype=torch.float32), 0.0, 1.0)
            idx = torch.floor(q * max_idx).to(dtype=torch.long)
            idx = torch.clamp(idx, 0, self.num_quantiles - 1)
            if quantiles.dim() == 3:
                gather_idx = idx.view(-1, 1, 1).expand(-1, self.action_dim, 1)
                return torch.gather(quantiles, dim=-1, index=gather_idx).squeeze(-1)
            raise ValueError("Tensor selection_quantile expects quantiles with shape [batch, action_dim, num_quantiles].")
        q = float(np.clip(selection_quantile, 0.0, 1.0))
        idx = int(np.floor(q * max_idx))
        return quantiles[..., idx]


def _selection_quantiles_from_obs(
    obs: np.ndarray | torch.Tensor,
    cfg: DqnConfig,
    device: torch.device | None = None,
) -> float | torch.Tensor:
    if not cfg.conditional_risk_selection or cfg.drqn_profile:
        return cfg.action_selection_quantile

    if isinstance(obs, np.ndarray):
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device)
    else:
        obs_t = obs.to(device=device, dtype=torch.float32) if device is not None else obs.to(dtype=torch.float32)

    if obs_t.dim() == 1:
        obs_t = obs_t.unsqueeze(0)

    # Non-drqn profile state layout:
    # [bwp_mode, mcs_offset, sinr_norm, arrived_bytes_norm, queue_bytes_norm, aoi_norm,
    #  in_flight_bytes_norm, time_since_last_delivery_norm]
    signal_metric = torch.clamp((obs_t[:, 2] + 1.0) * 0.5 * 15.0, 0.0, 15.0)

    tau = torch.full_like(signal_metric, float(cfg.risk_quantile_mid))
    low_mask = signal_metric < float(cfg.risk_low_cqi_threshold)
    tau = torch.where(low_mask, torch.full_like(tau, float(cfg.risk_quantile_low)), tau)
    high_mask = signal_metric > float(cfg.risk_mid_cqi_threshold)
    tau = torch.where(high_mask, torch.full_like(tau, float(cfg.risk_quantile_high)), tau)
    return tau


def make_replay_buffer(capacity: int) -> TensorDictReplayBuffer:
    return TensorDictReplayBuffer(storage=ListStorage(capacity))


def epsilon_by_step(cfg: DqnConfig, step: int) -> float:
    eps = cfg.eps_start * (cfg.eps_decay ** max(0, step))
    return max(cfg.eps_end, float(eps))


def select_action(
    net: DqnNet,
    obs: np.ndarray,
    epsilon: float,
    device: torch.device,
    cfg: DqnConfig,
) -> int:
    obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
    with torch.no_grad():
        quantiles = net(obs_t)
        q = net.q_values(quantiles)
    if np.random.random() < epsilon:
        action = int(np.random.randint(0, net.action_dim))
    else:
        action = int(torch.argmax(q, dim=-1).item())
    return action


def _quantile_huber_loss(pred: torch.Tensor, target: torch.Tensor, taus: torch.Tensor, kappa: float) -> torch.Tensor:
    td_error = target.unsqueeze(1) - pred.unsqueeze(2)
    abs_err = td_error.abs()
    huber = torch.where(
        abs_err <= kappa,
        0.5 * td_error.pow(2),
        kappa * (abs_err - 0.5 * kappa),
    )
    tau = taus.view(1, -1, 1)
    weight = (tau - (td_error.detach() < 0).float()).abs()
    return (weight * huber / kappa).mean(dim=(1, 2))


def train_batch(
    online_net: DqnNet,
    target_net: DqnNet,
    optimizer: torch.optim.Optimizer,
    batch: TensorDict,
    cfg: DqnConfig,
    device: torch.device,
) -> dict[str, float | list[float]]:
    obs = batch["obs"].to(device)
    action = batch["action"].to(device)
    reward = batch["reward"].to(device)
    next_obs = batch["next_obs"].to(device)
    done = batch["done"].to(device).float()

    batch_size = obs.shape[0]
    
    quantiles = online_net(obs)
    batch_idx = torch.arange(batch_size, device=device)
    pred_quantiles = quantiles[batch_idx, action, :]
    
    with torch.no_grad():
        next_quantiles_online = online_net(next_obs)
        next_q_online = online_net.q_values(next_quantiles_online)
        next_action = torch.argmax(next_q_online, dim=-1)
        
        next_quantiles_target = target_net(next_obs)
        target_quantiles = next_quantiles_target[batch_idx, next_action, :]
        target = reward.unsqueeze(1) + cfg.gamma * (1.0 - done).unsqueeze(1) * target_quantiles
        
        td_abs_quantile = (target.unsqueeze(1) - pred_quantiles.unsqueeze(2)).abs().mean(dim=2)

    loss = _quantile_huber_loss(pred_quantiles, target, online_net.taus.to(device), cfg.huber_kappa).mean()
    
    optimizer.zero_grad(set_to_none=True)
    loss.backward()
    torch.nn.utils.clip_grad_norm_(online_net.parameters(), 10.0)
    optimizer.step()
    
    quantile_td_abs = td_abs_quantile.mean(dim=0)
    
    return {
        "loss": float(loss.item()),
        "quantile_td_abs": [float(x) for x in quantile_td_abs.detach().cpu().tolist()],
    }

