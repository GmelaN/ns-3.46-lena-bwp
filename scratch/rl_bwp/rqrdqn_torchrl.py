#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch
from tensordict import TensorDict
from torch import nn
from torchrl.data import ListStorage, TensorDictReplayBuffer


@dataclass
class RqrdqnConfig:
    obs_dim: int
    action_dim: int
    hidden_dim: int = 128
    num_quantiles: int = 51
    seq_len: int = 8
    burn_in: int = 4
    gamma: float = 0.98
    lr: float = 1e-3
    batch_size: int = 32
    replay_capacity: int = 2000
    warmup_sequences: int = 64
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


class RqrdqnNet(nn.Module):
    def __init__(self, obs_dim: int, hidden_dim: int, action_dim: int, num_quantiles: int):
        super().__init__()
        self.obs_dim = obs_dim
        self.hidden_dim = hidden_dim
        self.action_dim = action_dim
        self.num_quantiles = num_quantiles
        self.in_proj = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
        )
        self.rnn = nn.GRUCell(hidden_dim, hidden_dim)
        self.head = nn.Linear(hidden_dim, action_dim * num_quantiles)
        taus = (torch.arange(num_quantiles, dtype=torch.float32) + 0.5) / float(num_quantiles)
        self.register_buffer("taus", taus)

    def forward_step(self, obs: torch.Tensor, hidden: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        x = self.in_proj(obs)
        next_hidden = self.rnn(x, hidden)
        quantiles = self.head(next_hidden).view(-1, self.action_dim, self.num_quantiles)
        return quantiles, next_hidden

    def zero_hidden(self, batch_size: int, device: torch.device) -> torch.Tensor:
        return torch.zeros(batch_size, self.hidden_dim, dtype=torch.float32, device=device)

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
    cfg: RqrdqnConfig,
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
    # [current_bwp, signed_log_mcs_offset, log_cqi, log_queue_backlog,
    #  signed_log_queue_delta, log_recent_goodput, log_rel_se, log_drop_rate,
    #  log_time_since_last_switch]
    cqi = torch.expm1(torch.clamp(obs_t[:, 2], min=0.0))

    tau = torch.full_like(cqi, float(cfg.risk_quantile_mid))
    low_mask = cqi < float(cfg.risk_low_cqi_threshold)
    tau = torch.where(low_mask, torch.full_like(tau, float(cfg.risk_quantile_low)), tau)
    high_mask = cqi > float(cfg.risk_mid_cqi_threshold)
    tau = torch.where(high_mask, torch.full_like(tau, float(cfg.risk_quantile_high)), tau)
    return tau


def make_replay_buffer(capacity: int) -> TensorDictReplayBuffer:
    return TensorDictReplayBuffer(storage=ListStorage(capacity))


def epsilon_by_step(cfg: RqrdqnConfig, step: int) -> float:
    eps = cfg.eps_start * (cfg.eps_decay ** max(0, step))
    return max(cfg.eps_end, float(eps))


def select_action(
    net: RqrdqnNet,
    obs: np.ndarray,
    hidden: torch.Tensor,
    epsilon: float,
    device: torch.device,
    cfg: RqrdqnConfig,
) -> tuple[int, torch.Tensor]:
    obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
    with torch.no_grad():
        quantiles, next_hidden = net.forward_step(obs_t, hidden)
        selection_quantile = _selection_quantiles_from_obs(obs_t, cfg, device)
        q = net.lower_quantile_values(quantiles, selection_quantile)
    if np.random.random() < epsilon:
        action = int(np.random.randint(0, net.action_dim))
    else:
        action = int(torch.argmax(q, dim=-1).item())
    return action, next_hidden.detach()


def episode_to_sequences(
    transitions: list[dict],
    seq_len: int,
    stride: int | None = None,
) -> list[TensorDict]:
    if len(transitions) < 2:
        return []
    if stride is None:
        stride = max(1, seq_len // 2)
    out: list[TensorDict] = []
    start = 0
    obs_dim = int(np.asarray(transitions[0]["obs"]).shape[0])
    while start < len(transitions):
        chunk = transitions[start : start + seq_len]
        obs = torch.zeros(seq_len, obs_dim, dtype=torch.float32)
        action = torch.zeros(seq_len, dtype=torch.long)
        reward = torch.zeros(seq_len, dtype=torch.float32)
        next_obs = torch.zeros(seq_len, obs_dim, dtype=torch.float32)
        done = torch.ones(seq_len, dtype=torch.float32)
        mask = torch.zeros(seq_len, dtype=torch.float32)
        for i, tr in enumerate(chunk):
            obs[i] = torch.as_tensor(tr["obs"], dtype=torch.float32)
            action[i] = int(tr["action"])
            reward[i] = float(tr["reward"])
            next_obs[i] = torch.as_tensor(tr["next_obs"], dtype=torch.float32)
            done[i] = 1.0 if tr["done"] else 0.0
            mask[i] = 1.0
        out.append(
            TensorDict(
                {
                    "obs": obs,
                    "action": action,
                    "reward": reward,
                    "next_obs": next_obs,
                    "done": done,
                    "mask": mask,
                },
                batch_size=[seq_len],
            )
        )
        if start + seq_len >= len(transitions):
            break
        start += stride
    return out


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
    online_net: RqrdqnNet,
    target_net: RqrdqnNet,
    optimizer: torch.optim.Optimizer,
    batch: TensorDict,
    cfg: RqrdqnConfig,
    device: torch.device,
) -> dict[str, float | list[float]]:
    obs = batch["obs"].to(device)
    action = batch["action"].to(device)
    reward = batch["reward"].to(device)
    next_obs = batch["next_obs"].to(device)
    done = batch["done"].to(device)
    mask = batch["mask"].to(device)

    batch_size, seq_len, _ = obs.shape
    h_online = online_net.zero_hidden(batch_size, device)
    h_target = target_net.zero_hidden(batch_size, device)
    losses: list[torch.Tensor] = []
    td_abs_quantile_num = torch.zeros(cfg.num_quantiles, device=device)
    td_abs_quantile_den = torch.zeros((), device=device)

    for t in range(seq_len):
        quantiles_t, h_online_next = online_net.forward_step(obs[:, t, :], h_online)
        with torch.no_grad():
            _, h_target_next = target_net.forward_step(obs[:, t, :], h_target)

        if t >= cfg.burn_in:
            batch_idx = torch.arange(batch_size, device=device)
            pred_quantiles = quantiles_t[batch_idx, action[:, t], :]
            with torch.no_grad():
                next_quantiles_online, _ = online_net.forward_step(next_obs[:, t, :], h_online_next)
                next_selection_quantile = _selection_quantiles_from_obs(next_obs[:, t, :], cfg, device)
                next_q_online = online_net.lower_quantile_values(next_quantiles_online, next_selection_quantile)
                next_action = torch.argmax(next_q_online, dim=-1)
                next_quantiles_target, _ = target_net.forward_step(next_obs[:, t, :], h_target_next)
                target_quantiles = next_quantiles_target[batch_idx, next_action, :]
                target = reward[:, t].unsqueeze(1) + cfg.gamma * (1.0 - done[:, t]).unsqueeze(1) * target_quantiles
                td_abs_quantile = (target.unsqueeze(1) - pred_quantiles.unsqueeze(2)).abs().mean(dim=2)
            td_loss = _quantile_huber_loss(pred_quantiles, target, online_net.taus.to(device), cfg.huber_kappa)
            losses.append(td_loss * mask[:, t])
            td_abs_quantile_num += torch.sum(td_abs_quantile * mask[:, t].unsqueeze(1), dim=0)
            td_abs_quantile_den += torch.sum(mask[:, t])

        h_online = h_online_next
        h_target = h_target_next

    if not losses:
        return 0.0

    loss_mat = torch.stack(losses, dim=1)
    denom = torch.clamp(torch.sum(mask[:, cfg.burn_in:]), min=1.0)
    loss = torch.sum(loss_mat) / denom
    optimizer.zero_grad(set_to_none=True)
    loss.backward()
    torch.nn.utils.clip_grad_norm_(online_net.parameters(), 10.0)
    optimizer.step()
    quantile_td_abs = torch.zeros(cfg.num_quantiles, device=device)
    if float(td_abs_quantile_den.item()) > 0.0:
        quantile_td_abs = td_abs_quantile_num / td_abs_quantile_den
    return {
        "loss": float(loss.item()),
        "quantile_td_abs": [float(x) for x in quantile_td_abs.detach().cpu().tolist()],
    }
