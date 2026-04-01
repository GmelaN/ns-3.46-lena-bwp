#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch
from tensordict import TensorDict
from torch import nn
from torchrl.data import ListStorage, TensorDictReplayBuffer


@dataclass
class DrqnConfig:
    obs_dim: int
    action_dim: int
    hidden_dim: int = 128
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


class DrqnNet(nn.Module):
    def __init__(self, obs_dim: int, hidden_dim: int, action_dim: int):
        super().__init__()
        self.obs_dim = obs_dim
        self.hidden_dim = hidden_dim
        self.action_dim = action_dim
        self.in_proj = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
        )
        self.rnn = nn.GRUCell(hidden_dim, hidden_dim)
        self.head = nn.Linear(hidden_dim, action_dim)

    def forward_step(self, obs: torch.Tensor, hidden: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        x = self.in_proj(obs)
        next_hidden = self.rnn(x, hidden)
        q = self.head(next_hidden)
        return q, next_hidden

    def zero_hidden(self, batch_size: int, device: torch.device) -> torch.Tensor:
        return torch.zeros(batch_size, self.hidden_dim, dtype=torch.float32, device=device)


def make_replay_buffer(capacity: int) -> TensorDictReplayBuffer:
    return TensorDictReplayBuffer(storage=ListStorage(capacity))


def epsilon_by_step(cfg: DrqnConfig, step: int) -> float:
    eps = cfg.eps_start * (cfg.eps_decay ** max(0, step))
    return max(cfg.eps_end, float(eps))


def select_action(
    net: DrqnNet,
    obs: np.ndarray,
    hidden: torch.Tensor,
    epsilon: float,
    device: torch.device,
) -> tuple[int, torch.Tensor]:
    obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
    with torch.no_grad():
        q, next_hidden = net.forward_step(obs_t, hidden)
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
        t = len(chunk)
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
        td = TensorDict(
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
        out.append(td)
        if start + seq_len >= len(transitions):
            break
        start += stride
    return out


def train_batch(
    online_net: DrqnNet,
    target_net: DrqnNet,
    optimizer: torch.optim.Optimizer,
    batch: TensorDict,
    cfg: DrqnConfig,
    device: torch.device,
) -> float:
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

    for t in range(seq_len):
        q_t, h_online_next = online_net.forward_step(obs[:, t, :], h_online)
        with torch.no_grad():
            _, h_target_next = target_net.forward_step(obs[:, t, :], h_target)

        if t >= cfg.burn_in:
            q_taken = q_t.gather(1, action[:, t].unsqueeze(1)).squeeze(1)
            with torch.no_grad():
                q_next, _ = target_net.forward_step(next_obs[:, t, :], h_target_next)
                target = reward[:, t] + cfg.gamma * (1.0 - done[:, t]) * torch.max(q_next, dim=1).values
            td_loss = nn.functional.smooth_l1_loss(q_taken, target, reduction="none")
            losses.append(td_loss * mask[:, t])

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
    return float(loss.item())
