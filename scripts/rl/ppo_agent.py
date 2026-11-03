"""PPO-Clip + GAE Actor-Critic for the VVenC-RL QP-only research agent.

Observations:
    spatial: float32 [batch, 3, 3, 4] (activity, temporal proxy, QPA)
    global_: float32 [batch, 8]      (see README_PPO.md)
Actions:
    12 Gaussian latents, tanh-squashed to [-2, 2], quantized to integer
    delta-QP [-2, 2] for the 4 x 3 CTU map (row-major).

The differentiable PPO log-probability is computed for the *raw sampled*
Gaussian action; the encoder receives the rounded map. Do not recompute
log-probabilities from rounded actions. No PPO inference is performed by the
C++ encoder yet: the live observation/action transport is developed separately.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import math

import torch
from torch import Tensor, nn
from torch.distributions import Normal
from torch.nn import functional as F


SPATIAL_SHAPE = (3, 3, 4)
GLOBAL_FEATURES = 8
ACTION_DIM = 12


class PPOActorCritic(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.spatial_encoder = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, padding=1), nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, padding=1), nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)), nn.Flatten(),
        )
        self.global_encoder = nn.Sequential(
            nn.Linear(GLOBAL_FEATURES, 64), nn.ReLU(),
            nn.Linear(64, 32), nn.ReLU(),
        )
        self.fusion = nn.Sequential(nn.Linear(64, 64), nn.Tanh())
        self.actor_mean = nn.Linear(64, ACTION_DIM)
        self.actor_log_std = nn.Parameter(torch.full((ACTION_DIM,), -0.7))
        self.critic = nn.Linear(64, 1)

    def forward(self, spatial: Tensor, global_: Tensor) -> Tuple[Normal, Tensor]:
        if spatial.ndim != 4 or tuple(spatial.shape[1:]) != SPATIAL_SHAPE:
            raise ValueError(f"Expected spatial [B,{SPATIAL_SHAPE}], got {tuple(spatial.shape)}")
        if global_.ndim != 2 or global_.shape[0] != spatial.shape[0] or global_.shape[1] != GLOBAL_FEATURES:
            raise ValueError("Expected global_ [B,8] with same batch size as spatial")

        latent = self.fusion(torch.cat(
            (self.spatial_encoder(spatial), self.global_encoder(global_)), dim=1
        ))
        mean = self.actor_mean(latent)
        std = self.actor_log_std.clamp(-5.0, 1.0).exp().expand_as(mean)
        distribution = Normal(mean, std)
        value = self.critic(latent).squeeze(-1)
        return distribution, value

    @staticmethod
    def _squashed_log_prob(distribution: Normal, raw_action: Tensor) -> Tensor:
        # Log det d[2*tanh(z)]/dz. Stable expression from tanh derivative.
        log_two = math.log(2.0)
        correction = log_two + 2.0 * (
            log_two - raw_action - F.softplus(-2.0 * raw_action)
        )
        return (distribution.log_prob(raw_action) - correction).sum(dim=-1)

    def sample(self, spatial: Tensor, global_: Tensor, deterministic: bool = False):
        distribution, value = self.forward(spatial, global_)
        raw_action = distribution.mean if deterministic else distribution.sample()
        bounded = 2.0 * torch.tanh(raw_action)
        discrete = bounded.round().clamp(-2, 2).to(torch.int32)
        log_prob = self._squashed_log_prob(distribution, raw_action)
        return raw_action, discrete, log_prob, value

    def evaluate(self, spatial: Tensor, global_: Tensor, raw_action: Tensor):
        distribution, value = self.forward(spatial, global_)
        if tuple(raw_action.shape) != tuple(distribution.mean.shape):
            raise ValueError("Expected raw_action [B,12]")
        log_prob = self._squashed_log_prob(distribution, raw_action)
        # Entropy of underlying Gaussian is a well-defined exploration proxy.
        entropy = distribution.entropy().sum(dim=-1)
        return log_prob, entropy, value


@dataclass
class Transition:
    spatial: Tensor                # [3,3,4]
    global_: Tensor                # [8]
    raw_action: Tensor             # [12], before tanh/round
    log_prob: float                # log probability of sampled continuous action
    reward: float                  # available only after encode/decode/reward
    value: float                   # critic value at the observation
    next_value: float              # critic at the next observation; 0 if done
    done: bool                     # environment episode ended after this step


def compute_gae(
    rewards: Tensor,
    values: Tensor,
    next_values: Tensor,
    dones: Tensor,
    gamma: float = 0.99,
    gae_lambda: float = 0.95,
) -> Tuple[Tensor, Tensor]:
    """GAE in trajectory order; done=True cuts credit across video boundaries."""
    n = rewards.numel()
    if any(x.numel() != n for x in (values, next_values, dones)):
        raise ValueError("GAE vectors must have equal length")
    advantages = torch.zeros_like(rewards)
    gae = torch.zeros((), device=rewards.device, dtype=rewards.dtype)
    for t in range(n - 1, -1, -1):
        nonterminal = 1.0 - dones[t].float()
        delta = rewards[t] + gamma * nonterminal * next_values[t] - values[t]
        gae = delta + gamma * gae_lambda * nonterminal * gae
        advantages[t] = gae
    return advantages, advantages + values


class PPOTrainer:
    def __init__(
        self,
        model: PPOActorCritic,
        learning_rate: float = 3e-4,
        gamma: float = 0.99,
        gae_lambda: float = 0.95,
        clip_eps: float = 0.2,
        epochs: int = 5,
        minibatch_size: int = 64,
        target_kl: float = 0.03,
        value_coef: float = 0.5,
        entropy_coef: float = 0.01,
        max_grad_norm: float = 0.5,
    ) -> None:
        self.model = model
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_eps = clip_eps
        self.epochs = epochs
        self.minibatch_size = minibatch_size
        self.target_kl = target_kl
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm

    def update(self, rollout: List[Transition]) -> Dict[str, float]:
        if not rollout:
            raise ValueError("PPO update requires a non-empty rollout")
        device = next(self.model.parameters()).device
        sp = torch.stack([x.spatial for x in rollout]).to(device=device, dtype=torch.float32)
        gl = torch.stack([x.global_ for x in rollout]).to(device=device, dtype=torch.float32)
        raw = torch.stack([x.raw_action for x in rollout]).to(device=device, dtype=torch.float32)
        logp_old = torch.tensor([x.log_prob for x in rollout], device=device, dtype=torch.float32)
        rewards = torch.tensor([x.reward for x in rollout], device=device, dtype=torch.float32)
        values = torch.tensor([x.value for x in rollout], device=device, dtype=torch.float32)
        next_values = torch.tensor([x.next_value for x in rollout], device=device, dtype=torch.float32)
        dones = torch.tensor([x.done for x in rollout], device=device, dtype=torch.float32)

        with torch.no_grad():
            advantage, returns = compute_gae(
                rewards, values, next_values, dones, self.gamma, self.gae_lambda
            )
            advantage = (advantage - advantage.mean()) / (advantage.std(unbiased=False) + 1e-8)

        losses = []
        kls = []
        optimizer_steps = 0
        early_stop = False
        for _ in range(self.epochs):
            for indices in torch.randperm(len(rollout), device=device).split(self.minibatch_size):
                new_logp, entropy, new_values = self.model.evaluate(
                    sp[indices], gl[indices], raw[indices]
                )
                ratio = torch.exp(new_logp - logp_old[indices])
                objective = torch.minimum(
                    ratio * advantage[indices],
                    torch.clamp(ratio, 1 - self.clip_eps, 1 + self.clip_eps) * advantage[indices],
                )
                policy_loss = -objective.mean()
                value_loss = F.mse_loss(new_values, returns[indices])
                loss = (
                    policy_loss
                    + self.value_coef * value_loss
                    - self.entropy_coef * entropy.mean()
                )
                if not torch.isfinite(loss):
                    raise FloatingPointError("Non-finite PPO loss")
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), self.max_grad_norm)
                self.optimizer.step()
                optimizer_steps += 1
                losses.append(float(loss.detach().cpu()))
                approx_kl = float((logp_old[indices] - new_logp.detach()).mean().cpu())
                kls.append(approx_kl)
                if approx_kl > self.target_kl:
                    early_stop = True
                    break
            if early_stop:
                break
        return {
            "loss": sum(losses) / len(losses),
            "approx_kl": sum(kls) / len(kls),
            "optimizer_steps": float(optimizer_steps),
            "early_stop": float(early_stop),
            "mean_reward": float(rewards.mean().cpu()),
        }


def rate_f1_reward(
    f1: float,
    cumulative_bits: int,
    target_bps: float,
    elapsed_seconds: float,
    beta: float,
) -> float:
    """Task-oriented reward based on frozen detector's per-frame pseudo-GT F1."""
    if target_bps <= 0 or elapsed_seconds <= 0 or beta < 0:
        raise ValueError("Invalid target bitrate / elapsed time / reward weight")
    if not (0.0 <= f1 <= 1.0) or cumulative_bits < 0:
        raise ValueError("Invalid F1 or cumulative bits")
    actual_bps = max(1.0, cumulative_bits / elapsed_seconds)
    return f1 - beta * abs(math.log(actual_bps / target_bps))


__all__ = [
    "PPOActorCritic", "PPOTrainer", "Transition", "compute_gae",
    "rate_f1_reward", "SPATIAL_SHAPE", "GLOBAL_FEATURES", "ACTION_DIM",
]
