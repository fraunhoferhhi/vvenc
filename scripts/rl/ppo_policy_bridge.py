#!/usr/bin/env python3
"""Per-POC PPO policy bridge: causal observation -> 4x3 delta-QP action.

Run alongside VVenC (NOT a trained reward-learning loop). The encoder emits
pre-action STATE lines, and this process atomically publishes one <POC>.txt
action file. Mode 'ppo' runs PPOActorCritic forward inference; mode 'zero'
is a deterministic boundary test. Each response is keyed by POC to avoid
mixing encoding order with display order. No RL weights are updated here.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

import torch

from ppo_agent import PPOActorCritic


def parse_observation(line: str):
    tokens = line.strip().split()
    if len(tokens) != 46 or tokens[0] != "STATE":
        raise ValueError(f"Expected STATE poc + 8 global + 36 maps, got {len(tokens)} fields")
    poc = int(tokens[1])
    values = [float(v) for v in tokens[2:]]
    if len(values) != 44 or not all(__import__("math").isfinite(v) for v in values):
        raise ValueError(f"Invalid features for POC {poc}")
    globals_ = torch.tensor(values[:8], dtype=torch.float32).view(1, 8)
    spatial = torch.tensor(values[8:], dtype=torch.float32).view(1, 3, 3, 4)
    return poc, globals_, spatial


def write_action_atomic(directory: Path, poc: int, actions):
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / f"{poc}.txt"
    if target.exists():
        raise RuntimeError(f"Duplicate or stale action path: {target}")
    temp = directory / f".{poc}.{os.getpid()}.tmp"
    try:
        with temp.open("w", encoding="ascii") as f:
            f.write(str(poc) + " " + " ".join(map(str, actions)) + "\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(temp, target)
    finally:
        temp.unlink(missing_ok=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--state-file", type=Path, required=True)
    p.add_argument("--action-dir", type=Path, required=True)
    p.add_argument("--trajectory-file", type=Path, required=True)
    p.add_argument("--frames", type=int, required=True)
    p.add_argument("--policy", choices=["ppo", "zero"], default="ppo")
    p.add_argument("--checkpoint", type=Path)
    p.add_argument("--deterministic", action="store_true")
    p.add_argument("--idle-timeout", type=float, default=120.0)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--ready-file", type=Path)
    a = p.parse_args()

    if a.frames <= 0 or a.idle_timeout <= 0:
        p.error("frames and idle-timeout must be positive")
    if a.state_file.exists():
        p.error("state file must not exist before starting a fresh episode")
    if a.action_dir.exists() and any(a.action_dir.iterdir()):
        p.error("action directory must be empty before a fresh episode")
    if a.trajectory_file.exists():
        p.error("trajectory file already exists; use a new episode path")

    torch.manual_seed(a.seed)
    model = PPOActorCritic().eval()
    if a.checkpoint:
        ckpt = torch.load(a.checkpoint, map_location="cpu", weights_only=True)
        model.load_state_dict(ckpt.get("model_state", ckpt))

    a.action_dir.mkdir(parents=True, exist_ok=True)
    a.state_file.parent.mkdir(parents=True, exist_ok=True)
    a.trajectory_file.parent.mkdir(parents=True, exist_ok=True)
    observed = set()
    last_activity = time.monotonic()

    if a.ready_file:
        a.ready_file.write_text("READY\n", encoding="ascii")

    # Wait for VVenC to open its pre-QPA observation output.
    while not a.state_file.is_file():
        if time.monotonic() - last_activity > a.idle_timeout:
            raise TimeoutError(f"Encoder did not create state file: {a.state_file}")
        time.sleep(0.02)

    with a.state_file.open("r", encoding="utf-8") as src, \
         a.trajectory_file.open("w", encoding="utf-8") as trajectory:
        while len(observed) < a.frames:
            line = src.readline()
            if not line:
                if time.monotonic() - last_activity > a.idle_timeout:
                    raise TimeoutError(
                        f"Policy waited for observation: got {len(observed)} / {a.frames}"
                    )
                time.sleep(0.02)
                continue
            last_activity = time.monotonic()
            poc, globals_, spatial = parse_observation(line)
            if poc in observed:
                raise RuntimeError(f"Duplicate observation POC {poc}")
            if poc < 0 or poc >= a.frames:
                raise RuntimeError(f"Unexpected POC {poc} for {a.frames} frame episode")

            with torch.no_grad():
                raw, action, log_prob, value = model.sample(
                    spatial, globals_, deterministic=a.deterministic
                )
            actions = ([0] * 12 if a.policy == "zero" else action[0].tolist())
            write_action_atomic(a.action_dir, poc, actions)

            # Policy output and raw sampled actions are kept for PPO ratios.
            # Reward must be appended from VVdeC+YOLO post-encode by POC.
            record = {
                "poc": poc,
                "spatial": spatial[0].tolist(),
                "global": globals_[0].tolist(),
                "raw_action": raw[0].tolist() if a.policy == "ppo" else None,
                "discrete_action": actions,
                "log_prob": float(log_prob.item()) if a.policy == "ppo" else None,
                "value": float(value.item()) if a.policy == "ppo" else None,
                "reward": None,
                "done": None,
            }
            trajectory.write(json.dumps(record, separators=(",", ":")) + "\n")
            trajectory.flush()
            observed.add(poc)
            print(f"PPO_ACTION poc={poc} values={actions}", flush=True)

    print(f"POLICY_BRIDGE_PASS states={len(observed)} actions={len(observed)}", flush=True)


if __name__ == "__main__":
    main()
