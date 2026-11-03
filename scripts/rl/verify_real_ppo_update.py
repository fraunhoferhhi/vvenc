#!/usr/bin/env python3
"""Single strict PPO update on a REAL frozen-YOLOv5n BDD100K Train rollout.

This is an integration GATE, not multi-episode training. Before taking any
gradient, reconstruct EXACTLY the untrained bridge policy that sampled the
episode, then assert state/value/logprob/discrete action parity. A mismatch
aborts instead of performing an invalid off-policy PPO update.

On success: one PPOTrainer.update, atomically save full model/optimizer
checkpoint, reload it into fresh objects and verify round-trip identity.
No more updates may be performed on the same rollout.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random

import numpy as np
import torch

from ppo_agent import PPOActorCritic, PPOTrainer, Transition


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as inp:
        for buf in iter(lambda: inp.read(1024 * 1024), b""):
            h.update(buf)
    return h.hexdigest()


def read_real_rollout(episode: Path):
    summary_path = episode / "summary.json"
    rollout_path = episode / "joined_reward_ppo.jsonl"
    if not summary_path.is_file() or not rollout_path.is_file():
        raise FileNotFoundError("Expected successful real Reward pilot summary and joined rollout")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if (
        summary.get("split") != "train"
        or summary.get("validation_status") not in (
            "UNTRAINED_POLICY_REWARD_DATA_COLLECTION",
            "UPDATED_POLICY_REWARD_DATA_COLLECTION",
        )
        or summary.get("ppo_policy_training") is not False
        or summary.get("target_kbps") != 120
        or summary.get("au_sum_matches_stream") is not True
    ):
        raise ValueError("Invalid or non-Train real Reward pilot summary")
    with rollout_path.open(encoding="utf-8") as f:
        rows = [json.loads(line) for line in f if line.strip()]
    frames = summary.get("frames")
    if type(frames) is not int or frames < 32 or len(rows) != frames:
        raise ValueError("Missing/incorrect Train episode frames")
    if {x.get("poc") for x in rows} != set(range(frames)):
        raise ValueError("Missing or duplicated POC")
    if any(x.get("video_id") != summary.get("video_id") for x in rows):
        raise ValueError("Video identity mismatch")
    for i, row in enumerate(rows):
        if (
            row.get("policy_decision_step") != i + 1
            or row.get("done") is not (i == frames - 1)
            or row.get("reward_diagnostics", {}).get("poc") != row["poc"]
            or row.get("reward_diagnostics", {}).get("video_id") != summary["video_id"]
            or row.get("reward_diagnostics", {}).get("encoded_step") != i + 1
        ):
            raise ValueError("POC/order/terminal boundary mismatch at transition " + str(i))
    return summary, rows, rollout_path


def validate_unchanged_collecting_policy(
    model: PPOActorCritic,
    rows: list[dict],
    atol: float = 3e-4,
):
    """Assert checkpoint/current policy is the SAME as sample-generating policy."""
    sp = torch.tensor([r["spatial"] for r in rows], dtype=torch.float32)
    gl = torch.tensor([r["global"] for r in rows], dtype=torch.float32)
    raw = torch.tensor([r["raw_action"] for r in rows], dtype=torch.float32)
    actions = torch.tensor([r["discrete_action"] for r in rows], dtype=torch.int32)
    old_logp = torch.tensor([r["log_prob"] for r in rows], dtype=torch.float32)
    old_value = torch.tensor([r["value"] for r in rows], dtype=torch.float32)
    reward = torch.tensor([r["reward"] for r in rows], dtype=torch.float32)
    nxt = torch.tensor([r["next_value"] for r in rows], dtype=torch.float32)
    if (
        tuple(sp.shape) != (len(rows), 3, 3, 4)
        or tuple(gl.shape) != (len(rows), 8)
        or tuple(raw.shape) != (len(rows), 12)
        or tuple(actions.shape) != (len(rows), 12)
    ):
        raise ValueError("Wrong PPO observation/action shape")
    for tensor in (sp, gl, raw, old_logp, old_value, reward, nxt):
        if not torch.isfinite(tensor).all():
            raise ValueError("Non-finite real PPO transition")
    expected_actions = (2.0 * torch.tanh(raw)).round().clamp(-2, 2).int()
    if not torch.equal(expected_actions, actions):
        raise ValueError("Rounded latent Gaussian action != encoder QP delta action")
    model.eval()
    # Single-sample eval matches the CPU inference bridge's numerical path.
    with torch.no_grad():
        evaluated = [
            model.evaluate(sp[i:i + 1], gl[i:i + 1], raw[i:i + 1])
            for i in range(len(rows))
        ]
        new_logp = torch.cat([p[0] for p in evaluated], dim=0)
        new_value = torch.cat([p[2] for p in evaluated], dim=0)
    max_logp_error = (old_logp - new_logp).abs().max().item()
    max_value_error = (old_value - new_value).abs().max().item()
    if max_logp_error > atol or max_value_error > atol:
        raise ValueError(
            f"Collector policy mismatch: logp_error={max_logp_error:.9g}, "
            f"value_error={max_value_error:.9g}. STOP: cannot perform PPO update "
            "from a different/random policy."
        )
    if any(abs(float(rows[i]["next_value"]) - (
            float(rows[i + 1]["value"]) if i + 1 < len(rows) else 0.0
        )) > 1e-6 for i in range(len(rows))):
        raise ValueError("next_value does not match next policy decision/value")
    transitions = []
    for i, row in enumerate(rows):
        transitions.append(Transition(
            spatial=sp[i], global_=gl[i], raw_action=raw[i],
            log_prob=float(old_logp[i]),
            reward=float(reward[i]), value=float(old_value[i]),
            next_value=float(nxt[i]), done=bool(row["done"]),
        ))
    return transitions, {
        "max_old_logp_error": max_logp_error,
        "max_old_value_error": max_value_error,
        "sampled_nonzero_qp_actions": int((actions != 0).sum().item()),
    }


def checkpoint_and_verify(
    trainer: PPOTrainer, output: Path, summary: dict, metrics: dict,
    rollout_path: Path, seed: int, collector_sha: str | None = None,
    ppo_iteration: int = 1,
):
    if output.exists():
        raise FileExistsError("Checkpoint already exists; refuse double update on same rollout")
    state = {
        "schema": "vvenc-ppo-single-real-rollout-gate/1",
        "experiment_stage": "one_update_only_not_full_training",
        "model_state": trainer.model.state_dict(),
        "optimizer_state": trainer.optimizer.state_dict(),
        "model_hparams": {
            "lr": 3e-4, "gamma": trainer.gamma, "gae_lambda": trainer.gae_lambda,
            "clip_eps": trainer.clip_eps, "epochs": trainer.epochs,
            "minibatch_size": trainer.minibatch_size, "target_kl": trainer.target_kl,
            "value_coef": trainer.value_coef, "entropy_coef": trainer.entropy_coef,
            "max_grad_norm": trainer.max_grad_norm,
        },
        "rollout_sha256": sha256(rollout_path),
        "video_id": summary["video_id"], "split": "train",
        "frames": summary["frames"], "baseline_lock_sha256": summary["baseline_lock_sha256"],
        "ppo_bridge_seed": int(summary.get("policy_bridge_seed", 7)),
        "optimizer_update_seed": seed,
        "ppo_iteration": ppo_iteration,
        "collector_checkpoint_sha256": collector_sha,
        "metrics": metrics,
        "torch_rng_state": torch.get_rng_state(),
        "python_rng_state": random.getstate(),
        # Serialize numpy state as only primitive types: torch.load(weights_only=True)
        # intentionally rejects arbitrary numpy objects.
        "numpy_rng_state": (
            np.random.get_state()[0],
            np.random.get_state()[1].tolist(),
            int(np.random.get_state()[2]),
            int(np.random.get_state()[3]),
            float(np.random.get_state()[4]),
        ),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp = output.with_name(output.name + f".{os.getpid()}.tmp")
    try:
        torch.save(state, tmp)
        restored = torch.load(tmp, map_location="cpu", weights_only=True)
        restored_model = PPOActorCritic()
        restored_model.load_state_dict(restored["model_state"], strict=True)
        restored_trainer = PPOTrainer(
            restored_model, learning_rate=restored["model_hparams"]["lr"],
            gamma=restored["model_hparams"]["gamma"],
            gae_lambda=restored["model_hparams"]["gae_lambda"],
            clip_eps=restored["model_hparams"]["clip_eps"],
            epochs=restored["model_hparams"]["epochs"],
            minibatch_size=restored["model_hparams"]["minibatch_size"],
            target_kl=restored["model_hparams"]["target_kl"],
            value_coef=restored["model_hparams"]["value_coef"],
            entropy_coef=restored["model_hparams"]["entropy_coef"],
            max_grad_norm=restored["model_hparams"]["max_grad_norm"],
        )
        restored_trainer.optimizer.load_state_dict(restored["optimizer_state"])
        if any(not torch.equal(v, restored_model.state_dict()[k])
               for k, v in trainer.model.state_dict().items()):
            raise AssertionError("Checkpoint model params differ on reload")
        if restored_trainer.optimizer.state_dict()["state"].keys() != trainer.optimizer.state_dict()["state"].keys():
            raise AssertionError("Optimizer state missing on reload")
        # Verify optimizer states exactly (not just state dict count).
        a = trainer.optimizer.state_dict()
        b = restored_trainer.optimizer.state_dict()
        for param_key, state in a["state"].items():
            for key, value in state.items():
                recovered = b["state"][param_key][key]
                if torch.is_tensor(value):
                    if not torch.equal(value, recovered):
                        raise AssertionError("Optimizer tensor changed in checkpoint roundtrip")
                elif value != recovered:
                    raise AssertionError("Optimizer scalar changed in checkpoint roundtrip")
        os.replace(tmp, output)
    finally:
        if tmp.exists():
            tmp.unlink()
    return sha256(output)


def one_true_rollout_update(
    episode: Path, checkpoint: Path, seed: int = 20261103,
    checkpoint_in: Path | None = None,
):
    if checkpoint.exists():
        raise FileExistsError("Checkpoint already exists; use new output, never replay one rollout")
    summary, rows, rollout_path = read_real_rollout(episode)
    torch.set_num_threads(2)
    torch.manual_seed(7)
    model = PPOActorCritic()
    collector_sha = None
    prev = None
    iteration = 1
    if checkpoint_in is not None:
        if not checkpoint_in.is_file():
            raise FileNotFoundError("Collector checkpoint missing: " + str(checkpoint_in))
        collector_sha = sha256(checkpoint_in)
        if summary.get("policy_checkpoint_sha256") != collector_sha:
            raise ValueError("Collector checkpoint hash mismatch: episode was sampled by another policy")
        prev = torch.load(checkpoint_in, map_location="cpu", weights_only=True)
        if (prev.get("schema") != "vvenc-ppo-single-real-rollout-gate/1"
                or prev.get("split") != "train"
                or prev.get("baseline_lock_sha256") != summary["baseline_lock_sha256"]):
            raise ValueError("Collector checkpoint is not a compatible Train PPO checkpoint")
        if prev.get("rollout_sha256") == sha256(rollout_path):
            raise ValueError("Refusing PPO update with rollout already used in source checkpoint")
        model.load_state_dict(prev["model_state"], strict=True)
        iteration = int(prev.get("ppo_iteration", 1)) + 1
    elif summary.get("policy_checkpoint_sha256") is not None:
        raise ValueError("This episode used a trained policy: --checkpoint-in is REQUIRED")
    elif summary.get("validation_status") != "UNTRAINED_POLICY_REWARD_DATA_COLLECTION":
        raise ValueError("Updated-policy episode is missing collector checkpoint identity")

    rollout, verification = validate_unchanged_collecting_policy(model, rows)

    # Only AFTER checking that collector policy == current trainable policy:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if prev is not None:
        hp = prev["model_hparams"]
        trainer = PPOTrainer(
            model, learning_rate=hp["lr"], gamma=hp["gamma"],
            gae_lambda=hp["gae_lambda"], clip_eps=hp["clip_eps"],
            epochs=hp["epochs"], minibatch_size=hp["minibatch_size"],
            target_kl=hp["target_kl"], value_coef=hp["value_coef"],
            entropy_coef=hp["entropy_coef"],
            max_grad_norm=hp["max_grad_norm"],
        )
        trainer.optimizer.load_state_dict(prev["optimizer_state"])
    else:
        trainer = PPOTrainer(model, epochs=2, minibatch_size=32)
    before = {key: value.detach().clone() for key, value in model.state_dict().items()}
    metrics = trainer.update(rollout)
    if not all(math.isfinite(metrics[k]) for k in ("loss", "mean_reward", "approx_kl")):
        raise ValueError("PPO loss/reward nonfinite")
    if metrics["optimizer_steps"] <= 0:
        raise ValueError("PPO never updated optimizer")
    actor_changed = not torch.equal(before["actor_mean.weight"], model.state_dict()["actor_mean.weight"])
    critic_changed = not torch.equal(before["critic.weight"], model.state_dict()["critic.weight"])
    if not (actor_changed and critic_changed):
        raise AssertionError("Both actor and critic must change during real-data PPO update")
    metrics.update(verification)
    metrics["actor_changed"] = actor_changed
    metrics["critic_changed"] = critic_changed

    digest = checkpoint_and_verify(
        trainer, checkpoint, summary, metrics, rollout_path, seed,
        collector_sha=collector_sha, ppo_iteration=iteration,
    )
    print(json.dumps({
        "gate": "REAL_ROLLOUT_SINGLE_PPO_UPDATE_PASS",
        "video_id": summary["video_id"],
        "frames": len(rows),
        "ppo_iteration": iteration,
        "collector_checkpoint_sha256": collector_sha,
        "optimizer_steps": metrics["optimizer_steps"],
        "mean_real_reward": metrics["mean_reward"],
        "ppo_loss": metrics["loss"],
        "approx_kl": metrics["approx_kl"],
        "early_stop": metrics["early_stop"],
        "collector_max_logprob_error": metrics["max_old_logp_error"],
        "collector_max_value_error": metrics["max_old_value_error"],
        "actor_updated": actor_changed, "critic_updated": critic_changed,
        "checkpoint": str(checkpoint), "checkpoint_sha256": digest,
        "checkpoint_reload": "EXACT",
    }, indent=2))
    print("CHECKPOINT_READY_FOR_NEXT_FRESH_ROLLOUT: NOT full PPO training")
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--checkpoint-out", type=Path, required=True)
    parser.add_argument("--checkpoint-in", type=Path,
                        help="Exactly the checkpoint used to collect this NEW rollout")
    args = parser.parse_args()
    one_true_rollout_update(args.episode, args.checkpoint_out,
                            checkpoint_in=args.checkpoint_in)


if __name__ == "__main__":
    main()
