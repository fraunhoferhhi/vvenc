#!/usr/bin/env python3
"""A bounded three-Train-video PPO-QP continuation gate, not full training.

Prerequisites: Kaggle GPU, locked BDD100K inputs and 2nd real checkpoint.
Each episode is collected with the *current* policy, gets genuine frozen-YOLO
F1/actual AU rate reward, and causes exactly ONE PPO update before next clip.

Safety:
 - Train-only indices in fixed explicit order; Test and Val untouched
 - SHA-chain checkpoint identity + optimizer continuation validated
 - crash-safe manifest with atomic replacement
 - no automatic replay of a partial episode after errors
 - checkpoint-only ZIP exported AFTER EVERY successful new episode
 - beta=0.1 is a diagnostic pilot weight, NOT an optimized final value
 - all samples are 64..512-frame prefixes, NOT complete BDD100K videos
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

import torch

from real_reward_pilot import read_lock_manifest, read_baseline_config

SCHEMA = "vvenc-ppo-multi-train-gate-v1"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for buf in iter(lambda: f.read(1048576), b""):
            h.update(buf)
    return h.hexdigest()


def atomic_json(path: Path, data) -> None:
    tmp = path.with_name(path.name + f".{os.getpid()}.tmp")
    try:
        with tmp.open("w", encoding="utf-8") as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


def load_checkpoint(path: Path):
    if not path.is_file():
        raise FileNotFoundError(f"Checkpoint missing: {path}")
    meta = torch.load(path, map_location="cpu", weights_only=True)
    if (meta.get("schema") != "vvenc-ppo-single-real-rollout-gate/1"
            or meta.get("split") != "train"
            or not isinstance(meta.get("ppo_iteration"), int)
            or meta["ppo_iteration"] < 1
            or "model_state" not in meta or "optimizer_state" not in meta):
        raise ValueError(f"Unsupported PPO checkpoint: {path}")
    return meta


def checkpoint_bundle(folder: Path, manifest: dict) -> Path:
    """Compact checkpoint/provenance ZIP; no large YUVs or bitstreams."""
    dst = folder / "PPO_QP_TRAIN_GATE_CHECKPOINTS.zip"
    tmp = dst.with_name(dst.name + ".tmp")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.write(folder / "progress.json", "progress.json")
        for row in manifest["checkpoint_files"]:
            filename = row["name"]
            path = folder / filename
            if sha256(path) != row["sha256"]:
                raise RuntimeError("Checkpoint checksum changed before archive: " + filename)
            archive.write(path, filename)
        for record in manifest["completed"]:
            summary = folder / record["episode_dir"] / "summary.json"
            if not summary.is_file():
                raise RuntimeError("Missing completed episode summary: " + str(summary))
            # Keep the original relative directory: extracted ZIP can resume.
            archive.write(summary, f"{record['episode_dir']}/summary.json")
    os.replace(tmp, dst)
    return dst


def run_checked(cmd: list[str], log: Path) -> None:
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w", encoding="utf-8") as stream:
        stream.write("$ " + " ".join(cmd) + "\n")
        stream.flush()
        returncode = subprocess.call(cmd, stdout=stream, stderr=subprocess.STDOUT)
    if returncode:
        tail = "\n".join(log.read_text(errors="replace").splitlines()[-45:])
        raise RuntimeError(f"Subprocess failed rc={returncode}. Inspect {log}:\n{tail}")


def parse_train_indices(text: str) -> list[int]:
    try:
        selected = [int(x.strip()) for x in text.split(",")]
    except ValueError as exc:
        raise ValueError("Train indices must be comma-separated integers") from exc
    if not selected or len(selected) > 3 or len(set(selected)) != len(selected):
        raise ValueError("For this bounded gate: provide 1..3 unique Train indices")
    if any(not 0 <= x < 65 for x in selected):
        raise ValueError("Train indices must lie in the locked range [0,64]")
    return selected


def run_gate(args) -> None:
    if not torch.cuda.is_available():
        raise RuntimeError("Kaggle T4 CUDA GPU required for frozen YOLO inference")
    if not args.repo.is_dir() or not (args.repo / "scripts/rl/real_reward_pilot.py").is_file():
        raise FileNotFoundError("RL repository absent")
    if not args.decoder.is_file():
        raise FileNotFoundError("VVdeC executable absent")
    if not (args.repo / "bin/release-static/vvencapp").is_file():
        raise FileNotFoundError("VVenC-RL executable absent")
    selected = parse_train_indices(args.train_indices)
    if not (32 <= args.frames <= 512):
        raise ValueError("Pilot sample frames must be 32..512")
    if not (0.0 <= args.beta <= 10.0):
        raise ValueError("Invalid beta")

    splits, _ = read_lock_manifest(Path("/kaggle/input"))
    lock = read_baseline_config(Path("/kaggle/input"))
    ids = [splits["train"][idx] for idx in selected]
    if set(ids) & set(splits["val"] + splits["test"]):
        raise AssertionError("Train/Val/Test split leakage")
    source = load_checkpoint(args.initial_checkpoint)
    locksha = lock["lock_sha256"]
    if source["baseline_lock_sha256"] != locksha:
        raise ValueError("Initial checkpoint has a different locked Baseline protocol")

    root = args.output
    progress_path = root / "progress.json"
    if progress_path.is_file():
        progress = json.loads(progress_path.read_text(encoding="utf-8"))
        if (progress.get("schema") != SCHEMA
                or progress.get("train_indices") != selected
                or progress.get("video_ids") != ids
                or progress.get("frames") != args.frames
                or progress.get("beta") != args.beta
                or progress.get("lock_sha256") != locksha):
            raise RuntimeError("Incompatible resume plan; use a new directory")
        base = root / progress["checkpoint_files"][0]["name"]
        if sha256(base) != progress["checkpoint_files"][0]["sha256"]:
            raise RuntimeError("Initial copied checkpoint changed")
        if sha256(args.initial_checkpoint) != sha256(base):
            raise RuntimeError("Supplied initial checkpoint differs from saved plan")
        completed = progress["completed"]
        if len(completed) > len(selected) or any(
            x["train_index"] != selected[i] for i, x in enumerate(completed)
        ):
            raise RuntimeError("Invalid completed history in progress manifest")
        for row in progress["checkpoint_files"]:
            if sha256(root / row["name"]) != row["sha256"]:
                raise RuntimeError("Stored checkpoint chain checksum mismatch")
        current_ckpt = root / progress["checkpoint_files"][-1]["name"]
    else:
        if root.exists() and any(root.iterdir()):
            raise FileExistsError("Output folder is nonempty but progress.json missing; do not overwrite")
        root.mkdir(parents=True, exist_ok=True)
        copied = root / f"ckpt_iter_{source['ppo_iteration']:03d}.pt"
        shutil.copy2(args.initial_checkpoint, copied)
        progress = {
            "schema": SCHEMA, "train_indices": selected,
            "video_ids": ids, "frames": args.frames,
            "beta": args.beta, "lock_sha256": locksha,
            "checkpoint_files": [
                {"name": copied.name, "sha256": sha256(copied),
                 "iteration": source["ppo_iteration"]}
            ],
            "completed": [],
            "role": "bounded_real_Train_continuation_gate_not_full_training",
        }
        atomic_json(progress_path, progress)
        current_ckpt = copied

    print(f"GATE PLAN: {len(progress['completed'])}/{len(selected)} completed. "
          f"Next checkpoint: {current_ckpt}", flush=True)
    for offset in range(len(progress["completed"]), len(selected)):
        idx = selected[offset]
        episode_dir = root / f"episode_{offset + 1:02d}_train_{idx:02d}"
        if episode_dir.exists():
            raise RuntimeError(
                f"Partial/replayed episode directory exists: {episode_dir}. "
                "Manual audit needed; never silently reuse old rollout."
            )
        seed = args.seed_base + offset
        print(f"\n=== NEW POLICY ROLLOUT {offset+1}/{len(selected)} "
              f"Train index={idx}, video_id={ids[offset]}, "
              f"seed={seed}, frames={args.frames} ===", flush=True)
        run_checked([
            sys.executable, str(args.repo / "scripts/rl/real_reward_pilot.py"),
            "--repo", str(args.repo),
            "--decoder", str(args.decoder),
            "--train-index", str(idx),
            "--frames", str(args.frames), "--beta", str(args.beta),
            "--policy-checkpoint", str(current_ckpt),
            "--policy-seed", str(seed),
            "--out", str(episode_dir),
        ], root / f"episode_{offset + 1:02d}_collect.log")

        summary_path = episode_dir / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if (summary["video_id"] != ids[offset]
                or summary.get("split") != "train"
                or summary.get("frames") != args.frames
                or summary.get("baseline_lock_sha256") != locksha
                or summary.get("policy_checkpoint_sha256") != sha256(current_ckpt)
                or summary.get("validation_status") != "UPDATED_POLICY_REWARD_DATA_COLLECTION"):
            raise RuntimeError("Gathered PPO episode has wrong Train/policy/lock identity")
        ckpt_next = root / f"ckpt_iter_{load_checkpoint(current_ckpt)['ppo_iteration'] + 1:03d}.pt"
        if ckpt_next.exists():
            raise FileExistsError("Next checkpoint filename already exists")
        run_checked([
            sys.executable, str(args.repo / "scripts/rl/verify_real_ppo_update.py"),
            "--episode", str(episode_dir),
            "--checkpoint-in", str(current_ckpt),
            "--checkpoint-out", str(ckpt_next),
        ], root / f"episode_{offset + 1:02d}_ppo_update.log")

        meta = load_checkpoint(ckpt_next)
        if (meta["ppo_iteration"] != load_checkpoint(current_ckpt)["ppo_iteration"] + 1
                or meta["collector_checkpoint_sha256"] != sha256(current_ckpt)
                or meta["video_id"] != ids[offset]
                or meta["baseline_lock_sha256"] != locksha):
            raise RuntimeError("Next checkpoint does not reference exact source policy")
        checkpoint_sha = sha256(ckpt_next)
        progress["completed"].append({
            "train_index": idx, "video_id": ids[offset],
            "episode_dir": episode_dir.name,
            "seed": seed, "frames": args.frames,
            "bitrate_kbps": summary["actual_kbps"],
            "tp": summary["total_tp"],
            "fp": summary["total_fp"],
            "fn": summary["total_fn"],
            "mean_reward": summary["mean_reward"],
            "checkpoint": ckpt_next.name,
            "checkpoint_sha256": checkpoint_sha,
            "ppo_iteration": meta["ppo_iteration"],
            "ppo_metrics": meta["metrics"],
        })
        progress["checkpoint_files"].append({
            "name": ckpt_next.name, "sha256": checkpoint_sha,
            "iteration": meta["ppo_iteration"],
        })
        atomic_json(progress_path, progress)
        archive = checkpoint_bundle(root, progress)
        print(
            f"NEW_EPISODE_PPO_PASS train={idx}, iteration={meta['ppo_iteration']}, "
            f"mean_reward={summary['mean_reward']:.6f}, "
            f"checkpoint={ckpt_next}, checkpoint_zip={archive}",
            flush=True,
        )
        current_ckpt = ckpt_next

    # Also rebuild a missing archive after a successful fully completed resume.
    if progress["completed"]:
        checkpoint_bundle(root, progress)
    print(f"BOUNDED_TRAIN_CONTINUATION_GATE_PASS: "
          f"{len(progress['completed'])} fresh Train episodes, "
          f"latest_checkpoint={current_ckpt}")
    print(f"SAVE THE ZIP: {root / 'PPO_QP_TRAIN_GATE_CHECKPOINTS.zip'}")
    print("No Validation/Test used; full 65-video training NOT run.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--decoder", type=Path, required=True)
    parser.add_argument("--initial-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--train-indices", default="1,2,3")
    parser.add_argument("--frames", type=int, default=64)
    parser.add_argument("--beta", type=float, default=0.1)
    parser.add_argument("--seed-base", type=int, default=3100)
    args = parser.parse_args()
    if args.seed_base < 0:
        parser.error("--seed-base must not be negative")
    run_gate(args)


if __name__ == "__main__":
    main()
