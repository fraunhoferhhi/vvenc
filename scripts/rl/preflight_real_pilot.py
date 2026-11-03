#!/usr/bin/env python3
"""Preflight check for locked Train-only YOLO/VVenC reward pilot.

Stops BEFORE rebuilding codecs if the GPU or locked Kaggle Inputs are absent.
No Test data are opened or processed.
"""
from pathlib import Path
import argparse
import json

from real_reward_pilot import read_lock_manifest, read_baseline_config, dataset_source


def main():
    import torch
    p = argparse.ArgumentParser()
    p.add_argument("--train-index", type=int, default=0)
    args = p.parse_args()
    if not 0 <= args.train_index < 65:
        p.error("--train-index must be 0..64")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA GPU not found: enable Kaggle GPU T4x2 before real YOLO inference")
    print("CUDA:", torch.cuda.get_device_name(0))
    splits, records = read_lock_manifest(Path("/kaggle/input"))
    lock = read_baseline_config(Path("/kaggle/input"))
    chosen = splits["train"][args.train_index]
    source = dataset_source(records, chosen)
    assert chosen not in splits["val"] and chosen not in splits["test"]
    print("REAL_PILOT_INPUT_GATE_PASS", json.dumps({
        "selected_train_id": chosen,
        "selected_train_index": args.train_index,
        "video_path": str(source),
        "split_counts": {k: len(v) for k, v in splits.items()},
        "baseline_lock_sha256": lock["lock_sha256"],
        "gpu": torch.cuda.get_device_name(0),
    }, ensure_ascii=False))
    print("No Validation or Test video was opened or encoded.")


if __name__ == "__main__":
    main()
