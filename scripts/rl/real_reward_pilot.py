#!/usr/bin/env python3
"""A genuine frozen-YOLOv5n QP-only reward episode on LOCKED BDD100K Train.

This pilot does NOT update the PPO network and does not access Validation/Test
videos. One TRAIN .mov -> exact Baseline color/resize -> VVenC online PPO QP
-> VVdeC -> frozen YOLO on reference/reconstruction -> POC-joined reward.

Kaggle inputs required:
  1. robikscube/driving-video-with-object-tracking (BDD100K Train00 MOV)
  2. VCM_BDD100K_LOCK_v1.zip or extracted dataset_lock_v1.json + split lists
  3. VVenC_Pilot_v1_2_QA_LOCKED_SNAPSHOT.zip or its extracted
     baseline_config.lock.json; refuses mismatching model/preprocess config.

Requires internet for YOLOv5 v7.0 weights/repository if not previously cloned,
and a CUDA GPU for detector. VVenC/VVdeC binaries built beforehand.

The sample's reduced duration is for validation ONLY. Its rate/F1 is not a
full-video result and must never be combined with the 500-run Test baseline.
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import urllib.request
import zipfile

import numpy as np

from au_rate_audit import read_au_bits
from reward_join import join_episode

MANIFEST_SHA = "f7a600f501de622f66ace04bb0b12ec5da0dab988cd54732d614f6b16495aaea"
DATASET_HINT = Path("/kaggle/input/datasets/robikscube/driving-video-with-object-tracking")
W, H, FPS = 480, 320, 30
FBY, FB10 = W * H * 3 // 2, W * H * 3
YOLO_REF = "v7.0"
CAR_CLASS = 2
BITRATE_KBPS = 120
SPACE_MAP = {
    "bt709": "bt709", "bt470bg": "bt470bg", "smpte170m": "smpte170m",
    "smpte240m": "smpte240m", "bt2020nc": "bt2020nc"
}
PRIMARIES_MAP = {
    "bt709": "bt709", "bt470bg": "bt470bg", "smpte170m": "smpte170m",
    "smpte240m": "smpte240m", "bt2020": "bt2020", "bt470m": "bt470m",
}
TRC_MAP = {
    "bt709": "bt709", "bt470bg": "bt470bg", "bt470m": "bt470m",
    "smpte170m": "smpte170m", "smpte240m": "smpte240m",
    "iec61966-2-1": "srgb", "srgb": "srgb", "bt2020-10": "bt2020-10",
    "bt2020-12": "bt2020-12",
}
UNSPECIFIED = (None, "unknown", "unspecified", "reserved")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def run_cmd(command: list[str], log: Path, env=None, timeout=900) -> None:
    with log.open("w", encoding="utf-8") as f:
        f.write("$ " + " ".join(map(str, command)) + "\n")
        f.flush()
        result = subprocess.run(command, stdout=f, stderr=subprocess.STDOUT,
                                env=env, timeout=timeout)
    if result.returncode:
        raise RuntimeError(
            f"Command failed rc={result.returncode}: {' '.join(command)}\n"
            + "\n".join(log.read_text(errors="replace").splitlines()[-30:])
        )


def read_lock_manifest(root: Path):
    candidates = []
    for path in root.rglob("dataset_lock_v1.json"):
        if sha256_file(path) == MANIFEST_SHA:
            candidates.append(path.read_bytes())
    for path in root.rglob("*LOCK*.zip"):
        if not path.is_file():
            continue
        with zipfile.ZipFile(path) as z:
            if "dataset_lock_v1.json" in z.namelist():
                raw = z.read("dataset_lock_v1.json")
                if sha256_bytes(raw) == MANIFEST_SHA:
                    candidates.append(raw)
    if not candidates:
        raise RuntimeError(
            "Missing LOCKED split manifest. Kaggle Add Input: VCM_BDD100K_LOCK_v1 "
            "(dataset_lock_v1.json or VCM_BDD100K_LOCK_v1.zip). "
            "Never select an arbitrary first MOV; it may belong to held-out Test."
        )
    assert all(a == candidates[0] for a in candidates), "Conflicting split manifests"
    data = json.loads(candidates[0])
    splits = data["splits"]
    if (data["status"] != "LOCKED" or
            [len(splits[s]) for s in ("train", "val", "test")] != [65, 7, 100] or
            len(set(splits["train"] + splits["val"] + splits["test"])) != 172):
        raise ValueError("Manifest split is not the approved 65/7/100 lock")
    records = {r["video_id"]: r for r in data["video_records"]}
    return splits, records


def read_baseline_config(root: Path):
    candidates = []
    for path in root.rglob("baseline_config.lock.json"):
        record = json.loads(path.read_text())
        if record.get("schema") == "vvc-anchor-configuration-lock/1" and record.get("status") == "LOCKED":
            candidates.append(record)
    for path in root.rglob("VVenC_Pilot_v1_2_QA_LOCKED_SNAPSHOT.zip"):
        if not path.is_file():
            continue
        with zipfile.ZipFile(path) as z:
            if "baseline_config.lock.json" in z.namelist():
                record = json.loads(z.read("baseline_config.lock.json"))
                if record.get("status") == "LOCKED":
                    candidates.append(record)
    if not candidates:
        raise RuntimeError(
            "Missing validated Baseline protocol lock. Kaggle Add Input "
            "VVenC_Pilot_v1_2_QA_LOCKED_SNAPSHOT (ZIP or extracted files). "
            "Without it we cannot verify frozen YOLOv5n weights/protocol."
        )
    hashes = {x["lock_sha256"] for x in candidates}
    if len(hashes) != 1:
        raise RuntimeError("Conflicting Baseline locks")
    locked = candidates[0]
    assert locked["source_manifest_sha256"] == MANIFEST_SHA
    cfg = locked["config"]
    assert cfg["detector"]["tag"] == "v7.0"
    assert cfg["detector"]["class_id"] == CAR_CLASS
    assert cfg["detector"]["conf"] == 0.25
    assert cfg["detector"]["nms_iou"] == 0.45
    assert cfg["detector"]["match_iou"] == 0.5
    assert cfg["encoder"]["tag"] == "v1.14.0"
    assert cfg["decoder"]["tag"] == "v3.2.0"
    assert cfg["encoder"]["gop_size_requested"] == 32
    assert cfg["encoder"]["lookahead_enabled_requested"] == 1
    assert cfg["encoder"]["mt_profile_requested"] == -1
    return locked


def dataset_source(records: dict, vid: str):
    roots = [DATASET_HINT]
    for root in Path("/kaggle/input").rglob("bdd100k_videos_train_00"):
        roots.append(root.parent)
    rel = records[vid]["relative_path"]
    for root in roots:
        path = root / rel
        if path.is_file():
            if path.stat().st_size != records[vid]["size_bytes"]:
                raise RuntimeError("Train MOV size differs from locked manifest: " + str(path))
            expected = records[vid].get("sha256")
            if expected and sha256_file(path) != expected:
                raise RuntimeError("Train MOV hash differs from locked manifest: " + str(path))
            return path
    raise FileNotFoundError("Locked Train MOV not present in Kaggle Add Input: " + rel)


def preprocess_mov(source: Path, output: Path, frames: int, log: Path):
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=codec_name,avg_frame_rate,nb_frames,"
         "color_range,color_space,color_transfer,color_primaries,side_data_list",
         "-of", "json", str(source)],
        check=True, text=True, capture_output=True,
    )
    meta = json.loads(probe.stdout)
    if not meta.get("streams"):
        raise ValueError("Source has no video track")
    stream = meta["streams"][0]
    props = {
        "matrix": (stream.get("color_space"), "bt709", SPACE_MAP),
        "primaries": (stream.get("color_primaries"), "bt709", PRIMARIES_MAP),
        "transfer": (stream.get("color_transfer"), "bt709", TRC_MAP),
        "range": (stream.get("color_range"), "tv", {"tv": "tv", "pc": "pc"}),
    }
    colour = {}
    fallbacks = {}
    for name, (value, fallback, allowed) in props.items():
        if value in UNSPECIFIED:
            fallbacks[name] = fallback
            value = fallback
        if value not in allowed:
            raise ValueError(f"Unsupported source metadata {name}={value!r}")
        colour[name] = allowed[value]
    cs = (
        f"colorspace=ispace={colour['matrix']}:"
        f"iprimaries={colour['primaries']}:itrc={colour['transfer']}:"
        f"irange={colour['range']}:space=bt709:primaries=bt709:"
        "trc=bt709:range=tv:format=yuv420p"
    )
    vf = f"fps={FPS},{cs},scale={W}:{H}:flags=bicubic,format=yuv420p"
    run_cmd(
        ["ffmpeg", "-hide_banner", "-nostdin", "-loglevel", "error", "-y",
         "-i", str(source), "-an", "-sn", "-vf", vf, "-pix_fmt", "yuv420p",
         "-fps_mode", "passthrough", "-frames:v", str(frames),
         "-f", "rawvideo", str(output)],
        log, timeout=300,
    )
    assert output.is_file() and output.stat().st_size == frames * FBY, (
        f"Preprocessing frame count/format mismatch: expected {frames*FBY} bytes"
    )
    return {"metadata": stream, "color_input": colour, "fallbacks": fallbacks, "ffmpeg_vf": vf}


def encode_with_ppo(
    repo: Path, encoder: Path, source_yuv: Path, out: Path, frames: int,
    policy_checkpoint: Path | None = None, policy_seed: int = 7,
):
    import torch
    out.mkdir(parents=True, exist_ok=False)
    action_dir = out / "actions"
    state_file = out / "states.txt"
    trajectory = out / "trajectory.jsonl"
    ready = out / "ready.txt"
    bridge_log = (out / "bridge.log").open("w", encoding="utf-8")
    bridge_cmd = [
        sys.executable, "-u", str(repo / "scripts/rl/ppo_policy_bridge.py"),
        "--state-file", str(state_file), "--action-dir", str(action_dir),
        "--trajectory-file", str(trajectory), "--frames", str(frames),
        "--policy", "ppo", "--seed", str(policy_seed),
        "--idle-timeout", "600", "--ready-file", str(ready)
    ]
    if policy_checkpoint is not None:
        bridge_cmd.extend(["--checkpoint", str(policy_checkpoint)])
    bridge = subprocess.Popen(bridge_cmd, stdout=bridge_log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 60
        while not ready.exists() and bridge.poll() is None:
            if time.monotonic() >= deadline:
                raise TimeoutError("PPO bridge startup timed out, inspect bridge.log")
            time.sleep(0.05)
        if not ready.exists():
            raise RuntimeError("PPO bridge quit early: " + (out / "bridge.log").read_text(errors="replace"))

        env = dict(os.environ)
        for key in ("VVENC_RL_QP_MAP", "VVENC_RL_ACTION_DIR", "VVENC_RL_STATE_FILE", "VVENC_RL_AU_BITS_FILE"):
            env.pop(key, None)
        env["VVENC_RL_ACTION_DIR"] = str(action_dir)
        env["VVENC_RL_STATE_FILE"] = str(state_file)
        env["VVENC_RL_AU_BITS_FILE"] = str(out / "au_bits.csv")
        stream = out / "ppo.266"

        cmd = [
            str(encoder), "-i", str(source_yuv), "-s", f"{W}x{H}",
            "-r", str(FPS), "-f", str(frames), "-c", "yuv420",
            "--preset", "medium", "--internal-bitdepth", "10",
            "--bitrate", f"{BITRATE_KBPS}k", "--passes", "1", "--qpa", "1",
            "--intraperiod", "32", "--refreshtype", "cra",
            "--sdr", "sdr_709", "--threads", "4", "--mtprofile", "-1",
            "--additional", "GOPSize=32:LookAhead=1", "-o", str(stream),
        ]
        run_cmd(cmd, out / "encoder.log", env=env, timeout=1200)
        status = bridge.wait(timeout=60)
        if status:
            raise RuntimeError("PPO bridge returned nonzero: " + (out / "bridge.log").read_text(errors="replace")[-2000:])
        au = read_au_bits(out / "au_bits.csv", stream, frames)
        with trajectory.open("r", encoding="utf-8") as f:
            records = [json.loads(s) for s in f if s.strip()]
        assert len(records) == frames and len({r["poc"] for r in records}) == frames
        assert {r["poc"] for r in records} == set(range(frames))
        assert any(any(v != 0 for v in r["discrete_action"]) for r in records), "PPO produced zero actions only"
        assert au[-1].cumulative_bits == stream.stat().st_size * 8
        return stream, records, au
    finally:
        if bridge.poll() is None:
            bridge.kill()
            bridge.wait()
        bridge_log.close()


def convert_10bit_to_8bit(path10: Path, path8: Path, frames: int):
    if path10.stat().st_size != frames * FB10:
        raise RuntimeError(
            f"Expected {frames} 10-bit decoded frames ({frames * FB10} bytes), "
            f"got {path10.stat().st_size}"
        )
    with path10.open("rb") as inp, path8.open("wb") as out:
        for _ in range(frames):
            raw = inp.read(FB10)
            if len(raw) != FB10:
                raise RuntimeError("Decoded YUV10 truncated")
            pixels = np.frombuffer(raw, dtype="<u2").astype(np.uint32)
            if pixels.max() > 1023:
                raise RuntimeError("Invalid 10-bit sample values")
            out.write(np.minimum(255, (pixels + 2) // 4).astype(np.uint8).tobytes())


def temporal_alignment_check(ref8: Path, decoded10: Path, frames: int):
    if frames < 5:
        return {"checked": False}
    samples = sorted(set(max(1, min(frames - 2, x)) for x in
                         (frames // 4, frames // 2, 3 * frames // 4)))
    suspects = 0
    values = []
    with ref8.open("rb") as a, decoded10.open("rb") as b:
        for frame in samples:
            b.seek(frame * FB10)
            y = np.frombuffer(b.read(W * H * 2), dtype="<u2").astype(np.float64)
            by_shift = {}
            for shift in (-1, 0, 1):
                a.seek((frame + shift) * FBY)
                yy = np.frombuffer(a.read(W * H), dtype=np.uint8).astype(np.float64) * 4
                by_shift[shift] = float(np.mean((yy - y) ** 2))
            if min(by_shift[-1], by_shift[1]) < 0.75 * by_shift[0]:
                suspects += 1
            values.append({"frame": frame, "mse_minus1": by_shift[-1],
                           "mse_0": by_shift[0], "mse_plus1": by_shift[1]})
    if suspects >= 2:
        raise RuntimeError("Possible VVdeC frame misalignment, inspect frame/POC mapping")
    return {"checked": True, "sampled": values, "suspect_count": suspects}


def rgb_batches_yuv8(path: Path, frames: int, batch_size: int):
    vf = (
        "scale=in_range=tv:out_range=pc:"
        "in_color_matrix=bt709:out_color_matrix=bt709,format=rgb24"
    )
    p = subprocess.Popen(
        ["ffmpeg", "-hide_banner", "-nostdin", "-loglevel", "error",
         "-f", "rawvideo", "-pixel_format", "yuv420p",
         "-video_size", f"{W}x{H}", "-framerate", str(FPS),
         "-i", str(path), "-vf", vf, "-pix_fmt", "rgb24",
         "-f", "rawvideo", "-"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        bufsize=4 * W * H * 3,
    )
    counter = 0
    try:
        while counter < frames:
            batch = []
            for _ in range(min(batch_size, frames - counter)):
                raw = bytearray()
                while len(raw) < W * H * 3:
                    part = p.stdout.read(W * H * 3 - len(raw))
                    if not part:
                        raise RuntimeError("FFmpeg RGB conversion truncated")
                    raw.extend(part)
                batch.append(np.frombuffer(bytes(raw), np.uint8).reshape(H, W, 3))
            counter += len(batch)
            yield batch
        if p.stdout.read(1):
            raise RuntimeError("FFmpeg RGB conversion emitted extra frames")
        if p.wait() != 0:
            raise RuntimeError("FFmpeg RGB conversion exited with failure")
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()


def load_frozen_yolov5(repo: Path, lock):
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError("Turn ON Kaggle GPU T4x2, restart Session and rerun; CPU is for earlier unit tests")
    torch.manual_seed(0)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    yolodir = repo.parent / "yolov5_v7_locked"
    if not (yolodir / ".git").is_dir():
        subprocess.run(["git", "clone", "--branch", YOLO_REF, "--depth", "1",
                        "https://github.com/ultralytics/yolov5.git",
                        str(yolodir)], check=True)
    got = subprocess.check_output(["git", "-C", str(yolodir),
                                   "rev-parse", "HEAD"], text=True).strip()
    expected_commit = lock["config"]["detector"]["commit"]
    if got != expected_commit:
        raise RuntimeError(f"YOLO commit {got} differs from locked {expected_commit}")
    weights = repo.parent / "yolov5n_v7_locked.pt"
    if not weights.is_file():
        urllib.request.urlretrieve(
            "https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.pt",
            weights,
        )
    got_hash = sha256_file(weights)
    expected_hash = lock["config"]["detector"]["weights_sha256"]
    if got_hash != expected_hash:
        raise RuntimeError(f"YOLO weights SHA256 {got_hash} differs from Baseline {expected_hash}")

    original = torch.load
    try:
        if "weights_only" in inspect.signature(original).parameters:
            def trusted_load(*args, **kwargs):
                kwargs.setdefault("weights_only", False)
                return original(*args, **kwargs)
            torch.load = trusted_load
        detector = torch.hub.load(
            str(yolodir), "custom", path=str(weights),
            source="local", device="cuda:0", autoshape=True,
        )
    finally:
        torch.load = original
    detector.eval()
    for parameter in detector.parameters():
        parameter.requires_grad_(False)
    detector.conf = 0.25
    detector.iou = 0.45
    detector.classes = [2]
    detector.max_det = 300
    detector.amp = False
    if detector.names[CAR_CLASS] != "car":
        raise RuntimeError("Model COCO class_id=2 is not car")
    return detector, {"commit": got, "weights_sha256": got_hash, "device": "cuda:0"}


def detect_all(model, file: Path, frames: int, video_id: str, output: Path):
    import torch
    n = 0
    cars = 0
    with output.open("w", encoding="utf-8") as f:
        for batch in rgb_batches_yuv8(file, frames, batch_size=8):
            with torch.inference_mode():
                result = model(batch, size=640)
            for detections in result.xyxy:
                pred = detections.detach().cpu().numpy()
                boxes = []
                for row in pred:
                    if int(row[5]) != CAR_CLASS:
                        continue
                    boxes.append({
                        "x1": round(float(row[0]), 5),
                        "y1": round(float(row[1]), 5),
                        "x2": round(float(row[2]), 5),
                        "y2": round(float(row[3]), 5),
                        "confidence": round(float(row[4]), 5),
                        "class_id": CAR_CLASS,
                    })
                f.write(json.dumps({
                    "video_id": video_id,
                    "poc": n,
                    "detections": boxes,
                }, separators=(",", ":"), allow_nan=False) + "\n")
                cars += len(boxes)
                n += 1
    if n != frames:
        raise RuntimeError(f"Detector generated {n} detections frames, expected {frames}")
    return cars


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--encoder", type=Path)
    parser.add_argument("--decoder", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=64)
    parser.add_argument("--train-index", type=int, default=0)
    parser.add_argument("--beta", type=float, default=0.1,
                        help="Pilot only; beta must later be chosen on Validation")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--policy-checkpoint", type=Path,
                        help="Previously updated PPO checkpoint for a NEW on-policy episode")
    parser.add_argument("--policy-seed", type=int, default=7,
                        help="Sampling RNG seed; model weights must come from checkpoint")
    args = parser.parse_args()

    if args.frames < 32 or args.frames > 512:
        parser.error("Pilot frames must be between 32 and 512")
    if not 0 <= args.train_index < 65:
        parser.error("Train index must be in [0,64]")
    if args.out.exists():
        parser.error("Output directory already exists; use a unique path")
    if not np.isfinite(args.beta) or args.beta < 0:
        parser.error("Beta must be finite and nonnegative")
    if args.policy_seed < 0:
        parser.error("Policy seed must be nonnegative")
    if args.policy_checkpoint and not args.policy_checkpoint.is_file():
        parser.error("Policy checkpoint not found: " + str(args.policy_checkpoint))
    args.encoder = args.encoder or args.repo / "bin/release-static/vvencapp"
    for path in (args.encoder, args.decoder):
        if not path.is_file():
            parser.error("Missing executable: " + str(path))
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError("Enable Kaggle GPU (T4x2) for the frozen YOLOv5n real-video gate")
    root = Path("/kaggle/input")
    splits, records = read_lock_manifest(root)
    lock = read_baseline_config(root)
    vid = splits["train"][args.train_index]
    source_mov = dataset_source(records, vid)

    args.out.mkdir(parents=True)
    source_yuv = args.out / "reference_8bit.yuv"
    preprocess = preprocess_mov(source_mov, source_yuv,
                                args.frames, args.out / "ffmpeg_source.log")
    stream, observations, au = encode_with_ppo(
        args.repo, args.encoder, source_yuv, args.out / "online",
        args.frames, policy_checkpoint=args.policy_checkpoint,
        policy_seed=args.policy_seed,
    )
    decode = args.out / "decoded_10bit.yuv"
    run_cmd([str(args.decoder), "-b", str(stream), "-o", str(decode),
             "--threads", "4"], args.out / "decoder.log", timeout=600)
    rec8 = args.out / "decoded_8bit.yuv"
    convert_10bit_to_8bit(decode, rec8, args.frames)
    alignment = temporal_alignment_check(source_yuv, decode, args.frames)

    detector, provenance = load_frozen_yolov5(args.repo, lock)
    source_detections = args.out / "source_detections.jsonl"
    reconstructed_detections = args.out / "decoded_detections.jsonl"
    cars_source = detect_all(detector, source_yuv, args.frames, vid, source_detections)
    cars_reconstructed = detect_all(detector, rec8, args.frames, vid, reconstructed_detections)
    joined = join_episode(
        video_id=vid,
        trajectory_file=args.out / "online" / "trajectory.jsonl",
        au_csv=args.out / "online" / "au_bits.csv",
        bitstream=stream,
        source_detections=source_detections,
        decoded_detections=reconstructed_detections,
        frame_rate=float(FPS),
        target_bps=BITRATE_KBPS * 1000.0,
        beta=args.beta,
        iou_threshold=0.5,
    )
    joined_path = args.out / "joined_reward_ppo.jsonl"
    with joined_path.open("w", encoding="utf-8") as f:
        for r in joined:
            f.write(json.dumps(r, allow_nan=False, separators=(",", ":")) + "\n")

    total_bitstream_bits = stream.stat().st_size * 8
    total_real_kbps = total_bitstream_bits * FPS / args.frames / 1000.0
    stats = {
        "purpose": "TRAIN_ONLY_REAL_REWARD_VALIDATION_NOT_TRAINING",
        "video_id": vid, "split": "train", "train_index": args.train_index,
        "frames": args.frames,
        "target_kbps": BITRATE_KBPS, "actual_kbps": total_real_kbps,
        "bits": total_bitstream_bits,
        "au_sum_matches_stream": au[-1].cumulative_bits == total_bitstream_bits,
        "pseudo_source_boxes": cars_source,
        "decoded_boxes": cars_reconstructed,
        "mean_f1_reward_term": sum(r["reward_diagnostics"]["detection_term"] for r in joined)/len(joined),
        "mean_reward": sum(r["reward"] for r in joined)/len(joined),
        "total_tp": sum(r["reward_diagnostics"]["counts"]["tp"] for r in joined),
        "total_fp": sum(r["reward_diagnostics"]["counts"]["fp"] for r in joined),
        "total_fn": sum(r["reward_diagnostics"]["counts"]["fn"] for r in joined),
        "decision_order_first_8_pocs": [r["poc"] for r in joined[:8]],
        "au_output_first_8_pocs": [r.poc for r in au[:8]],
        "preprocess": preprocess, "detector": provenance,
        "baseline_lock_sha256": lock["lock_sha256"],
        "alignment_screen": alignment,
        "joined_path": str(joined_path),
        "ppo_policy_training": False,
        "policy_checkpoint_sha256": (sha256_file(args.policy_checkpoint)
                                    if args.policy_checkpoint else None),
        "policy_checkpoint": (str(args.policy_checkpoint)
                              if args.policy_checkpoint else None),
        "policy_bridge_seed": args.policy_seed,
        "validation_status": ("UPDATED_POLICY_REWARD_DATA_COLLECTION"
                              if args.policy_checkpoint else
                              "UNTRAINED_POLICY_REWARD_DATA_COLLECTION"),
    }
    (args.out / "summary.json").write_text(
        json.dumps(stats, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8"
    )
    if cars_source == 0:
        print("WARNING: selected Train clip has no reference cars; choose another train index for a meaningful detection pilot")
    print(json.dumps({
        "video_id": vid, "frames": args.frames,
        "source_boxes": cars_source,
        "decoded_boxes": cars_reconstructed,
        "actual_kbps": round(total_real_kbps, 4),
        "tp": stats["total_tp"], "fp": stats["total_fp"], "fn": stats["total_fn"],
        "mean_reward": round(stats["mean_reward"], 6),
        "output": str(args.out), "ppo_weights_updated": False,
        "collector_checkpoint_sha256": stats["policy_checkpoint_sha256"],
    }, indent=2))
    print("REAL_REWARD_PILOT_PASS: detector and AU bits joined by locked Train video ID + POC")
    print("NOT TRAINED: no PPO parameter updates occurred")


if __name__ == "__main__":
    main()
