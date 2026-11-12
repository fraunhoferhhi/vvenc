"""Paired locked BDD100K full/Test100 or mini/Test30 evaluation with verified resume."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np

from .data import ASSETS, FrozenDetector, VideoCatalog, atomic_json, digest, source_fingerprint
from .environment import VVenCEnv
from .method import FRAME_BYTES
from .model import AuxPPO
from .artifacts import RUN_SCHEMA,policy_digest,import_run,export_evaluation

class EvaluationStop(Exception):pass


def verify_decoded(stream, trajectory, frames, ffmpeg, decoder=None):
    destination = stream.with_suffix(".decoded.yuv")
    if decoder:
        command = [str(decoder), "-b", str(stream), "-o", str(destination), "--threads", "2"]
    else:
        command = [ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "error", "-y", "-i", str(stream),
                   "-pix_fmt", "yuv420p10le", "-fps_mode", "passthrough", "-f", "rawvideo", str(destination)]
    subprocess.run(command, check=True, stdout=subprocess.DEVNULL, timeout=1200)
    with gzip.open(trajectory, "rt", encoding="utf-8") as source:
        rows = [json.loads(line) for line in source]
    hashes = {row["poc"]: row["reconstruction_sha256"] for row in rows}
    if len(rows) != frames or set(hashes) != set(range(frames)) or destination.stat().st_size != frames * FRAME_BYTES * 2:
        raise ValueError("Decoded output and scored reconstruction POCs differ")
    with destination.open("rb") as decoded:
        for poc in range(frames):
            if hashlib.sha256(decoded.read(FRAME_BYTES * 2)).hexdigest() != hashes[poc]:
                raise ValueError(f"Actual decoded frame differs from the scored frame at POC {poc}")
    destination.unlink()


def run_pair_method(model, catalog, detector, args, root, video_id, rate, name, identity):
    point = root / f"{name}.json"
    if point.exists():
        saved = json.loads(point.read_text())
        if saved["identity"] != identity:
            raise ValueError("Completed evaluation point belongs to a different experiment")
        for evidence in saved["evidence"]:
            if digest(root / evidence["path"]) != evidence["sha256"]:
                raise ValueError("Completed evaluation evidence has changed")
        return saved
    env = VVenCEnv(catalog, detector, args.encoder, root / name, split="test", frame_limit=args.frames,
                   threads=args.encoder_threads, fixed_video=video_id, fixed_rate=rate,
                   mode="anchor" if name == "vvenc" else "live")
    start = time.monotonic()
    try:
        obs, _ = env.reset(seed=args.seed)
        done = False
        while not done:
            if time.monotonic()>=args.deadline:raise EvaluationStop("Soft evaluation budget reached")
            action = np.zeros(35) if name == "vvenc" else model.predict(obs, deterministic=True)[0]
            obs, _, done, _, info = env.step(action)
        env.trajectory.close()
        env.trajectory = None
        trajectory = env.native.output.parent / "trajectory.jsonl.gz"
        stream = root / f"{name}.266"
        shutil.copyfile(env.native.output / "stream.266", stream)
        verify_decoded(stream, trajectory, env.frames, args.ffmpeg, args.decoder)
        result = {"identity": identity, "video_id": video_id, "target_kbps": rate, "method": name,
                  "decoded_reconstruction_verified": True, "elapsed_seconds": time.monotonic() - start,
                  **info["episode"], "evidence": []}
        for path in (stream, trajectory):
            result["evidence"].append({"path": path.relative_to(root).as_posix(), "sha256": digest(path)})
        atomic_json(point, result)
        return result
    finally:
        env._close_episode()


def bd_metrics(anchor, agent):
    from scipy.interpolate import PchipInterpolator

    def frontier(points):
        best={}
        for p in points:
            rate,quality=p["actual_kbps"],p["mean_precision"]
            if not np.isfinite([rate,quality]).all() or rate<=0 or not 0<=quality<=1:
                raise ValueError("Invalid RD point")
            best[rate]=max(best.get(rate,-1.),quality)
        ordered = sorted(best.items())
        result = []
        for rate, quality in ordered:
            if not result or quality > result[-1][1]:
                result.append((rate, quality))
        return np.asarray(result)

    a, b = frontier(anchor), frontier(agent)
    if len(a) < 2 or len(b) < 2:
        return {"bd_rate_percent": None, "bd_precision_points": None, "reason": "insufficient monotone points"}
    low, high = max(a[0, 1], b[0, 1]), min(a[-1, 1], b[-1, 1])
    rate = None
    if high > low:
        first = PchipInterpolator(a[:, 1], np.log(a[:, 0]))
        second = PchipInterpolator(b[:, 1], np.log(b[:, 0]))
        rate = 100 * np.expm1((second.integrate(low, high) - first.integrate(low, high)) / (high - low))
    low_rate, high_rate = max(a[0, 0], b[0, 0]), min(a[-1, 0], b[-1, 0])
    quality = None
    if high_rate > low_rate:
        lo, hi = np.log(low_rate), np.log(high_rate)
        first = PchipInterpolator(np.log(a[:, 0]), a[:, 1])
        second = PchipInterpolator(np.log(b[:, 0]), b[:, 1])
        quality = 100 * (second.integrate(lo, hi) - first.integrate(lo, hi)) / (hi - lo)
    return {"bd_rate_percent": float(rate) if rate is not None else None,
            "bd_precision_points": float(quality) if quality is not None else None}


def report(output, pairs, rates=(50,80,120,160,200)):
    import csv
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    by_video = {}
    for pair in pairs:
        by_video.setdefault(pair[0]["video_id"], []).append(pair)
    rows = []
    all_complete_pairs=[]
    for video_id, values in by_video.items():
        if len(values)!=len(rates) or {p[0]["target_kbps"] for p in values}!=set(rates):
            continue
        anchor,agent=zip(*values)
        all_complete_pairs.extend(values)
        if len(rates)==5:
            metrics=bd_metrics(anchor,agent)
        else:
            metrics={"bd_rate_percent":None,"bd_precision_points":None,
                     "reason":"Mini 3-point curve: BD-rate reserved for full 5-rate evaluation"}
        rows.append({"video_id":video_id,**metrics})
        fig, axis = plt.subplots()
        for points, label in ((anchor, "VVenC (matched CTU32)"), (agent, "VVenC + RL-RC-DoT")):
            points = sorted(points, key=lambda point: point["actual_kbps"])
            axis.plot([p["actual_kbps"] for p in points], [p["mean_precision"] for p in points], "o-", label=label)
        axis.set(xlabel="Actual bitrate (kbps)", ylabel="Mean frame Precision", title=video_id)
        axis.legend()
        fig.tight_layout()
        fig.savefig(output / f"{video_id}_rd.png", dpi=160)
        plt.close(fig)
    with (output / "bd_per_video.csv").open("w", newline="", encoding="utf-8") as out:
        writer = csv.DictWriter(out, fieldnames=["video_id", "bd_rate_percent", "bd_precision_points", "reason"])
        writer.writeheader()
        writer.writerows(rows)
    valid=[row["bd_rate_percent"] for row in rows if row["bd_rate_percent"] is not None]
    def macro(name, key):
        return float(np.mean([agent[key]-anchor[key] for anchor,agent in all_complete_pairs])) if all_complete_pairs else None
    rate_ratios=[agent["actual_kbps"]/anchor["actual_kbps"]-1
                 for anchor,agent in all_complete_pairs if anchor["actual_kbps"]>0]
    win_rates=[agent["mean_precision"]>anchor["mean_precision"] for anchor,agent in all_complete_pairs]
    atomic_json(output/"summary.json",{
        "completed_pairs":len(pairs),"complete_videos":len(rows),
        "rates_kbps":list(rates),"completed_comparable_pairs":len(all_complete_pairs),
        "mean_delta_precision":macro("agent","mean_precision"),
        "mean_delta_recall":macro("agent","mean_recall"),
        "mean_delta_f1":macro("agent","f1"),
        "mean_rate_ratio_vs_anchor":float(np.mean(rate_ratios)) if rate_ratios else None,
        "precision_win_fraction":float(np.mean(win_rates)) if win_rates else None,
        "bd_rate_valid_videos":len(valid),
        "mean_bd_rate_percent":float(np.mean(valid)) if valid else None,
        "bd_rate_policy":"5 bitrate points only; mini 3-rate runs report paired metrics, no BD-rate claim",
        "anchor_profile":"matched CTU32/DQP2/FPP0 research profile; not stock CTU128",
        "curve_rule":"Pareto frontier sorted by actual bitrate; PCHIP on overlap only"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("input-root", "encoder", "yolo-repo", "weights", "checkpoint", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--decoder", type=Path,required=True)
    parser.add_argument("--resume-from",type=Path)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--dataset-mode",choices=("mini","full"),default="full")
    parser.add_argument("--rates",default="50,80,120,160,200",
                        help="Mini: 50,120,200; Full: 50,80,120,160,200")
    parser.add_argument("--encoder-threads", type=int, default=2)
    parser.add_argument("--seed", type=int, default=20261111)
    parser.add_argument("--max-pairs", type=int, default=0)
    parser.add_argument("--max-hours", type=float, default=0)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--ffprobe", default="ffprobe")
    args = parser.parse_args()
    if args.frames<0 or args.max_pairs<0 or args.max_hours<0 or args.encoder_threads<1:
        parser.error("Invalid evaluation budget")
    try:
        args.rate_list=[int(s.strip()) for s in args.rates.split(",")]
    except ValueError:
        parser.error("Invalid bitrate list")
    if (not args.rate_list or len(set(args.rate_list))!=len(args.rate_list)
            or any(x not in (50,80,120,160,200) for x in args.rate_list)):
        parser.error("Evaluation rate list must be a unique subset of 50,80,120,160,200")
    if args.dataset_mode=="mini" and (args.frames!=256 or args.rate_list!=[50,120,200]):
        parser.error("Mini Test30 requires 256 frames and 50,120,200 kbps")
    if args.dataset_mode=="full" and len(args.rate_list)!=5:
        parser.error("Full Test100 requires all five bitrate points")
    manifest_path=args.checkpoint.parent/"manifest.json"
    method_path=args.checkpoint.parents[2]/"method.json"
    sealed=json.loads(manifest_path.read_text());method=json.loads(method_path.read_text())
    if (sealed.get("schema")!=RUN_SCHEMA or digest(args.checkpoint)!=sealed["files"]["model.zip"]
            or sealed["method_sha256"]!=hashlib.sha256(json.dumps(method,sort_keys=True).encode()).hexdigest()
            or method["encoder_sha256"]!=digest(args.encoder) or method["source_fingerprint"]!=source_fingerprint()
            or method["dataset_sha256"]!=digest(ASSETS/"dataset_lock.json") or method["encoder_threads"]!=args.encoder_threads
            or method["detector_weights_sha256"]!=digest(args.weights)
            or method.get("dataset_mode","full")!=args.dataset_mode):
        raise ValueError("Evaluation checkpoint/source/encoder/detector does not match the training contract")
    if args.resume_from:import_run(args.resume_from,args.output)
    args.output.mkdir(parents=True, exist_ok=True)
    identity = {"schema": "vvenc-rcdot-evaluation/2", "checkpoint_sha256": digest(args.checkpoint),
                "encoder_sha256": digest(args.encoder), "source_fingerprint": source_fingerprint(),
                "dataset_sha256": digest(ASSETS / "dataset_lock.json"), "frames": args.frames,
                "dataset_mode":args.dataset_mode,"rates":args.rate_list,
                "decoder_sha256": digest(args.decoder) if args.decoder else None,
                "detector_device": args.device, "encoder_threads": args.encoder_threads,
                "seed":args.seed,"profile":"ctu32_dqp2_fpp0","method":method}
    config = args.output / "evaluation.json"
    if config.exists() and json.loads(config.read_text()) != identity:
        raise ValueError("Output contains a different evaluation configuration")
    atomic_json(config, identity)
    model = AuxPPO.load(args.checkpoint, device="cpu")
    model.assert_checkpoint_finite()
    if policy_digest(model)!=sealed["policy_sha256"]:raise ValueError("Loaded evaluation policy differs")
    catalog = VideoCatalog(args.input_root, args.ffmpeg, args.ffprobe,
                           mode=args.dataset_mode,seed=args.seed)
    catalog.check_available(("test",))
    print("EVALUATION_DATASET_SELECTION_PASS",json.dumps({"mode":args.dataset_mode,
           "test_videos":catalog.selected["test"],"rates":args.rate_list,"frames":args.frames}),flush=True)
    detector = FrozenDetector(args.yolo_repo, args.weights, args.device)
    deadline = time.monotonic() + args.max_hours * 3600 if args.max_hours else float("inf")
    if args.max_hours:deadline-=min(600,args.max_hours*360)
    args.deadline=deadline
    pairs, added = [], 0
    try:
        for video_id in catalog.selected["test"]:
            for rate in args.rate_list:
                pair_root = args.output / "pairs" / f"{video_id}_{rate}"
                existed = (pair_root / "rl_rc_dot.json").exists() and (pair_root / "vvenc.json").exists()
                if not existed and (time.monotonic() >= deadline or (args.max_pairs and added >= args.max_pairs)):
                    continue
                pair_root.mkdir(parents=True, exist_ok=True)
                results = [run_pair_method(model, catalog, detector, args, pair_root, video_id, rate, name, identity)
                           for name in ("vvenc", "rl_rc_dot")]
                pairs.append(results)
                added += not existed
                print("TEST_PAIR", video_id, rate, "completed", len(pairs), flush=True)
    except EvaluationStop as error:
        print("EVALUATION_SOFT_STOP",str(error),flush=True)
    finally:
        detector.close()
        report(args.output,pairs,args.rate_list)
        print("EVALUATION_EXPORT",json.dumps(export_evaluation(args.output,args.output/"exports"/f"RL_RC_DOT_EVAL_{time.time_ns()}.zip")),flush=True)


if __name__ == "__main__":
    main()
