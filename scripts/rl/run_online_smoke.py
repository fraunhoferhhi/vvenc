#!/usr/bin/env python3
"""Kaggle smoke test: online PPO policy bridge coupled to VVenC encoder.

Only one short YUV clip. No YOLO rewards, no model updates, no BDD100K Test.
Stage-1 bitstream parity and the online PPO action transport are tested
separately. An untrained PPO policy is used ONLY to verify action delivery.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from au_rate_audit import read_au_bits


def run_case(phase, args, work):
    case = work / phase
    if case.exists():
        raise RuntimeError(f"Refusing to reuse stale episode directory {case}")
    case.mkdir(parents=True)
    bitstream = case / f"{phase}.266"
    trace = case / "encoder.log"
    encoder_env = dict(os.environ)
    for key in ("VVENC_RL_QP_MAP", "VVENC_RL_ACTION_DIR",
                "VVENC_RL_STATE_FILE", "VVENC_RL_TRACE", "VVENC_RL_AU_BITS_FILE"):
        encoder_env.pop(key, None)
    au_bits_path = case / "au_bits.csv"
    encoder_env["VVENC_RL_AU_BITS_FILE"] = str(au_bits_path)

    server = None
    if phase != "vanilla":
        states = case / "states.txt"
        action_dir = case / "actions"
        trajectory = case / "trajectory.jsonl"
        ready = case / "ready.txt"
        with (case / "policy.log").open("w", encoding="utf-8") as logger:
            cmd = [
                sys.executable, "-u", str(args.repo / "scripts/rl/ppo_policy_bridge.py"),
                "--state-file", str(states),
                "--action-dir", str(action_dir),
                "--trajectory-file", str(trajectory),
                "--frames", str(args.frames),
                "--policy", "zero" if phase == "zero" else "ppo",
                "--idle-timeout", "180",
                "--seed", "7",
                "--ready-file", str(ready),
            ]
            server = subprocess.Popen(cmd, stdout=logger, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 30
        while not ready.exists() and server.poll() is None:
            if time.monotonic() > deadline:
                server.kill()
                raise RuntimeError(f"Policy bridge never signaled READY; check {case}/policy.log")
            time.sleep(0.05)
        if server.poll() is not None:
            raise RuntimeError(f"Policy bridge exited early; check {case}/policy.log")
        encoder_env["VVENC_RL_STATE_FILE"] = str(states)
        encoder_env["VVENC_RL_ACTION_DIR"] = str(action_dir)
        encoder_env["VVENC_RL_TRACE"] = "1"

    cmd = [
        str(args.encoder), "-i", str(args.video), "-s", "480x320",
        "-r", "30", "-f", str(args.frames),
        "--format", "yuv420", "--preset", "medium",
        "--bitrate", "120000", "--passes", "1", "--qpa", "1",
        "--threads", "4", "--internal-bitdepth", "10",
        "--intraperiod", "32", "--refreshtype", "cra",
        "-o", str(bitstream),
    ]
    try:
        with trace.open("w", encoding="utf-8") as logger:
            completed = subprocess.run(
                cmd, env=encoder_env, stdout=logger, stderr=subprocess.STDOUT,
                timeout=240
            )
        if completed.returncode:
            raise RuntimeError(
                f"{phase} encoder failure: {completed.returncode}\n" +
                "\n".join(trace.read_text(errors="replace").splitlines()[-30:])
            )
        if server is not None:
            try:
                status = server.wait(timeout=30)
            except subprocess.TimeoutExpired:
                server.kill()
                raise RuntimeError(f"{phase} policy bridge still running; inspect policy.log")
            if status:
                raise RuntimeError(
                    f"{phase} bridge failed: {status}\n" +
                    (case / "policy.log").read_text(errors="replace")[-2000:]
                )
    finally:
        if server is not None and server.poll() is None:
            server.kill()
            server.wait()

    if not bitstream.is_file() or not bitstream.stat().st_size:
        raise RuntimeError(f"No {phase} bitstream")
    sha = hashlib.sha256(bitstream.read_bytes()).hexdigest()
    au_records = read_au_bits(au_bits_path, bitstream, args.frames)
    print(
        f"  AU_RATE_ACCOUNTING_PASS mode={phase} frames={len(au_records)} "
        f"total_bits={au_records[-1].cumulative_bits} "
        f"output_order_pocs={[x.poc for x in au_records[:8]]}",
        flush=True
    )

    if args.decoder:
        decoded = case / "decoded.yuv"
        decode_log = case / "decoder.log"
        with decode_log.open("w", encoding="utf-8") as f:
            dec = subprocess.run(
                [str(args.decoder), "-b", str(bitstream), "-o", str(decoded), "-t", "2"],
                stdout=f, stderr=subprocess.STDOUT, timeout=180
            )
        sizes = {args.frames*480*320*3//2, args.frames*480*320*3}
        if dec.returncode or not decoded.is_file() or decoded.stat().st_size not in sizes:
            raise RuntimeError(f"{phase} VVdeC decode failed; see {decode_log}")

    actions = []
    if phase != "vanilla":
        records = [
            json.loads(line)
            for line in (case / "trajectory.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        if len(records) != args.frames or len({r["poc"] for r in records}) != args.frames:
            raise RuntimeError(f"{phase}: expected {args.frames} unique POC observations")
        actions = [r["discrete_action"] for r in records]
        for r in records:
            if len(r["global"]) != 8 or len(r["spatial"]) != 3:
                raise RuntimeError(f"{phase}: invalid state tensor dimensions")
        if phase == "zero" and any(any(v != 0 for v in a) for a in actions):
            raise RuntimeError("Zero-policy returned a nonzero action")

    nonzero = sum(v != 0 for m in actions for v in m)
    print(
        f"{phase.upper()}: bits={bitstream.stat().st_size*8} "
        f"sha={sha[:16]} states={len(actions)} nonzero_actions={nonzero}",
        flush=True
    )
    return sha, nonzero


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", type=Path, default=Path("/kaggle/working/vvenc_rl_stage1"))
    p.add_argument("--encoder", type=Path)
    p.add_argument("--decoder", type=Path)
    p.add_argument("--video", type=Path,
                   default=Path("/kaggle/working/rl_qp_smoke/smoke_480x320.yuv"))
    p.add_argument("--output", type=Path, default=Path("/kaggle/working/rl_agent_smoke"))
    p.add_argument("--frames", type=int, default=16)
    args = p.parse_args()
    args.encoder = args.encoder or args.repo / "bin/release-static/vvencapp"
    for path in (args.encoder, args.video):
        if not path.is_file():
            p.error(f"Missing required file: {path}")
    if args.output.exists():
        p.error(f"Choose a new --output directory; {args.output} already exists")
    args.output.mkdir(parents=True)

    hashes = {}
    nonzero = {}
    for phase in ("vanilla", "zero", "ppo"):
        hashes[phase], nonzero[phase] = run_case(phase, args, args.output)
    checks = {
        "zero action bitstream parity": hashes["zero"] == hashes["vanilla"],
        "PPO supplied nonzero actions": nonzero["ppo"] > 0,
        "PPO actions affected bitstream": hashes["ppo"] != hashes["vanilla"],
    }
    for name, ok in checks.items():
        print(f"{'PASS' if ok else 'FAIL'}: {name}")
    if all(checks.values()):
        print("OVERALL: PASS — PPO inference reached live VVenC QP action hook")
    else:
        print("OVERALL: FAIL — fix policy bridge/encoder before reward or training")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
