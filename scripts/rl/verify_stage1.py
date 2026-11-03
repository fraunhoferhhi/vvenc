#!/usr/bin/env python3
"""End-to-end stage-1 gate for the fixed per-POC VVenC RL-QP action hook.

Runs four encodes, tests zero-action bitstream parity, audits *candidate*
QG traces separately from *selected* CTU traces, and decodes all streams.
No training data or PPO weights are used.
"""
import argparse
from collections import Counter
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys

CANDIDATE = re.compile(
    r"\[VVENC_RL_QP\] poc=(\d+) x=(\d+) y=(\d+) "
    r"qp_base=(-?\d+) delta_requested=(-?\d+) qp_applied=(-?\d+) "
    r"subdiv=(\d+) depth=(\d+) cu_w=(\d+) cu_h=(\d+)"
)
FINAL = re.compile(
    r"\[VVENC_RL_CTU_FINAL\] poc=(\d+) ctu=(\d+) x=(\d+) y=(\d+) "
    r"delta_requested=(-?\d+) num_luma_cus=(\d+) min_qp=(-?\d+) max_qp=(-?\d+)"
)
CASES = {"vanilla": None, "zero": 0, "minus2": -2, "plus2": 2}


def run_checked(command, log_path, env=None):
    with log_path.open("w", encoding="utf-8") as output:
        process = subprocess.run(command, env=env, stdout=output, stderr=subprocess.STDOUT)
    if process.returncode:
        tail = log_path.read_text(errors="replace").splitlines()[-35:]
        raise RuntimeError(
            f"Command returned {process.returncode}: {' '.join(map(str, command))}\n"
            + "\n".join(tail)
        )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--encoder", required=True, type=Path)
    p.add_argument("--decoder", required=True, type=Path)
    p.add_argument("--video", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--frames", type=int, default=16)
    p.add_argument("--width", type=int, default=480)
    p.add_argument("--height", type=int, default=320)
    p.add_argument("--ctu-size", type=int, default=128)
    a = p.parse_args()

    for path in (a.encoder, a.decoder, a.video):
        if not path.is_file():
            p.error(f"Missing: {path}")
    if a.frames < 1 or a.width < 1 or a.height < 1 or a.ctu_size < 1:
        p.error("Invalid frame count or dimensions")

    a.output.mkdir(parents=True, exist_ok=True)
    expected_ctus_per_frame = (
        (a.width + a.ctu_size - 1) // a.ctu_size
        * ((a.height + a.ctu_size - 1) // a.ctu_size)
    )
    expected_ctus = a.frames * expected_ctus_per_frame

    hashes = {}
    qp_final = {}
    errors = []

    for name, delta in CASES.items():
        bitstream = a.output / f"{name}.266"
        log = a.output / f"{name}.log"
        env = dict(os.environ)
        env.pop("VVENC_RL_QP_MAP", None)
        env.pop("VVENC_RL_TRACE", None)

        if delta is not None:
            action_file = a.output / f"{name}_actions.txt"
            with action_file.open("w", encoding="ascii") as file:
                for poc in range(a.frames):
                    file.write(str(poc) + " " + " ".join([str(delta)] * 12) + "\n")
            env["VVENC_RL_QP_MAP"] = str(action_file)
            env["VVENC_RL_TRACE"] = "1"

        command = [
            str(a.encoder), "-i", str(a.video), "-s", f"{a.width}x{a.height}",
            "-r", "30", "-f", str(a.frames), "--format", "yuv420",
            "--preset", "medium", "--bitrate", "120000", "--passes", "1",
            "--qpa", "1", "--threads", "4", "--internal-bitdepth", "10",
            "--intraperiod", "32", "--refreshtype", "cra", "-o", str(bitstream),
        ]
        print(f"ENCODE {name} ...", flush=True)
        run_checked(command, log, env)
        if not bitstream.is_file() or not bitstream.stat().st_size:
            errors.append(f"{name}: empty bitstream")
            continue

        hashes[name] = hashlib.sha256(bitstream.read_bytes()).hexdigest()

        decoded = a.output / f"{name}_decoded.yuv"
        decode_log = a.output / f"{name}_decode.log"
        run_checked([str(a.decoder), "-b", str(bitstream), "-o", str(decoded), "-t", "2"], decode_log)
        valid_sizes = {
            a.frames * a.width * a.height * 3 // 2,
            a.frames * a.width * a.height * 3,
        }
        if not decoded.is_file() or decoded.stat().st_size not in valid_sizes:
            errors.append(f"{name}: decoded size invalid; check frames/bit depth")
        print(f"  decode={'OK' if decoded.is_file() and decoded.stat().st_size in valid_sizes else 'FAIL'}"
              f"  bits={bitstream.stat().st_size * 8}")

        if delta is None:
            continue

        contents = log.read_text(errors="replace")
        candidates = [tuple(map(int, hit)) for hit in CANDIDATE.findall(contents)]
        finals = [tuple(map(int, hit)) for hit in FINAL.findall(contents)]

        if not candidates:
            errors.append(f"{name}: no QP candidate traces (new binary not built?)")
            continue
        if len(finals) != expected_ctus:
            errors.append(f"{name}: selected CTU records {len(finals)} != {expected_ctus}")
        final_keys = [(poc, ctu) for poc, ctu, *_ in finals]
        if len(set(final_keys)) != expected_ctus:
            errors.append(f"{name}: duplicate/missing final CTUs in POC x CTU grid")
        if any(
            poc not in range(a.frames) or ctu not in range(expected_ctus_per_frame)
            or changed != delta or count <= 0 or minimum > maximum
            for poc, ctu, x, y, changed, count, minimum, maximum in finals
        ):
            errors.append(f"{name}: invalid final CTU POC/address/delta/CU QP")
        if any(requested != delta or applied != base + delta for
               poc, x, y, base, requested, applied, subdiv, depth, cw, ch in candidates):
            errors.append(f"{name}: QG action differs from fresh QPA base plus requested delta")
        if any((subdiv == 0) != (depth == 0) for
               poc, x, y, base, requested, applied, subdiv, depth, cw, ch in candidates):
            # Depth and subdiv differ for QT/MTT in certain cases; diagnostic only.
            print(f"  info: nontrivial depth/subdiv mapping in {name}")

        unique_qg = len({(c[0], c[1], c[2]) for c in candidates})
        duplicates = len(candidates) - unique_qg
        sub_events = sum(c[6] > 0 for c in candidates)
        repeat_detail = Counter(c[0] for c in candidates)
        print(
            f"  RDO candidate calls={len(candidates)}, "
            f"unique poc/qg-position={unique_qg}, revisits={duplicates}; "
            f"sub-QG candidates={sub_events}"
        )
        print(
            f"  FINAL CTUs={len(finals)}/{expected_ctus}, "
            f"unique FINAL CTUs={len(set(final_keys))}, "
            f"calls by POC: {sorted(repeat_detail.items())[:5]} ..."
        )
        qp_final[name] = {(poc, ctu): (minimum, maximum)
                          for poc, ctu, x, y, dd, count, minimum, maximum in finals}

    if hashes.get("zero") != hashes.get("vanilla"):
        errors.append("zero-action stream differs from disabled-action stream")
    for name in ("minus2", "plus2"):
        if hashes.get(name) == hashes.get("vanilla"):
            errors.append(f"{name}: action stream has same hash as vanilla")
        if name in qp_final and "zero" in qp_final:
            changed_ctus = sum(
                qp != qp_final["zero"].get(key)
                for key, qp in qp_final[name].items()
            )
            print(f"Final CTUs with QP-range changed from zero: {name}={changed_ctus}")
            if changed_ctus == 0:
                errors.append(f"{name}: no selected CTU QP range changed")

    print("\n=== STAGE-1 QP ACTION GATE ===")
    if errors:
        for error in errors:
            print("FAIL:", error)
        print("OVERALL: FAIL — do not start PPO training")
        return 1

    print("PASS: disabled and zero actions are bit-identical")
    print("PASS: +/-2 candidate actions are consistent with fresh QPA QP")
    print(f"PASS: exactly {expected_ctus} final CTUs per action mode, without duplicate final CTUs")
    print("PASS: all four bitstreams decoded to expected-size YUV files")
    print("PASS: nonzero action affects final encoded CU QP ranges")
    print("OVERALL: PASS — stage-1 action hook is ready for PPO integration")
    return 0


if __name__ == "__main__":
    sys.exit(main())
