#!/usr/bin/env python3
"""Strict independent audit for the untrained VVenC PPO policy bridge.

Read-only audit of run_online_smoke.py outputs: exact POC identity, 12 action
values, original observation lines, file-exchange integrity, final CTU count,
per-CTU delta vs action, QG candidate QP and absence of silent action loss.
Only checks integration; does NOT validate a trained policy or rewards.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys

from au_rate_audit import read_au_bits

FINAL = re.compile(
    r"\[VVENC_RL_CTU_FINAL\] poc=(\d+) ctu=(\d+) x=(\d+) y=(\d+) "
    r"delta_requested=(-?\d+) num_luma_cus=(\d+) min_qp=(-?\d+) max_qp=(-?\d+)"
)
CANDIDATE = re.compile(
    r"\[VVENC_RL_QP\] poc=(\d+) x=(\d+) y=(\d+) "
    r"qp_base=(-?\d+) delta_requested=(-?\d+) qp_applied=(-?\d+) "
    r"subdiv=(\d+) depth=(\d+) cu_w=(\d+) cu_h=(\d+)"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def assert_true(test, reason):
    if not test:
        raise AssertionError(reason)


def audit_case(case: Path, n: int, expected_delta=None):
    au_data = read_au_bits(
        case / "au_bits.csv", case / f"{case.name}.266", n
    )
    assert_true(len({r.poc for r in au_data}) == n, "Duplicate AU POC")
    raw_lines = (case / "states.txt").read_text().splitlines()
    records = [json.loads(s) for s in (case / "trajectory.jsonl").read_text().splitlines()]
    expected_pocs = set(range(n))
    assert_true(len(records) == n, f"{case.name}: {len(records)} records instead of {n}")
    assert_true(len(raw_lines) == n, f"{case.name}: {len(raw_lines)} observations instead of {n}")
    pocs = [int(r["poc"]) for r in records]
    assert_true(set(pocs) == expected_pocs and len(set(pocs)) == n,
                f"{case.name}: missing/duplicated episode POCs")
    state_pocs = []
    for line in raw_lines:
        f = line.split()
        assert_true(len(f) == 46 and f[0] == "STATE",
                    f"{case.name}: malformed state line")
        state_pocs.append(int(f[1]))
        assert_true(all(math.isfinite(float(x)) for x in f[2:]),
                    f"{case.name}: non-finite feature in state")
    assert_true(state_pocs == pocs,
                f"{case.name}: mismatched encoder-state vs trajectory ordering")

    action_by_poc = {}
    for rec in records:
        poc = int(rec["poc"])
        assert_true(len(rec["global"]) == 8 and len(rec["spatial"]) == 3
                    and all(len(chan) == 3 and all(len(row) == 4 for row in chan)
                            for chan in rec["spatial"]),
                    f"{case.name} POC={poc}: bad state tensor shape")
        action = rec["discrete_action"]
        assert_true(len(action) == 12 and all(type(x) is int and -2 <= x <= 2
                                              for x in action),
                    f"{case.name} POC={poc}: bad action map")
        disk = [int(v) for v in (case / "actions" / f"{poc}.txt").read_text().split()]
        assert_true(disk == [poc] + action,
                    f"{case.name} POC={poc}: action file differs from policy trajectory")
        if expected_delta == 0:
            assert_true(action == [0] * 12, "Zero-policy produced non-zero action")
        else:
            assert_true(rec["log_prob"] is not None
                        and rec["value"] is not None
                        and len(rec["raw_action"]) == 12
                        and math.isfinite(rec["log_prob"])
                        and math.isfinite(rec["value"])
                        and all(math.isfinite(x) for x in rec["raw_action"]),
                        f"{case.name} POC={poc}: invalid PPO sample/log_prob/value")
        action_by_poc[poc] = action

    text = (case / "encoder.log").read_text(errors="replace")
    finals = [tuple(map(int, r)) for r in FINAL.findall(text)]
    candidates = [tuple(map(int, r)) for r in CANDIDATE.findall(text)]
    assert_true(len(finals) == n * 12,
                f"{case.name}: expected {n * 12} final CTUs, found {len(finals)}")
    assert_true(len({(r[0], r[1]) for r in finals}) == n * 12,
                f"{case.name}: duplicated final POC/CTU")
    assert_true(len(candidates) > 0, f"{case.name}: no candidate QP logs")
    assert_true({r[0] for r in finals} == expected_pocs,
                f"{case.name}: missing final POC")

    for poc, ctu, x, y, delta, cus, qp_min, qp_max in finals:
        assert_true(0 <= ctu < 12 and 0 <= x < 480 and 0 <= y < 320,
                    f"{case.name}: invalid final CTU coordinates")
        assert_true(ctu == (y // 128) * 4 + (x // 128),
                    f"{case.name}: CTU raster alignment mismatch")
        assert_true(delta == action_by_poc[poc][ctu],
                    f"{case.name}: wrong PPO delta applied to final CTU")
        assert_true(cus > 0 and 0 <= qp_min <= qp_max <= 63,
                    f"{case.name}: invalid encoded CU QP range")

    for poc, x, y, base, delta, applied, subdiv, depth, cw, ch in candidates:
        idx = (y // 128) * 4 + (x // 128)
        assert_true(poc in action_by_poc and 0 <= idx < 12,
                    f"{case.name}: orphan candidate")
        assert_true(delta == action_by_poc[poc][idx],
                    f"{case.name}: candidate received wrong action")
        assert_true(applied == min(63, max(0, base + delta)),
                    f"{case.name}: QP action accumulation or clipping mismatch")

    print(
        f"PASS {case.name}: states={n}, actions={n}, "
        f"final_ctus={len(finals)}, QG_candidates={len(candidates)}, "
        f"actual_bits={au_data[-1].cumulative_bits}"
    )
    return {(poc, ctu): (mn, mx) for poc, ctu, x, y, d, nc, mn, mx in finals}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=16)
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames must be positive")
    root = args.directory
    zero = audit_case(root / "zero", args.frames, expected_delta=0)
    ppo = audit_case(root / "ppo", args.frames)
    hv = sha256(root / "vanilla" / "vanilla.266")
    hz = sha256(root / "zero" / "zero.266")
    hp = sha256(root / "ppo" / "ppo.266")
    assert_true(hv == hz, "Zero-policy bitstream differs from vanilla")
    assert_true(hp != hz, "Untrained PPO policy did not alter the bitstream")
    n_changes = sum(qp != zero.get(key) for key, qp in ppo.items())
    assert_true(n_changes > 0, "No final CTU QP range changed under PPO")
    print(f"PASS: zero bit-exact, PPO different bitstream; {n_changes} selected CTUs changed QP")
    print("OVERALL: PASS — causal state/action delivery and POC/CTU mapping only")
    print("Reward/YOLO/true policy learning NOT covered by this audit")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, ValueError, KeyError, OSError) as exc:
        print(f"OVERALL: FAIL — {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
