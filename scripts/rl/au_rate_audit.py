"""Strict VVenC access-unit (AU) rate logging and validation.

The modified vvencapp writes VVENC_RL_AU_BITS_FILE while emitting each
Annex-B access unit. The recorded payloadUsedSize includes start codes.
The AU rows are ordered by *output* order, not numeric POC. The sum
of byte sizes MUST equal the exact raw bitstream size on disk.

This is an output-rate accounting module, not reward or video decoding.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class AUBits:
    output_step: int
    poc: int
    au_bytes: int
    au_bits: int
    cumulative_bits: int


def read_au_bits(
    log_path: Path,
    bitstream_path: Path,
    expected_frames: int,
) -> list[AUBits]:
    if expected_frames < 1:
        raise ValueError("expected_frames must be positive")
    if not log_path.is_file() or not bitstream_path.is_file():
        raise FileNotFoundError("Missing AU log or encoded bitstream")

    with log_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        expected_fields = ["output_step", "poc", "au_bytes", "au_bits"]
        if reader.fieldnames != expected_fields:
            raise ValueError(f"Bad AU CSV header {reader.fieldnames}; expected {expected_fields}")
        entries = list(reader)

    if len(entries) != expected_frames:
        raise ValueError(f"Expected {expected_frames} access units, got {len(entries)}")
    result = []
    cumulative = 0
    pocs = set()
    for i, row in enumerate(entries, start=1):
        if None in row:
            raise ValueError(f"Malformed AU row {i}")
        try:
            step = int(row["output_step"])
            poc = int(row["poc"])
            nbytes = int(row["au_bytes"])
            nbits = int(row["au_bits"])
        except (ValueError, TypeError) as e:
            raise ValueError(f"Bad AU integers in row {i}: {row}") from e
        if step != i or poc < 0 or poc >= expected_frames or poc in pocs:
            raise ValueError(f"Invalid output_step / POC / duplicate in row {i}")
        if nbytes <= 0 or nbits != 8 * nbytes:
            raise ValueError(f"AU size mismatch in row {i}")
        pocs.add(poc)
        cumulative += nbits
        result.append(AUBits(step, poc, nbytes, nbits, cumulative))
    if pocs != set(range(expected_frames)):
        raise ValueError("Access units missing POCs")
    bitstream_bytes = bitstream_path.stat().st_size
    if cumulative != bitstream_bytes * 8:
        raise ValueError(
            f"AU accounting mismatch: sum={cumulative} bits, "
            f"bitstream={bitstream_bytes * 8} bits"
        )
    return result
