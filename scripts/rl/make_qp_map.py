#!/usr/bin/env python3
"""Generate fixed POC-indexed 4x3 delta-QP maps for VVenC-RL smoke tests."""
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", required=True, type=int,
                        help="Number of frames/POCs, starting from --start-poc")
    parser.add_argument("--start-poc", type=int, default=0)
    parser.add_argument("--delta", required=True, type=int, choices=range(-2, 3))
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.frames <= 0 or args.start_poc < 0:
        parser.error("--frames must be positive and --start-poc >= 0")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="ascii") as file:
        for poc in range(args.start_poc, args.start_poc + args.frames):
            file.write(f"{poc} " + " ".join([str(args.delta)] * 12) + "\n")
    print(f"Created {args.frames} per-POC actions at {args.output}")

if __name__ == "__main__":
    main()
