# VVenC-RL: QP-only integration (experimental)

**Baseline:** VVenC v1.14.0, commit `9428ea8636ae7f443ecde89999d16b2dfc421524`.

This branch is the *first integration checkpoint*, **not a trained PPO encoder**.
It provides a fixed per-POC 4 x 3 integer delta-QP map. The original encoder
remains untouched when `VVENC_RL_QP_MAP` is not set.

## What is implemented

- `source/Lib/EncoderLib/RLQpAction.h`: immutable, per-POC delta-QP reader, valid values [-2, 2].
- `source/Lib/EncoderLib/EncCu.cpp`: optional action at luma QG initialization,
  after built-in QPA QP selection and before the coding-mode search. For a nonzero
  delta, synchronize RDO/quantization lambda using VVenC's `updateLambda()`.
- Optional tracing with `VVENC_RL_TRACE=1`: reports POC, QG top-left pixel
  coordinates, base QP, requested delta and applied QP to stderr.

All existing Partition and MTS decisions remain under VVenC.

## Build on Kaggle

```bash
git clone --branch rl-ppo https://github.com/youngan2810/vvenc_rl.git
cd vvenc_rl
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 2
find build -type f \( -name vvencapp -o -name vvencFFapp \)
```

No input action map (Vanilla behavior):

```bash
unset VVENC_RL_QP_MAP
unset VVENC_RL_TRACE
# Run your EXACT locked baseline command here using the built vvenc binary.
```

Generate a 120-frame example (POCs 0..119):

```bash
python3 scripts/rl/make_qp_map.py --frames 120 --delta 0 --output /kaggle/working/dqp_zero.txt
python3 scripts/rl/make_qp_map.py --frames 120 --delta=-2 --output /kaggle/working/dqp_minus2.txt
python3 scripts/rl/make_qp_map.py --frames 120 --delta=2 --output /kaggle/working/dqp_plus2.txt
export VVENC_RL_QP_MAP=/kaggle/working/dqp_zero.txt
export VVENC_RL_TRACE=1
# Run the same locked VVenC command; redirect stderr to a trace log.
```

The action-file format is exactly 13 integers per row: one POC and twelve
row-major deltas, e.g. `0 0 0 0 0 0 0 0 0 0 0 0 0`.
The map must include **every POC that the encoder actually encodes**. An
unknown POC, duplicate POC, invalid delta, or incorrect CTU grid fails fast.

## Audit: why there are repeated QP-hook calls

`xCompressCU()` recursively evaluates multiple hypothetical QT/MTT CU
partitions during RDO. Therefore, a log entry from inside `xCompressCU()`
is a **candidate evaluation**, not an independent RL policy step, and
revisited POC/QG positions are expected. At the selected-QP branches, the
original QPA code resets `baseQP` before the fixed action is added; the
action is not accumulated by revisiting a candidate. The immutable map
supplies only one logical action per frame/POC.

To verify the final winning encoding decisions independently, this branch
now emits `[VVENC_RL_CTU_FINAL]` records **after** `xCompressCtu()` has
selected its CU tree. A CTU record contains the POC, CTU index, number
of chosen luma CUs, and their min/max QP. Each final CTU must appear
exactly once per POC in the final encoding pass.

After pulling and rebuilding the updated branch (the old binary cannot
emit final-CTU records), run:

```bash
python3 scripts/rl/verify_stage1.py \
  --encoder bin/release-static/vvencapp \
  --decoder /kaggle/working/vvdec_3_2_0/bin/release-static/vvdecapp \
  --video /kaggle/working/rl_qp_smoke/smoke_480x320.yuv \
  --output /kaggle/working/rl_qp_audit_final \
  --frames 16 --width 480 --height 320 --ctu-size 128
```

Expected selected-CTU count: 16 frames * 4 columns * 3 rows = **192**,
not the number of recursive RDO candidate calls (e.g. 464).
The gate checks bit-identical zero actions, 0 / +/-2 action consistency,
192 unique final CTU records, decoded output size, and that the chosen CU
QP ranges respond to the nonzero actions. **Do not begin PPO training
until the gate passes.** Passing this gate validates the test configuration,
not all arbitrary codec modes and resolutions.

## Required verification before PPO

1. `VVENC_RL_QP_MAP` unset: output identical to unmodified v1.14.0 for the
   same input, options, and deterministic threading configuration.
2. Zero actions: bitstream identical to the disabled-action run.
3. +2 and -2 maps: trace QP changes as expected, allowing for legal QP
   clipping and VVenC's additional QPA behavior; `delta_requested` is not
   necessarily the final coded CU QP.
4. Decode with baseline VVdeC v3.2.0; check frame count and POC alignment.
5. Compare actual bits, per-frame detection TP/FP/FN and rate–F1. A one-frame
   bitrate response need not be monotone because of rate control.
6. Validate intra/sub-CTU QPA and QG signaling with a representative short
   clip before the full 65-video PPO train subset.

**Constraints:** This integration requires QPA + PPS differential QP at the QG,
a 4 x 3 CTU grid, and externally supplied action maps. This code is *not* a
live Python↔C++ bridge. Future work: encoder state extraction, a per-POC
low-latency action protocol, trajectory/reward logging, and the PPO training loop.

**Do not train on the 100-video holdout Test.** Initial pilot: Train subset,
120 kbps; later 50/80/120/160/200 kbps with frozen YOLOv5n for reward.
