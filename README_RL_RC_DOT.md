# RL-RC-DoT + VVenC v1.14.0 — isolated research branch

Branch: feature/rl-rc-dot-vvc, derived from rl-agent-dev.
Baseline: VVenC v1.14.0 at 9428ea8636ae7f443ecde89999d16b2dfc421524.

## Source delivered in this branch

- source/Lib/EncoderLib/RLDotObservation.h: causal 16 globals plus 4 x 20 x 30 maps
- source/Lib/EncoderLib/RLDotAction.h: 600-int per-POC deltaQP file protocol,
  area-weighted QP action on real CTU/QG, delta in [-12,12]
- scripts/rl_rc_dot/agent.py: PPO with 3-layer CNN, 16 globals, 64-D latent,
  5x7 Gaussian action, bilinear interpolation, actor/critic/auxiliary block head
- scripts/rl_rc_dot/bridge.py: strict POC-keyed live inference and action files
- scripts/rl_rc_dot/train.py: verified on-policy PPO update joining true AU bits
  and frozen-YOLO detections; checkpoint of both model and optimizer
- scripts/rl_rc_dot/reward.py: precision/bitrate reward with lambda=20
- scripts/rl_rc_dot/test_contract.py and test_agent.py
- The established 8+3, 4x3 rl-agent-dev pilot code is preserved.

## Scientific status: PROTOTYPE, not full reproduction

The encoder writes DOT_PROXY, not DOT_STATE, to make gaps explicit. The
current 16-entry observation has zero placeholders for G09-G12.
L01 is luma-only AC variance; L02 is spatial prediction error rather than
true lookahead intra SATD; L03 is temporal SAD rather than MB-tree propagated
cost; L04 is QPA scale proxy rather than x264 inverse qscale. G15 is an
Intra flag, not full I/P/B encoding.

An encoder-statistics patch now uses actual RC-completed picture counts
(not POC) to estimate G13/G14, and records last picture bit count and QP for
G03/G04. These still do not equal final AU-size or all paper-defined metrics.

The bridge and the trainer REJECT DOT_PROXY unless --allow-proxy-state is
explicitly provided. That flag is for transport and optimizer smoke tests
ONLY. Do not report its metrics as scientific RL-RC-DoT reproduction.

Action is a 30x20 requested map, but actual QP signalling follows v1.14.0
QPA/QG constraints. The mapping is area-averaged over the available QG;
it is not independent 16x16 Inter QP signalling.

Agent architecture now follows the public reimplementation's 3 convolutions
(3x3, 3x3, 4x4), flatten, FC128, FC64 and 64->128->600 reward head.
Prior 4x6 adaptive pooling and the direct FC64 have been removed.
This changes checkpoint tensor dimensions: existing experimental checkpoints
from the previous 16+4 implementation are not compatible.

The masked auxiliary reward head now receives a centre-of-detection surrogate
block target: one-to-one matched YOLO predictions score 1; unmatched boxes
score 0, with occupied-cell masks including zero-reward false positives.
This activates auxiliary training but the exact original block-target
mapping remains unverified; it is NOT yet paper-faithful.

G06/G07/G08 now reflect area-weighted, non-overlapping winning VVC luma
CU classes (Intra-like / Inter non-Skip / Skip) in the previous completed
picture, with IBC/Palette grouped into Intra-like. The fields remain in a
DOT_PROXY observation until all 16+4 features are validated.
Full-video 65-train/7-val/100-test orchestration, frozen-YOLO collection
for this NEW 16+4 bridge, and final BD-rate results remain unvalidated.

## Build and unit tests

    git clone -b feature/rl-rc-dot-vvc https://github.com/youngan2810/vvenc_rl.git
    cd vvenc_rl
    cmake -S . -B build/dot -DCMAKE_BUILD_TYPE=Release
    cmake --build build/dot --parallel 2
    python3 -m unittest discover -s scripts/rl_rc_dot -p 'test_contract.py' -v
    python3 -m unittest discover -s scripts/rl_rc_dot -p 'test_agent.py' -v
    python3 -m unittest discover -s scripts/rl_rc_dot -p 'test_auxiliary.py' -v
    python3 -m unittest discover -s scripts/rl_rc_dot -p 'test_bridge.py' -v
    python3 -m unittest discover -s scripts/rl_rc_dot -p 'test_codec_contract.py' -v

The bridge waits for a complete newline-terminated state before parsing or
publishing an action. Empty-detection minibatches still train the auxiliary
zero targets using `aux_empty_weight=0.05`; setting that weight to zero
explicitly disables supervision of unoccupied cells.

Export VVENC_DOT_QP_TRACE_FILE=/tmp/dot_final_qp.csv to audit winning
luma CU min/max and area-weighted QP for each POC+CTU. A trace without
VVENC_DOT_ACTION_DIR is disabled, and the result MUST be checked against
the full POC×CTU grid before making claims about spatial control.

## Smoke test protocol

Start the bridge in background using a fresh directory and the matching
encoder environment variables:

    python3 scripts/rl_rc_dot/bridge.py --states /tmp/dot_states.txt \
      --actions /tmp/dot_actions --trajectory /tmp/dot_trajectory.jsonl \
      --frames 16 --mode zero --allow-proxy-state &
    export VVENC_DOT_STATE_FILE=/tmp/dot_states.txt
    export VVENC_DOT_ACTION_DIR=/tmp/dot_actions

Then run the locked 480x320, 30fps, QPA-on VVenC command for 16 frames.
The synthetic-YUV smoke gate requires complete state POC coverage, exact
requested-map/CTU pooling, zero-action/vanilla bitstream parity, positive and
negative action outcomes, and the expected left/right QP direction on the
first policy decision. Two further runs share a zero-action prefix and vary
only the last decision POC, checking that its final QP responds while earlier
POC QPs remain unchanged. These are CTU-level checks; independent 16x16 QP
control and VVdeC decoding remain separate requirements before training.

Do not simultaneously set VVENC_RL_QP_MAP / VVENC_RL_ACTION_DIR and
VVENC_DOT_ACTION_DIR.

For a pilot, use --mode ppo and --allow-proxy-state to generate a
trajectory. Real reward/update requires the SAME 100%-verified baseline
YOLO preprocessing, frame-by-frame source/decoded detection JSONL and
the exact AU bits file from VVENC_RL_AU_BITS_FILE. The train.py update
rejects untrusted/missing detections and off-policy trajectories.

## Next steps for scientific reproduction

1. Replace all proxy features with causal faithful x264-equivalent
   statistics from VVenC; instrument actual final-CU modes, PSNR/SSIM,
   motion/coeff bits and valid temporal dependency information.
2. Establish finer independent Inter-QG QP control, or explicitly report
   the coarser CTU adaptation as a limitation.
3. Build and run Kaggle encoder/decoder tests, then train on all 65 train
   videos; freeze validation decisions on 7 videos; protect 100 test.
4. Use baseline-matched precision/recall, bitrate, PSNR and BD-rate.
5. Retain original experimental setup / PPO reward-head mapping in the
   methodology documentation; do not claim published results before
   these gates pass.

