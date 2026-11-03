# VVenC RL Agent development branch (`rl-agent-dev`)

## Status and scientific reproducibility

- **Validated Stage 1:** Branch `rl-ppo`, VVenC v1.14.0, QPA enabled,
  4x3 QP action map. Stage-1 smoke, VVdeC decoding and unique selected-CTU
  audit PASS on the 16-frame synthetic clip.
- **Experimental Stage 2:** This branch `rl-agent-dev` adds a *working-code
  proposal* for pre-action state extraction, an online Python↔VVenC policy
  bridge, and PPO-Clip + GAE. **Not yet compiled or run on Kaggle.**
- **Not yet implemented:** per-frame decoded YOLOv5n frozen pseudo-GT/TP-FP-FN
  reward collector, state/action/reward alignment by POC, batched RL
  training episodes, early stopping on held-out validation, trained policy
  checkpoint, and final 100-video Test evaluation.
  Do not report any PPO accuracy/bitrate gains based on these files.

## Fixed architecture (QP-only pilot)

- One Actor-Critic with three spatial convolution layers (3→16→32→32).
- A global-state MLP (8→64→32), fused with 32-D CNN latent to 64-D vector.
- Actor: 12 independent Gaussian latents; tanh to [-2,+2], integer rounding
  for 4x3 QP action map; Critic: one scalar value.
- PPO-Clip epsilon=0.2, GAE gamma=0.99 / lambda=0.95, Adam=3e-4,
  epochs=5, minibatch=64, target-KL=0.03, grad norm=0.5.
- Use original continuous *raw* action and its log probability for PPO
  optimization. The rounded discrete QP action alone does not define the
  Gaussian log-probability. Keep raw action in trajectories.
- PPO episodes = individual video × bitrate target. One step for each
  picture that is actually sent to final VVenC encoding, POC-keyed.
  The encoder may process frames in a different order from POC; maintain
  explicit POC mapping for source, reconstruction, reward, and bits.

## Observation contract and timing

The encoder emits `STATE <poc> <8 globals> <36 spatial>` after
`EncSlice::xInitSliceLambdaQP()` (built-in picture QPA complete), before
any CTU coding. Output via `VVENC_RL_STATE_FILE` environment variable.
The 36 spatial features are the row-major concatenation of 3 channels,
each shaped 3x4:

- Activity: local luma pixel gradient over sparsely sampled original pixels.
- Temporal: local absolute-difference to the previously available source
  picture (from VVenC's preprocessor; zero if unavailable).
- QPA: built-in CTU QP divided by 63.

The eight global features are currently:

| Index | Feature | Normalization |
|---|---|---|
| 0 | Target bitrate | / 200000 |
| 1 | POC progress | / configured frame count |
| 2 | Previously used bitrate budget | used bits / sequence target bits |
| 3 | Remaining budget fraction | max(0,1-used fraction) |
| 4 | Previous average QP from RC | /63 (fallback picture QP) |
| 5 | Picture/slice QP after QPA | /63 |
| 6 | Intra-frame flag | 0 or 1 |
| 7 | Temporal layer | /6 |

Important: `EncRCSeq` bit accumulation corresponds to *previously
encoded* frames and hence encoder scheduling order; it is not necessarily
the same as POC order. Review data sharing if enabling parallel GOPs.
This input contract is preliminary: no reward or finished-frame
information is allowed in the current picture's observation.

## Agent policy transport

The process `scripts/rl/ppo_policy_bridge.py` tails the observation
file and creates `VVENC_RL_ACTION_DIR/<POC>.txt`, atomically, containing
`POC delta00 ... delta23`. The encoder waits for the matching POC file
before encoding QG(s) for that picture, with a 60 s timeout. This
separates Python GPU inference from the C++ VVenC library without Python
linkage in the encoder binary. File I/O is **only for the first prototype**;
optimize IPC and runtime only after correctness.

The fixed-map environment variable `VVENC_RL_QP_MAP` and live-action
`VVENC_RL_ACTION_DIR` cannot be used together. If both are absent,
the hook is disabled and VVenC runs the unmodified algorithm.
Use a fresh per-episode state file, action directory, and trajectory file
to avoid stale cross-video actions. Empty/new directories only.

## Kaggle smoke test for the Stage-2 bridge

After pulling `rl-agent-dev` and rebuilding, in a fresh Kaggle session
with the **same 16-frame synthetic YUV** and existing VVdeC v3.2.0:

```bash
python3 -m unittest discover -s scripts/rl -p 'test_ppo_agent.py'
python3 scripts/rl/run_online_smoke.py \
  --repo /kaggle/working/vvenc_rl_stage1 \
  --encoder /kaggle/working/vvenc_rl_stage1/bin/release-static/vvencapp \
  --decoder /kaggle/working/vvdec_3_2_0/bin/release-static/vvdecapp \
  --video /kaggle/working/rl_qp_smoke/smoke_480x320.yuv \
  --frames 16 \
  --output /kaggle/working/rl_agent_smoke
```

The test invokes:
1. Vanilla encode (no RL environment variables).
2. Online `zero` mode: observations and policy transport must leave the
   bitstream bit-identical to Vanilla.
3. Online `ppo` mode (randomly initialized **untrained** policy): consumes
   genuine VVenC observations; supplies per-POC QP actions and records a
   trajectory with raw Gaussian latents and log-probabilities.
4. Optionally VVdeC decode each bitstream and verify expected YUV size.

No model is trained and no detector accuracy or reward is calculated
during this bridge smoke test. The training phase must include the
frozen source/reconstructed YOLOv5n evaluation and bitrate reward,
using the paper's TP/FP/FN protocol and consistent POC mapping.

## Stage-2 Kaggle verification (2026-11-03)

On the 16-frame 480x320 synthetic clip, the user verified:
- VVenC and VVdeC builds complete.
- PPO math unit tests 4/4 PASS.
- Vanilla 16856 bits; online zero-action 16856 bits and bit-identical.
- Untrained PPO inference generated 118 nonzero QP values and a 17160-bit stream.
- 16 observations, 16 matching actions, and 192 unique final CTUs per run.
- 99 of 192 final CTUs had different QP ranges under PPO.
- Both PPO policy bridge and strict POC/CTU gate PASS.

These are integration results only. In particular, the test video's tiny
output bitrate and an untrained PPO action must NOT be used to claim improved
detection, rate control or bitrate savings on BDD100K.

## Stage-3 Reward implementation status

`scripts/rl/reward_metrics.py` implements **standalone reward primitives**:
one-to-one car-box matching (prediction confidence order, IoU>=0.5),
TP/FP/FN and F1, an empty-reference rule (no free reward, FP penalty),
and the cumulative actual-bps log penalty evaluated in encoding step order.

`scripts/rl/test_reward_metrics.py` covers one-to-one matching, misses,
rate over/undershoot, empty-car frames, input validation and POC vs
encoding-order indexing. Run:

```bash
python3 -m unittest discover -s scripts/rl -p 'test_reward_metrics.py' -v
```

**NOT YET INTEGRATED:** actual YOLOv5n detector inference; frozen detector
preprocessing/confidence/NMS parity with the locked 500-run Baseline;
bit extraction by access-unit/POC; corresponding VVdeC display order;
joining actual per-frame bits + source/decoded detections + PPO trajectory;
Beta validation; frame-level and episode-level RL optimization; checkpoint
saving/resumption. This code must not be presented as a finished reward
pipeline or a trained PPO Agent.

The reward assumes the inputs are already matched by (video ID, POC), and
that actual bits are from the VVC access unit, not from YUV file size.
The chosen one-to-one matching rule **must be verified against the Baseline**
before it becomes an evaluation protocol; do not silently change it.

## Python PPO test commands

```bash
python3 -m unittest discover -s scripts/rl -p 'test_ppo_agent.py'
```

Requires PyTorch; CPU is sufficient for the unit tests and the tiny
prototype. PPO training should use Kaggle GPU.

## Scientific constraints

Use the training split (65 videos) to optimize the model and validation
(7 videos) for checkpoint/parameter selection. The 100-video Test split is
a fixed, untouched final holdout. The initial RL pilot targets 120 kbps;
the five-rate single-policy experiment uses 50, 80, 120, 160, 200 kbps.

Freeze YOLOv5n and use **source detections as pseudo-labels**, not ground
truth from human annotation. Each encoded/reconstructed frame yields a
post-encode detection F1 and actual bits; the initial reward is:
`r_t = F1_t - beta * abs(log(R_cum,t / R_target))`.
Do not call this mAP against annotated GT. Do not optimize on Test.
