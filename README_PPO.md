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

## AU bitstream accounting (Stage-3 prerequisite)

The next gate reads the **actual output AU size**, rather than a bitrate
estimate from QP. The modified `vvencapp` accepts the environment variable
`VVENC_RL_AU_BITS_FILE=/path/au_bits.csv` and emits:
`output_step,poc,au_bytes,au_bits`.

- The AU size is `vvencAccessUnit.payloadUsedSize` after copying the
  Annex-B bitstream, so its byte sum must exactly equal the output
  `.266` file size.
- `output_step` is the **emission/encoding order**. POC labels the
  picture, which may not increase monotonically. The two must never be
  treated as identical under B-frame reorder.
- `scripts/rl/au_rate_audit.py` checks each row, unique POC, exact sum,
  and output order. `scripts/rl/test_au_rate_audit.py` covers reordered
  POC and bad logs.
- The revised `run_online_smoke.py` sets the environment variable for
  Vanilla, Zero and PPO and verifies AU byte sums. The strict
  `verify_online_episode.py` also requires AU records.

**Current status:** AU-logging code was added AFTER the Stage-2 online
smoke passed, and has not yet been rebuilt or tested on Kaggle. This is
not a completed YOLO Reward pipeline.

Rebuild after pulling `rl-agent-dev`, then run the short smoke again:

```bash
python3 -m unittest discover -s scripts/rl -p 'test_au_rate_audit.py' -v
python3 scripts/rl/run_online_smoke.py \
  --repo /kaggle/working/vvenc_rl_stage1 \
  --encoder /kaggle/working/vvenc_rl_stage1/bin/release-static/vvencapp \
  --decoder /kaggle/working/vvdec_3_2_0/bin/release-static/vvdecapp \
  --video /kaggle/working/smoke_480x320.yuv \
  --frames 16 --output /kaggle/working/rl_au_rate_check
python3 scripts/rl/verify_online_episode.py \
  --directory /kaggle/working/rl_au_rate_check --frames 16
```

## Second PPO update from fresh Train rollout — user-verified PASS (2026-11-03)

User's Kaggle log on `17d9eec` confirms:
- **23/23** Python unit tests PASS.
- Fresh 64-frame episode, locked Train video `0000f77c-6257be58`,
  sampled from *updated* PPO checkpoint #1 (SHA
  `8368b8a67f4d4d504bfb6c5c4a4fa622cd95594e44ce22f5fad3160799cab23b`).
- Target 120 kbps, actual 125.355 kbps, source boxes 119;
  decoded boxes 93; TP=91 FP=2 FN=28; mean reward 0.784614.
- On-policy collector logprob/value error 0, second PPO update:
  4 optimizer steps, Actor and Critic both changed, KL 0.003494,
  checkpoint reload EXACT.
- Second checkpoint at user Kaggle path
  `/kaggle/working/rl_second_fresh_1793704553/ppo_second_update.pt`,
  SHA256 `8a7a84b92df00a9f589b0eac39bffa2b1062c4c996d1b4a4f4e5937362b1e622`.
- The user produced `/kaggle/working/PPO_QP_PILOT_CHECKPOINTS.zip`.
  Export off ephemeral Kaggle before session termination.

CAUSALITY WARNING: first vs second sample used seed 7 vs 17,
so better F1 (0.8095→0.8585) and slight bitrate decrease are
NOT evidence of trained-policy generalization or rate gains.

### Next step: bounded multi-VIDEO Train continuation gate

`scripts/rl/run_multi_train_gate.py` implements a **3 distinct Train
video pilot**, not full training. It loads the second checkpoint and
keeps a SHA-verified policy + Adam chain over fresh Train clips.
It takes only indices from the existing 65-ID locked Train manifest
(default 1,2,3); cannot use Validation/Test. Each episode at 120kbps:
frozen YOLOv5n → genuine per-POC AU bitrate/F1 Reward →
single on-policy PPO update → new checkpoint. Saves progress.json
atomically and checkpoint/provenance ZIP **after every successful video**.
A partial failed episode is never silently recycled.

On the *same* GPU Kaggle Session with the second checkpoint:

```bash
cd /kaggle/working/vvenc_rl_stage1
git pull --ff-only origin rl-agent-dev
python3 -m unittest discover -s scripts/rl -p 'test_*.py' -v
python3 scripts/rl/run_multi_train_gate.py \
  --repo /kaggle/working/vvenc_rl_stage1 \
  --decoder /kaggle/working/vvdec_3_2_0/bin/release-static/vvdecapp \
  --initial-checkpoint /kaggle/working/rl_second_fresh_1793704553/ppo_second_update.pt \
  --train-indices 1,2,3 --frames 64 --beta 0.1 \
  --output /kaggle/working/ppo_qp_multi_train_gate
```

Expected gate (must be verified on Kaggle):
`BOUNDED_TRAIN_CONTINUATION_GATE_PASS`;
3 fresh, distinct Train episodes, checkpoints at PPO iterations 3,4,5;
archive `ppo_qp_multi_train_gate/PPO_QP_TRAIN_GATE_CHECKPOINTS.zip`.

This is ACTUAL pilot training with small samples, not just unit tests,
but NOT yet full training over 65 complete videos.
`--frames 64` tests only the first 64 frames per Train clip, and
`beta=0.1` is not a validated hyperparameter. Full training needs
full-video/window sampling strategy, normalization and a fixed-seed,
no-update Validation evaluation protocol, resource budgets,
resume, and protected Test split. DO NOT directly interpret pilot
F1 differences as proof of RL improvement.

## First PPO update from real Reward — user-verified PASS (2026-11-03)

Kaggle GPU log on previous commit `6c17517` established:
- 22/22 unit tests PASS.
- Real 64-frame Train episode `0000f77c-6257be58` is on-policy
  for seed=7's untrained Actor/Critic, logprob/value max error 0.
- PPOOptimizer steps=4; real mean Reward=0.743876934.
- Both Actor and Critic weights changed; approx KL=0.00228209,
  and model+optimizer checkpoint reloaded EXACTLY.
- First checkpoint (user's active Kaggle Session):
  `/kaggle/working/rl_real_train_1793702575/ppo_first_update.pt`;
  SHA256 `8368b8a67f4d4d504bfb6c5c4a4fa622cd95594e44ce22f5fad3160799cab23b`.
- This proves a single valid PPO update, not an improved rate-F1 curve.

### Next gate: fresh episode sampled from the UPDATED policy

The updated `real_reward_pilot.py` now takes
`--policy-checkpoint /path/ppo_first_update.pt` and
`--policy-seed 17`; the bridge loads the checkpoint before
sampling actions. The pilot records the collecting checkpoint SHA
in the reward episode summary.

The updated `verify_real_ppo_update.py` takes
`--checkpoint-in /path/ppo_first_update.pt` and
`--checkpoint-out /path/ppo_second_update.pt`. The checkpoint SHA
must match the new episode's recorded collecting policy. It restores
BOTH model and Adam state, validates raw latent QP action, logprob/value,
and performs one update on the NEW rollout. Reusing the same previous
rollout or a mismatched checkpoint is forbidden. New checkpoint records
the iteration count and collector provenance.

```bash
cd /kaggle/working/vvenc_rl_stage1
git pull --ff-only origin rl-agent-dev
python3 -m unittest discover -s scripts/rl -p 'test_*.py' -v
python3 scripts/rl/real_reward_pilot.py \
  --repo /kaggle/working/vvenc_rl_stage1 \
  --decoder /kaggle/working/vvdec_3_2_0/bin/release-static/vvdecapp \
  --train-index 0 --frames 64 --beta 0.1 \
  --policy-checkpoint /kaggle/working/rl_real_train_1793702575/ppo_first_update.pt \
  --policy-seed 17 \
  --out /kaggle/working/rl_second_fresh_64frames
python3 scripts/rl/verify_real_ppo_update.py \
  --episode /kaggle/working/rl_second_fresh_64frames \
  --checkpoint-in /kaggle/working/rl_real_train_1793702575/ppo_first_update.pt \
  --checkpoint-out /kaggle/working/rl_second_fresh_64frames/ppo_second_update.pt
```

NOTE: These new changes are NOT YET Kaggle verified. The 64-frame
Train pilot is not the full train split. Keep 100 Test holdout
untouched. Save checkpoints out of ephemeral Kaggle /working.
The real optimizer config is still the *gate* config (2 epochs,
minibatch 32). Select full training hyperparameters on Train/Val,
not on Test, after passing the fresh-collector gate.

## Real frozen-YOLO Train reward pilot — user-verified PASS (2026-11-03)

The GPU Kaggle log from commit `58a2d7e` verifies:
- **20/20 code unit tests PASS** before the real test.
- Correct locked manifest (65 Train / 7 Val / 100 Test), GPU Tesla T4
  and Train-only clip `0000f77c-6257be58` (64 frames).
- Correct encoder/decoder compilation and detector YOLOv5 v7.0 loading.
- 119 reference/pseudo-GT car boxes, 91 decoded car boxes;
  TP=85, FP=6, FN=34, micro F1=170/210=0.8095238.
- Real measured bitrate: 125.6063 kbps, target 120 kbps.
- Mean per-frame reward: 0.743877 for pilot beta=0.1.
- Actual AU bits were joined by Train video ID + POC and the
  complete read-only PPO reward rollout was written.
- `ppo_weights_updated: false`. No learning claim is supported.

Output episode location in the user Kaggle session:
`/kaggle/working/rl_real_train_1793702575`.
This path only exists if that Kaggle Session is still available or its
output was persisted. Do NOT claim it survives a Session reset.

## Next mandatory gate: one on-policy PPO update from genuine reward

New `scripts/rl/verify_real_ppo_update.py` reads this complete
real Train-only episode, reconstructs the exact seed=7 bridge model,
re-evaluates all *unrounded* Gaussian actions and compares
collector logprob, value, action quantization, POC, terminal boundary
and next critic value. If any differ, it FAILS before training.

After verifying collector policy identity it performs exactly ONE
PPOTrainer.update on the actual frozen-detector reward, tests that
both Actor and Critic weights changed, and atomically saves a
`torch.load(weights_only=True)` compatible checkpoint containing
model/optimizer/PRNG/provenance; new objects reload it and must match
exactly. Reusing a checkpoint path is forbidden to prevent accidental
multiple updates on one on-policy batch.

Unit tests `test_real_ppo_update.py` cover real-schema synthetic
collector parity, tampered collector logprob/action rejection, an
optimizer update, checkpoint fidelity, and repeat-write refusal.
**No Kaggle execution of this new update gate has been reported yet.**

On the SAME Kaggle Session as the real 64-frame test:

```bash
cd /kaggle/working/vvenc_rl_stage1
git pull --ff-only origin rl-agent-dev
python3 -m unittest discover -s scripts/rl -p 'test_*.py' -v
python3 scripts/rl/verify_real_ppo_update.py \
  --episode /kaggle/working/rl_real_train_1793702575 \
  --checkpoint-out /kaggle/working/rl_real_train_1793702575/ppo_first_update.pt
```

**IMPORTANT:** This is the LAST pre-training gate, not evidence of
better task-rate behavior and NOT the full 65-video training loop.
Even after PASS the next fresh rollout must load the saved updated
model (policy bridge supports `--checkpoint`), and the full
train/validation experiment requires repeated fresh rollout-then-update,
logging and resume across Kaggle Sessions. Never reuse old trajectories
after changing the policy.

## Next mandatory gate: real frozen-YOLOv5n Train reward pilot (not training)

User-tested state-action integration passed on 2026-11-03, including
16/16 observations, 192/192 final CTUs and exact POC AU-bit totals.
Standalone Reward F1 (6/6 tests), AU-accounting (4/4 tests) and
Reward POC join (5/5 tests) passed on Kaggle CPU. These are NOT
evidence of task-specific RL optimization.

The newly implemented `scripts/rl/real_reward_pilot.py` runs a real
**locked Train-only** sample at 120 kbps. This is still experimental
code and has NOT yet been executed on Kaggle.

A sample run needs three Kaggle Add Inputs:
1. BDD100K Train00 video data: `robikscube/driving-video-with-object-tracking`.
2. The existing **VCM_BDD100K_LOCK_v1** manifest with Train=65, Val=7,
   Test=100. SHA-256 of `dataset_lock_v1.json` must match
   `f7a600f501de622f66ace04bb0b12ec5da0dab988cd54732d614f6b16495aaea`.
3. The existing **VVenC_Pilot_v1_2_QA_LOCKED_SNAPSHOT** (ZIP or
   extracted): `baseline_config.lock.json` is required. The detector
   YOLO commit/weight SHA and codec settings are checked against the
   approved lock; refuse to substitute fresh/retrained weights.

Preflight on a GPU Kaggle Session (CUDA is required by this pilot):

```bash
python3 scripts/rl/preflight_real_pilot.py --train-index 0
```

Then after building `vvencapp` and `vvdecapp`, and checking the
updated Reward unit tests:

```bash
python3 scripts/rl/real_reward_pilot.py \
  --repo /kaggle/working/vvenc_rl_stage1 \
  --encoder /kaggle/working/vvenc_rl_stage1/bin/release-static/vvencapp \
  --decoder /kaggle/working/vvdec_3_2_0/bin/release-static/vvdecapp \
  --train-index 0 --frames 64 --beta 0.1 \
  --out /kaggle/working/rl_real_train_64frames
```

The script preprocesses locked Train MOV exactly as in the Baseline
with FFmpeg metadata-dependent colorspace conversion, 480x320 bicubic
and CFR 30. It uses VVenC medium, 1-pass VBR/QPA, internal 10-bit,
GOPSize=32, LookAhead=1, `--mtprofile -1` and freezes the original
YOLOv5n v7.0 car detector (`conf=0.25`, NMS=0.45, IoU match=0.5).
It checks decoded frame count and a temporal-alignment screen, writes
one source and one reconstructed detection list per POC, joins exact
AU bits, and writes a complete read-only reward rollout.

This pilot **does not train PPO** or pick beta; `beta=0.1` is an
illustrative diagnostic value. It only examines the first 64 frames,
so the observed bitrate and detection metrics must not be compared
with full-video 500-run Test results. The next gate must verify true
PPO update on on-policy, detector-derived transitions, including
checkpoint save/load/reproducibility, before training over 65 Train.

Reward causal-accounting clarification: `reward_join.py` now computes
the cumulative true AU bits along **policy decision order** for a
per-decision reward/GAE trajectory; AU output order and its own
cumulative bits remain in `au_bits.csv` as independent telemetry.
The user-reported 5/5 reward_join PASS was before this correction;
the updated tests must pass again.

## Reward-to-trajectory join (Stage 3 — code available, NOT validated on BDD100K)

`scripts/rl/reward_join.py` now takes (a) `ppo_policy_bridge.py`
trajectory, (b) a real output-AU CSV and exact VVC bitstream,
(c) **frame-matched frozen YOLOv5n detections from the reference and
decoded video**. Both detection files are JSON Lines with exactly one
record per input frame, including no-car frames:

```json
{"video_id":"TRAIN_VIDEO","poc":0,"detections":[{"x1":10,"y1":10,"x2":40,"y2":40,"confidence":0.91,"class_id":2}]}
```

The join requires identical video_id and complete, unique POC sets.
Rate penalty is computed from AU emission order. Returns are emitted in
**policy decision order**, with `next_value` from the next policy
observation, and `done` at the episode end; these orders can differ.
It refuses Zero-policy trajectories because they lack a raw sampled
Gaussian policy action/log probability.

Unit tests `test_reward_join.py` deliberately use different POC orders
for policy observations (2,0,1), AU emission (0,2,1), and detection JSONL
(1,2,0) to catch accidental row-number pairing. Run:

```bash
python3 -m unittest discover -s scripts/rl -p 'test_reward_join.py' -v
```

The converter DOES NOT execute YOLO, preprocess frames, derive picture
order from decoded YUV, or train a model. Those are required next.
No task-rate result has been measured for PPO-QP.

### Locked detector / codec evaluation protocol

The newer completed 100-video/500-run archive
`VVenC_Full_Baseline(1).zip` contains `baseline_config.lock.json`.
Its locked settings, which take precedence over earlier 98-video draft
notebooks, are:

- **Video:** 480x320, 30 fps, YUV420p8 normalized using BT.709
  limited range and the locked ffmpeg preprocessing pipeline.
- **Encoder:** pinned VVenC v1.14.0, `medium`, 1-pass VBR, QPA=1,
  intra=32, CRA, GOPSize=32, LookAhead=1, `--mtprofile -1`,
  4 threads and internal 10-bit. Decoder VVdeC v3.2.0.
- **Detector:** YOLOv5 v7.0, `yolov5n.pt`, FP32,
  COCO class_id=2 (car), confidence=0.25, NMS IoU=0.45,
  detection-match IoU>=0.5. On decoded 10-bit YUV use
  `clip((YUV10 + 2)//4, 0, 255)` before the exact BT.709 limited
  YUV420p-to-RGB transform used on the source.
- **Pseudo-reference:** the frozen detector's results on the normalized
  uncompressed encoder input (NOT human annotations). Preserve source
  box reference and one-to-one confidence-ordered matching rules.
- **Evaluation:** test split 100 videos x 5 targets; 50/80/120/160/200
  kbps; locked Test is only for final evaluation. Train split has 65,
  Validation 7.

**Important discrepancy:** Stage-2 16-frame *synthetic* smoke CLI only
validated integration; it omitted the explicit `--mtprofile -1` and
`--additional GOPSize=32:LookAhead=1` needed for exact scientific
protocol parity. Never treat the smoke bit counts as BDD100K results.
The locked Baseline has an authoritative configuration/protocol lock.
Use it for any future real Train/Val encoding, not the smoke settings.

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
