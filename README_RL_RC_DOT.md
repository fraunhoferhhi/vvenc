## Adapter/3 research update (2026-11-12)

**Important: old adapter/2 checkpoints cannot be resumed.** The QP reference,
auxiliary supervision, and PPO learning reward scale changed. Use a fresh
source-pinned Kaggle notebook and `MODE='new'`.

- Delta actions now modify native VVenC QPA QP; native preflight requires
  zero-action bit-exact parity with the matched research anchor.
- PPO receives 0.05 times the paper's scalar reward to reduce Critic target
  magnitudes; reported validation/episode rewards stay in paper units.
- Auxiliary loss uses a detection-region mask (source and reconstruction box
  centers) and 5x weighting for positive cells, not uniform full-frame MSE.
- Every checkpoint is compared to matched anchor AND zero-action. Validation
  tracks actual bitrate ratios and Precision changes; controls are cached.
- **No performance improvement has been demonstrated yet:** run native
  preflight, smoke, Trial, and multiple-seed rate–task comparisons.

See `scripts/rl_rc_dot/METHOD_MAPPING.md` for the scientific rationale.

---

# RL-RC-DoT → VVenC: integrated H.266 adaptation (adapter/2)

Supported branch: **feature/rl-rc-dot-vvc**. Supported training entry:
**python -m scripts.rl_rc_dot.session** (or train.py without --trajectory).
The old DOT_PROXY pilot is retired and fails fast. Existing v3/v4 and
custom-PPO prototype checkpoints are **not compatible** with this schema.

This is an implementation of the method on H.266 with explicitly declared
codec adaptations. **After the 4096-transition Trial, the critic branch is
isolated from the actor/auxiliary branch, gradients are clipped per branch,
state cumulative bits are log-scaled and auxiliary output is bounded.** It is not the authors' released implementation, and
working tests do not establish convergence or the paper's performance.
Read [METHOD_MAPPING.md](scripts/rl_rc_dot/METHOD_MAPPING.md) before experiments.

## What is now connected

- One causal decision → actual encoded picture → Annex-B AU bits and native
  reconstruction → frozen YOLO reward → next state.
- All 16 global fields and 4 local maps are populated. CU modes and QP come
  from final CABAC coding before cleanup; completed PSNR/SSIM and actual
  syntax-bit counters enter the next state.
- Source-only ten-picture lookahead provides SATD and a declared temporal
  dependency-cost approximation, not a relabelled temporal SAD.
- Gaussian 5×7 action → floating bilinear 20×30 map → actual VVC QG pooling.
  The encoder echoes the consumed map; the bridge and rollout buffer verify it.
- Stable-Baselines3 **2.7.0 PPO**, with independent actor and critic CNN/64
  latents, auxiliary block reward-prediction head on the actor branch and
  separate actor/critic gradient clipping. This isolates large critic return-MSE
  gradients observed in the 4096-step Trial. The auxiliary head now uses sigmoid
  outputs in [0,1], matching its labels. Provisional settings: learning rate
  **3e-5**, **3** optimizer epochs, **target KL 0.05**, auxiliary MSE coefficient **0.1**.
  This is a declared training adaptation, not a claim of original-paper fidelity.
- Finite checks on observations, actions, old logprobs, rewards, losses,
  gradients, parameters and optimizer state. The entire rollout is checked
  against the collecting policy before the first optimizer step.
- Train/Validation separation, deterministic paired validation, best checkpoint,
  restored optimizer/RNG, soft session stop, transactional checkpoint pointers,
  sealed exports and Kaggle-safe resume.
- Two collectors by default: worker0 uses GPU0 and worker1 uses GPU1 for YOLO.
  PPO defaults to CPU. VVenC itself remains a CPU encoder.

## Locked data and detector

Bundled assets contain the approved Train65 / Validation7 / internal Test100
split and frozen baseline protocol. They contain **no MOV videos**.

Dataset SHA256:
f7a600f501de622f66ace04bb0b12ec5da0dab988cd54732d614f6b16495aaea

Kaggle still needs the actual BDD100K Train00 MOV dataset
(Driving Video with Object Tracking). Each MOV is checked against the lock.
The existing bdd100k.zip and snapshot.zip match the bundled lock content;
they are metadata, not replacement video inputs.

Preprocessing: 480×320, 30fps, BT.709 TV I420. YOLOv5n v7.0, car class2,
confidence0.25, NMS0.45, matchIoU0.5, maxdet300, FP32. Source/weights/dirty
tracked YOLO files are checked. RGB conversion must match the frozen FFmpeg
protocol byte-for-byte. Detection matching is confidence-ordered, one-to-one;
empty prediction Precision is0. These details are explicit definitions where
the publication does not provide complete implementation code.

## Codec profile and comparison

The integrated profile is medium, 480×320@30, internal10bit, one-pass RC,
GOP32, native LookAhead1, CTU32, DQPsubdiv2, frame parallelism0. CTU threads
remain enabled. This makes the frame boundary causal and permits finer QGs;
the actual CU/QG partition can still coalesce cells. It does **not** promise
600 independent 16×16 quantizers.

In agent mode, deltaQP is relative to the frame QP and replaces the local
QPA offset. Zero action therefore need not equal the native-QPA anchor.
The supplied paired anchor uses native QPA under the **same declared
CTU32 research profile**. Label it accordingly. It is not the old stock
CTU128 VVenC result. **Do not reuse the previous 500 baseline points as a
matched anchor for this profile.** A stock-VVenC comparison is an additional
experiment with its own configuration record.

The mapping adds no SAC/Partition/MTS policy and no changed task reward.
The source-derived lookahead and VVC syntax categories are adaptations that
must be disclosed, not claimed numerically identical to x264 internals.

## Setup on Kaggle

Enable Internet and T4×2. Clone this branch at a fixed commit, then:

    python -m scripts.rl_rc_dot.setup_kaggle \
      --repo /kaggle/working/vvenc_rl_source \
      --source-commit THE_CHECKED_OUT_COMMIT \
      --output /kaggle/working/rl_rc_dot_tools --jobs 2 --install-system

Setup preserves installed CUDA Torch, pins VVdeC source and YOLO, builds
VVenC with JSON ON, and records actual binary hashes. It does not compare
a new compiler's binary with an unrelated historical hard-coded SHA.

Run the complete tests:

    python -m unittest discover -s scripts/rl_rc_dot -p 'test_*.py' -v

Run native preflight (encoder/decoder paths from TOOLCHAIN.json):

    python -m scripts.rl_rc_dot.preflight \
      --encoder /path/vvencapp --decoder /path/vvdecapp \
      --output /kaggle/working/preflight --frames 32

This verifies complete state/AU/POC/CU coverage, causal history statistics,
consumed actions, spatial direction, an isolated POC probe, and byte equality
of decoded reconstruction for every POC. The session runner also runs it.

## Short pilot before long training

Use a new output. This is a reduced technical/learning pilot, not a reportable
full experiment. The CLI's default full-video training horizon is preserved.

    python -m scripts.rl_rc_dot.session \
      --input-root /kaggle/input \
      --encoder /path/vvencapp --decoder /path/vvdecapp \
      --yolo-repo /path/yolov5 --weights /path/yolov5n_v7_locked.pt \
      --output /kaggle/working/rcdot_pilot --mode new \
      --workers 2 --gpu-ids 0,1 --encoder-threads 2 \
      --rollout-steps 64 --additional-timesteps 256 \
      --ppo-learning-rate 3e-5 --ppo-epochs 3 --ppo-target-kl 0.05 --ppo-aux-loss-coef 0.1 \
      --validation-videos 2 --validation-rates 50,120 --validation-frames 64 \
      --val-every-updates 2 --max-hours 10

Inspect metrics, validation, action spread, target/actual bitrate, KL,
clip fraction, `max_approx_kl_observed` and `stopped_by_target_kl`.
The prior real-data smoke completed 256 interactions and two PPO updates,
with approximate KL 3.95/69.14 before the first stabilization. A subsequent
stabilized 4096-transition Trial ran end-to-end but saw degraded Validation
reward after update2, 5/8 target-KL stops, growing auxiliary MSE (0.7→426.8)
and enormous critic-dominated global gradients (up to 220k). See
[specific diagnosis](scripts/rl_rc_dot/TRIAL_4096_DIAGNOSIS.md).

**The corrected policy architecture is not compatible with those previous
checkpoints. Start MODE=new, PHASE=pilot for a fresh 256-step smoke; only
advance to a new Trial after checking the persisted diagnostic metrics.** A finite optimizer update alone does
not prove a good policy. Do not select reward/hyperparameters using Test100.

## Full experiment

Start a new official run after the pilot is reviewed. Default training budget
is **20,000,000 interactions**, following the reported paper budget rather
than inventing a fixed3-epoch rule. Two collectors replace the paper's8 to fit
the available hardware. Full Validation is7 clips ×5 fixed target rates.
Time required on VVC/T4×2 must be measured, not inferred from the paper's x264
runtime. A reduced frame budget must be reported as a reduced experiment.

    python -m scripts.rl_rc_dot.session \
      --input-root /kaggle/input \
      --encoder /path/vvencapp --decoder /path/vvdecapp \
      --yolo-repo /path/yolov5 --weights /path/yolov5n_v7_locked.pt \
      --output /kaggle/working/rcdot_official --mode new \
      --workers 2 --gpu-ids 0,1 --max-hours 11.75 \
      --ppo-learning-rate 3e-5 --ppo-epochs 3 --ppo-target-kl 0.05

For a **bounded new run**, add `--additional-timesteps 20480`; the 20M
maximum remains in place, but the session stops at the smaller budget.
An optional `--validation-frames 64` explicitly reduces validation cost;
keep it unchanged when resuming and report it as a reduced validation
protocol. Before final Test100 claims, do an appropriately long held-out
Validation assessment. Never use Test100 to choose policy hyperparameters.

max-hours includes this command's setup/preflight. To count notebook setup
too, pass --session-start-utc with time.time() captured in its first cell.
A reserve (default10min, capped at10% for very short sessions) is kept for
closing encoders and exporting. This is a soft stop, not recovery from an
instant VM kill. Checkpoints are committed after updates and at normal stop;
an abrupt kill recovers the last committed generation available on disk.

## Resume

Upload the exported **_KAGGLE_SAFE.zip** package as Input. Kaggle can extract
the outer ZIP; session_payload.bin retains the inner ZIP bytes. Both raw
exports and verified extracted folders are supported.

    python -m scripts.rl_rc_dot.session \
      --input-root /kaggle/input \
      --encoder /path/vvencapp --decoder /path/vvdecapp \
      --yolo-repo /path/yolov5 --weights /path/yolov5n_v7_locked.pt \
      --output /kaggle/working/rcdot_official --mode resume \
      --resume-from /kaggle/input/MY_DATASET/session_payload.bin \
      --workers 2 --gpu-ids 0,1 --max-hours 11.75 \
      --ppo-learning-rate 3e-5 --ppo-epochs 3 --ppo-target-kl 0.05

Keep the same training and validation configuration as the imported run.
Do not promote a reduced pilot's validation score into the full experiment.
Changing total/session interaction budget is allowed; changing method/source/
profile/horizon/collector count is rejected.

Resume restores policy, optimizer, counters, parent RNG and sampler RNG,
then starts fresh codec episodes and a fresh on-policy rollout. It does not
pretend to serialize an in-flight VVenC reference-picture buffer. Partial
rollouts may be discarded. num_timesteps counts observed interactions;
trained_frames counts completed PPO-rollout interactions.

## Outputs

- method.json: source/dataset/profile/algorithm/validation contract
- progress.json: committed latest/previous/best checkpoint references
- checkpoints/: at most current, previous and best generations, with hashes/RNG
- metrics/: update and episode logs; validations/: completed paired validation
- episodes/, validation_media/: compressed states, actions, AU/reconstruction
  hashes, detections and encoder logs/bitstreams (raw YUV is cleaned)
- exports/: sealed session ZIP and byte-preserving Kaggle-safe ZIP
- history.json: index of previous-session evidence, retained in the user's
  previous export; previous media is not repeatedly copied into new sessions

Best selection uses the macro mean per-frame reward on Validation. Clip
Precision/Recall/F1/PSNR/actual bitrate are also saved. Incomplete validation
does not replace the best checkpoint. Historical exports should be retained.

## Evaluation

Use the sealed best-validation model.zip from checkpoints, retaining its
manifest and the run's method.json. Evaluation verifies them before loading.

    python -m scripts.rl_rc_dot.evaluate \
      --input-root /kaggle/input --encoder /path/vvencapp \
      --decoder /path/vvdecapp --yolo-repo /path/yolov5 \
      --weights /path/yolov5n_v7_locked.pt \
      --checkpoint /path/run/checkpoints/OBJECT/model.zip \
      --output /kaggle/working/rcdot_test --max-pairs 20

Default Test100 ×5 rates uses full clips. Resume reuses only points with
matching identity and intact stream/trajectory hashes. Report uses actual
bitrate and overlap-only PCHIP on an explicitly reported Pareto frontier;
undefined BD results stay null. BD-Precision is **not mAP**. The paired
VVenC anchor here is the matched CTU32 profile described above.

## Evidence and limits

CPU tests and real codec fixtures validate implementation contracts.
YOLO fixtures in unit tests are labelled synthetic and prove no detection
gain. A 256-interaction Kaggle T4×2 pilot with real BDD100K/YOLO has now
confirmed end-to-end execution, while revealing unstable PPO updates.
The new stability configuration still requires a **fresh Kaggle 256-step
smoke test**, followed by a longer controlled training/validation run.

Method sources:
- https://arxiv.org/html/2501.12216v2
- https://github.com/Puiching-Memory/RL-RC-DoT (independent reconstruction)
- https://stable-baselines3.readthedocs.io/en/v2.7.0/modules/ppo.html

The integrated SB3 extension is independently implemented from the documented
API/update, with SB3's MIT notice in scripts/rl_rc_dot/licenses/SB3-MIT.txt.
No source from the independent reconstruction is redistributed.

Evaluation also accepts --resume-from for its own sealed export; it always
requires the separately verified training checkpoint. Each evaluation session
exports completed/partial evidence plus a _KAGGLE_SAFE.zip and reports only
verified completed pairs. Retain the previous session packages for comparison.
