# RL-RC-DoT to VVenC: technical specification, adapter/3

Current method definitions are below; older results are explicitly historical.
Operational instructions: [project README](../../README_RL_RC_DOT.md).

## State mapping

| Paper information | VVC adapter measurement |
|---|---|
| Next QP | Current decision's native slice QP |
| Frame number | POC; coding order is recorded separately |
| Bitstream size | Completed Annex-B cumulative bits |
| Current QP | Previous completed picture's area-weighted actual luma CU QP |
| Average QP | Area-weighted luma CU QP across completed pictures |
| Intra fraction | Final luma CU area, non-Inter/non-Skip class |
| Inter fraction | Final luma CU area, Inter excluding Skip |
| Skip fraction | Final luma CU Skip area |
| PSNR | Completed native Y-PSNR, lossless capped99 |
| SSIM | Completed luma SSIM averaged over non-overlapping8×8 windows |
| Motion bits | Final CABAC inter-prediction syntax bits / actual AU bits |
| Coefficient bits | Final residual-coefficient syntax bits / actual AU bits |
| Progress | Completed decisions / requested frames |
| Rate error | Signed log(completed-prefix bitrate / target) |
| Frame type | B=0, P=1, I=2 |
| Complexity | Mean source-lookahead intra SATD |
| Local energy |8×8 low-resolution source variance per16×16 source cell |
| Local intra cost | Min DC/H/V low-resolution Hadamard SATD |
| Local propagation | Ten-frame backward propagation using a bounded single-previous-frame motion/dependency graph |
| Inverse qscale |2^(-(native QPA QP-4)/6), sampled to local cells |


## Native measurements and causality

Final CU statistics come from winning luma CABAC coding before CU cleanup.
Completed reconstruction/PSNR/SSIM/AU output precede their publication in the
next state. history_poc=-1 explicitly denotes cold-start history. No detection
or current-picture coded outcome is used as policy input before its action.

The four local channels are VVC-oriented source analogues. Their simple
intra SATD search is not full VVC RDO; propagation is a bounded
single-previous-reference dependency graph, not x264 MB-tree. Inter syntax
bits include more than MV components; transform syntax is not DCT-only.
SSIM uses non-overlapping8x8 luma windows. These differences must be declared.

Additional source preanalysis is bounded by POC+10. Native GOP/RC lookahead
also remains enabled; this is not equivalent to x264's complete latency or
reference buffering. YOLO is needed for reward/evaluation, not deployment.

## State, action and native profile

- Globals: QPs/63, POC/clip frames, log1p(completed bits/100000), PSNR/60,
  tanh(rate error), frame type/2 and tanh(complexity/1500).
- Maps: log1p on energy/intra/propagation, then per-channel spatial z-score
  with epsilon1e-6. These are versioned implementation choices.
- 35 Gaussian values are clipped to[-10,10], transformed by12*tanh, and
  half-pixel bilinear interpolated5x7->20x30. Float packets use canonical
  nine-significant-digit values; echoed encoder consumption is checked exactly.
- Adapter/3 delta adds to the **native QPA QP**, using area pooling at a fresh
  legal QG and legal-QP clipping. Zero must preserve QPA QP/lambda and reproduce
  matched anchor bit-for-bit. This differs from frame-QP-relative x264 action.
- Profile: medium,480x320@30, internal10bit, one-pass RC, GOP32, LookAhead1,
  CTU32, DQPsubdiv2, MaxParallelFrames0, default2 CTU threads.
- Legal CU/QG grouping and zero-residual QP prediction still coalesce cells;
  there is no guarantee of600 independently signalled16x16 quantizers.

Anchor uses the same research CTU32 profile with native QPA. Stock CTU128
baseline points must not be silently reused or relabelled as this anchor.

## Current reward, labels and optimization

Original-unit reward is20*frame Precision-abs(log(prefix actual bps/target bps)).
Prefix rate is completed Annex-B bits*30/completed picture decisions. PPO
receives0.05 times that scalar; report/validation retain original units.
Positive rescaling preserves reward ordering for the isolated objective,
but does not preserve all loss/gradient/optimizer dynamics.

Detector protocol: YOLOv5n v7.0, car class2, conf0.25, NMS IoU0.45,
maxdet300, FP32/autoshape640. Source commit915bbf294bb74c859f0b41f1c23bc395014ea679;
weights SHA2564f180cf23ba0717ada0badd6c685026d73d48f184d00fc159c2641284b2ac0a3.
Source/weights/clean tracked files are checked. BT.709 TV I420->RGB must match
the frozen FFmpeg byte protocol. Explicit logical CUDA placement avoids the
pinned YOLO helper's late remasking. Coordinates follow frozen rounding.

Pseudo-label matching is confidence-ordered, one-to-one, IoU>=0.5; empty
Precision=0. A reconstructed prediction contributes matched1/unmatched0 at its
box-centre16x16 cell; multiple predictions in a cell are averaged.
The adapter/3 mask is the union of source/reconstruction detection-centre
cells, including missed-object and false-positive locations. Weighted MSE
uses occupancy*(1+4*target); target1 receives5x weight relative to active
target0. Fractional labels receive intermediate weights. Empty-object batches
have zero auxiliary gradients. This is detector-derived supervision, not a
causal label of how QP changes task quality.

SB3 2.7.0 PPO uses GAE and clipped surrogate, normalized advantages, auxiliary
coefficient0.1, learning_rate3e-5, n_epochs3, target_kl0.05, threshold0.075.
Actor/critic have independent CNNs, kernel3x3/3x3/4x4, stride1, channels16/32/32;
flatten+16 globals ->128 ->64. Gaussian actor and scalar value head use their
own latents. Actor auxiliary head64->128->600 ends with sigmoid. Actor+aux
and critic gradient groups are disjoint and clipped separately at0.5.

The paper describes a shared64-D representation feeding its three heads.
Independent branches, masked/weighted auxiliary, reward scaling, exact
normalization and PPO settings above are **declared adaptations**. Their
presence is not proof of improvement, and this is not a bit-identical authors'
implementation. No SAC/Partition/MTS head has been added. Some widths/block
assignment conventions are explicit choices where released detail is absent.

Before optimization, the whole rollout is checked against the collecting
policy's values/logprobs and echoed consumed maps. NaN/Inf guards cover states,
actions, labels, objectives, gradients, parameters and optimizer state.
KL, clipping, return/value moments, explained variance, auxiliary occupancy
and branch gradient norms are recorded. Sampling std alone is not evidence
of useful content-adaptive decisions.

## Dataset, selection and reporting

The frozen full lock is65/7/100; Mini deterministically selects16/4/30 by
car-box-count quartiles and seeded SHA ordering. It does not change the full
asset. No train/validation/test overlap is allowed. MOV bytes are hash checked.
Mini training uses256-frame windows starting at0/128/256/384/512, with random
Train video and integer target50..200 kbps. Validation/Test use fixed starts.
Full training defaults to20M interactions; reduced budgets must be disclosed.

Validation compares native anchor, zero and learned policy, choosing best
by macro mean original-unit per-frame reward. Clip F1 and final bitrate are
diagnostics. Incomplete validation cannot replace best. Keep test for final
assessment. Pilot/trial/full have different contracts, not resumable by
silently switching the phase or dataset mode.

Report builds Pareto frontiers using actual bitrate and PCHIP only on overlap,
without extrapolation; undefined BD stays null. Five-rate Mini results cover
short256-frame clips, not full videos. Three-rate quick mode reports paired
metrics without BD claims. BD-Precision is not mAP or annotation-ground-truth
accuracy. Segmentation/tracking are not implemented here.

Required main comparison is H264+RL-RC-DoT, VVenC without RL, and VVenC+RL.
**The current runner implements only the latter two under matched CTU32.**
H264+RL needs a compatible trained reference and common evaluation protocol;
it is not replaced by H264 alone or zero action. Comparing different codec
and policy variants measures a combined change unless policy differences are
controlled or explicitly analysed. Use the same splits/frame windows/detector
and disclose budgets, seeds, rate-control settings and deviations.

## Checkpoint and evidence contracts

Native schema vvenc-rcdot-vvc-adapter/3; run/export containers remain/2.
Source fingerprint, native schema, model/optimizer/RNG, dataset, profile,
workers, horizon, PPO and validation settings are sealed in the contract.
MD documents and notebook paths are excluded from source_fingerprint, so this
documentation consolidation does not invalidate a compatible adapter/3 run.
Adapter/2 and older V3/V4/DOT_PROXY checkpoints are incompatible.

Atomic generation folders plus progress pointer preserve current/previous/best.
SAFE keeps the inner ZIP byte-for-byte in session_payload.bin; all file hashes
are verified before load. Archive traversal/duplicates/symlinks are rejected.
Sampler and global RNG restore before fresh codec episodes/rollouts; no claim
of resuming an in-flight reference buffer. Previous-media history remains in
retained original exports. Validation control cache currently recomputes after
export/import, which can add time without changing the policy.

## Verification and experiment history

Current native source was built/tested on Windows/GCC15.2, Release, JSON ON,
SIMD OFF.62 tests executed:61 passed and1 shared-libswscale RGB test skipped.
Two actual native-codec/PPO/checkpoint/validation tests passed with a clearly
labelled synthetic detector; they are not real BDD/YOLO learning evidence.

Five32-frame native runs (anchor, zero, spatial pattern, isolated POC-/+)
pass: full CU pixel coverage, finite mode/history statistics, actual Annex-B
bits, source/reconstruction POC identity, exact consumed actions, spatial QP
direction, isolated-picture effect and all-frame decoder parity. Zero stream
equals anchor byte-for-byte. Tested local encoder SHA256:
65efc45cf9c85a9543d34e9b83d4fb42476a3fefbedd9958748a8f0740711924.
This is provenance for the local binary, not a required Kaggle compiler hash.

Linux CI/Kaggle mandates RGB byte equality. Actual adapter/3 BDD100K/YOLO
T4x2 execution, GPU placement, learning curves and coding efficiency still
require observed pilot results. GitHub CI results are not inferred from the
local test summary. Input pin recovery explicitly fetches exact dependency
commits; old training_session/decoder_alignment import failures belong to
earlier notebooks, not a reason to remove current verification gates.

Historical evidence, retained to explain changes rather than claim gains:

| Run/version | Evidence | Interpretation |
|---|---|---|
| Early real256 smoke |2 PPO updates; KL3.95/69.14; clip fractions0.799/0.938 | End-to-end execution, unstable optimization |
| Shared-feature4096 Trial |270/270 files verified;4096 transitions;8 updates;4 reduced validations. Best update2/1024 reward16.069; latest14.307; anchor13.770. KL guard stopped5/8, clipping up to0.406 | No monotone improvement or coding-efficiency proof |
| Same4096 Trial diagnostics |Aux MSE0.707->426.778; pre-clip norm19,677->220,549.82 sampled states: mean abs latent6.85->41.56; value55.57->347.05; mean abs aux3.00->16.69, extremes12.75->72.07. Cumulative bits3.46M gave normalized34.6 under old linear scaling | Evidence consistent with branch interference, not causal proof |
| Adapter/2 20,480 Trial |40 updates; best5,120 reward15.30682; latest15.20359; anchor14.67397. Latest Precision+3.40pp/F1+9.13pp with bitrate+9.27%; auxiliary still poor | Higher detection with higher rate; no convergence or rate-saving conclusion |
| Current adapter/3 |Native parity gate and technical tests above pass | Real-data learning benefit not established |

Historical changes: linear cumulative bits ->log1p; shared->separate branches;
separate clip groups; bounded aux predictions; later QPA-relative deltas,
masked/weighted aux and reward rescaling. Frequent old validation interrupted
long source episodes. These are reasons for controlled experiments, not
grounds to resume incompatible old weights or declare the new architecture
better without measurement.

## Sources and licensing

- Paper: https://arxiv.org/html/2501.12216v2
- Independent public reconstruction: https://github.com/Puiching-Memory/RL-RC-DoT
- SB3 PPO: https://stable-baselines3.readthedocs.io/en/v2.7.0/modules/ppo.html

Integrated SB3 extension is independently implemented; SB3 MIT notice is in
licenses/SB3-MIT.txt. No independent-reconstruction source is redistributed
as authors' released training code. Standard codec licenses remain applicable.
