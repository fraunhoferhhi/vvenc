# Method mapping: publication → declared VVC adapter/2

The goal is a method-level port before further RL improvements. The authors'
training code/checkpoint and all implementation details are unavailable.
This table defines what the implementation actually measures.

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

## Important qualifications

Mode statistics are captured in final CABAC coding, before CU memory is
released. They are published only after reconstruction and AU output.
Cold-start history is explicitly identified by history_poc=-1. No current
frame's coded outcome is included before choosing its action.

The four local statistics are **source-derived VVC analogues**. The intra
search is not VVC's full RDO search. The propagation graph is not x264's
full MB-tree/reference hierarchy. Motion syntax includes more than just MV
components, and VVC transforms differ from DCT-only x264 coefficients.
Luma SSIM implementation details also differ. These are declared adaptations,
not claims of exact x264 feature equality. There are no silent zero
placeholders for unavailable fields.

The added pre-analysis uses raw source only and no detector output. Its own
source window is bounded by POC+10. VVenC's native GOP/RC lookahead remains
enabled as declared in the profile; this is not a claim that the entire
encoder has the same buffering/latency as x264 lookahead10.
The policy does not need YOLO at deployment.
Training/reward still require the frozen detector.

## State and action definitions

Globals use fixed scaling: QPs/63, POC/clip frames, completed bits/100000,
PSNR/60, tanh(rate error), type/2 and tanh(complexity/1500).
For each local channel: log1p on energy/intra/propagation, then per-channel
spatial z-score with epsilon1e-6. These normalization choices are explicit
because the paper does not release all normalization constants.

35 Gaussian actions are clipped to[-10,10] as SB3 Box actions, transformed
by12*tanh and half-pixel bilinear interpolation to20×30. Floating maps are
written atomically and echoed by the encoder. At each fresh QG, area pooling
and legal QP clipping determine the native QP relative to the slice QP.
No repeated addition to descendants is intended. Zero map replaces local
QPA with the slice-QP reference; it is not a vanilla-parity assertion.

CTU32 / DQPsubdiv2 limits coalescing compared with the old CTU128 path.
Native legal CU/QG partition and zero-residual QP prediction still govern
the effective signalling resolution. Final CU traces are the evidence.

## Reward, auxiliary target and PPO

Task reward is20*frame Precision; the rate term is
-abs(log(actual completed-prefix bps / target bps)).
This same per-frame objective selects checkpoints on held-out Validation.
Aggregated clip F1 and bitrate are diagnostics, not a different selection
formula.

Matching is confidence-ordered, one-to-one, IoU>=0.5. Empty Precision=0.
A prediction contributes1 if matched, otherwise0, to the16×16 cell
containing its centre. Multiple predictions in a cell are averaged.
Empty cells have target0 and **uniform** MSE weight. False positives remain
supervised at0. This label placement/matching convention is disclosed; exact
author block-assignment code is unavailable.

PPO is SB3 v2.7.0 with its defaults (including GAE), except for the additional
0.1 auxiliary MSE. The shared CNN is3×3,3×3,4×4, stride1, channels16/32/32;
flatten +16 globals →128 →64; Gaussian actor and value head use this latent;
auxiliary head is64 →128 →600. Intermediate widths/coarse5×7 action follow
the public reconstruction where the publication is underspecified.

Official budget defaults to20M interactions, full training clips, randomly
sampled Train65 and integer targets50..200kbps. This is not a975-episode
curriculum. Two environments are a hardware adaptation from the reported8.
A shorter run must be described with its actual frame/update budget.

## Scientific comparison

The supplied anchor and RL use the same declared CTU32 profile, inputs,
preprocessing, detector and metric convention. It is a **matched research
anchor**, not an assertion of stock-default VVenC equivalence.
Stock CTU128 baseline points from previous experiments must not be silently
substituted. Test100 remains frozen and is not used for reward or checkpoint
selection. No claim of improvement over VVenC or the paper is justified
until real held-out measurements are complete.
