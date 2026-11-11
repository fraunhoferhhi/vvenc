# Trial 4096: root-cause diagnosis and corrective changes (2026-11-11)

## Evidence from the supplied Kaggle-safe export

- Manifest: 270/270 payload files verified; 4096/4096 on-policy transitions, 8 PPO updates, four reduced paired validations.
- Best checkpoint: update2/1024 transitions, validation reward 16.069; latest: update8/4096, reward 14.307; matched anchor 13.770.
- KL guard halted 5/8 updates; clip_fraction reached 0.406.
- Auxiliary uniform [0,1] label MSE: 0.707 (update1) -> 426.778 (update8).
- Shared global pre-clip gradient norm: 19,677 (update1) -> 220,549 (update8).
- Independent direct PyTorch evaluation of stored weights on 82 sampled, normalized training-encoder states: mean |shared latent| 6.85 -> 41.56; value prediction mean 55.57 -> 347.05; mean |aux output| 3.00 -> 16.69; extremes ~12.75 -> ~72.07.
- The original global observation scaled cumulative bits linearly by 100,000; actual trial source states reached 3.46 million bits, implying a normalized feature ~34.6.
- Validation every two 512-frame rollouts explicitly suspended live encoder processes; observed training episodes rarely completed 1200-frame source clips.

These observations support a specific gradient-interference mechanism, not a proven claim of causality or coding-efficiency improvement.

## Fixes and evaluation protocol

1. Split actor/critic CNN feature extraction to prevent high-return critic gradients from perturbing actor features.
2. Clip actor(+auxiliary) and critic gradients **separately**, retaining 0.5 norm per branch; global norm had allowed critic gradients to dominate the clip.
3. Route Auxiliary Head only through actor features; sigmoid output constrains its predictions to [0,1], matching labels. Auxiliary weight remains 0.1 by default.
4. Replace cumulative_bits/100000 with log1p(cumulative_bits/100000); the feature remains causal and monotone.
5. Persist explained variance, return/value moments, RMSE, and branch gradient norms to updates.jsonl, not merely SB3 logger.
6. Make aux coefficient a validated, resume-contract-sealed CLI option for future controlled 0-vs-0.1 ablation.
7. In new Trial notebook, use 7 held-out Validation clips, two rates and a less frequent validation interval (4 updates) to reduce episode interruption.

**Do not resume old adapter/2 checkpoints into changed source.** All old exports and pre-fix validation results are retained as reference evidence. Use a new source-pinned run. Before any larger experiment, rerun a fresh 256-transition smoke and then 4096-transition Trial on the *new* configuration. Compare exact actual bitrates, not only nominal rates. A longer validation (7 videos and 5 target rates) and multiple seeds are required before claiming improved coding performance. The standard 20M interaction budget remains an aspirational reference, not achieved.
