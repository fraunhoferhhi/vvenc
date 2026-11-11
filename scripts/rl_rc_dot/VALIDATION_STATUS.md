# Implementation validation — 2026-11-11

This describes engineering checks, not a trained-policy result.

- Windows / GCC15.2 / CMake Release / JSON ON / SIMD OFF: build successful.
- 44 tests executed:43 passed;1 shared-libswscale RGB test skipped on Windows.
  The skipped test is mandatory in Linux CI/Kaggle setup.
- Real native tests include source→codec→policy→SB3 update→checkpoint→fresh
  rollout and a final-update paired validation/best-checkpoint selection.
  Their detector is an explicitly labelled synthetic fixture.
- Native preflight: five32-frame runs (matched anchor, zero, spatial pattern,
  isolated POC minus/plus). All actual decoded frames equal the native scored
  reconstructions by POC; source fixture POC alignment is also checked.
- Final CU coverage is checked pixel-by-pixel, not only by total area.
  Mode fractions, PSNR/SSIM history and actual Annex-B cumulative bits agree.
- Raw Gaussian actions are checked against canonical9-significant-digit maps
  actually consumed by the encoder before they enter the PPO rollout buffer.
- NaN/Inf, off-policy replay, action mismatch, corrupt exports, unsafe archive
  paths, interrupted checkpoint generations and pending-seed RNG overwrite
  have regression coverage.
- Evaluation math covers rate scaling, duplicate rates, nonoverlap and invalid
  RD points. BD-Precision is not mAP.
- Two spawned CPU collectors pass a shared on-policy update. Explicit GPU routing and loader restoration have mock regression coverage; actual T4 placement remains a pilot gate.
- No real BDD100K/YOLO Kaggle T4×2 training/quality result has been produced
  during this source repair. It remains the next acceptance experiment.
- No claim of bit-identical x264 feature extraction or published paper gains
  is made. METHOD_MAPPING.md lists the codec adaptations and limitations.
