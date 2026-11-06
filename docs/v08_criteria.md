# Research-ready V0.8 acceptance criteria

Written 2026-11-06 before Stage 1 implementation. A research-ready prototype is
not production navigation. Overall PASS requires every mandatory gate below;
Stage 1 may advance a coherent subset without declaring V0.8 achieved.

* PRIMARY (mandatory): documented batch and causal stream contracts, legacy_cnn
  `results/motionnet.pt` primary, hybrid/fixed NHC fallback, ACI, robust GNSS and
  GNSS-free dual guard; matched-grid regression tests and loaded-checkpoint run.
  TCN stays opt-in and unpromoted until independent evidence supports promotion.
* RECOVERY (mandatory): opt-in consecutive-M healthy-epoch recovery in both
  batch and learned stream, excluded alarm epochs, no guard reset while alarmed,
  interruption resets streak, finite-statistic validation, first alarm retained,
  and focused batch/stream parity tests. Defaults preserve historical latching.
  This is a heuristic fail-safe research mechanism, not authenticated recovery.
* DETECTION (mandatory): thresholds selected only on validation seeds 500–503,
  then frozen before evaluation on 900–909, retaining attack/noise definitions.
  Report clean alarm counts, exposure and attack delays/misses separately.
  Default-noise ramp reliability must be evaluated; a failed candidate cannot
  replace the current primary settings. Until this exists this gate FAILS.
* COVERAGE (mandatory evidence gate): retain locked fifth-batch protocol and
  report its pre-existing measured coverage 0.691387, below nominal 0.95.
  Explain bounded/sliding ACI and outage inflation; no mathematical guarantee.
  Any consistency improvement needs validation-only selection and frozen test
  evaluation, including kappa saturation and outage/healthy breakdowns. V0.8 may
  accept explicitly budgeted under-coverage; it cannot claim nominal coverage.
* REPRODUCIBILITY (mandatory): full pytest green, focused tests for new claims,
  commands and validation/test separation in new result artifacts, honest matrix
  item counts and preserved historical tables/checkpoints. No completion percent.
* HPL (optional): currently FAIL/unimplemented. V0.8 may pass with HPL open;
  a future report must label covariance-derived HPL experimental and report
  HPE>HPL without claiming certified integrity.

Pre-existing authoritative baseline: 149 tests passed; TCN test RMSE about
31.5 m versus legacy about 5.15 m, not promoted; low-noise ramp detects but
noise-default ramp misses; no recovery/HPL. Matrix counts complete 5, partial 17,
experiment_substitute 2, unimplemented 4. These are not new session measurements.

Production/later exclusions: certified integrity, FA/hour guarantees, flight/ASIL
certification, UrbanNav multi-satellite raw TC with clocks/NLOS, online 18-state
mounting, last-layer SGD, RTS smoother, TCN promotion and rewriting historical
published tables. UrbanNav scalar experiment remains experiment_substitute.
