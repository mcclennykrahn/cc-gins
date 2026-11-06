# MotionNetTCN regression investigation (2026-11-06)

All runs in this batch use local CPU. Data are simulated IMU and **synthetic GNSS**.
The authoritative baseline is `58b3dc976046e4f2107b84e5eee8fff9a9bfe59a`.
Historical simulation/KITTI/i2Nav tables and published checkpoints are untouched.

## Reproduction and localization

Commands (from repository root):

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_experiment --out results/proposal_alignment_debug/smoke
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_debug/probe.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_debug/tail_probe.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_debug/smooth_pilot.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_next
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q
```

The rerun reproduces the frozen smoke aggregate metrics exactly (runtime is excluded).
`probe.json` decomposes NLL into channels and variance quantiles. For TCN seed 900,
forward/lateral/vertical NLL is 4.919387 / 0.420892 / 35.277800;
seed 901 is 4.067101 / 0.471885 / 24.418560. Thus the catastrophic aggregate
NLL is principally a **vertical variance tail**, despite small vertical RMSE.
Seed 900 reaches variance `exp(-12) = 6.144212e-6`.
This is a standalone reference-conditioned diagnostic: filter attitude/bias mismatch
cannot cause this particular failure. The probe includes the original two test
seeds solely to diagnose the already-published failure; new selection uses validation only.

The hard clamp kills the variance-head gradient whenever its raw output leaves
[-12,4]. The new regression explicitly demonstrates zero gradient for raw values
-13 and 5 under hard clipping, and finite restoring gradients under smooth clipping.
The optional smooth activation is `-12 + softplus(raw+12) - softplus(raw-4)`.
It keeps the same limiting range and retains gradients near either boundary.
It is stored in checkpoint metadata; old plain state dictionaries and old TCN
metadata default to hard clipping with unchanged predictions. Both architectures
can use either activation. This is an optimization defect/risk, not proof that all
out-of-distribution variance errors have been solved.

`tail_probe.py` inspects the actual pre-clamp outputs of the frozen smoke TCN.
Neither training seed has vertical raw log-variance below -12; seed 900 does
for 3.286979% of diagnostic windows. Seed 901 has minimum -11.414389, so its
large NLL occurs even without hitting the lower clamp. Lower-bound dead gradients
during training are therefore **not established as the cause of vertical collapse**.
This is held-out variance extrapolation/overconfidence. Conversely, forward raw
log-variance exceeds the upper bound on 82.932996% / 96.080910% of training
windows (seeds 100/101): the zero-gradient upper-clamp problem is present in
the actual trained head, not merely a toy possibility. More training and smooth
bounds are helpful validation evidence, not a complete generalization remedy.

The isolated six-epoch **pilot/debug**, using the original smoke training seeds
100/101 and validation 500, changes TCN validation NLL from 1.363208 (hard) to
1.340219 (smooth). No pilot test results select the configuration. More training
and activation are separated by the larger hard/smooth paired validation pilots
in `results/proposal_alignment_debug/validation_pilots.json`.

## Checks that did not identify the suspected bugs

- β-NLL is `.5*(logvar + error²/var)*stop_gradient(var)^.5`, averaged over samples
  and channels. The proposal omits the conventional `.5`; that is a global loss
  factor, not a wrong target or missing variance. Gaussian reporting includes
  `.5*log(2π)`; this constant does not change training gradients. Toy value and
  detached-weight gradient tests pass. We did not change the objective.
- No BatchNorm exists. Mean/std buffers are independently estimated for every
  channel over training windows, including the two radian channels. Scaling is
  not shared between IMU and attitude. Normalization and scale invariance have
  regression coverage. Float64 is used in the mathematical invariance probe to
  avoid artificial cancellation from adding large offsets to tiny float32 signals.
- Roll/pitch extraction matches ZYX body→ENU rotations and is invariant to yaw.
  Constant and per-sample gyro/accel bias subtraction are tested.
- Windows `[idx-100:idx]` and targets `v_body[idx-1]` refer to the same final sample.
  Serving applies that prediction at navigation state `idx`, a 10 ms lag shared
  by both architectures, not the explanation for their relative standalone NLL.
- TCN dilation 1/2/4, kernel 5, padding 2/4/8 preserves window length; receptive
  field is 29 samples before global pooling. Legacy striding gives 29 samples too.
  Global pooling/statistics use the full 100-sample trailing window. Internal
  features are same-padded, not strictly causal per sample; endpoint serving
  consumes no future sample. No architecture redesign was made.
- In hybrid mode only forward learned variance enters navigation; lateral/up
  use fixed NHC. Thus the vertical raw-NLL failure cannot itself explain hybrid
  navigation RMSE. Forward mean error/domain gap and ACI interaction remain
  separate issues. Fixed correlation scale and ACI settings are paired; this
  batch does not claim they are calibrated integrity thresholds.

## Locked assessment and limits

`configs/proposal_alignment_next.yaml` was written before new training: four
600 s training seeds 100–103, validation 500–503, tests 900–909, nominal outages
150/30, 280/60, 420/120 seconds, identical 12-epoch training and fixed ACI settings.
The only candidate is hard versus smooth variance activation; choose the mean
validation raw Gaussian NLL over both architectures, then use the same selected
activation for both. Navigation validation ranks architectures by horizontal RMSE;
test results report the locked comparison, never select hyperparameters.
Calibration seeds 700–705 are unused. This is **reduced-train / not full M3 acceptance**.

The detailed final table, selection record and per-seed measurements are in
[the next report](../results/proposal_alignment_next/report.md).
Reference-conditioned training versus filter-conditioned serving remains a domain
gap; no filter-conditioned training ablation establishes its contribution here.
A finite variance bound and β-weighting do not guarantee calibrated tails.
Full-chain streaming learning+ACI+integrity remains incomplete; so do GNSS κ_g,
online mounting/18-state, HPL/exceedance/recovery, full 20×600 s training,
real-drive TCN validation and UrbanNav raw-pseudorange tightly coupled processing.
