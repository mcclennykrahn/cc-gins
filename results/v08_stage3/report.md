# V0.8 Stage 3 guard-reference health (2026-11-06)

## Verdict and scope

**DETECTION still FAILS. Stage 3 does not close DETECTION. CUSUM and TCN remain
unpromoted; confirmed_v08=false.** The opt-in whole-reset-interval mechanism has
unit and matched-grid parity evidence, but the diagnostic campaign does not
establish improved navigation or detection. Held-out outcomes worsen with the
flag enabled. No acceptance class or runtime default is promoted.

Implementation is `e45900a`; [protocol.md](protocol.md) and the evaluator were
frozen in that commit. The mechanism holds ACI/bias references and drops learned
forward aiding to fixed lateral/vertical NHC after an unhealthy epoch, keeps that
health sticky through the scheduled interval, and excludes alarm/recovery evidence
from reseeding. Each scheduled boundary renews the assessment interval, even when
reseed is skipped. `guard_ref_health=False` remains the default. This is research
mechanism evidence, not an integrity or production acceptance claim.

## Completed reproduction

The interrupted campaign was rerun in full, without changing source or evaluator:

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/v08_stage3/evaluate.py all --workers 8 > results/v08_stage3/evaluate.log 2>&1
```

The process exited **0**. [evaluate.log](evaluate.log) records all validation seeds
500–503 and held_out_diagnostic seeds 900–909 completed. [metrics.json](metrics.json)
contains `parity`, `validation` (32 rows), and `held_out_diagnostic` (80 rows), with
all four rates and both flag values for each seed. Each trajectory lasts 600 s;
attacks start at 300 s at 0.2/0.5/1.0 m/s. The Stage 2 frozen diagnostic `(k,h)=(3,50)`
and scenario/noise definitions were retained; no threshold selection occurred.

The resumed workspace lacked `.venv`; an initial launch failed before execution.
The environment was restored and the command above then completed. Completed-run
environment: Python 3.13.5, NumPy 2.5.4, SciPy 1.18.1, PyTorch 2.15.0+cpu,
PyYAML 6.0.3. This is a newly restored environment, not a claim of identical
packages to the interrupted run. Flag-off campaign counts reproduce Stage 2.

Evaluator SHA-256: `1643755c12ba9e5b38df3e38d8bbd46800db5cba0493dde587b98169115c4819`.
Loaded legacy checkpoint SHA-256:
`e2495238df1cf3093155f26cec929d2341f03abb489f04b8acb9dd6a9f515445`.

The preserved pre-interruption full-suite result in [pytest.log](pytest.log) is
**170 passed in 67.90s (0:01:07)** from `.venv/bin/python -m pytest -q`.
No source code changed during packaging, so pytest was not rerun. This result is
prior-session evidence, not a new full-suite run in the restored environment.

## Mechanism and loaded-checkpoint parity

| Case | Flag | Batch / stream fallback epochs | Reseeds / skipped boundaries | Max final position / full covariance delta |
|---|---|---:|---:|---:|
| Clean, 40 s, seed 500 | off | 0 / 0 | not instrumented | 0 / 0 |
| Clean, 40 s, seed 500 | on | 0 / 0 | 1 / 0 | 0 / 0 |
| Outage 10–14 s, 50 s, seed 500 | off | 0 / 0 | not instrumented | 0 / 0 |
| Outage 10–14 s, 50 s, seed 500 | on | 195 / 195 | 0 / 1 | 0 / 0 |

The outage records one unhealthy interval and `history_reasons=["stale"]`;
health is renewed at the 30 s boundary. Both evaluator mechanism booleans pass.
The evaluator clean no-op boolean alone compares batch/stream deltas, rather than
direct flag-on/off trajectories; the preserved full-suite clean no-op unit test
compares flag-on/off trajectories and final covariance directly. These are narrow
matched-grid loaded-checkpoint checks, not general off-grid covariance equality.

## Validation and reused held-out diagnostics

Clean exposure is 2,400 s per flag in validation and 6,000 s per flag in held-out.
Each rate has 4 validation or 10 held-out trajectories per flag.

| Split | Health flag | Clean alarms | Detected at 0.2 / 0.5 / 1.0 m/s | Misses per rate | Pre-onset alarms per rate |
|---|---|---:|---|---:|---:|
| Validation | off | 2/4 | 2/4, 2/4, 2/4 | 2 | 2 |
| Validation | on | 2/4 | 2/4, 2/4, 2/4 | 2 | 2 |
| Held-out diagnostic | off | 6/10 | 5/10, 5/10, 5/10 | 5 | 5 |
| Held-out diagnostic | on | 7/10 | 4/10, 4/10, 4/10 | 6 | 6 |

Misses include pre-onset latched alarms: the miss and pre-onset counters refer to
the same failed trajectories at each rate, not disjoint failures. Early latched
alarms are not successful ramp detections. Raw alarm times and delays are retained
in metrics; no success-conditioned mean delay is presented as overall performance.
Seeds 900–909 were already used in Stage 2, so these are reused diagnostic seeds,
not fresh independent evidence for generalization or a calibrated false-alarm rate.

| Split | Health flag | Mean / max final horizontal error (m) | Fallback epochs | Recorded reseeds / skips |
|---|---|---:|---:|---:|
| Validation | off | 15.488622 / 68.798859 | 0 | not instrumented |
| Validation | on | 25.292962 / 92.752697 | 54,030 | 113 / 207 |
| Held-out diagnostic | off | 17.231177 / 31.720281 | 0 | not instrumented |
| Held-out diagnostic | on | 20.870568 / 75.649998 | 141,540 | 255 / 545 |

Navigation summaries pool clean and attacked runs (16 validation / 40 held-out
per flag); they are final horizontal errors, not trajectory RMSE. Flag-off reset
counts in JSON are zero placeholders because guard-health instrumentation is
absent, not evidence of zero historical reseeds. Enabled fallback and reset/skip
counters are cumulative. Both pooled mean and maximum final error worsen with the
flag enabled; this campaign supports no navigation-improvement claim.

## Measurement limits and remaining gates

Although the protocol calls reason codes cumulative, the implementation/evaluator
only exports final-interval reason-code presence in summaries. All campaign
summary reason counts are zero; they do not mean no earlier unhealthy epochs.
`history_reasons` preserves unique earlier reason names, not event counts. The
outage parity fixture explicitly records historical `stale`. Cumulative per-reason
counts therefore remain a measurement gap; the protocol's metric description is
not fully met. No code was changed to repair or reinterpret this after evaluation.

Coverage, kappa saturation, elapsed reseed age and DR covariance growth were not
measured here. Pre-existing coverage_h=0.691387 remains disclosed, not nominal 0.95.
HPL remains unimplemented and UrbanNav raw multi-satellite TC remains
`experiment_substitute`. Whole-interval health is now implemented, but broad
robustness, long-alarm recovery validity, independent review and fresh-seed
acceptance remain unresolved. Matrix counts remain complete=5, partial=17,
experiment_substitute=2, unimplemented=4.

The mechanism checks and preserved regression suite support review of the opt-in
implementation. They do not override the failed default-noise detection campaign.
Independent Stage 3 review is pending; packaging selfcheck is not independent
approval. See [handoff/README.md](handoff/README.md).

## S3-01 independent-review fix addendum

Independent review found that interval renewal could clear an active alarm and
resume learned forward aiding at 30.1–31.0 s. Fixed in `d8167d8`: batch and learned
stream now record all boundary-epoch evidence in the new assessment interval,
including alarm, recovery, rejection, forced and nonfinite disqualifiers. DR
reseeding remains alarm-gated; `guard_ref_health=False` remains the default.
Loaded-checkpoint regression checks fixed-NHC fallback across the alarmed boundary,
zero alarm-time reseeds and batch/stream position/full-covariance parity. Additional
boundary tests cover rejection, forced fixes and completing recovery in both paths.
The original defect-asserting repro now exits with `Defect no longer reproduced`.

This is a mechanism fix awaiting independent re-review. Historical campaign tables
above describe the pre-fix implementation and have not been rerun or rewritten.
**DETECTION remains FAIL; confirmed_v08=false.**

Post-fix full suite: `.venv/bin/python -m pytest -q` — **177 passed in 106.79s
(0:01:46)**; preserved in [s301_pytest.log](s301_pytest.log).
