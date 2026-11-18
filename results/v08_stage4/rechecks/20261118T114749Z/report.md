# Stage4 diagnostics: evidence-limited stop (20261118T114749Z)

DETECTION remains FAIL; confirmed_v08=false. Stage3 PASS is preserved. TCN and CUSUM remain unpromoted. No new held-out campaign, tuning, or implementation change was made.

Restored regression in an isolated CPython 3.13.5 CPU environment: **180 passed in 71.62s (0:01:11)**. Separate revalidation and selfcheck PASS; independently recomputed clean alarms, first-alarm miss partitions and reason totals for all 112 stored rows. All 38 historical evidence file hashes match the prior provenance. Same-agent source/artifact review PASS; historical independent Stage3/Stage4 reviews are preserved, with no new external reviewer.

Stored enabled diagnostic seeds 900–909: 7/10 clean runs alarm over nominal 6000 s. At each of 0.2, 0.5 and 1.0 m/s: 4/10 detections, 6/10 misses, 6/10 preattack alarms, 0/10 no-alarm misses. These seeds were already exposed and provide no fresh held-out generalization evidence.

Cumulative enabled counters across validation and diagnostic splits:

| Reason | Invalidation calls | Affected intervals |
| --- | ---: | ---: |
| missing | 182182 | 637 |
| stale | 7342 | 70 |
| rejected | 20618 | 762 |
| alarm | 20583 | 739 |

Forced/nonfinite/recovery are zero in the stored enabled rows. Reasons overlap; repeated freshness checks count again. These are bookkeeping calls and per-reason intervals, not independent failures, durations or rates. Disabled counters are absent, not measured zeros. Preattack alarms latch the first alarm and are counted as misses; an alarm at onset is detected with zero delay.

Coverage, kappa, reseed age and DR covariance growth remain unmeasured. Historical Stage3 differences predate S3-01 and do not isolate a Stage4 effect. This stage closes the local regression-environment blocker, not the DETECTION evidence gap.

Reproduce regression from the repo with an isolated environment (outside tracked paths):

```sh
uv venv /tmp/ccgins-s4-venv
uv pip install --python /tmp/ccgins-s4-venv/bin/python pytest numpy scipy matplotlib pyyaml --index-url https://pypi.org/simple
uv pip install --python /tmp/ccgins-s4-venv/bin/python torch --index-url https://download.pytorch.org/whl/cpu
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/ccgins-s4-venv/bin/python -m pytest -q
```

Exact tested package versions are in environment.txt. Audit source is retained in the delivery directory. No fresh simulation was run; selfcheck verifies stored parity assertions only.

Driver note: stage_complete cc_diagnostics_s4; regression and stored-evidence checks PASS; evidence-limited stop; DETECTION FAIL; confirmed_v08=false.
