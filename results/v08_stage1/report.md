# V0.8 Stage 1: opt-in integrity recovery

Stage 0 criteria were written before implementation in `docs/v08_criteria.md`
and the runner control directory. This stage advances RECOVERY, not overall
V0.8 acceptance. See `docs/v08_recovery.md` for API, failure modes and contracts.
Legacy `results/motionnet.pt` remains primary and unchanged. TCN is not promoted.

## Frozen protocol and commands

```
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/v08_stage1/evaluate.py > results/v08_stage1/evaluate.log
.venv/bin/python -m pytest -q > results/v08_stage1/pytest.log 2>&1
```

The evaluator uses unmodified `make_scenario` default sensor noise, 12-second
runs, hybrid + ACI + fixed NHC + Huber 1.5 + dual CUSUM k=3/h=50, correlation
inflation (40,40,40), canonical legacy checkpoint, and explicit engineering
recovery M=3/q<=3. No parameter search or test-based selection occurred. Epoch-zero
GNSS is excluded in both paths. Attack: a separate gross +10000 m horizontal
transient at GNSS epochs 3–5 s. It is a mechanism fixture, not a replacement for
any historical low-noise/default-noise ramp or evidence of subtle attack detection.

Validation mechanism seeds 500–503 and held-out mechanism seeds 900–909 are
reported separately. Both are short simulation probes, not the locked 600-second
outage protocol. Historical ramp seeds/labels/noise were not changed. This campaign
does not claim validation-tuned thresholds. Settings must remain opt-in.

## Measurements from this session

| Split | Clean alarms / runs | Clean GNSS epochs | Gross attacks detected / runs | Recovery completed / attacked runs | Accepted fix after recovery within horizon |
|---|---:|---:|---:|---:|---:|
| Validation mechanism | 0 / 4 | 48 | 4 / 4 | 4 / 4 | 4 / 4 |
| Held-out mechanism | 0 / 10 | 120 | 10 / 10 | 10 / 10 | 9 / 10 |

All attack first alarms occurred at 3 s (zero epoch delay for this gross injection).
Recovery completed at 8 s except held-out seed 904 at 12 s, the final horizon;
that seed therefore has no subsequent fix to demonstrate reconnection. The
completing fix remains excluded. Nine other held-out runs accepted again at 9 s.
Each clean trajectory spans 12 s; this limited exposure cannot estimate a reliable
FA/hour rate or guarantee a bound. No such rate is claimed.

Maximum final position and full covariance absolute batch/stream differences
were both 0 across all evaluator runs. This is matched-grid final-state evidence,
not every-epoch or off-grid equality. JSON retains each GNSS decision, q, g,
exclusion, acceptance, recovery count and streak, plus batch recovery times.

Focused tests cover default permanent latch, interrupted healthy streak,
nonfinite/negative q, invalid recovery config, repeat alarm, exclusion of recovery
completion, loaded legacy batch/stream agreement and post-recovery ACI resumption
in a separate 40-second validation-seed regression. Full-suite outcome is recorded
in `pytest.log` and the runner selfcheck.

## Gate accounting and limits

RECOVERY advances to implemented/tested for the opt-in research mechanism;
PRIMARY compatibility and REPRODUCIBILITY gain evidence. Default historical
behavior stays latched. ACI can resume from current healthy state instead of being
permanently blocked by historical first alarm. No certified fail-safe is claimed.

DETECTION remains FAIL: no validation-tuned CUSUM or default-noise ramp improvement,
no full-protocol false-alarm/attack benchmark. Whole-window guard-reset health
remains unresolved; q can be permissive when DR covariance grows and cannot
authenticate GNSS. A long drift can also prevent recovery. Stage 2 must tune only
on validation, freeze settings, then evaluate original clean/ramp protocols on
held-out seeds; failed validation blocks promotion and redirects effort to primary
fallback robustness.

COVERAGE evidence retains the pre-existing fifth-batch 0.691387 measured fraction,
not nominal 0.95. Bounded/sliding ACI and correlated references confer no closed-loop
mathematical guarantee. No new coverage improvement is claimed. HPL remains
FAIL/unimplemented. UrbanNav raw TC remains experiment_substitute. No 18-state
mount, online last-layer SGD, RTS or TCN promotion. Matrix counts remain complete 5,
partial 17, experiment_substitute 2, unimplemented 4; no completion percentage.

Final full-suite command above completed: **157 passed in 57.06s**. Focused
recovery tests separately completed: **8 passed in 3.52s**. `git diff --check`
was clean. These are session measurements, not the pre-existing baseline count.
