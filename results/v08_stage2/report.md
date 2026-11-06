# V0.8 Stage 2 detection campaign (2026-11-06)

## Scope and preregistration

This stage tested the preregistered dual-CUSUM grid in
[protocol.md](protocol.md): `(k,h)` = `(0,20)`, `(3,50)`, `(6,100)`, with the
historical 600 s default-noise scenario, 300 s east ramp onset, rates 0.2, 0.5
and 1.0 m/s, and clean runs without attacks or outages. Selection used only
validation seeds 500–503. The winning (least-bad) candidate was frozen in
[selection.json](selection.json) before any held-out run; seeds 900–909 were
then evaluated once by the separate `test` command. No candidate was promoted
to runtime defaults, no attack or noise definition was retuned, and no grid
expansion followed the held-out result.

Commands (repository root, matching the protocol):

```
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/v08_stage2/evaluate.py select > results/v08_stage2/selection.log 2>&1
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/v08_stage2/evaluate.py test > results/v08_stage2/test.log 2>&1
.venv/bin/python -m pytest -q > results/v08_stage2/pytest.log 2>&1
```

The frozen evaluator SHA-256 is
`9d27085fba400b12b8d7ef5bf40bace319b42a4cbe583ae5ce4e7f73f9bbefe1`; the legacy
checkpoint SHA-256 is
`e2495238df1cf3093155f26cec929d2341f03abb489f04b8acb9dd6a9f515445`.
`selection.json` records `held_out_used_for_selection: false`.

## Validation: all three candidates fail

Validation used 4 clean trajectories and 4 attacked trajectories per rate
(2,400 s clean exposure; 2,400 clean GNSS epochs). Acceptance required zero
clean alarms and all 12 attacks detected after onset on the validation split.

| Candidate (k, h) | Clean alarms | Detected 0.2 | Detected 0.5 | Detected 1.0 | Misses / pre-onset per rate | Result |
|---|---:|---:|---:|---:|---|---|
| (0, 20) | 4/4 | 0/4 | 0/4 | 0/4 | 4 / 4 | FAIL |
| (3, 50) | 2/4 | 2/4 | 2/4 | 2/4 | 2 / 2 | FAIL |
| (6, 100) | 2/4 | 2/4 | 2/4 | 2/4 | 2 / 2 | FAIL |

No candidate met the zero-clean-alarm condition; `(0,20)` also detected no
post-onset attack. `(3,50)` and `(6,100)` each lost seeds 501 and 503 to
pre-onset latched alarms at every rate. The evaluator's miss count includes
pre-onset alarms (`delay_s` is null for both), so the "misses / pre-onset"
entries describe the same two trajectories per rate, not four distinct
failures.

Per protocol, the least-bad candidate was frozen for diagnosis even though
validation failed. Ranking first minimizes clean plus pre-onset alarms, then
misses, then total detected delay with grid-order tie-breaking. `(3,50)` and
`(6,100)` tie on the first two keys; `(3,50)` wins on detected delay
(validation sums 125 s versus 150 s). [selection.json](selection.json) therefore
records `validation_pass: false` and `frozen_candidate: {cusum_k: 3.0,
cusum_h: 50.0}`. This is a diagnostic freeze only, **not a promotion**.

## Held-out diagnostic: 6/10 clean alarms, 5/10 detected per rate

The frozen `(3,50)` candidate was run on held-out seeds 900–909 at each rate,
with 10 clean runs (6,000 s clean exposure, 6,000 clean GNSS epochs). The raw
rows and summary are in [metrics.json](metrics.json).

| Rate (m/s) | Runs | Detected after onset | Misses | Pre-onset alarms | Detected delays (s) |
|---|---:|---:|---:|---:|---|
| 0.0 (clean) | 10 | — | — | 6 alarms | alarm times 74, 90, 103, 156, 158, 416 |
| 0.2 | 10 | 5 | 5 | 5 | 16, 33, 123, 14, 15 |
| 0.5 | 10 | 5 | 5 | 5 | 16, 10, 8, 6, 13 |
| 1.0 | 10 | 5 | 5 | 5 | 7, 6, 5, 5, 8 |

For each attack rate, five of ten trajectories were detected after onset and
five were not. Those same five non-detections are the five pre-onset latched
alarms (seeds 900, 903, 904, 905 and 908 at 103, 158, 74, 90 and 156 s), so the
reported "misses" and "pre-attack alarms" are overlapping counters for one set
of five failed trajectories; there were no additional post-onset misses. On
the clean split, six of ten trajectories latched: seeds 900, 903, 904, 905 and
908 (the same pre-onset times) plus seed 909 at 416 s. An early latched alarm
is not a successful ramp detection.

The held-out summary contains no clean run free of alarms for the frozen
candidate, and detection coverage is only 50% at every rate. No delay average
is presented as a detection-performance result because it conditions on the
five successful runs only. This diagnostic does not establish a false-alarm
rate per hour; 6,000 s of clean exposure is a finite campaign, not an
operational integrity standard.

## Gate and promotion decision

`selection.json` has `validation_pass: false`; `metrics.json` has
`detection_pass: false` because the frozen validation split did not pass and
the held-out summary also fails the same engineering acceptance rule. The
mandatory V0.8 **DETECTION gate therefore FAILS**.

- No detector parameters were promoted; runtime defaults are unchanged.
- No attack, noise, checkpoint, scenario or correlation-setting retuning was
  performed for this campaign.
- The frozen `(3,50)` candidate exists only as a documented diagnostic in
  `selection.json` / `metrics.json`; it is not a new primary configuration.
- This result does not prove that every possible CUSUM setting fails, only that
  this preregistered three-point grid fails validation and its least-bad member
  does not generalize on the held-out diagnostic.

## Reproducibility

- Full suite: **159 passed in 89.44s** (`pytest.log`).
- Focused protocol tests: 10 passed in 3.56s (`focused.log`).
- Raw held-out rows, per-rate delays/misses and clean alarm times are preserved
  in `metrics.json`; validation rows are preserved in `validation.json`.
- Historical tables, checkpoints and published numbers were not rewritten.

## Next step: robust navigation, not another threshold sweep

[blocked_followup.md](blocked_followup.md) proposes the next falsifiable step: an
opt-in, shared batch/stream guard-reference health state covering the whole DR
reset interval, paired with the existing hybrid-to-fixed-NHC fallback. It would
hold bias-monitor/ACI reference updates when the reference is unhealthy, drop
learned forward aiding with a recorded reason, and add unit/parity/fallback
tests before a new preregistered validation-only experiment. The proposal also
requires fresh independent seeds before any later generalization claim.

That robust-nav proposal is **not implemented in this part**, and it is not
asserted to close the DETECTION gate. Whole-window reference health may reduce
contaminated-reference propagation while increasing DR drift and false alarms.
