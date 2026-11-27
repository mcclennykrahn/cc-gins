# NHC cadence trial: NOT_IMPROVED

Candidate `nhc_cadence_1hz` sets `pseudo_every=100`: 1 Hz fixed NHC versus the baseline 10 Hz. All defaults remain unchanged; candidate unpromoted. See protocol.md for architecture review and frozen seed/config protocol.

| Seed | Baseline horizontal RMSE (m) | Candidate (m) |
|---|---:|---:|
| 900 | 7.173489 | 7.923964 |
| 901 | 5.619521 | 6.379096 |
| 902 | 3.258814 | 4.256097 |
| 29201 | 1.632407 | 1.587251 |
| 29202 | 11.249125 | 13.557263 |
| 29203 | 1.662705 | 1.723919 |
| 29204 | 4.175897 | 4.138994 |
| 29205 | 4.935118 | 7.048534 |

development mean horizontal RMSE: 5.350608 → 6.186386 m. Pooled RMSE: 5.587421 → 6.366486 m. Mean horizontal coverage: 0.795026 → 0.928270; normalized position ANEES: 1.683885 → 1.200308.

held_out mean horizontal RMSE: 4.731050 → 5.611192 m. Pooled RMSE: 5.895179 → 7.156867 m. Mean horizontal coverage: 0.717921 → 0.822252; normalized position ANEES: 2.300604 → 1.483684.

All three development seeds and three of five held-out seeds regress. Lower NHC frequency weakens useful aiding in this simulation; this experiment does not support the correlation hypothesis as an accuracy improvement. No further tuning on these seeds. Full precision metrics include outage errors. Small simulation-only evidence does not establish real-data or detection performance.

Review provenance: initial implementation incorrectly used motion_every=100 (stationary aiding schedule), yielding identical baseline/candidate horizontal RMSE on all eight seeds. This no-op was detected by self-check and independent review (FAIL), corrected to pseudo_every=100, and the entire campaign rerun. The table and JSON contain only the corrected campaign. Independent re-review passed after the correction. The initial no-op identity is self-check provenance; the independent reviewer verified the scheduling error and audited only corrected rows.

Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. /tmp/ccgins-accuracy-venv/bin/python scripts/evaluate_nhc_cadence.py`. Regression: same interpreter with `-m pytest -q`.

DETECTION remains FAIL; confirmed_v08=false. Existing Stage3 and prior negative evidence remain preserved.

Final independent re-review: PASS after correction; independent targeted regression 35 passed in 7.22s. Full regression: 199 passed in 75.96s. Self-check and diff check PASS. Candidate NOT_IMPROVED and unpromoted.
