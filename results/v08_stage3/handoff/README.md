# Stage 3 independent-review handoff

Implementation: `e45900a` (opt-in guard-reference health). A separate packaging
commit adds the completed evaluator output, report, regression log and matrix
append. Review the implementation and evidence independently; this handoff is
an implementer packaging record, not an independent approval.

Reproduce from the repository root:

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/v08_stage3/evaluate.py all --workers 8 > results/v08_stage3/evaluate.log 2>&1
.venv/bin/python -m pytest -q > results/v08_stage3/pytest.log 2>&1
```

Read `../protocol.md`, `../report.md` and `../metrics.json`. Check sticky interval
health, scheduled boundary renewal, alarm/recovery exclusion, fixed-NHC fallback,
flag-off behavior and loaded-checkpoint parity. The evaluator's clean no-op boolean
alone compares batch/stream deltas; the full-suite clean no-op unit test directly
compares flag-on/off trajectories and final covariance.

Reason-code summaries count final-interval presence, not cumulative event counts;
`history_reasons` records unique earlier reason names. Reset/fallback counters are
cumulative. Coverage, kappa saturation, elapsed reseed age and DR covariance growth
are not measured in this Stage 3 campaign. Reused seeds 900–909 are diagnostic
only; fresh independent seeds are required for a later generalization claim.

DETECTION remains FAIL; CUSUM and TCN remain unpromoted; confirmed_v08=false.
External relay status is in `/workspace/ccgins-v08/review.json` and
`/workspace/ccgins-v08/reviews/stage3_selfcheck.json`; independent review is pending.

Completed recovery: evaluator exited 0 with all 4 validation and 10 held-out
seeds; see `evaluation_completion.json`. Report discloses environment restoration,
prior-session pytest provenance, measurement gaps and worse enabled held-out
outcomes (7/10 clean alarms; 4/10 detection per rate). Independent review remains
pending. Relay push status is updated externally after the packaging commit.
