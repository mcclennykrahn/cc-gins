# Stage4 revalidation

This invocation found Stage4 already implemented and independently reviewed at
75ec525 (implementation 57a7d90). Preserve that evidence and Stage3 PASS.
No detector/navigation/model changes, tuning, or fresh campaign were performed.

The separate `revalidate.py` arithmetic audit imports no evaluator code and
checks all 112 stored rows: clean alarms; onset-derived detection delays;
preattack versus no-alarm miss partitions; reason-event and interval sums;
reset/fallback totals; pooled final horizontal errors. PASS. Existing
`selfcheck.py` was rerun: PASS, including historical differences and final
batch/stream parity. This is independent arithmetic, not a new external reviewer;
the prior independent review remains at `stage4_independent_review.json`.

Enabled reused diagnostic seeds still show 7/10 clean alarms and 4/10 detections
per rate, with 6/10 misses all overlapping preattack alarms. Reason events count
invalidation calls; reasons overlap and repeated checks are not independent
GNSS failures. Seeds 900–909 were already exposed. Coverage/kappa/reseed-age/DR
covariance growth remain unmeasured. No fresh held-out or generalization claim.
DETECTION FAIL; confirmed_v08=false; TCN and CUSUM remain unpromoted.

Regression outcome is recorded in `revalidation_pytest.log`. Initial shell
`python` lookup failed before any tests ran; rerun uses the existing `.venv`.
