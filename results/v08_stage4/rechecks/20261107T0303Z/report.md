# Stage4 evidence-limited stop

Separate stored-row arithmetic audit and selfcheck PASS; 112 stored rows checked. Existing independent source review retained; this invocation has no new external reviewer or simulation. Prior Stage3 and Stage4 evidence hashes are unchanged (provenance.json). Regression results are recorded in pytest.log and review.json.

Enabled reused diagnostic seeds: clean alarms 7/10; each rate detects 4/10, misses 6/10, preattack alarms 6/10, no-alarm misses 0. Pooled reason events: missing 130416, stale 6103, rejected 14846, alarm 14818; forced/nonfinite/recovery 0. These overlapping counts measure invalidation calls, including repeated freshness checks, not independent GNSS failures.

No fresh held-out evidence or new research gap closed. Coverage/kappa/reseed-age/DR covariance growth remain unmeasured. DETECTION FAIL; confirmed_v08=false; TCN/CUSUM unpromoted. Stop at evidence limit. The stage is already implemented; repeated verification does not establish research progress. Future scheduling should require a concrete new diagnostic question or an unexposed evaluation protocol before rerunning this stage. Other repo untouched; unrelated .dsh-prompts excluded.
