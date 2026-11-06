# cc_diagnostics_s4 repeated invocation

Stage4 was already complete at 8053bc0. Separate arithmetic audit and selfcheck PASS; full regression: 180 passed in 87.87s (0:01:27). No implementation or campaign rerun was necessary. Existing independent source review remains PASS; no new external reviewer was used. Provenance hashes cover original Stage3 and Stage4 evidence.

All 112 stored rows audited. Enabled reused diagnostic seeds: clean alarms 7/10; each attack rate detected 4/10, missed 6/10, with all six misses overlapping preattack alarms and zero no-alarm misses. Pooled cumulative calls: missing 130416, stale 6103, rejected 14846, alarm 14818; affected intervals respectively 456, 51, 548, 530. Forced/nonfinite/recovery calls and intervals are zero. Causes overlap; calls include repeated freshness checks and are not independent GNSS failures or durations.

Seeds 900–909 were already exposed. This invocation stops at the evidence limit: no fresh held-out experiment, generalization or detection improvement. Coverage/kappa/reseed-age/DR covariance growth remain unmeasured. DETECTION FAIL; confirmed_v08=false; TCN/CUSUM unpromoted. Prior Stage3 PASS and campaign artifacts preserved.
