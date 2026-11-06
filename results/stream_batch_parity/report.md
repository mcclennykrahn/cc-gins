# Stream / batch consistency on frozen published configurations

Replay frozen published configurations, without changing model/scoring defaults.

Protocol: all 64 guarded motion paths (4 IMUs x 2 folds x 2 filters x
clean/outage x baseline/ZUPT), plus all 24 NO-NHC stream_motion paths.
Compare every 10th IMU epoch, all position/velocity components and all 225
covariance entries, atol=1e-12, rtol=0. No fitting or case exclusion.
Synthetic wheel remains reference-derived, NOT real odometry validation.


88 complete batch + stream pairs; all cases retained. Every logged epoch checked. All motion counters/times equal. Published batch RMSE, coverage and ANEES unchanged to 1e-12.

| Maximum absolute discrepancy | Value |
|---|---:|
| position_max_abs | 0 |
| velocity_max_abs | 0 |
| position_cov_max_abs | 0 |
| full_cov_max_abs | 0 |
| velocity_block_cov_max_abs | 0 |
| position_block_cov_max_abs | 0 |
| vel_pos_cross_cov_max_abs | 0 |

## Diagnosis and scope

Streaming previously silently omitted fixed mounted NHC, making motion-report configurations inequivalent. It now applies the same fixed mounted NHC after ZUPT/wheel and before GNSS, with the same WINDOW warmup and pseudo cadence. Unsupported learned/ACI/monitor/adaptive-R/range configurations raise an error.
Batch propagates with nominal truth.dt. Timestamp subtraction in streaming introduced tiny interval errors on real resampled grids; these can accumulate during outages. from_truth now uses nominal dt for whole fixed-grid intervals within 1e-9 s; off-grid GNSS subintervals still use their actual durations. Batch estimator behavior and default stop confirmation are unchanged.
Full covariance uses each filter’s internal error coordinates (ESKF and InEKF differ); compared only within the same filter. Velocity/position diagonal and cross blocks are included, together with nav-frame position covariance.
Intentional differences: streaming applies start-epoch GNSS while batch starts at the next IMU epoch. Replay explicitly skips valid index-0 GNSS, counts recorded per case. Off-grid GNSS splits propagation in streaming; batch uses indices. Replay checks alignment and never snaps timestamps. Wheel data must be queued before its propagated grid epoch; late GNSS cannot be reconstructed. from_truth supplies fixed dt; a manually constructed navigator without dt uses actual timestamp differences.

## Scientific limits

This is numerical equivalence, not covariance calibration. The NO-NHC synthetic-wheel coverage/ANEES regressions remain: reference-derived speed is an oracle input, not measured odometry accuracy. Previously isolated wheel-stop RMSE 6.4053 → 6.4273 m was worse. No new independent-drive or real-wheel evidence is supplied, and no stop-assist generalization claim is made. Existing adaptive_motion NHC baselines and stream_motion reports are retained.

Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.parity_experiment --workers 4`
