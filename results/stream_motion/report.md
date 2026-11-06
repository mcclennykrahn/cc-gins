# SYNTHETIC wheel speed: exploratory outage-stop experiment

i2Nav has NO real wheel-speed log. Reference/body velocity is intentionally used as a synthetic measurement here; this is an oracle-assisted mechanism study, not measured odometry accuracy or held-out validation.
8 paired cases, 24 batch trajectories + 8 full stream replays. Four co-mounted IMUs, folds A/B, InEKF, initial seed 0. All cases retained. Same drive/folds as prior development; no significance claim.
No NHC/learned pseudo-measurements: streaming supports IMU/GNSS/motion only. Guarded baseline is therefore a new matched baseline, not the earlier NHC baseline.
Frozen opposite-half IMU variance calibration; existing opposite-half mounting calibration. Synthetic wheel point is the IMU centre (explicit zero lever), vehicle forward axis from calibrated mounting. 10 Hz signed reference/body forward speed, 1% scale error, independent Gaussian sigma=0.01+0.02|v_forward| m/s; +10% slip for seconds 40–45 of each 90 s period, explicitly marked invalid and excluded. Synthetic RNG seed 2026. Assumed known slip is optimistic; no slip estimator.
Methods: guarded ZUPT; guarded ZUPT + synthetic wheel; guarded ZUPT + synthetic wheel + opt-in wheel-zero stop confirmation. The latter two isolate the incremental benefit of stop confirmation from continuous speed aiding.
Wheel stop guard requires two accepted |speed|+3sigma<0.1 m/s observations <=0.2 s apart; expires after 0.2 s. ZUPT still requires causal IMU quiet/dwell, gravity/rotation and filter-speed veto, and is never forced. Invalid observations are not updates; previous evidence expires naturally. Forward zero does not exclude lateral slip.
Fixed six-outage schedule first=60 s, durations 30/120/60/30/120/60 s with 40 s gaps. Real RTK receiver sigma with 0.05 m floor; Earth compensation. Global scoring excludes first 30 s. Reference also initializes the existing protocol and scores trajectories.

| IMU/fold | Guarded RMSE | + synthetic wheel | + wheel stop | Outage ZUPTs (baseline / wheel / stop) |
|---|---:|---:|---:|---:|
| ICM20602/A | 356.8530 | 10.2630 | 10.2630 | 0 / 0 / 0 |
| ICM20602/B | 44.4514 | 10.4627 | 10.4627 | 0 / 0 / 0 |
| ADIS16460/A | 74.5638 | 9.3592 | 9.3093 | 0 / 0 / 277 |
| ADIS16460/B | 22.7202 | 4.5441 | 5.4187 | 0 / 0 / 276 |
| ADIS16465/A | 40.0606 | 5.7066 | 5.6656 | 0 / 0 / 37 |
| ADIS16465/B | 14.0126 | 4.6135 | 4.0595 | 0 / 0 / 69 |
| i300/A | 7.1063 | 3.3438 | 3.3334 | 0 / 0 / 265 |
| i300/B | 8.9167 | 2.9493 | 2.9060 | 0 / 0 / 270 |

Aggregate (arithmetic mean across cases):

| Method | RMSE m | Coverage95 | ANEES/3 |
|---|---:|---:|---:|
| guarded | 71.0856 | 0.7615 | 6.7861 |
| synthetic_wheel | 6.4053 | 0.4691 | 9.6400 |
| synthetic_wheel_stop | 6.4273 | 0.4692 | 9.6991 |

## All paired outage events, including regressions

| Comparison | n | Mean max before → after m | Mean end before → after m | Max improved / tied / regressed |
|---|---:|---:|---:|---:|
| guarded → synthetic_wheel_stop | 48 | 104.0914 → 14.6001 | 104.0914 → 8.8398 | 34 / 0 / 14 |
| synthetic_wheel → synthetic_wheel_stop | 48 | 14.7941 → 14.6001 | 8.8608 → 8.8398 | 12 / 29 / 7 |

Whole-run RMSE guarded → synthetic_wheel_stop: improved / tied / regressed = 8 / 0 / 0.

Whole-run RMSE synthetic_wheel → synthetic_wheel_stop: improved / tied / regressed = 6 / 1 / 1.

Accepted ZUPTs total / inside outages: guarded=423 / 0, synthetic_wheel=373 / 0, synthetic_wheel_stop=1680 / 1194.

Reference-defined new outage stops (speed norm <0.1 m/s for >=1 s, onset strictly inside outage): 12 co-mounted sensor/fold episodes (correlated; three physical stop onsets viewed by four sensors). Counts with at least one accepted ZUPT before stop/outage ends: guarded=0, synthetic_wheel=0, synthetic_wheel_stop=9.
Accepted ZUPTs at reference speed >0.5 m/s: guarded=0, synthetic_wheel=0, synthetic_wheel_stop=0. Creep and near-zero motion are not excluded by this diagnostic.

Stream/batch maximum absolute position discrepancy: 6.93e-07 m. Full per-case position/velocity/covariance discrepancies and motion counts are in metrics.json.

Limits: synthetic speed directly imports reference information. No real wheel accuracy, slip robustness, timing/geometry uncertainty or independent-drive generalization is measured. Large outage drift can keep the filter-speed veto active and prevent real stops. Raw gyro bias can also veto rest. Continuous wheel aiding and stop confirmation effects must be read separately. No tuning on these outcomes.

Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.stream_motion_experiment --workers 4`
