# Research batch: adaptive ZUPT and odometry integration

96 paired cases / 192 actual trajectory runs across three retained profiles. Each profile: 4 IMUs × 2 time folds × 2 filters × clean/outage × baseline/updates. One initialization seed (0), six outages per outage case; 96 paired outage events per profile, 288 including ablations. These are repeated observations of one shared drive, not independent trials.

The conservative pilot made no updates. The unguarded vibration profile exposed false stops and severe divergence; it is retained as a failed ablation. The guarded default was developed in response to those failures. This is exploratory research on available data, not held-out confirmation.

| Profile / case | Mean horizontal RMSE baseline → updates (m) | Improved / tied / regressed cases | Accepted ZUPTs | Accepts at reference speed >0.5 m/s |
|---|---:|---:|---:|---:|
| conservative / clean | 0.0282 → 0.0282 | 0 / 16 / 0 | 0 | 0 |
| conservative / outage | 5.9700 → 5.9700 | 0 / 16 / 0 | 0 | 0 |
| vehicle / clean | 0.0282 → 2.4824 | 10 / 0 / 6 | 3670 | 16 |
| vehicle / outage | 5.9700 → 41.8525 | 10 / 0 / 6 | 3480 | 16 |
| guarded / clean | 0.0282 → 0.0275 | 16 / 0 / 0 | 3582 | 0 |
| guarded / outage | 5.9700 → 5.9694 | 8 / 7 / 1 | 1010 | 0 |

## Guarded default by IMU (both folds and filters)

| IMU | Clean RMSE baseline → updates (m) | Outage RMSE baseline → updates (m) | Outage 95% horizontal coverage baseline → updates | Outage position ANEES baseline → updates |
|---|---:|---:|---:|---:|
| ICM20602 | 0.0284 → 0.0283 | 14.0610 → 14.0610 | 0.9161 → 0.9161 | 6.7475 → 6.7475 |
| ADIS16460 | 0.0411 → 0.0388 | 4.0023 → 3.9998 | 0.7423 → 0.7423 | 2.2149 → 2.2140 |
| ADIS16465 | 0.0254 → 0.0253 | 3.2207 → 3.2207 | 0.8271 → 0.8271 | 1.4318 → 1.4318 |
| i300 | 0.0178 → 0.0176 | 2.5962 → 2.5962 | 0.9836 → 0.9836 | 0.8731 → 0.8731 |

## Verification and interpretation

`OPENBLAS_NUM_THREADS=1 .venv/bin/python -m pytest -q`: **83 passed in 32.45s** (all original 73 plus 10 new test cases).
Tests cover calibration response/caps, causal dwell, speed/rotation/gravity vetoes, ESKF/InEKF velocity and mounted-wheel Jacobians by finite differences, lever-arm turn correction, NIS rejection, invalid timestamps/geometry, real main-loop ZUPT/odometry application, default stop confirmation and revocation even on a rejected moving fix.
Guarded default requires two accepted slow RTK displacement pairs; any moving valid fix or IMU dwell reset revokes the stop latch. It cannot confirm a new stop during an outage. Zero observed >0.5 m/s accepts does not establish a safety guarantee: slower creep, receiver errors, correlated GNSS noise and causal departure delay remain possible.
Odometry is wired into both main filter pipelines with explicit mounting rotation and wheel-point lever arm, gyro-bias/Earth-rate terms, timestamps, validity mask and NIS gate. No real wheel-speed data exists in these i2Nav files. Odometry accuracy is therefore unvalidated; no truth/RTK-derived wheel speeds were fabricated. Wheel slip, scale, time offset and geometry uncertainty remain caller responsibilities.
Updates are opt-in. No new dataset or paid dependency was added. Streaming integration remains future work.

Detailed per-run metrics, detector thresholds, accepted timestamps, data hashes, mounting calibration, every outage maximum/endpoint and per-IMU tables are retained in each profile directory:

- [Guarded default](report.md) / [metrics](metrics.json)
- [Conservative pilot](../adaptive_motion_conservative/report.md)
- [Failed unguarded vibration profile](../adaptive_motion_vehicle/report.md)
