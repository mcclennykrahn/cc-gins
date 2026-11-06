# Adaptive ZUPT on real i2Nav IMU + RTK

32 paired cases / 64 trajectory runs. All cases retained. Profile: conservative.
Vehicle/guarded caps: accel std 3 m/s², gyro std 0.05 rad/s. Conservative profile: 0.25 m/s² and 0.01 rad/s.
Guarded profile (default): two consecutive accepted RTK displacement upper-speed bounds <0.25 m/s (distance + 2 sigma, epochs <=2 s apart) confirm rest. Any IMU dwell reset or moving RTK fix (even filter-rejected) revokes confirmation. New stops during outages cannot be confirmed. Other profiles omit this independent stop guard.
Baseline: ESKF/InEKF + mounting-calibrated NHC with correlation-inflated sigma; Earth compensation and real receiver epoch sigma (0.05 m floor).
Same seed 0, IMU, RTK, geometry, initialization, and outages within each pair. First 30 s excluded from global metrics.
Calibration: lower-decile one-second IMU variance on the opposite time half, 4x margin, sensor noise density floor and physical caps. Existing training-half RTK/INS mounting calibration reused.
Causal 0.5 s trailing window, 1 s dwell; speed <0.5 m/s, raw gyro mean <0.02 rad/s and gravity magnitude within 0.25 m/s². ZUPT sigma 0.05 m/s at 10 Hz, chi-square 0.999 gate; never forced.
IMU rates indexed k cover [t[k], t[k+1]]; detector sees only propagated samples. No test truth or future samples used by detection.
Clean and fixed outage schedule (first=60 s, 30/120/60/30/120/60 s, 40 s gaps) run on both test halves (~803 s each).

| IMU | Fold | Filter | Case | RMSE baseline → adaptive (m) | Coverage baseline → adaptive | ANEES baseline → adaptive | ZUPT accepted | Speed >0.5 m/s |
|---|---|---|---|---:|---:|---:|---:|---:|
| ICM20602 | A | eskf | clean | 0.0338 → 0.0338 | 1.000 → 1.000 | 2.042 → 2.042 | 0 | 0 |
| ICM20602 | A | eskf | outage | 7.2315 → 7.2315 | 0.965 → 0.965 | 5.063 → 5.063 | 0 | 0 |
| ICM20602 | A | inekf | clean | 0.0339 → 0.0339 | 1.000 → 1.000 | 2.042 → 2.042 | 0 | 0 |
| ICM20602 | A | inekf | outage | 7.4026 → 7.4026 | 0.960 → 0.960 | 5.109 → 5.109 | 0 | 0 |
| ICM20602 | B | eskf | clean | 0.0230 → 0.0230 | 1.000 → 1.000 | 2.073 → 2.073 | 0 | 0 |
| ICM20602 | B | eskf | outage | 20.6684 → 20.6684 | 0.939 → 0.939 | 4.124 → 4.124 | 0 | 0 |
| ICM20602 | B | inekf | clean | 0.0231 → 0.0231 | 1.000 → 1.000 | 2.072 → 2.072 | 0 | 0 |
| ICM20602 | B | inekf | outage | 20.9414 → 20.9414 | 0.925 → 0.925 | 4.662 → 4.662 | 0 | 0 |
| ADIS16460 | A | eskf | clean | 0.0418 → 0.0418 | 0.984 → 0.984 | 0.970 → 0.970 | 0 | 0 |
| ADIS16460 | A | eskf | outage | 4.5654 → 4.5654 | 0.838 → 0.838 | 1.655 → 1.655 | 0 | 0 |
| ADIS16460 | A | inekf | clean | 0.0418 → 0.0418 | 0.984 → 0.984 | 0.972 → 0.972 | 0 | 0 |
| ADIS16460 | A | inekf | outage | 4.5587 → 4.5587 | 0.826 → 0.826 | 1.685 → 1.685 | 0 | 0 |
| ADIS16460 | B | eskf | clean | 0.0403 → 0.0403 | 0.983 → 0.983 | 0.946 → 0.946 | 0 | 0 |
| ADIS16460 | B | eskf | outage | 3.4779 → 3.4779 | 0.868 → 0.868 | 1.616 → 1.616 | 0 | 0 |
| ADIS16460 | B | inekf | clean | 0.0403 → 0.0403 | 0.983 → 0.983 | 0.947 → 0.947 | 0 | 0 |
| ADIS16460 | B | inekf | outage | 3.4072 → 3.4072 | 0.869 → 0.869 | 1.598 → 1.598 | 0 | 0 |
| ADIS16465 | A | eskf | clean | 0.0250 → 0.0250 | 0.999 → 0.999 | 0.369 → 0.369 | 0 | 0 |
| ADIS16465 | A | eskf | outage | 3.5233 → 3.5233 | 0.908 → 0.908 | 1.126 → 1.126 | 0 | 0 |
| ADIS16465 | A | inekf | clean | 0.0252 → 0.0252 | 0.999 → 0.999 | 0.371 → 0.371 | 0 | 0 |
| ADIS16465 | A | inekf | outage | 3.5461 → 3.5461 | 0.922 → 0.922 | 1.134 → 1.134 | 0 | 0 |
| ADIS16465 | B | eskf | clean | 0.0257 → 0.0257 | 0.998 → 0.998 | 0.430 → 0.430 | 0 | 0 |
| ADIS16465 | B | eskf | outage | 2.9026 → 2.9026 | 0.892 → 0.892 | 0.774 → 0.774 | 0 | 0 |
| ADIS16465 | B | inekf | clean | 0.0256 → 0.0256 | 0.998 → 0.998 | 0.429 → 0.429 | 0 | 0 |
| ADIS16465 | B | inekf | outage | 2.9108 → 2.9108 | 0.892 → 0.892 | 0.771 → 0.771 | 0 | 0 |
| i300 | A | eskf | clean | 0.0174 → 0.0174 | 1.000 → 1.000 | 0.377 → 0.377 | 0 | 0 |
| i300 | A | eskf | outage | 2.5232 → 2.5232 | 0.983 → 0.983 | 0.718 → 0.718 | 0 | 0 |
| i300 | A | inekf | clean | 0.0180 → 0.0180 | 1.000 → 1.000 | 0.376 → 0.376 | 0 | 0 |
| i300 | A | inekf | outage | 2.5151 → 2.5151 | 0.981 → 0.981 | 0.707 → 0.707 | 0 | 0 |
| i300 | B | eskf | clean | 0.0180 → 0.0180 | 1.000 → 1.000 | 0.423 → 0.423 | 0 | 0 |
| i300 | B | eskf | outage | 2.6740 → 2.6740 | 1.000 → 1.000 | 0.583 → 0.583 | 0 | 0 |
| i300 | B | inekf | clean | 0.0180 → 0.0180 | 1.000 → 1.000 | 0.423 → 0.423 | 0 | 0 |
| i300 | B | inekf | outage | 2.6725 → 2.6725 | 1.000 → 1.000 | 0.580 → 0.580 | 0 | 0 |

## Paired outage maxima (all events, including regressions)

| IMU | Duration | n | Mean maximum baseline → adaptive (m) | Mean endpoint baseline → adaptive (m) | Improved / tied / regressed |
|---|---:|---:|---:|---:|---:|
| ICM20602 | 30s | 8 | 8.0885 → 8.0885 | 6.3950 → 6.3950 | 0 / 8 / 0 |
| ICM20602 | 60s | 8 | 50.5833 → 50.5833 | 40.0210 → 40.0210 | 0 / 8 / 0 |
| ICM20602 | 120s | 8 | 61.1652 → 61.1652 | 16.5080 → 16.5080 | 0 / 8 / 0 |
| ADIS16460 | 30s | 8 | 2.8182 → 2.8182 | 2.7591 → 2.7591 | 0 / 8 / 0 |
| ADIS16460 | 60s | 8 | 13.6918 → 13.6918 | 7.9409 → 7.9409 | 0 / 8 / 0 |
| ADIS16460 | 120s | 8 | 11.0760 → 11.0760 | 8.8471 → 8.8471 | 0 / 8 / 0 |
| ADIS16465 | 30s | 8 | 3.8741 → 3.8741 | 3.8493 → 3.8493 | 0 / 8 / 0 |
| ADIS16465 | 60s | 8 | 3.4464 → 3.4464 | 2.5608 → 2.5608 | 0 / 8 / 0 |
| ADIS16465 | 120s | 8 | 10.8233 → 10.8233 | 8.9959 → 8.9959 | 0 / 8 / 0 |
| i300 | 30s | 8 | 3.2512 → 3.2512 | 3.1869 → 3.1869 | 0 / 8 / 0 |
| i300 | 60s | 8 | 5.4189 → 5.4189 | 4.2346 → 4.2346 | 0 / 8 / 0 |
| i300 | 120s | 8 | 9.6782 → 9.6782 | 5.0604 → 5.0604 | 0 / 8 / 0 |
| ALL | 30s | 32 | 4.5080 → 4.5080 | 4.0476 → 4.0476 | 0 / 32 / 0 |
| ALL | 60s | 32 | 18.2851 → 18.2851 | 13.6893 → 13.6893 | 0 / 32 / 0 |
| ALL | 120s | 32 | 23.1857 → 23.1857 | 9.8528 → 9.8528 | 0 / 32 / 0 |

Global RMSE improved / tied / regressed: 0 / 32 / 0. Accepted ZUPTs: 0; accepted at reference speed >0.5 m/s: 0.

## Limits

This is one shared drive, four co-mounted IMUs, two correlated time folds and one initial seed. Event counts are descriptive; no independence or statistical significance claim.
The filter-speed veto can suppress real stops after large outage drift, and can admit smooth motion if the filter is already wrong. Zero observed false-motion accepts is not a safety guarantee; stops below 0.5 m/s include creep. Raw gyro mean conservatively rejects sensors with large turn-on bias.
The vehicle caps were revised after the conservative pilot rejected the calibration-half vibration floor (0.40–0.72 m/s² accel std). This is exploratory development, not a blind confirmatory evaluation. The pilot is retained in results/adaptive_motion_conservative. The unguarded vehicle profile is retained in results/adaptive_motion_vehicle; its large regressions motivated the guarded default, so this evaluation must not be treated as held-out validation. Calibration is frozen after the training half; caps can reject highly vibrating stationary vehicles. No online bias/threshold learning from accepted ZUPTs, avoiding self-reinforcing detection.
Odometry is integrated in the same pipeline with explicit mounting rotation and IMU-to-wheel lever arm, Earth-rate correction, timestamp validation, validity mask and NIS gating. i2Nav has no wheel-speed stream: no odometry accuracy claim or synthetic wheel experiment is reported. Slip, scale, timing offsets and geometry uncertainty remain unmodelled; caller must calibrate geometry and exclude slipping epochs.
Reference is used for initial-state construction (existing protocol) and scoring only; this is not unaided cold-start navigation.

Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.motion_experiment --workers 4 --profile conservative --out results/adaptive_motion_conservative`
