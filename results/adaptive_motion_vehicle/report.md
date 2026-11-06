# Adaptive ZUPT on real i2Nav IMU + RTK

32 paired cases / 64 trajectory runs. All cases retained. Profile: vehicle.
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
| ICM20602 | A | eskf | clean | 0.0338 → 0.0529 | 1.000 → 0.993 | 2.042 → 2.063 | 64 | 0 |
| ICM20602 | A | eskf | outage | 7.2315 → 7.2315 | 0.965 → 0.965 | 5.063 → 5.057 | 21 | 0 |
| ICM20602 | A | inekf | clean | 0.0339 → 0.0530 | 1.000 → 0.993 | 2.042 → 2.064 | 64 | 0 |
| ICM20602 | A | inekf | outage | 7.4026 → 7.4026 | 0.960 → 0.960 | 5.109 → 5.101 | 21 | 0 |
| ICM20602 | B | eskf | clean | 0.0230 → 0.0226 | 1.000 → 1.000 | 2.073 → 1.989 | 108 | 0 |
| ICM20602 | B | eskf | outage | 20.6684 → 20.7352 | 0.939 → 0.940 | 4.124 → 4.152 | 52 | 0 |
| ICM20602 | B | inekf | clean | 0.0231 → 0.0227 | 1.000 → 1.000 | 2.072 → 1.988 | 108 | 0 |
| ICM20602 | B | inekf | outage | 20.9414 → 21.0430 | 0.925 → 0.925 | 4.662 → 4.689 | 52 | 0 |
| ADIS16460 | A | eskf | clean | 0.0418 → 0.0392 | 0.984 → 0.993 | 0.970 → 0.920 | 542 | 0 |
| ADIS16460 | A | eskf | outage | 4.5654 → 3.5420 | 0.838 → 0.890 | 1.655 → 1.344 | 543 | 0 |
| ADIS16460 | A | inekf | clean | 0.0418 → 0.0393 | 0.984 → 0.993 | 0.972 → 0.922 | 542 | 0 |
| ADIS16460 | A | inekf | outage | 4.5587 → 3.5231 | 0.826 → 0.878 | 1.685 → 1.378 | 543 | 0 |
| ADIS16460 | B | eskf | clean | 0.0403 → 0.0382 | 0.983 → 0.991 | 0.946 → 0.892 | 370 | 0 |
| ADIS16460 | B | eskf | outage | 3.4779 → 3.3112 | 0.868 → 0.805 | 1.616 → 1.832 | 369 | 0 |
| ADIS16460 | B | inekf | clean | 0.0403 → 0.0382 | 0.983 → 0.991 | 0.947 → 0.892 | 370 | 0 |
| ADIS16460 | B | inekf | outage | 3.4072 → 3.2614 | 0.869 → 0.807 | 1.598 → 1.818 | 369 | 0 |
| ADIS16465 | A | eskf | clean | 0.0250 → 0.5837 | 0.999 → 0.853 | 0.369 → 11.556 | 70 | 4 |
| ADIS16465 | A | eskf | outage | 3.5233 → 22.8594 | 0.908 → 0.659 | 1.126 → 68.083 | 74 | 4 |
| ADIS16465 | A | inekf | clean | 0.0252 → 20.5812 | 0.999 → 0.002 | 0.371 → 13626.564 | 9 | 4 |
| ADIS16465 | A | inekf | outage | 3.5461 → 274.1290 | 0.922 → 0.001 | 1.134 → 27554.739 | 9 | 4 |
| ADIS16465 | B | eskf | clean | 0.0257 → 0.0255 | 0.998 → 0.998 | 0.430 → 0.427 | 99 | 0 |
| ADIS16465 | B | eskf | outage | 2.9026 → 1.6469 | 0.892 → 0.884 | 0.774 → 0.709 | 99 | 0 |
| ADIS16465 | B | inekf | clean | 0.0256 → 0.0254 | 0.998 → 0.998 | 0.429 → 0.426 | 99 | 0 |
| ADIS16465 | B | inekf | outage | 2.9108 → 1.6493 | 0.892 → 0.883 | 0.771 → 0.706 | 99 | 0 |
| i300 | A | eskf | clean | 0.0174 → 0.4900 | 1.000 → 0.879 | 0.377 → 5.462 | 506 | 4 |
| i300 | A | eskf | outage | 2.5232 → 18.6187 | 0.983 → 0.617 | 0.718 → 35.393 | 506 | 4 |
| i300 | A | inekf | clean | 0.0180 → 17.6702 | 1.000 → 0.001 | 0.376 → 7565.079 | 9 | 4 |
| i300 | A | inekf | outage | 2.5151 → 277.2522 | 0.981 → 0.000 | 0.707 → 30027.261 | 9 | 4 |
| i300 | B | eskf | clean | 0.0180 → 0.0179 | 1.000 → 1.000 | 0.423 → 0.415 | 355 | 0 |
| i300 | B | eskf | outage | 2.6740 → 1.7181 | 1.000 → 1.000 | 0.583 → 0.697 | 357 | 0 |
| i300 | B | inekf | clean | 0.0180 → 0.0179 | 1.000 → 1.000 | 0.423 → 0.415 | 355 | 0 |
| i300 | B | inekf | outage | 2.6725 → 1.7165 | 1.000 → 1.000 | 0.580 → 0.694 | 357 | 0 |

## Paired outage maxima (all events, including regressions)

| IMU | Duration | n | Mean maximum baseline → adaptive (m) | Mean endpoint baseline → adaptive (m) | Improved / tied / regressed |
|---|---:|---:|---:|---:|---:|
| ICM20602 | 30s | 8 | 8.0885 → 8.0885 | 6.3950 → 6.3950 | 0 / 8 / 0 |
| ICM20602 | 60s | 8 | 50.5833 → 50.8034 | 40.0210 → 40.2411 | 0 / 6 / 2 |
| ICM20602 | 120s | 8 | 61.1652 → 61.1652 | 16.5080 → 16.4687 | 0 / 8 / 0 |
| ADIS16460 | 30s | 8 | 2.8182 → 2.8226 | 2.7591 → 2.7640 | 0 / 6 / 2 |
| ADIS16460 | 60s | 8 | 13.6918 → 7.6885 | 7.9409 → 7.5597 | 4 / 2 / 2 |
| ADIS16460 | 120s | 8 | 11.0760 → 10.4482 | 8.8471 → 8.2193 | 2 / 4 / 2 |
| ADIS16465 | 30s | 8 | 3.8741 → 80.6060 | 3.8493 → 80.5714 | 0 / 4 / 4 |
| ADIS16465 | 60s | 8 | 3.4464 → 91.0236 | 2.5608 → 90.5081 | 3 / 2 / 3 |
| ADIS16465 | 120s | 8 | 10.8233 → 191.8748 | 8.9959 → 182.3901 | 2 / 2 / 4 |
| i300 | 30s | 8 | 3.2512 → 83.3829 | 3.1869 → 83.3214 | 1 / 4 / 3 |
| i300 | 60s | 8 | 5.4189 → 94.8762 | 4.2346 → 94.7843 | 3 / 2 / 3 |
| i300 | 120s | 8 | 9.6782 → 189.5848 | 5.0604 → 176.6881 | 2 / 2 / 4 |
| ALL | 30s | 32 | 4.5080 → 43.7250 | 4.0476 → 43.2629 | 1 / 22 / 9 |
| ALL | 60s | 32 | 18.2851 → 61.0980 | 13.6893 → 58.2733 | 10 / 12 / 10 |
| ALL | 120s | 32 | 23.1857 → 113.2683 | 9.8528 → 95.9415 | 6 / 16 / 10 |

Global RMSE improved / tied / regressed: 20 / 0 / 12. Accepted ZUPTs: 7150; accepted at reference speed >0.5 m/s: 32.

## Limits

This is one shared drive, four co-mounted IMUs, two correlated time folds and one initial seed. Event counts are descriptive; no independence or statistical significance claim.
The filter-speed veto can suppress real stops after large outage drift, and can admit smooth motion if the filter is already wrong. Zero observed false-motion accepts is not a safety guarantee; stops below 0.5 m/s include creep. Raw gyro mean conservatively rejects sensors with large turn-on bias.
The vehicle caps were revised after the conservative pilot rejected the calibration-half vibration floor (0.40–0.72 m/s² accel std). This is exploratory development, not a blind confirmatory evaluation. The pilot is retained in results/adaptive_motion_conservative. The unguarded vehicle profile is retained in results/adaptive_motion_vehicle; its large regressions motivated the guarded default, so this evaluation must not be treated as held-out validation. Calibration is frozen after the training half; caps can reject highly vibrating stationary vehicles. No online bias/threshold learning from accepted ZUPTs, avoiding self-reinforcing detection.
Odometry is integrated in the same pipeline with explicit mounting rotation and IMU-to-wheel lever arm, Earth-rate correction, timestamp validation, validity mask and NIS gating. i2Nav has no wheel-speed stream: no odometry accuracy claim or synthetic wheel experiment is reported. Slip, scale, timing offsets and geometry uncertainty remain unmodelled; caller must calibrate geometry and exclude slipping epochs.
Reference is used for initial-state construction (existing protocol) and scoring only; this is not unaided cold-start navigation.

Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.motion_experiment --workers 4 --profile vehicle --out results/adaptive_motion_vehicle`
