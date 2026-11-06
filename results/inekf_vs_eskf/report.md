# i300: InEKF vs ESKF

Real i300 IMU + RTK + reference, 599.99 s.
Injected outages (start, duration), seconds: [(60.0, 30.0), (130.0, 120.0), (290.0, 60.0)].
Paired initial seeds; no learned or motion pseudo-observations. Earth compensation enabled.
Receiver epoch sigma floor: 0.05 m. Initialization excluded for first 30 s.

| Filter | Horizontal RMSE (m) | 95% horizontal coverage | Outage coverage | Position ANEES | Outage ANEES |
|---|---:|---:|---:|---:|---:|
| eskf | 4.4747 | 0.9539 | 0.8807 | 0.7695 | 1.3090 |
| inekf | 4.6711 | 0.9512 | 0.8738 | 0.7840 | 1.3356 |

Coverage uses the 2D position covariance ellipse and chi-square threshold 5.9915.
ANEES is mean 3D position NEES divided by 3 (ideal 1).
These are time-correlated samples from one drive, not independent Monte Carlo trials;
coverage is descriptive, not a confidence claim or proof of consistency.
Both filters use the same gating and forced reacquisition policy. No tuning to truth.

Reproduce: `OPENBLAS_NUM_THREADS=1 python -m ccgins.comparison`
