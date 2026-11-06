# cc-gins simulation report

- Config: `configs/proposal_primary.yaml`
- Scenario: seed=900, duration=80s, dt=0.01s, IMU samples=8001, GNSS epochs=80
- Outages: 45s+20s
- Filter: `inekf`, pseudo=`hybrid`, monitor=`dual`, ACI=on
- Metrics window: t >= 30s

## Metrics

| Metric | Value |
| --- | --- |
| Horizontal RMSE [m] | 2.542 |
| 3D RMSE [m] | 2.571 |
| Maximum horizontal error [m] | 6.041 |
| Maximum 3D error [m] | 6.095 |
| ANEES (position, 3D) | 1.430 |
| Mean 2D NEES | 3.900 |
| Mean 3D NEES | 4.290 |
| 2D NEES 95th percentile | 7.275 |
| 95% NEES coverage (2D) | 0.754 |
| Runtime per IMU step [us] | 121.4 |
| Total run wall time [s] | 0.973 |

## Outages

| Start [s] | Duration [s] | Max horizontal error [m] | End error [m] | ANEES | 95% coverage |
| --- | --- | --- | --- | --- | --- |
| 45.0 | 20.0 | 6.041 | 5.756 | 2.130 | 0.435 |

NEES consistency: ANEES is the mean 3D position NEES divided by 3 (target 1.0); the 95% coverage uses the 2D threshold chi2_2(0.95) = 5.991.
