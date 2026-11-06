# cc-gins simulation report

Data source for all metrics: synthetic simulation IMU and synthetic GNSS (truth + noise/outage).

- Config: `configs/proposal_primary.yaml`
- Scenario: seed=900, duration=80s, dt=0.01s, IMU samples=8001, GNSS epochs=80
- Outages: 45s+20s
- Filter: `inekf`, pseudo=`hybrid`, monitor=`dual`, ACI=on
- Metrics window: t >= 30s

## Metrics

| Metric | Value |
| --- | --- |
| Horizontal RMSE [m] | 3.829 |
| 3D RMSE [m] | 3.873 |
| Maximum horizontal error [m] | 11.943 |
| Maximum 3D error [m] | 12.079 |
| ANEES (position, 3D) | 1.671 |
| Mean 2D NEES | 4.544 |
| Mean 3D NEES | 5.014 |
| 2D NEES 95th percentile | 7.634 |
| 95% NEES coverage (2D) | 0.525 |
| Runtime per IMU step [us] | 271.0 |
| Total run wall time [s] | 2.169 |

## Outages

| Start [s] | Duration [s] | Max horizontal error [m] | End error [m] | ANEES | 95% coverage |
| --- | --- | --- | --- | --- | --- |
| 45.0 | 20.0 | 11.943 | 11.943 | 2.482 | 0.000 |

NEES consistency: ANEES is the mean 3D position NEES divided by 3 (target 1.0); the 95% coverage uses the 2D threshold chi2_2(0.95) = 5.991.
