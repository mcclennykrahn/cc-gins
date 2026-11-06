# ADIS16465 RTK-baseline pseudo-range pilot

160 s real IMU + RTK, identical seed 0; Earth compensation enabled.
Fixed ENU anchor (-100, -100, 0) m. After 120 s vector-RTK warmup, one scalar update replaces each RTK vector update.
Injection: body-forward antenna displacement starts at 120 s, grows at 0.05 m/s.

| Lever drift (m/s) | First CUSUM alarm (s) | Horizontal RMSE (m) |
|---|---:|---:|
| 0.0 | None | 0.786 |
| 0.05 | 150.97999999998137 | 0.767 |

This scalar geometry is rank one; tangential drift is not constrained.
An alarm indicates residual inconsistency and is not a diagnosis of lever error.
Baseline false alarms must be considered alongside the injected run.
RTK-derived pseudo-ranges are not raw satellite observations; receiver clock is not estimated.
Reproduce: `OPENBLAS_NUM_THREADS=1 python -m ccgins.range_experiment`
