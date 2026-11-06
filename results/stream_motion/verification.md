# Verification record

Implementation and experiment source: `a5a9823`.

- Full suite: `.venv/bin/pytest -q` → **96 passed in 38.12s** (83 existing + 13 new).
- Real-data experiment: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.stream_motion_experiment --workers 4`.
- Report enrichment/parity validation: same module with `--report-only` (does not rerun filters). Position, velocity and position-covariance absolute discrepancies must each be <1e-5; motion dictionaries must match exactly. All eight real-data replays pass.
- 8 matched cases, 24 batch trajectories, 8 streaming replays, 48 paired outage events per comparison. Three physical new outage stops viewed by four co-mounted IMUs yield 12 correlated sensor/fold stop episodes.
- The source estimator is unchanged between the experiment and final verification; later edits add report scoring and documentation. Input and mounting-calibration SHA256 hashes are in metrics.json.

The wheel stream is **SYNTHETIC and reference-derived**, not a real i2Nav sensor. The matched baseline omits NHC and differs from the prior adaptive-motion report. Read report.md for both gains and regressions, including worse mean RMSE when isolating stop confirmation and worse consistency than the GNSS-only baseline.
