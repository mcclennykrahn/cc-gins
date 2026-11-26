# cc_accuracy_s1: anisotropic NHC experiment (not improved)

Fixed candidate: lateral sigma 0.1 m/s, vertical sigma 0.03 m/s. Legacy sigma
is 0.1 m/s for both channels. Defaults remain unchanged. The optional setting is
available in `configs/example_sim_accuracy.yaml`; it is not promoted.

Architecture review selected uncertainty tuning as a bounded experiment. InEKF
uses a right-invariant SE_2(3) error with additive IMU biases, first-order covariance
propagation, and Joseph measurement updates. Mechanization matches the simulator's
left-end sample convention; changing it would break truth parity. NHC's lateral
slip and vertical motion differ, motivating independent uncertainty settings.
Outages retain causal IMU/NHC aiding but lack strong forward-speed observations.
Bias initialization is already configurable; random-walk/vibration calibration,
covariance reset/discretization, and offline smoothing need separate validation.
Smoothing would expand scope and change latency. No such changes were made here.

Protocol was fixed before evaluating: example_sim configuration, 180 s, 100 Hz,
GNSS outages 60–90 s and 120–180 s, report from 30 s, identical truth/IMU/GNSS
objects per pair and default initialization seed 0. Development seeds 900–902;
new held-out seeds 19101–19105, with no retuning after exposure. This small nominal
simulation campaign is not real-data or spoof-detection validation.

| Seed | Baseline horizontal RMSE (m) | Candidate (m) |
|---|---:|---:|
| 900 | 7.173489 | 6.934223 |
| 901 | 5.619521 | 5.531739 |
| 902 | 3.258814 | 3.022665 |
| 19101 | 2.853497 | 2.817678 |
| 19102 | 2.473900 | 2.464793 |
| 19103 | 1.784488 | 1.779594 |
| 19104 | 1.956633 | 2.046688 |
| 19105 | 3.398527 | 3.417533 |

Development arithmetic mean RMSE: 5.350608 → 5.162876 m. Held-out arithmetic
mean: **2.493409 → 2.505257 m (+0.475%)**; pooled RMSE across equal-duration
held-out runs: 2.562242 → 2.570990 m. Two held-out seeds regress. Held-out mean
horizontal 95% coverage: 0.839973 → 0.838774; normalized 3D ANEES:
1.551159 → 1.616694. Tightening vertical uncertainty improves the exposed example
but does not establish generalized positioning improvement. No accuracy promotion.

Full precision metrics including each outage maximum/end error, 3D RMSE, coverage
and ANEES are in `paired_metrics.json`. Runtime is observed, not an accuracy gate.
Reproduce from repo root:

```
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. python scripts/evaluate_nhc_accuracy.py
python -m pytest -q
```

Self-check: 16 rows, exact paired seed/config membership, finite metrics and separate
arithmetic recomputation; baseline seed 900 reproduces the stated 7.173 m. Independent
review and final regression are recorded in `review.json` and `validation.json`.
The optional variance is applied consistently to batch/stream NHC, hybrid and fixed
fallbacks, in the mounted vehicle frame. Learned variances are untouched. Legacy
and unequal-channel full-covariance parity, learned hybrid parity, and invalid sigma
checks cover this change.

DETECTION remains FAIL; confirmed_v08=false. Stage3 PASS and historical evidence
are preserved. This stage stops honestly with the accuracy objective unconfirmed.
