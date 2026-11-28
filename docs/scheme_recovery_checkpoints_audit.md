# Positioning scheme audit and GNSS recovery checkpoints

Candidate `scheme_recovery_checkpoints` from `17b1539`. Mid-audit. DETECTION remains FAIL and confirmed_v08 remains false.

The handoff and project manifest show the prior joint entry delivery reviewed and pushed. The recovery study began on a clean tracked tree aligned with origin/main. This finishing session inherited its uncommitted diagnostic changes, completed evidence, and review notes; it reruns regression, segment budgets and isolated recovery before committing. The untracked runner prompt was left alone.

## Measured error budget

Both runs used configs/example_sim.yaml, 180 s, t_start=30 s, native initialization and seeds 900, 114001 and 114002. Budget JSON and all three signed position-error histories match byte for byte. Each cell below is before and after, expressed as horizontal/vertical RMSE in metres. Ten-second bins and individual available intervals are in the paired JSON.

| Seed | Available | 60–90 s | 120–180 s | Overall | Long-gap horizontal SSE share |
| --- | ---: | ---: | ---: | ---: | ---: |
| 900 | 0.887/0.902 | 1.488/0.677 | 11.016/0.640 | 7.019/0.763 | 98.46% |
| 114001 | 0.749/0.949 | 4.167/1.219 | 2.929/1.003 | 2.669/1.029 | 48.15% |
| 114002 | 0.961/0.691 | 4.980/0.573 | 4.818/0.328 | 3.822/0.547 | 63.53% |

Seed 900's 60 s gap dominates horizontal SSE and reaches 27.684 m before the terminal fix. The historical 7.173 m overall and 28 m peak describe an earlier baseline. Available H RMSE is below 1 m in these three draws. Earlier ensembles include exceptions. The 120–180 s interval consumes 40% of the scored time and one third of scenario time. The terminal posterior at 180 s is separately available, outside the half-open gap.

| Seed | GNSS-only H/V RMSE m | Raw 30 s INS H RMSE/end m | Raw 60 s INS H RMSE/end m | Mean gap speed 30 s/60 s m/s |
| --- | ---: | ---: | ---: | ---: |
| 900 | 1.427/2.125 | 9.458/21.390 | 42.694/101.889 | 10.131/8.954 |
| 114001 | 1.567/1.896 | 12.468/28.180 | 13.459/10.814 | 12.689/9.439 |
| 114002 | 1.481/2.277 | 16.575/37.152 | 71.639/163.521 | 0.041/17.322 |

GNSS uses 1 m per horizontal axis, 2 m vertical, and 0.5 m colored multipath with 30 s correlation time. Stationary raw-fix H/V RMS is 1.581/2.062 m, with finite-sample and zero-start effects. GNSS-only scores valid fixes against the true antenna. Online scores same-time IMU-origin position. Raw INS starts each interval at exact truth and propagates noisy, uncompensated IMU through the interval endpoint before GNSS.

In `ccgins/sim.py:45–53,181–187`, gyro turn-on bias is 30 deg/h (1.45444e-4 rad/s), accelerometer bias is 5 mg (0.04905 m/s²), ARW is 0.3 deg/√h (8.72665e-5 rad/√s), and VRW is 0.1 m/s/√h (1.66667e-3 m/s/√s). Straight, level single-axis 30/60 s endpoint scales are 22.073/88.290 m from accelerometer bias, 6.421/51.365 m from gyro-bias tilt coupling, 0.944/5.338 m from ARW tilt coupling, and 0.158/0.447 m from VRW. Those scales omit estimated compensation, vibration, maneuvers, and bias random walk. Actual effects can reinforce or cancel.

A 60 s outage at 20 m/s spans 1.2 km and is a reasonable stress test. The scenario uses a 20 m/s ceiling and seed 900 averages 8.954 m/s during its long gap. For planning, use roughly 0.7–1.5 m horizontal available, a few metres over 30 s, and metres to tens of metres over 60 s. Prior short and long gaps reached 17.142 and 45.892 m horizontal RMSE. Neither sub-metre outage accuracy nor a narrow tail bound is supported.

## Confirmed scoring bug

At parent `scripts/audit_calibration.py:19–20`, position consistency did not pass the true endpoint through to either budget or segment masks. Both inferred an endpoint from the last logged sample (`scripts/audit_scheme.py:14`). For an available final decimated sample before the actual endpoint, the report relabeled it as terminal and omitted it from the disjoint available interval. The overall and aggregate available budgets were unaffected. Current `scripts/audit_calibration.py:15–20` and six experiment call sites pass the true endpoint, and the new regression exercises an unlogged endpoint. This fix changes diagnostic segment membership only.

## End-to-end audit

- Truth and IMU synthesis use local ENU, body FLU, and body-to-navigation rotation matrices (`ccgins/sim.py:152–168`, `ccgins/filters.py:33–45`). The old attitude maps specific force, velocity/position update, and the right rotation increment advances attitude. There is no quaternion in this path. Constant downward gravity and omitted Earth rotation agree across default truth and filter. Optional fixed local Earth rate and Coriolis omit geodetic transport. Exact inversion of the same integrator makes closure optimistic for real sensors.
- GNSS (`ccgins/sim.py:191–212`, `ccgins/filters.py:122–124,282–286`) uses the same positive body lever in p+Rlever. Multipath evolves at rounded GNSS spacing through gaps. Filter measurement covariance treats updates as temporally independent. Propagation precedes the update at the matching epoch (`ccgins/pipeline.py:256–260,416–420`).
- InEKF bias couplings and noise mapping are at `ccgins/filters.py:233–255`. The GNSS and body velocity Jacobians at 282–291 agree with invariant errors, including NHC attitude cancellation. Joseph update and reset are retained. First-order transition and Q discretization at 94–103, fixed-speed vibration in Q at 55–67, and correlated GNSS remain model approximations. Low excitation limits bias observability. Existing truth-entry interventions identify sensitivity without providing a causal bias estimator.
- NHC constrains lateral/vertical velocity (`ccgins/pipeline.py:315–317`) while truth can slip (`ccgins/sim.py:154–157`). ZUPT and odometry are off. No receiver fix is supplied in the half-open gaps. IMU and NHC continue. No NHC sigma change was made.
- Initial attitude, position and velocity are perturbed truth (`ccgins/pipeline.py:119–132`), bias means start at zero and the initial covariance is transformed (`ccgins/filters.py:303–319`). This leaves a causal alignment evidence gap. The online log (`ccgins/pipeline.py:528–540`) is same-epoch posterior. The audit includes no smoother. A future manifold RTS benchmark would need separate offline scoring and an explicit account of future GNSS at the endpoint.

Prior bug fixes, regression evidence, and the Stage 3 review remain in the project manifest and archived reports.

## First-fix recovery evidence

The isolated-intervention results below come from `ensemble_isolated/recovery.json`.

The recorded example trajectory scenario was run on 12 fresh, untuned seeds 114101–114112. Each draws its own truth, IMU and GNSS and uses init_seed=seed+200000 for the synthetic initialization. The two counterfactuals separately invalidate just the valid GNSS fix at 90 s or at 180 s. Both retain the native prefix for their respective intervention, all other fixes, identical multipath and white draws, and the same IMU and initialization. Posterior GNSS age confirms the two return updates are applied only in the native comparison. Checkpoints at 91 and 95 s in the withheld column use the 90 s omission. Each fixed-time aggregate uses 12 independent draws. Epochs and paired arms reuse draws. Empirical coverage counts from this sample do not establish a calibrated tail.

| Epoch s | Native H/V RMSE m | Withheld H/V RMSE m | Native/withheld H NEES per dimension | Native/withheld H 95% coverage count | Native gives larger H error |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 89.9 | 7.527/0.587 | 7.527/0.587 | 0.839/0.839 | 12/12 vs 12/12 | 0/12 |
| 90.0 | 1.210/0.494 | 7.577/0.589 | 1.019/0.838 | 12/12 vs 12/12 | 0/12 |
| 91.0 | 1.136/0.513 | 1.484/0.514 | 1.438/1.429 | 11/12 vs 11/12 | 3/12 |
| 95.0 | 0.904/0.655 | 0.985/0.648 | 1.736/1.927 | 9/12 vs 9/12 | 2/12 |
| 179.9 | 19.999/0.814 | 19.999/0.814 | 1.401/1.401 | 11/12 vs 11/12 | 0/12 |
| 180.0 | 1.528/0.697 | 20.135/0.814 | 1.413/1.405 | 11/12 vs 11/12 | 0/12 |

The 180 s terminal fix cuts ensemble horizontal RMSE from 20.135 to 1.528 m at the same epoch. The effect is due to newly available aiding, so report terminal recovery and last in-gap accuracy separately. At 95 s, native horizontal RMSE is below 1 m while native nominal 95% ellipse coverage is 9/12. Report accuracy and covariance calibration separately. This experiment uses clean unspoofed fixes and supplies no DETECTION evidence.

## Residual plan

1. Evaluate causal sensor-only alignment with uncertainty and explicit refusal across independent starts and low observable motion.
2. Combine colored GNSS, speed-dependent IMU uncertainty, and NHC slip in a joint calibration protocol with frozen training and evaluation splits.
3. Predeclare per-segment online tail thresholds and validate on a larger fresh ensemble. Keep recovery, outage, and optional offline smoothing metrics distinct.

This round fixes diagnostic scoring and advances terminal recovery attribution. Whole-scheme validation remains open. There is no external blocker.

Reproduce with `PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1` and the project virtualenv. Run `python -m scripts.audit_scheme --seeds 900 114001 114002 --out <directory>`, `python -m scripts.audit_recovery_checkpoints --out <directory>`, and `python -m pytest -q`.

Final finishing-session validation: 320 passed in 94.09s (0:01:34). Fresh verified_budget/ reproduces before/ budget and all three signed histories byte for byte; final_reexecution/recovery.json reproduces ensemble_isolated/recovery.json byte for byte. See final_validation.json. Independent final review is stored under reviews/recovery_checkpoints_final_independent.md.
