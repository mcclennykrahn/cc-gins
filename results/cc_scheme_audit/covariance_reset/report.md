# Positioning scheme audit continuation

Candidate `scheme_covariance_reset`. Parent `842052c`. The prior whole-scheme audit and its fixes are documented in [the original audit](../report.md). Those measurements cover truth and IMU synthesis, GNSS antenna and timing, filter Jacobians, alignment and bias states, NHC and ZUPT, outage handling, and the absence of smoothing. This continuation resolves the finite-correction covariance defect and completes the prior frozen-discretization evidence item.

## Measured error budget first

Fresh paired runs use `configs/example_sim.yaml:4–25,27–39`, 180 s at 100 Hz, 1 Hz GNSS, a 30 s score start, and initialization seed 0. The historical `cfad161` baseline measured 7.173489 m horizontal RMSE and 28.257610 m peak during the final outage. That outage contributed 98.468% of the total horizontal squared error. At the immediate parent, the fresh rerun measured 7.035064 m overall, with 98.442% of squared horizontal error in the final outage. After reset it still contributes 98.462% for seed 900. In fresh seed 68432 the shorter outage instead contributes 58.5%.

Available intervals are [30,60) and [90,120), plus the separate terminal GNSS update at 180 s. Outages are [60,90) and [120,180). The trace compares IMU origins at the same post-update epoch. Raw GNSS is scored against the true antenna point. All paired inputs, outage sample counts, GNSS-only metrics and pure INS results agree. Table labels give boundaries in seconds. The before and after budget JSONs include counts, maxima, end points, SSE fractions and 10-second bins. The associated CSVs give the signed ENU error time histories.

The simulator applies 1 m horizontal white sigma per axis, 2 m vertical white sigma and 0.5 m per-axis correlated multipath. Seed 900 raw GNSS measured 1.427 m horizontal and 2.125 m vertical RMS. The steady-state noise model gives about 1.58 m horizontal and 2.06 m vertical RMS. This is approximately the raw GNSS target. Available filter accuracy around 0.7–1.5 m horizontal and 0.2–1.5 m vertical is plausible for these simulations.

The configured IMU ARW is 0.3 deg/√h and VRW 0.1 m/s/√h. The simulated turn-on biases are 30 deg/h and 5 mg, converted to rad/s and m/s² at `ccgins/sim.py:179–185`. Density conversion is in `ccgins/sim.py:43–52`. From exact truth initialization, unaided integration of that same raw IMU measured 21.39 m and 101.89 m horizontal endpoint errors for seed 900, at 30 and 60 s respectively. The corresponding segment horizontal RMS was 9.46 m and 42.69 m, with vertical endpoint errors of 37.58 m and 160.13 m. In a flat, straight approximation, the one-axis accelerometer-bias endpoint scales are 22.1 and 88.3 m, and gyro tilt-growth scales are 6.4 and 51.4 m. See the original audit for ARW, VRW, random walk and vibration terms. Seed 900 averaged 10.13 and 8.95 m/s in its two outages with a 20 m/s configured speed ceiling. A 60 s outage at 20 m/s is a reasonable 1.2 km stress test. Sub-metre online performance throughout would require substantially stronger aiding or calibration. For this simulated MEMS and existing NHC, roughly 1–5 m and 3–15 m horizontal RMS are planning ranges for the 30 and 60 s gaps, with vertical RMS roughly 0.2–2 m. The sample is small and these ranges are engineering targets. Fresh runs can fall on either side of these ranges.

## Verified bug and fix

At parent `842052c:ccgins/filters.py:110–115`, an accepted update formed the Joseph covariance in the pre-injection error coordinates and immediately injected the state correction. The subsequent updates and physical covariance projection then used a different tangent reference. The corrected shared update maps the Joseph covariance on both sides by the derivative of the reset before storing it (`ccgins/filters.py:110–119`).

- ESKF uses the attitude right Jacobian at the correction, derived from Log(Exp(−φ) Exp(φ + ε)). Additive velocity, position and biases retain identity blocks (`ccgins/filters.py:205–209`).
- InEKF uses the derivative of Log(Exp(ξ + ε) Exp(−ξ)), the integral of exp(s adξ) over s in [0,1]. A block matrix exponential evaluates it and retains the rotation–velocity and rotation–position coupling. Body biases are additive (`ccgins/filters.py:267–280`).

`tests/test_covariance_reset.py` checks both implementations by centrally differencing the physical state-to-error map for a finite correction with a distant ENU position. It independently reconstructs the expected covariance from the known Joseph posterior of a synthetic update. Rejected measurements bypass the reset through the existing gate.

The unfinished discretization diagnostic now has a corrected count of 48 rows and focused analytical tests. Its frozen-model Euler horizontal process-variance error was −0.082% at 30 s and −0.060% at 60 s for the example 10 ms step. That evidence supports keeping the current propagation discretization for this case. It does not quantify a full time-varying covariance trajectory.

## Paired online results

Seeds 49501 and 49502 repeat the previous comparison. Seeds 68431 and 68432 were added for this continuation, without parameter fitting. Overall horizontal RMSE rose on all four comparison seeds. Seed 900 final-outage peak was 27.795 → 27.684 m. Its descriptive position 95% coverage at 30–60 s was 47.0% → 43.3% and at 90–120 s was 40.7% → 39.0%. The correction fixes covariance coordinates, while the outcome leaves a substantial calibration problem. The stage makes no accuracy improvement claim.

| Seed | Segment | Horizontal RMSE (m) | Vertical RMSE (m) |
|---:|---|---:|---:|
| 900 | outage_60_90 | 1.512 → 1.488 | 0.674 → 0.677 |
| 900 | outage_120_180 | 11.040 → 11.016 | 0.643 → 0.640 |
| 900 | available | 0.885 → 0.887 | 0.887 → 0.902 |
| 900 | overall | 7.035 → 7.019 | 0.756 → 0.763 |
| 900 | available_30_60 | 0.987 → 0.989 | 1.163 → 1.186 |
| 900 | available_90_120 | 0.768 → 0.768 | 0.472 → 0.474 |
| 900 | terminal_available | 1.367 → 1.387 | 0.381 → 0.378 |
| 49501 | outage_60_90 | 0.825 → 0.814 | 0.419 → 0.418 |
| 49501 | outage_120_180 | 5.201 → 5.219 | 0.710 → 0.709 |
| 49501 | available | 1.102 → 1.099 | 0.566 → 0.567 |
| 49501 | overall | 3.381 → 3.392 | 0.604 → 0.604 |
| 49501 | available_30_60 | 1.035 → 1.032 | 0.522 → 0.525 |
| 49501 | available_90_120 | 1.162 → 1.159 | 0.606 → 0.605 |
| 49501 | terminal_available | 1.860 → 1.868 | 0.882 → 0.882 |
| 49502 | outage_60_90 | 3.432 → 3.611 | 0.148 → 0.146 |
| 49502 | outage_120_180 | 4.841 → 4.842 | 0.512 → 0.518 |
| 49502 | available | 0.940 → 0.943 | 0.216 → 0.215 |
| 49502 | overall | 3.475 → 3.512 | 0.358 → 0.361 |
| 49502 | available_30_60 | 0.660 → 0.664 | 0.158 → 0.164 |
| 49502 | available_90_120 | 1.143 → 1.148 | 0.261 → 0.256 |
| 49502 | terminal_available | 2.744 → 2.745 | 0.332 → 0.338 |
| 68431 | outage_60_90 | 1.273 → 1.241 | 0.361 → 0.359 |
| 68431 | outage_120_180 | 2.804 → 2.817 | 0.393 → 0.398 |
| 68431 | available | 0.652 → 0.652 | 0.284 → 0.274 |
| 68431 | overall | 1.907 → 1.910 | 0.347 → 0.345 |
| 68431 | available_30_60 | 0.606 → 0.609 | 0.336 → 0.322 |
| 68431 | available_90_120 | 0.693 → 0.689 | 0.220 → 0.217 |
| 68431 | terminal_available | 1.251 → 1.249 | 0.114 → 0.124 |
| 68432 | outage_60_90 | 5.167 → 5.293 | 0.239 → 0.225 |
| 68432 | outage_120_180 | 2.763 → 2.961 | 0.156 → 0.153 |
| 68432 | available | 1.078 → 1.076 | 0.272 → 0.270 |
| 68432 | overall | 2.975 → 3.093 | 0.225 → 0.220 |
| 68432 | available_30_60 | 0.798 → 0.818 | 0.347 → 0.342 |
| 68432 | available_90_120 | 1.299 → 1.284 | 0.165 → 0.168 |
| 68432 | terminal_available | 0.910 → 0.925 | 0.320 → 0.329 |

## Remaining work

Use independently drawn trajectories, sensors and alignment errors to isolate model mismatch in available intervals. Prioritize the temporally correlated GNSS multipath model and NHC model residuals, compare covariance consistency across trajectories, and characterize bias observability before and during each outage using scaled sensitivity to maneuvers. Replay time-varying process Jacobians before considering a higher-order propagation change. Forward-speed aiding needs an independent input. Optional manifold RTS would need offline storage of predicted and filtered states and cross-covariances, and an explicitly separate score. The terminal outage has one GNSS epoch of future information at its end. This stage delivers an online audit continuation. DETECTION remains FAIL and confirmed_v08 remains false.


## Reproduction

Run from the repo root at the parent and at the reset commit, using the same environment and a separate output directory for each checkout.

```sh
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/audit_scheme.py --out OUTPUT --seeds 900 49501 49502 68431 68432
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/audit_bias.py --out OUTPUT
OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python -m pytest -q
```

At the reset commit, run the frozen-model diagnostic with `PYTHONPATH=. python scripts/audit_discretization.py --out OUTPUT`. The stage's raw CSVs, JSON budgets and independent reviews are stored under `/workspace/ccgins-v08`.
