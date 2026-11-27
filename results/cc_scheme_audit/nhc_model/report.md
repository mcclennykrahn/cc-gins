# Whole-scheme continuation — motion constraint and multipath

The inherited multipath audit was the only unfinished delivery at entry. Its independent implementation review and 215-test regression passed. This run checked the recorded SHA-256 and reconstructed H/V SSE across all 24 stored rows before committing the intended files. Commit `f2e5a2f840ae868749500d4251a01dd197a3b8c1` is present on origin/main.

## Seed 900 error budget

The original `cfad161` measured horizontal RMSE was **7.173489 m**, and the 60 s outage contributed **98.468%** of horizontal SSE with a **28.257610 m** peak. Fresh nominal seed 900 exactly reproduced the current corrected budget at **7.019110 m** overall, **11.016203 m** in the long outage and **27.684186 m** peak. The long gap still contributes **98.462%** of horizontal SSE. The following table reports current H/V RMSE in metres. Removing the constraint is a diagnostic control on identical inputs, with no proposed filter default.

| Segment | Original baseline H/V | Current fixed NHC H/V | Same inputs without NHC H/V |
|---|---:|---:|---:|
|30–60 s GNSS available|0.985/1.147|0.989/1.186|1.037/1.591|
|60–90 s outage|1.544/0.652|1.488/0.677|12.682/2.043|
|90–120 s GNSS available|0.762/0.472|0.768/0.474|0.945/0.798|
|120–180 s outage|11.259/0.666|11.016/0.640|24.731/8.288|
|180 s reacquisition|1.389/0.430|1.387/0.378|1.108/0.286|
|Overall at t ≥ 30 s|7.173/0.755|7.019/0.763|16.644/5.378|

Config is `example_sim.yaml`, duration 180 s at 100 Hz, score from 30 s. Fixes are timestamped on the IMU grid and applied after propagation; score the IMU origin at the logged epoch. Outage windows exclude GNSS reacquisition at 90 and 180 s. GNSS-only residuals are evaluated at the true antenna, p + R lever. The previous nominal GNSS-only benchmark measured H/V **1.427/2.125 m**, including 0.5 m correlated multipath. Stationary expectations from the nominal 1 m per-horizontal-axis and 2 m vertical white sigmas plus multipath are 1.581/2.062 m. The trace files contain signed ENU errors every 0.1 s, and metrics.json includes 10 s bins.

The original truth-initialized raw-IMU INS test measured horizontal endpoints 21.39 and 101.89 m over 30 and 60 s. One-axis flat-motion endpoint scales for the configured 5 mg raw accelerometer bias are 22.07 and 88.29 m. For 30 deg/h raw gyro bias coupled through gravity they are 6.42 and 51.36 m. Corresponding ARW scales are 0.944 and 5.338 m, and VRW scales are 0.158 and 0.447 m. Bias conversions are at sim.py:179–180, ARW and VRW at sim.py:43–52. These are raw-INS scales. Prior GNSS estimates some of the biases, and the online filter continues to use NHC during outages.

## Fresh comparison draws

The full 2 × 2 experiment toggles all NHC updates and synthetic multipath independently, preserving sensor white draws, trajectory, initialization seed 0 and outages within each seed. Seeds 78201–78204 were reserved for comparison without tuning. The table gives unweighted mean per-run H/V RMSE across those four seeds. Individual rows and terminal metrics are in metrics.json.

| Segment | Nominal NHC | Nominal without NHC | Zero-MP NHC | Zero-MP without NHC |
|---|---:|---:|---:|---:|
|30–60 s available|0.851/0.367|1.096/0.928|0.648/0.301|0.886/0.846|
|60–90 s outage|4.535/0.362|12.302/4.413|4.016/0.350|13.154/3.930|
|90–120 s available|0.910/0.695|1.018/1.569|0.602/0.519|0.822/1.170|
|120–180 s outage|5.877/0.787|22.643/6.002|5.327/0.437|21.670/5.309|
|Overall|4.872/0.645|15.404/4.706|4.279/0.434|14.984/4.290|

NHC reduced long-outage H RMSE in all five seeds in both GNSS conditions, although on nominal seed 78204 it increased short-outage H RMSE from 5.497 to 6.555 m. Turning NHC off throughout changes both outage-entry estimates and in-gap updates. The experiment measures their combined effect. With NHC and nominal multipath, the fresh draws ranged 1.147–8.486 m H RMSE for the short gap and 1.445–12.510 m for the long one. Approximately 1 m available, several metres for 30 s and up to the low teens for 60 s are defensible planning scales for these runs. Five draws cannot establish tail gates. At 20 m/s, a 60 s gap spans 1.2 km and is a sensible stress test. The simulator varies speed, and seed 900 averaged 8.95 m/s in that gap. Sub-metre performance throughout the gap would require materially stronger aiding or a different IMU grade.

## Model findings and existing fixes

- At its actual 10 Hz update epochs, seed 900 truth-side lateral FLU residual RMS was 0.0384, 0.0163, 0.0222 and 0.0064 m/s across the four windows, with lag-one correlations of 0.999, 0.999, 0.999 and 0.994. The vertical residual RMS was 0.00125–0.00135 m/s. During the final gap, lateral lag-one correlations ranged 0.994–0.999 across the five seeds. Simulated slip and vertical motion enter at sim.py:152–156. Fixed NHC supplies zero lateral and vertical velocity at pipeline.py:310–317 with constant per-epoch variance at pipeline.py:93–100. The observed correlation establishes a temporal-model mismatch even though the residual RMS is below the configured 0.1 m/s.
- The nominal, NHC-aided comparison mean long-gap position NEES per dimension was 2.335, with 68.6% 3D ellipsoid coverage. Removing multipath with NHC retained gave 1.029 and 98.5%. Removing NHC while keeping multipath gave 0.855 and 100% while long-gap H RMSE increased to 22.643 m. Coverage changes along with reported uncertainty and is not an accuracy gain. Repeated time samples are dependent. These are descriptive results from four draws.
- Prior **8251fe0** corrected IMU density and sample interval handling (filters.py:55–66), rounded-grid multipath time evolution (sim.py:191–203), and outage score boundaries/empty windows/repeated-duration naming (metrics.py:28–37). Prior **70f21b4** added covariance transport after measurement injection (see the [reset audit](../covariance_reset/report.md)). These remain the production correctness fixes. This round adds controlled diagnostics and found no new production implementation bug in the examined NHC path.
- The [original audit](../report.md) covers inverse truth mechanization, fixed ENU gravity and omitted Earth rotation in the default synthetic path, body-to-nav matrices, lever-arm sign, bias and Jacobian conventions, Joseph update and first-order process discretization, causal ZUPT and outage handling. The [maneuver audit](../maneuver_observability/report.md) gives the corrected discrete observability analysis. Idealized truth-perturbed alignment, nominal vibration density, temporally correlated GNSS errors and approximately planar motion limit transfer to real data. No offline smoothing metric is reported.

## Review and remaining work

All 20 cases ran end to end. A standalone Cholesky-based check reconstructed segment errors and NEES/coverage from all traces, verified timestamps and counts, and matched both current seed 900 NHC cases to the previous multipath audit. Full regression **215 passed in 86.62 s**. The new diagnostic script has no separate implementation reviewer sign-off yet and is held out of the push. The inherited multipath audit has independent implementation review and was pushed.

Next, evaluate an optional colored-GNSS state with batch/stream parity and analytic Jacobians, including cross-covariance and propagation through gaps. Use a causal slip model or targeted in-gap versus pre-gap NHC ablation to separate NHC effects. Compare unchanged nominal inputs over independent sensor and alignment draws before setting gates. Evaluate optional RTS only as a separate offline experiment if future measurements support it. This is a mid-audit delivery. DETECTION remains FAIL and confirmed_v08=false.

Reproduce from repo root with `PYTHONPATH=. OPENBLAS_NUM_THREADS=1 python -m scripts.audit_nhc_model --out OUTPUT`. Run `verify.py` in the evidence directory for the independent numerical calculation. The source snapshot accompanies the run.

## Completion addendum — 2026-11-27

The historical pending implementation sign-off above is now closed: independent review PASS in `/workspace/ccgins-v08/reviews/cc_scheme_nhc_completion_review.json`. All 70 factorial traces, 20 model traces and 70 parent CLI comparisons passed independent verification. Final regression: 217 passed in 95.91 s. Intended source and report files are committed in the delivery recorded by review.json. The broader structural audit remains open; DETECTION FAIL, confirmed_v08=false.
