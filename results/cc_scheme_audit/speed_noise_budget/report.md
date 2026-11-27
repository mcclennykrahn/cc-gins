# Scheme continuation: speed-dependent IMU noise budget

New work follows completion of the inherited NHC/CLI candidate. All five draws use configs/example_sim.yaml, 180 s, dt=.01 s; diagnostics select half-open segments from 30 s. Seed900 plus comparison seeds91001–91004, no tuning. This diagnostic does not run a new positioning algorithm or claim new positioning RMSE.

## Measured noise power

We subtract exact simulated truth and stored time-varying bias offline from raw IMU. The resulting white residual is compared with the simulator conditional variance, averaged over each segment. Variance ratios below are per-axis scalar variances, with measured power pooled over all three axes. Truth speed is never supplied to the online filter.

| Seed | Segment | Mean forward speed m/s | Gyro expected / nominal variance | Accel expected / nominal variance | Gyro measured / expected | Accel measured / expected |
|---|---|---:|---:|---:|---:|---:|
|900|available_30_60|16.930|1.284|1.916|0.988|1.005|
|900|outage_60_90|10.131|0.909|0.792|0.985|0.992|
|900|available_90_120|6.883|0.747|0.396|0.985|0.977|
|900|outage_120_180|8.954|0.844|0.601|0.974|1.001|
|91001|available_30_60|15.096|1.182|1.608|1.011|1.022|
|91001|outage_60_90|2.943|0.582|0.181|0.999|1.014|
|91001|available_90_120|9.702|0.880|0.675|1.007|0.999|
|91001|outage_120_180|7.613|0.782|0.479|0.999|0.989|
|91002|available_30_60|15.623|1.211|1.688|1.007|0.974|
|91002|outage_60_90|9.169|0.856|0.636|0.988|0.998|
|91002|available_90_120|2.274|0.552|0.129|0.969|0.988|
|91002|outage_120_180|4.485|0.642|0.227|0.984|1.006|
|91003|available_30_60|1.222|0.507|0.061|1.005|0.986|
|91003|outage_60_90|1.039|0.498|0.043|1.025|1.021|
|91003|available_90_120|4.867|0.652|0.197|1.009|0.984|
|91003|outage_120_180|12.624|1.042|1.167|0.994|0.991|
|91004|available_30_60|18.844|1.404|2.351|1.012|1.002|
|91004|outage_60_90|18.865|1.406|2.356|0.990|1.007|
|91004|available_90_120|12.895|1.050|1.149|0.999|0.988|
|91004|outage_120_180|9.915|0.891|0.709|0.986|0.999|

## Interpretation and source evidence

Simulator ccgins/sim.py:182–185 uses abs(forward body speed) and adds base plus vibration standard deviations before drawing white noise. NoiseParams.from_spec at ccgins/filters.py:55–66 instead freezes vibration at 12 m/s. Propagation at filters.py:93–103 uses that density for every epoch. The earlier density-addition and dt bug is already fixed; the remaining speed approximation is documented. Seed900 final-gap expected acceleration variance is 0.601 of nominal and gyro variance 0.844, so underestimating white-noise power is not a sufficient explanation for its final-gap inconsistency. Other seeds have both excess and deficient nominal power. This does not identify bias, slip or colored-GNSS attribution from noise power alone. No new production equation bug is demonstrated here.

Fresh measurements agree with the simulator: measured/expected variance spans .969–1.025 for gyro and .974–1.022 for acceleration across these segments. These dependent-trajectory summaries are descriptive, not independent positioning trials or tail confidence bounds.

## Positioning budget and correctness fix carried to completion

The original measured seed900 overall H RMSE is 7.173489 m; the final outage contributes 98.468% H SSE and peaks at 28.257610 m. Original combined available H RMSE .881984 m supports the <=1 m order-of-magnitude hypothesis for this draw. Current nominal H RMSE after previously reviewed noise/reset fixes is 7.019110 m overall and 11.016203 m in the final gap, peak27.684186 m. Original/current H/V and held-out per-segment metrics, 10 s time bins, raw-INS scales, realistic targets and complete equation/frame/units audit are in [the factorial report](../nhc_timing_factorial/report.md) and [original whole-scheme report](../report.md). The original report is historical: its reset and observability open items have subsequent linked continuations.

The inherited CLI bug at parent f2e5a2f ccgins/cli.py:228 ignored t_start in detailed outage metrics although summarize respected it. The correction intersects the outage and scoring masks. The boundary fixture changes a reported detailed peak100 m to3 m; wholly unscored rows are omitted. Empty scoring is rejected before reductions at cli.py:207–210. All70 nominal/factorial traces preserve exact parent scoring at t_start30 because both outages follow convergence; there is no online trajectory change. Same-input and held-out per-segment parity evidence is in ../nhc_timing_factorial/scoring_validation.json.

## Residual plan and limits

Do not tune NHC sigma on this finding. A candidate causal noise model must estimate speed without truth, propagate speed uncertainty, preserve batch/stream parity, and undergo same-input comparisons across sensor and alignment seeds. Colored GNSS covariance and causal correlated-slip modeling remain plausible structural work; separate their effect from white noise. More independent draws are required for tail targets. Optional manifold RTS remains an offline metric only; terminal outage has little future support. Alignment is still truth-perturbed synthetic initialization; Earth rotation is absent in both default truth and filter. No V0.8 claim: DETECTION FAIL, confirmed_v08=false.

Reproduce: OPENBLAS_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python -m scripts.audit_speed_noise --out OUTPUT. metrics.json contains all measured powers, nominal powers, standardized residual moments and full configuration. Independent review and final regression are recorded in delivery.json once complete.
