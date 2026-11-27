# Segment ensemble: whole-scheme audit continuation

Candidate `scheme_segment_ensemble`; parent `f22fb26`. No production change or new implementation bug established. This is a bounded mid-audit delivery, not completion or tail validation. DETECTION remains FAIL; confirmed_v08=false.

## Error budget first

Fresh before/after: configs/example_sim.yaml, duration180s, t_start30s; seed900 and held-out104001/104002; init_seed0. Metrics dictionaries match exactly and full signed CSV histories are byte-identical. Online IMU-origin positions are scored against same-epoch truth. Half-open outages exclude the corrected180s GNSS endpoint; terminal_available is reported separately.

|Seed|Segment|Before H = after H (m)|Before V = after V (m)|H SSE share|
|---|---|---:|---:|---:|
|900|available_30_60|0.989350|1.185786|0.3971%|
|900|outage_60_90|1.488475|0.676636|0.8988%|
|900|available_90_120|0.767795|0.474157|0.2391%|
|900|outage_120_180|11.016203|0.640390|98.4624%|
|900|terminal_available|1.386857|0.378029|0.0026%|
|900|overall|7.019110|0.762560|100.0000%|
|104001|available_30_60|1.009398|0.326765|1.2487%|
|104001|outage_60_90|1.678252|0.808647|3.4518%|
|104001|available_90_120|0.751416|0.469349|0.6920%|
|104001|outage_120_180|6.212670|0.170401|94.6060%|
|104001|terminal_available|0.607722|0.155793|0.0015%|
|104001|overall|4.038352|0.455727|100.0000%|
|104002|available_30_60|1.200951|1.023707|0.4223%|
|104002|outage_60_90|17.141542|0.560684|86.0296%|
|104002|available_90_120|0.667892|0.713152|0.1306%|
|104002|outage_120_180|4.785936|0.540592|13.4126%|
|104002|terminal_available|2.241401|0.295199|0.0049%|
|104002|overall|8.262208|0.700580|100.0000%|

Historical cfad161 baseline7.173489m overall H and28.257610m maximum long-gap error are documented in results/cc_scheme_audit/report.md. They are not the current parent baseline. Current seed900 combined available H 0.886598m; long gap H 11.016203m and 98.462% SSE.

## Independent draws

Sixteen previously unused scenario/sensor seeds104101–104116, each with independent init_seed=seed+1000000. The same generator produces trajectory and sensors from each scenario seed; this is not an independent factorial of trajectory versus sensor noise. Truth-relative initial alignment remains optimistic. The table is per-run min / median / max, with equal run weight; no epoch pooling, confidence interval or percentile/tail guarantee.

|Segment|H RMSE min / median / max (m)|V RMSE min / median / max (m)|
|---|---:|---:|
|available|0.791 / 0.934 / 1.489|0.221 / 0.449 / 1.483|
|available_30_60|0.405 / 0.844 / 1.155|0.129 / 0.477 / 1.837|
|available_90_120|0.589 / 0.965 / 1.863|0.119 / 0.376 / 1.015|
|outage_120_180|1.033 / 5.089 / 10.305|0.135 / 0.529 / 0.982|
|outage_60_90|1.053 / 2.526 / 6.403|0.079 / 0.546 / 1.584|
|overall|1.242 / 3.672 / 6.732|0.244 / 0.512 / 1.220|
|terminal_available|0.824 / 1.305 / 2.478|0.089 / 0.360 / 1.347|

Long gap exceeds50% H SSE in13/16 draws; available H<=1m in11/16. Long-gap dominance is common, not universal; <=1m available performance is plausible, not a guaranteed gate. Short-gap H RMSE reaches6.403m and its peak17.139m, exceeding the earlier provisional1–5m/10m ranges. This is new evidence against interpreting those provisional targets as bounds. Long-gap peak across these16 runs reaches19.138m; seed900 remains worse, so this smaller ensemble maximum does not justify tightening a target.

### Pure INS and GNSS-only measurements

|Quantity|Min / median / max (m)|
|---|---:|
|GNSS antenna rmse_h|1.414 / 1.556 / 1.809|
|GNSS antenna rmse_v|1.715 / 2.003 / 2.330|
|Truth-initialized raw INS 30s rmse_h|1.313 / 11.284 / 28.907|
|Truth-initialized raw INS 30s end_h|3.946 / 25.763 / 66.584|
|Truth-initialized raw INS 30s end_v|1.022 / 11.165 / 32.708|
|Truth-initialized raw INS 60s rmse_h|12.068 / 42.960 / 132.983|
|Truth-initialized raw INS 60s end_h|31.269 / 97.145 / 317.295|
|Truth-initialized raw INS 60s end_v|0.019 / 41.637 / 132.898|

Actual 30s outage mean speeds: 4.687–19.727m/s (median 10.994); vmax20 is a cap, not sustained speed.

Actual 60s outage mean speeds: 2.368–18.258m/s (median 10.893); vmax20 is a cap, not sustained speed.

## Correctness audit and scenario limits

Current source references (historical bug locations and fixes remain in the original report):

- Units: ccgins/sim.py:181–187 resolves30deg/h and5mg, converted by pi/180/3600 and1e-3*9.81. ARW/VRW conversion is at sim.py:49–58. Additive speed-dependent vibration and bias random walks are present beyond the four YAML IMU fields. One-axis raw accel bias endpoint scale22.073/88.290m and gyro-bias gravity coupling6.421/51.365m at30/60s; independent contributions are not additive error predictions. Raw INS is reinitialized at truth, has no bias subtraction or NHC, and is not the aided online filter.
- Truth/integration: sim.py:1–5,158–169 and filters.py:32–45 use ENU/FLU body-to-nav matrices, old-attitude acceleration, second-order position and then attitude. No quaternion convention is involved in this path. Constant gravity and omitted Earth rotation match the synthetic default; real-Earth realism is a scenario limitation.
- GNSS: sim.py:191–212 uses rounded grid timing, p+Rlever,1m per-horizontal-axis white noise,2m vertical and.5m correlated multipath. filters.py:122–124 matches lever sign/frame. GNSS-only scores use antenna residuals, not origin residuals. Stationary GNSS radial H expectation1.581m, V2.062m; finite-track, zero-initialized multipath differs. White-only filter R is still a colored-noise approximation.
- InEKF: filters.py:93–102 retains first-order Phi and Qd; current filters.py:111–118 DOES reset covariance after injection. The original report’s reset-absence finding is historical and fixed. Prior discretization, observability, noise-phase and colored-GNSS evidence is retained; this study does not identify a new Jacobian/sign/discretization bug or justify a parameter change.
- Alignment: pipeline.py:119–132 samples around truth, not a deployable alignment algorithm. Independent draws improve coverage but do not solve causal initialization or establish bias observability. NHC pipeline.py:312–317 observes lateral/vertical motion; no forward-speed observation is added, and ZUPT remains disabled in this config. Existing maneuver-observability evidence remains relevant; all biases cannot be assumed well estimated.
- Outages: invalid GNSS is skipped while IMU/NHC continue. Original half-open endpoint, fractional-grid and metric-start corrections remain preserved. Traces and10s bins under the artifact directory expose time dependence; bins omit the terminal corrected sample intentionally.
- Smoothing: none applied. An RTS smoother would need manifold transition/cross-covariance storage and a separate offline score. The final outage has only one future GNSS epoch here; no smoothing benefit is asserted.

## Realistic targets and remaining work

For engineering planning, GNSS-only H~1.6m/V~2.1m and aided available H roughly0.8–1.5m/V0.2–1.5m are supported as scales, with recovery transients above them.The16-draw30s H range1–7m RMS and peaks below20m describes that sample only. Paired held-out104002 has30s H RMSE17.142m and peak31.025m (last error29.083m), so even that broader range is not a defensible upper target. A provisional stress envelope would need to allow at least~18m RMS/~32m peaks, with no reliable upper guarantee yet.60s H roughly1–15m RMS with tens-of-metres peaks remains a plausible working envelope. These are not acceptance limits; the16 runs plus seed900 cannot establish failure probabilities. Vertical NHC-aided RMS can remain small here because vehicle vertical velocity is constrained, unlike unconstrained raw INS.

A60s outage at20m/s spans1.2km. It is a fair stress test, but sub-metre online performance throughout is not a realistic requirement for this grade without forward-speed aiding, better calibration or another independent measurement. No odometer is introduced. No tiny NHC-sigma trial is warranted by these data.

Residual plan: implement and validate causal initial alignment before making deployability claims; perform a predeclared larger Monte Carlo separating sensor, trajectory and alignment variation; jointly assess colored GNSS/NHC/Q calibration and conditional bias uncertainty; consider structural changes only after those diagnostics isolate a cause. Existing process-noise factorial and causal-course trials are closed evidence. DETECTION remains a separate open FAIL gate. No external blocker; finite ensemble and truth-relative initialization limit the evidence.

## Evidence and reproduction

Run `OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/audit_segment_ensemble.py --out OUTPUT`. Run `python scripts/audit_scheme.py --out OUTPUT --seeds 900 104001 104002` before/after. Artifact root: `/workspace/ccgins-v08/results/cc_scheme_audit/segment_ensemble/`; contains before/after CSV histories, full budgets, ensemble.json, compressed signed traces and regression output. Repo summary.json retains compact numerical evidence. Independent review and regression must pass before commit/push.

Regression:260 passed in103.98s. Independent review initially failed the provisional-target prose; corrected to include held-out104002 and submitted for re-review.
