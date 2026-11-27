# Bias and covariance continuation of the whole-scheme audit

Candidate: scheme_bias_consistency; parent 8251fe0. New diagnostic work only; no filter tuning or accuracy promotion. Baseline/error-budget-first evidence and end-to-end bug audit remain in results/cc_scheme_audit/report.md. That report contains before/after segment H/V, pure INS expectations, bias unit resolution, GNSS-only accuracy, frames/lever-arm/timing, constraints, fairness and provisional segment targets. The three fixed bugs remain closed. No new algorithm bug is established by this continuation.

## Experiment and measured results

Same configs/example_sim.yaml, duration180s, scoring t>=30s, seeds900/49501/49502; initialization seeds0 and17. All data generated independently of estimated biases. Logging added at pipeline.py:149-152,245,525-526,542 is opt-in and occurs after updates at matching epochs. It never supplies truth to propagation/update. Body-frame bias error uses simulator bias at that logged epoch. Bias marginal covariance blocks are valid for both filters' additive body bias convention. Position uses transformed physical position covariance, not the invariant position sub-block.

Each NEES below is divided by dimension3. Coverage is fraction inside the nominal 95% 3D position ellipsoid. Time samples are correlated; these are descriptive diagnostics, not independent-sample confidence tests or formal observability results.

|seed|init|segment|H RMSE m|position NEES/3|95% coverage|gyro bias NEES/3|accel bias NEES/3|
|---|---|---|---:|---:|---:|---:|---:|
|900|0|available_30_60|0.986797|2.963416|0.470000|0.528293|2.044704|
|900|0|outage_60_90|1.512239|1.116252|1.000000|1.047862|1.838116|
|900|0|available_90_120|0.768319|2.591559|0.406667|2.275059|3.243979|
|900|0|outage_120_180|11.040119|2.052582|1.000000|2.616826|1.284974|
|900|17|available_30_60|0.951476|2.847030|0.510000|0.402246|1.874781|
|900|17|outage_60_90|1.569656|1.141262|1.000000|1.112840|1.798974|
|900|17|available_90_120|0.771085|2.625741|0.393333|2.245768|3.235164|
|900|17|outage_120_180|11.123544|1.987295|1.000000|2.557580|1.288987|
|49501|0|available_30_60|1.034644|1.929294|0.843333|0.470528|0.449282|
|49501|0|outage_60_90|0.824900|0.336720|1.000000|1.290367|0.284416|
|49501|0|available_90_120|1.161532|3.164380|0.480000|1.949918|0.869834|
|49501|0|outage_120_180|5.200525|3.308504|0.365000|1.328419|0.695329|
|49501|17|available_30_60|1.036333|1.872217|0.880000|0.457809|0.497866|
|49501|17|outage_60_90|0.821997|0.252437|1.000000|1.360496|0.356710|
|49501|17|available_90_120|1.170374|3.268674|0.483333|2.123124|1.008133|
|49501|17|outage_120_180|5.113479|3.468108|0.340000|1.511397|0.765990|
|49502|0|available_30_60|0.660441|0.800840|0.966667|0.274821|0.486628|
|49502|0|outage_60_90|3.431730|0.775785|1.000000|0.130635|0.601084|
|49502|0|available_90_120|1.143458|2.272503|0.693333|0.671988|0.299850|
|49502|0|outage_120_180|4.840543|0.652099|1.000000|0.417140|0.328045|
|49502|17|available_30_60|0.654602|0.796946|0.966667|0.350959|0.479990|
|49502|17|outage_60_90|3.960098|0.741589|1.000000|0.122164|0.487519|
|49502|17|available_90_120|1.153507|2.290703|0.683333|0.717245|0.283221|
|49502|17|outage_120_180|4.923612|0.685534|1.000000|0.469416|0.357223|

## Interpretation and remaining work

At119.9s seed900/init0 residual gyro bias is [-13.257,-8.509,12.785]deg/h and accel bias [-.504,.591,-.791]mg; reported marginal sigmas are [5.609,6.246,9.261]deg/h and [.436,.474,.371]mg. These residuals explain why raw turn-on bias scales are inappropriate for the aided filter, while also showing that bias estimation has not removed drift sources. They do not establish causal shares of position error: attitude, velocity, bias correlations and maneuvers also matter.

Seed900 position covariance is overconfident during available30–60s (coverage47%) and available90–120s (40.7%). Seed49501 long-outage coverage36.5% contrasts with seed49502's100%; a single successful trajectory does not validate uncertainty. Changing init seed to17 gives overall H7.088986/3.328350/3.630609m versus7.035064/3.381255/3.474680m at init0. Six runs are insufficient for tail targets. Both init seeds reuse sensor/trajectory per seed and are not six independent sensor draws.

No online outputs change: init0 reproduces the parent after-budget H segment metrics exactly to numerical precision. Optional diagnostic logging parity is regression-tested. No new before/after accuracy claim; existing paired algorithm before/after remains in the parent report. No smoothing or extra forward-speed aiding added.

Potential causes remain hypotheses: first-order Phi/Qd at filters.py:96-100; lack of post-injection covariance-coordinate reset at filters.py:110-115; colored multipath not represented in R; fixed nominal vibration density; approximate NHC. This experiment cannot distinguish them, so no structural modification is justified yet. Next perform isolated finite-injection reset and process-noise consistency tests in matched-model Monte Carlo, then paired real-scenario validation; compute maneuver-conditioned scaled observability singular values with independently varied initial alignment before changing bias estimation. Expand independent sensor/trajectory seeds before adopting provisional targets as gates. Optional RTS needs manifold/cross-covariance logging and a separate offline result; terminal outage has little future information.

Audit remains explicitly incomplete. DETECTION remains FAIL; confirmed_v08=false. No V0.8 claim or gate change. Reproduce: OPENBLAS_NUM_THREADS=1 PYTHONPATH=. /tmp/ccgins-accuracy-venv/bin/python scripts/audit_bias.py --out OUTPUT.
