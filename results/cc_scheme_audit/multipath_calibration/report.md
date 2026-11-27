# Colored GNSS covariance-calibration continuation

Candidate `scheme_multipath_calibration`; parent `e9060d2`. New diagnostic work; no online algorithm or default changed. Whole-scheme audit remains incomplete. DETECTION FAIL; confirmed_v08=false.

## Error budget first

Fresh nominal seed900/init0 reproduces every segment field in the previous maneuver-observability budget exactly. Config remains example_sim.yaml: duration180s, t_start30s, sigma_h1m/axis, sigma_v2m, multipath .5m with30s time constant, outages [60,90) and [120,180). Scoring uses same-epoch IMU-origin position;180s is a separate reacquisition. The final outage still contributes98.462% of horizontal SSE. Original cfad161 overall H7.173489m/peak28.257610m; current parent H7.019110m/peak27.684186m. Those are prior measured algorithm results, not results of a new filter change.

The following pairs are nominal / zero-multipath CONTROL H,V RMSE in metres. Removing simulator multipath is an unattainable-input diagnostic, not a promoted accuracy result or before/after algorithm improvement. Both use identical truth, IMU, white GNSS draws, availability and initialization. No online estimate receives truth.

|Seed|Init|30–60 available|30s outage|90–120 available|60s outage|Overall|
|---:|---:|---|---|---|---|---|
|900|0|0.989,1.186 / 0.929,0.420|1.488,0.677 / 1.367,0.064|0.768,0.474 / 0.467,0.233|11.016,0.640 / 8.027,0.437|7.019,0.763 / 5.133,0.351|
|900|1000|0.967,1.139 / 0.745,0.393|1.576,0.687 / 1.441,0.090|0.768,0.475 / 0.475,0.247|11.165,0.651 / 8.355,0.466|7.115,0.754 / 5.336,0.363|
|73101|0|0.774,0.811 / 0.558,0.253|1.558,1.342 / 1.428,0.647|0.921,0.689 / 0.692,0.436|4.261,0.293 / 3.808,0.385|2.835,0.788 / 2.523,0.440|
|73101|73201|0.816,0.785 / 0.516,0.269|1.511,1.286 / 1.441,0.590|0.901,0.667 / 0.673,0.416|4.465,0.281 / 3.999,0.373|2.953,0.758 / 2.637,0.417|
|73102|0|1.382,1.213 / 0.572,0.740|2.421,0.389 / 2.009,0.190|1.467,0.470 / 0.776,0.141|4.595,0.905 / 5.346,0.397|3.229,0.834 / 3.524,0.428|
|73102|73202|1.371,1.237 / 0.562,0.765|2.452,0.407 / 2.035,0.186|1.466,0.482 / 0.776,0.137|4.619,0.926 / 5.375,0.404|3.246,0.853 / 3.544,0.439|
|73103|0|0.789,0.964 / 0.706,0.826|1.103,0.885 / 0.893,0.849|0.923,0.851 / 0.517,0.702|1.255,0.853 / 0.745,0.649|1.080,0.882 / 0.731,0.740|
|73103|73203|0.792,0.930 / 0.711,0.792|1.081,0.877 / 0.874,0.839|0.925,0.863 / 0.514,0.713|1.258,0.861 / 0.741,0.657|1.078,0.879 / 0.725,0.735|
|73104|0|1.175,0.401 / 0.740,0.194|1.593,0.989 / 1.883,0.523|0.662,0.633 / 0.552,0.307|2.047,0.773 / 1.733,0.486|1.596,0.740 / 1.442,0.419|
|73104|73204|1.197,0.384 / 0.736,0.193|1.669,0.966 / 1.908,0.501|0.649,0.617 / 0.539,0.292|2.051,0.754 / 1.737,0.467|1.616,0.721 / 1.450,0.403|
|73105|0|0.836,0.680 / 0.743,0.267|7.209,1.216 / 6.024,0.255|1.112,0.889 / 0.643,0.170|3.939,1.229 / 4.486,0.334|4.120,1.073 / 3.936,0.279|
|73105|73205|0.816,0.716 / 0.741,0.251|7.387,1.263 / 6.214,0.297|1.105,0.909 / 0.638,0.184|3.741,1.233 / 3.969,0.333|4.108,1.094 / 3.769,0.286|

## Covariance evidence

Seeds73101–73105 are new comparison trajectory/sensor draws. Each seed has alignment0 plus seed+100; both alignments reuse sensors and are dependent. Twelve nominal runs and twelve paired controls are six independent trajectory/sensor draws, not24 independent trials. Table entries are unweighted means of per-run statistics (mean RMSE is not pooled RMSE). NEES uses the full physical3D position covariance and divides by3. Coverage uses chi-square3 threshold7.814727903251179. Samples within time windows are correlated; no binomial confidence intervals or formal ensemble calibration claim.

|Segment|Nominal H/V RMSE|Control H/V RMSE|Nominal NEES/3|Control NEES/3|Nominal95% coverage|Control95% coverage|
|---|---|---|---:|---:|---:|---:|
|available_30_60|0.992045/0.870408|0.688157/0.446982|2.257079|0.854922|0.653889|0.981111|
|outage_60_90|2.587468/0.915323|2.293149/0.419126|1.953172|0.755942|0.731111|1.000000|
|available_90_120|0.972170/0.668267|0.605260/0.331420|2.641695|0.908096|0.503333|0.980556|
|outage_120_180|4.534385/0.783160|4.026796/0.449081|2.120359|1.013506|0.784028|0.995833|

## Findings and limits

The white-noise covariance mismatch already identified in the original audit now has causal support: sim.py:197–203 adds AR(1) multipath while pipeline.py:179 builds Rm from white sigmas alone, and pipeline.py:437 supplies it to the update. filters.py:133–145 treats this as epoch measurement noise without a colored state. Removing only multipath substantially reduces position overconfidence in this sample. This is an established model limitation, not a newly discovered sign/unit implementation bug. No new production bug was established, so no speculative filter fix is claimed. Coverage near98% in controls also warns against simply inflating R universally. Remaining effects include nominal vibration density, NHC mismatch, nonlinear initialization and process discretization.

The model ablation changes GNSS residuals and therefore the estimated entry attitude/velocity/bias; long-outage differences cannot be uniquely assigned to one state. Five checkpoints per run save signed position and full covariance for independent recomputation, not a full time trace. The prior signed time traces,10s bins, GNSS-only residuals and exact-truth pure-INS diagnostics remain in the linked earlier audit; no new GNSS-only or pure-INS baseline is invented.

Unit/frame and full-path findings remain in [whole-scheme audit](../report.md), with corrected reset/discretization conclusions in [covariance reset](../covariance_reset/report.md) and [maneuver observability](../maneuver_observability/report.md). Bias30deg/h and5mg are resolved from sim.py:179–180; ARW/VRW conversions in sim.py:43–52. One-axis unknown-bias endpoint scales remain22.1/88.3m accel and6.4/51.4m gyro tilt over30/60s; seed900 truth-initialized raw INS endpoints21.39/101.89m horizontal are prior measurements. Available GNSS-only H1.427m/V2.125m includes multipath, versus stationary expectation1.581/2.062m. NHC-aided available H~.7–1.5m,30s H~1–5m,60s H~3–15m remain provisional planning ranges, not guarantees. A60s gap at20m/s spans1.2km and is a reasonable stress test; sub-metre online performance throughout is unrealistic for this grade without stronger independent aiding. Earth/transport omission, idealized alignment and planar constraints limit external realism. No smoother was added; any RTS result must be a separate offline metric.

## Residual plan and honest stop

Next implement an optional colored-GNSS state model or equivalently justified temporal-whitening model, including its cross-covariance and outage propagation. Validate analytic Jacobians, batch/stream parity and initial transient; compare against current nominal filter on unchanged seed900 and newly held-out seeds. Do not replace colored noise with an untested scalar sigma change. Separately isolate serial NHC mismatch and causal speed-dependent process noise; expand independent draws before setting tail gates. No accuracy/default promotion is justified by this ablation alone. DETECTION remains unresolved and its gate unchanged.

Reproduce: `OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/audit_calibration.py --out OUTPUT`. Evidence: calibration.json(24rows), pytest.txt, validation.json, independent reviews. This is a structured mid-audit delivery; no external blocker, remaining work is substantive model implementation and validation.

Regression:215 passed in185.00s, including white-draw pairing and full-covariance/half-open-segment fixtures. Independent review is recorded separately.
