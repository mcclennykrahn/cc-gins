# Outage entry transport: bounded whole-scheme audit continuation

Candidate `scheme_entry_transport`, parent `cdabe84`. New offline diagnostic evidence; no production change or new production bug established. Whole-scheme audit remains incomplete. DETECTION FAIL; confirmed_v08=false.

## Error budget first

Fresh before/after use configs/example_sim.yaml unchanged: duration180s, dt0.01s, seed900 plus held-out105001/105002, init_seed0, t_start30s. All budget dictionaries and signed position CSV bytes match exactly. Table values are before = after, metres. Full 10s H/V bins, GNSS-only and raw INS are in external before/budget.json; compact evidence is summary.json.

|Seed|Segment|H RMSE|V RMSE|H SSE share|
|---:|---|---:|---:|---:|
|900|available_30_60|0.989350|1.185786|0.3971%|
|900|outage_60_90|1.488475|0.676636|0.8988%|
|900|available_90_120|0.767795|0.474157|0.2391%|
|900|outage_120_180|11.016203|0.640390|98.4624%|
|900|terminal_available|1.386857|0.378029|0.0026%|
|900|available|0.886598|0.902407|0.6388%|
|900|overall|7.019110|0.762560|100.0000%|
|105001|available_30_60|0.780639|0.298740|2.7691%|
|105001|outage_60_90|3.367452|0.142540|51.5273%|
|105001|available_90_120|0.419331|0.359720|0.7990%|
|105001|outage_120_180|2.221544|0.422090|44.8513%|
|105001|terminal_available|1.876714|0.483912|0.0533%|
|105001|available|0.630734|0.330953|3.6214%|
|105001|overall|2.097263|0.345157|100.0000%|
|105002|available_30_60|1.054225|1.033593|4.3353%|
|105002|outage_60_90|2.346825|0.702245|21.4842%|
|105002|available_90_120|1.054226|0.676683|4.3354%|
|105002|outage_120_180|2.991767|0.930693|69.8301%|
|105002|terminal_available|1.075736|0.860537|0.0150%|
|105002|available|1.054261|0.873538|8.6857%|
|105002|overall|2.263558|0.866223|100.0000%|

Seed900 long-gap H maximum27.684186m and overall7.019110m are current-parent measurements; historical baseline7.173m/28m is prior evidence, not this round. Long-gap domination holds for900 but not105001 (short gap51.53% H SSE). Available<=1m is plausible, not universal (105002 combined1.054261m).

## Signed entry-state decomposition

For each outage, anchor the last logged epoch at/before start (60/120s here). This state includes propagation and NHC at that epoch; it is not a pre-update snapshot. At every scored epoch, exactly e(t)=p0+(t-anchor)*v0+r(t). p0 and v0 are signed ENU estimation errors. r includes changing attitude, residual biases, IMU noise, motion and subsequent NHC updates. This is algebraic bookkeeping, not a counterfactual replay or causal sensor attribution. The diagnostic reads truth-relative errors only after run returns; existing truth-relative initialization is unchanged and remains a limitation. No additional truth-fed online update is introduced.

|Seed|Gap|Observed H RMSE|Position+velocity transport H RMSE|Remainder H RMSE|
|---:|---:|---:|---:|---:|
|900|30s|1.488475|0.422956|1.690814|
|900|60s|11.016203|0.985817|10.612496|
|105001|30s|3.367452|3.218490|1.673145|
|105001|60s|2.221544|3.530924|3.884708|
|105002|30s|2.346825|1.221139|1.189455|
|105002|60s|2.991767|1.011249|2.724107|

Seed900 long-gap growth is not explained by constant entry-position/velocity transport alone (.986m vs observed11.016m). This does not isolate bias versus attitude versus NHC error. For105001, long-gap transport3.531m and remainder3.885m combine to observed2.222m because they cancel. Removing entry error cannot be assumed to improve this run. Full squared terms and signed cross terms (including negatives), ENU entry errors, vertical decomposition and endpoint vectors are retained in decomposition/decomposition.json. Cross terms are essential: no component percentage or additive RMSE claim is made.

## GNSS-only and pure INS comparison

GNSS residuals are scored at the true antenna p+Rlever, while online errors are same-epoch IMU-origin errors. Half-open outages60–90/120–180s exclude reacquisition; corrected180s is separate terminal_available. Raw INS starts from exact truth at each gap and uses uncorrected IMU, without NHC or bias subtraction; it is an optimistic initialization for an unaided sensor diagnostic, not the online filter.

|Seed|GNSS H/V RMSE|Raw INS30s H RMSE / endpoint|Raw INS60s H RMSE / endpoint|
|---:|---:|---:|---:|
|900|1.427 / 2.125|9.458 / 21.390|42.694 / 101.889|
|105001|1.350 / 2.282|10.010 / 22.244|30.389 / 66.494|
|105002|1.533 / 2.140|10.664 / 24.617|52.606 / 125.026|

## End-to-end correctness and limitations

No new production bug is demonstrated in this round; historical bugs with original file/line and fixes remain in ../report.md and linked prior reports. Current source evidence:

- IMU units at ccgins/sim.py:181–187 resolve30deg/h to30*pi/180/3600rad/s and5mg to.04905m/s². ARW/VRW density conversion at sim.py:45–58 divides hour-root by60. Bias random walks and speed vibration also contribute beyond YAML fields. One-axis raw accelerometer-bias endpoint scale bT²/2=22.073/88.290m; gyro-bias gravity coupling gbT³/6=6.421/51.365m at30/60s. White ARW/VRW endpoint scales .944/5.338m and .158/.447m exclude vibration and initial errors; these are not additive bounds.
- Truth/integration at sim.py:158–169 and filters.py:32–45 use ENU/FLU body-to-nav matrices, old-attitude acceleration, second-order position, then rotation. No quaternion path here. Constant gravity and absent Earth rotation match default synthetic truth; the optional fixed tangent Earth model omits transport and is not validated by this simulation.
- GNSS at sim.py:191–212 rounds epochs to grid, uses p+Rlever and actual epoch-spacing MP; filters.py:122–124 matches lever sign/frame. sigma_h1m is per axis, sigma_v2m, mp_sigma.5m. Stationary GNSS-only radial H/V expectation1.581/2.062m includes MP; finite initialized MP differs. Filter white R remains a colored-error approximation.
- InEKF at filters.py:91–118 uses first-order process discretization, Joseph update and covariance reset. Prior reset/discretization/observability evidence is closed; this decomposition neither demonstrates a Jacobian bug nor validates joint covariance calibration. Bias states are additive body-frame states, not guaranteed observable merely because they exist.
- Initial state at pipeline.py:119–132 is truth-relative with zero initial bias estimates, a deployability limitation. NHC at pipeline.py:312–317 constrains lateral/vertical body velocity, not forward speed; simulated slip means it is approximate. Default ZUPT is off. Outage handling skips invalid GNSS while IMU/NHC continue; same-epoch logging is pipeline.py:521–533.
- No RTS smoother or odometer added. Offline smoothing would require separate metrics; only one terminal future GNSS epoch is available after the last gap.

## Targets, review and residual plan

A60s gap at20m/s spans1.2km: reasonable as a MEMS stress scenario, unsupported as a sub-metre online gate. vmax is a cap; per-gap realized speeds are recorded in before/budget.json. GNSS-only planning H~1.6m/V~2.1m and aided available H~0.8–1.5m with recovery excursions remain plausible scales. Prior held-out104002 short-gap H17.142m/peak31.025m prevents treating1–5m as a30s bound. Working stress envelopes must at least allow~18m RMS/~32m peak for30s and tens-of-metres peaks for60s; neither is a validated tail target. Vertical NHC-aided errors remain small in this synthetic scenario and do not establish real vertical INS performance.

New diagnostic bug scripts/audit_entry_transport.py:30 in the initial draft passed an array as scalar boundary to timing.at_or_after; independent initial review FAIL caught it, corrected by an explicit upper-bound comparison, then re-reviewed. No change to production filtering. Regression and final independent review are external pytest.txt and reviews/entry_transport_final.json.

Residual plan: causal full initialization with uncertainty/refusal; counterfactual or sensitivity diagnostics separating entry attitude/bias from subsequent motion-constraint error; joint colored GNSS/NHC/process-noise calibration; predeclared independent trajectory/sensor/alignment tail study. The algebraic remainder cannot resolve these causes. No external blocker; bounded evidence stop. DETECTION remains FAIL and no V0.8 claim.

## Reproduction

Use OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. /tmp/ccgins-accuracy-venv/bin/python. Run scripts/audit_scheme.py --seeds 900 105001 105002 --out OUTPUT before/after and scripts/audit_entry_transport.py --out OUTPUT. Artifact root /workspace/ccgins-v08/results/cc_scheme_audit/entry_transport contains before/after CSVs, budgets, compressed ep/ev traces, signed terms and validation.json. No raw long logs committed.
