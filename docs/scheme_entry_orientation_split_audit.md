# Scheme audit: separate outage-entry velocity, heading and tilt

Candidate: scheme_entry_orientation_split. Base d878779. New offline diagnostic work; MID_AUDIT. No production filter/config change or online improvement claimed. DETECTION remains FAIL; confirmed_v08=false.

## State and error budget FIRST

Read pending_handoff and current project review.json before coding. HEAD=origin/main=d878779; only .dsh-prompts/ was untracked. Latest reviewed/pushed work and negative trials are CLOSED. No unfinished patch required resumption. Prior Stage3/v0.8 evidence preserved.

Fresh pre-edit baseline: configs/example_sim.yaml, seed900, duration180s, dt0.01, scoring t_start30s; held-out111001/111002 chosen before results. Native init_seed0 is shared: these are independent trajectory/sensor draws, not independent alignment validation. Native posterior at IMU origin, same-time truth, sampled10Hz; half-open outages[60,90),[120,180), terminal180s reacquisition separate. Before/after budget dictionaries and signed CSV bytes match exactly for all three draws (paired_check.json). Tables are before=after, metres.

|Seed|Segment|H RMSE|V RMSE|H SSE share %|
|---:|---|---:|---:|---:|
|900|available_30_60|0.989350|1.185786|0.3971|
|900|outage_60_90|1.488475|0.676636|0.8988|
|900|available_90_120|0.767795|0.474157|0.2391|
|900|outage_120_180|11.016203|0.640390|98.4624|
|900|terminal_available|1.386857|0.378029|0.0026|
|900|available|0.886598|0.902407|0.6388|
|900|overall|7.019110|0.762560|100.0000|
|111001|available_30_60|0.542677|0.721807|0.7592|
|111001|outage_60_90|5.690474|0.920050|83.4818|
|111001|available_90_120|1.391425|0.469002|4.9913|
|111001|outage_120_180|1.443342|0.463864|10.7415|
|111001|terminal_available|1.745759|0.615804|0.0262|
|111001|available|1.057590|0.608686|5.7767|
|111001|overall|2.784344|0.635251|100.0000|
|111002|available_30_60|0.497343|0.715861|0.3322|
|111002|outage_60_90|8.475547|0.314496|96.4752|
|111002|available_90_120|0.784231|0.166957|0.8260|
|111002|outage_120_180|0.934138|0.251665|2.3439|
|111002|terminal_available|2.254885|0.146692|0.0228|
|111002|available|0.662516|0.519377|1.1809|
|111002|overall|3.857715|0.391273|100.0000|

Seed900 long gap accounts for98.4624% of scored H SSE, confirming the hypothesis for this draw. Peak long-gap error27.684186m; overall7.019110m. Historical7.173m/28m numbers refer to an earlier revision, not a new baseline. Both held-out draws are dominated by the short gap (83.48% and96.48%); dominance is scenario dependent. Available H0.887/1.058/0.663m supports order-one-metre performance but refutes a strict <=1m bound.

Ten-second budgets and full signed errors are in before/ and after/. GNSS-only scores use true antenna p+Rlever; INS diagnostic starts each gap with exact R/v/p, no bias subtraction or aiding, propagates every raw IMU step, including the endpoint before GNSS correction.

|Seed|GNSS-only H/V RMSE|INS30s H RMSE/endpoint|INS60s H RMSE/endpoint|Mean gap speed30s/60s|
|---:|---|---|---|---|
|900|1.427/2.125|9.458/21.390|42.694/101.889|10.131/8.954|
|111001|1.738/1.880|23.712/54.567|109.282/253.018|7.333/10.939|
|111002|1.456/2.137|16.657/38.669|45.886/95.715|12.208/11.635|

## Physical budget and achievable targets

Config GNSS sigma_h1m per axis, sigma_v2m, mp_sigma0.5m (configs/example_sim.yaml:18-20); stationary asymptotic radial H sqrt(2*1.25)=1.581m, V sqrt(4.25)=2.062m. Multipath starts at zero; finite-run residuals above differ. Online available H roughly0.7–1.5m is a planning target, not a guaranteed bound. A few metres is a nominal30s goal, but held-out8.476m RMS here and prior counterexamples preclude a tight guarantee. Sixty-second RMS ranges from0.934 to11.016m here and prior45.892m counterexample remains valid; plan metres to tens of metres, not sub-metre. No tail bound has been validated.

Units resolved from code, not assumed: sim.py:181 converts30deg/h to1.45444e-4rad/s; sim.py:182 converts5mg to0.04905m/s². sim.py:45-51 converts ARW0.3deg/sqrt(h) to8.72665e-5rad/sqrt(s), VRW0.1m/s/sqrt(h) to0.00166667m/s/sqrt(s). Flat straight one-horizontal-axis unaided endpoint scales30/60s: initial accel bias bT²/2=22.073/88.290m; gyro bias gravity coupling gbT³/6=6.421/51.365m; ARW gqT^2.5/sqrt20=0.944/5.338m; VRW qT^1.5/sqrt3=0.158/0.447m. These are scale estimates, not additive online RMSE fractions. Real draws include bias RW, maneuvers, speed vibration and aided calibration. At20m/s a60s outage spans1.2km: reasonable stress test for this IMU grade, unsupported sub-metre online acceptance test. Actual speeds are lower and vary by gap.

## End-to-end audit and distinction from bugs

No new production bug established in this continuation. No production tuning justified. Previously fixed bugs remain archived in earlier reports, not reopened here.

* Truth/IMU: ccgins/sim.py:152-168 uses body FLU to local ENU R, second-order position, old-attitude specific force and relative SO(3) logarithm consistent with ccgins/filters.py:33-45. No quaternion representation/order. Constant gravity[0,0,-9.81]; default truth/filter omit Earth rate consistently. Optional Earth/Coriolis at filters.py:21-44 omits transport: scope limitation for real navigation, not a newly confirmed default sign bug.
* Sensor units: sim.py:45-51,181-188. Vibration and base sample standard deviations are added before a single Gaussian draw; this is a documented generative choice, not independent-source quadrature. Filter noise in filters.py:98-100 is fixed and does not model speed vibration. Calibration limitation, not evidence that another scalar tweak repairs drift.
* GNSS: sim.py:198-212 and filters.py:122-124 both use positive p+Rlever in body FLU. Multipath uses actual rounded sample period; nominal GNSS covariance does not represent its temporal correlation. pipeline.py:256-260,416-420 propagates then consumes same-index valid fix. No interpolation/lever-point mismatch in this score.
* InEKF: filters.py:233-255 bias couplings/F/G,282-291 GNSS/body-velocity Jacobians; invariant NHC attitude cancellation is intentional. First-order Phi and Qd at94-103 is an approximation, not newly proved dominant bug. Joseph covariance and finite reset at110-118,257-280 retain prior reviewed correction. Bias observability depends on maneuvers; position accuracy alone cannot establish bias calibration.
* Motion constraints: pipeline.py:315-317 assumes zero lateral/vertical velocity while sim.py:154-157 has small slip. Default ZUPT/odometry disabled. Existing trusted-GNSS stop-evidence fix remains intact. NHC feedback and entry-state errors interact; prior replay evidence did not justify tiny sigma tuning.
* Initialization: pipeline.py:119-132 uses truth-relative R/v/p perturbations and zero biases; filters.py:303-319 converts initial covariance to invariant coordinates. This is optimistic benchmark alignment, not deployable sensor-only initialization.
* Outages/scoring: native NHC/IMU continues during missing GNSS; pipeline.py:528-540 logs same-epoch posterior at IMU origin. No RTS exists and none is claimed. Terminal180s GNSS would be future information in any smoother; report that separately if implemented.

## Intervention definition

scripts/audit_entry_state.py adds three one-time privileged interventions immediately before first propagate in each gap, after logged posterior at entry. Velocity replaces nav v only. Heading replaces ZYX yaw by left nav-up rotation while preserving estimated yaw-free tilt. Tilt replaces yaw-free ZYX rotation while preserving estimated yaw. Vertical forward axes are refused (undefined yaw). This convention is coordinate dependent, not a globally unique tilt/heading decomposition.

R, v, biases and position evolve normally afterward. Internal covariance stays fixed, so these are conditional native-filter sensitivity experiments, not consistent oracle posteriors, additive causal fractions or deployable algorithms. Changing nominal state changes invariant covariance interpretation; subsequent NHC/bias feedback and cancellation can worsen errors. Prefix t<=entry p/v/P_pos must be bit-identical to native. Evidence records actual truth, intervention target, before and after separately.

|Seed|Gap s|Online H/V|Offline heading H/V|Offline tilt H/V|Offline velocity H/V|
|---:|---:|---|---|---|---|
|900|30|1.488/0.677|1.400/0.675|1.523/0.711|1.654/0.691|
|900|60|11.016/0.640|12.924/0.686|10.807/0.630|10.622/0.630|
|111001|30|5.690/0.920|5.649/0.917|3.216/0.791|4.592/0.945|
|111001|60|1.443/0.464|1.193/0.461|1.481/0.496|1.482/0.451|
|111002|30|8.476/0.314|8.444/0.313|5.586/0.306|6.689/0.327|
|111002|60|0.934/0.252|0.794/0.241|0.905/0.260|1.009/0.257|

Truth heading substitution worsens seed900 long-gap H11.016→12.924m, whereas tilt10.807m and velocity10.622m have modest effects. Held-out short gaps improve most under tilt (5.690→3.216m,8.476→5.586m); velocity also helps, while heading barely changes them. None is universally beneficial:111001 long-gap tilt/velocity worsen;111002 long-gap velocity worsens. This is evidence of interactions and cancellation, not proof of an additive error decomposition. Prior attitude+bias joint replay and these single interventions use different conditional assumptions and draws; do not subtract them. No production fix or accuracy trial is promoted.

## Verification, reproduction and residual plan

Use PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 with /tmp/ccgins-accuracy-venv/bin/python:

```
-m scripts.audit_scheme --seeds 900 111001 111002 --out /tmp/split_budget
-m scripts.audit_entry_state --seeds 900 111001 111002 --arms heading tilt velocity --out /tmp/split_interventions
-m pytest -q
```

Tests cover wrapped yaw, complementary ZYX component preservation, SO(3) validity, vertical-axis refusal, each intervention's exact timing and one-time application, covariance preservation, unchanged fields, native prefixes and configuration refusals. Independent reviewer recomputed all18 outage H/V metrics from signed traces, verified prefixes, intended fields and before/after parity (independent_check.py/json); independent focused regression51 passed. Final full regression:301 passed in74.78s; independent verdict PASS. Evidence is attached in pytest_final.txt and /workspace/ccgins-v08/reviews/entry_orientation_split.md.

Residual plan: (1) validate causal sensor-only initialization with uncertainty/refusal across independent starts; (2) test joint velocity/tilt/heading/bias interactions with specified covariance treatment on frozen splits; (3) jointly calibrate GNSS colored multipath, speed-dependent process uncertainty and NHC slip using training/validation separation; (4) predeclare segment-tail targets and evaluate independent trajectory, sensor and alignment draws. An optional RTS remains separately scoped offline work, especially because terminal180s reacquisition supplies future information. Three fixed-init draws do not close alignment/observability/calibration/tail validation. No external blocker; evidence limits prevent completion. DETECTION remains FAIL, confirmed_v08=false.
