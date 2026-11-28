# Scheme audit continuation: component replay interval correctness

Candidate `scheme_component_interval`, parent `917e449`. New work; structured mid-audit delivery. No production filter/config change or accuracy improvement claimed. DETECTION remains FAIL; confirmed_v08=false. No external blocker; remaining work requires further implementation and independent evidence.

## State and measured error budget FIRST

Read pending_handoff.md and project review.json before coding. HEAD equaled origin/main, previous delivery reviewed/pushed; only .dsh-prompts/ was untracked. Closed negative trials and prior audits were retained. The stale pending causal-ZUPT note in completed.md is superseded by its subsequent reviewed/pushed entry. No genuinely incomplete working-tree patch remained.

Fresh required-config measurement preceded edits: configs/example_sim.yaml, seed900 and new held-out109001/109002, duration180s, dt.01s, metrics.t_start30s, online InEKF+NHC, default init_seed0. Inputs and scoring unchanged before/after. All RMSE units metres. Before/after H/V columns below each apply identically to both runs; signed CSV and JSON equality is checked separately in paired_check.json. Segment terminal_available is the single180s reacquisition; outages are half-open [60,90), [120,180). Ten-second time budgets are in before/budget.json and after/budget.json; complete signed ENU errors are errors_<seed>.csv. GNSS residual is at the true antenna; filter error at the IMU origin at the same posterior epoch.

|Seed|Segment|Before=after H RMSE|Before=after V RMSE|H SSE share %|
|---:|---|---:|---:|---:|
|900|available_30_60|0.989350|1.185786|0.397|
|900|outage_60_90|1.488475|0.676636|0.899|
|900|available_90_120|0.767795|0.474157|0.239|
|900|outage_120_180|11.016203|0.640390|98.462|
|900|terminal_available|1.386857|0.378029|0.003|
|900|available|0.886598|0.902407|0.639|
|900|overall|7.019110|0.762560|100.000|
|109001|available_30_60|1.124314|1.405879|7.350|
|109001|outage_60_90|1.804247|1.477571|18.928|
|109001|available_90_120|1.290158|1.150761|9.678|
|109001|outage_120_180|2.346563|0.466593|64.033|
|109001|terminal_available|0.779609|0.336392|0.012|
|109001|available|1.209491|1.283673|17.040|
|109001|overall|1.854032|1.087733|100.000|
|109002|available_30_60|1.341819|0.468617|6.652|
|109002|outage_60_90|2.571263|0.464606|24.427|
|109002|available_90_120|1.012385|0.137384|3.787|
|109002|outage_120_180|2.964728|0.227508|64.949|
|109002|terminal_available|3.872680|0.360870|0.185|
|109002|available|1.198042|0.345335|10.624|
|109002|overall|2.325854|0.334041|100.000|

Seed900 confirms the hypothesis: long gap contributes98.462% horizontal SSE, max27.684186m. Current overall7.019110m differs from historical7.173489m/max28.257610m at cfad161 because of previously reviewed changes; this diagnostic claims no reduction. Held-out combined available H exceeds1m in both draws, so <=1m is a useful nominal aim, not a universal filter correctness criterion. Long-gap SSE shares are64.03% and64.95% on these two held-outs.

|Seed|GNSS-only H/V|Pure INS30s H RMS / endpoint|Pure INS60s H RMS / endpoint|Mean speed30s /60s m/s|
|---:|---|---|---|---|
|900|1.427 / 2.125|9.458 / 21.390|42.694 / 101.889|10.131 / 8.954|
|109001|1.855 / 2.425|9.069 / 21.043|47.899 / 106.058|5.551 / 7.157|
|109002|1.713 / 1.934|3.535 / 8.599|25.092 / 63.632|6.651 / 15.642|

Pure INS initializes exact R/v/p at each gap and uses raw IMU, unknown biases and no aiding for the complete30/60s, before reacquisition. It is an offline diagnostic, not a deployable filter result. Component ablations in components_before/components.json and components_after/components.json separately measure gyro bias, accelerometer bias, sampled gyro/accelerometer noise, combined biases/noise and all sources. Sampled noise includes vibration; biases include random walks. Norms cannot be summed because signed terms cancel and rotations couple them.

Measured component H RMSE / endpoint (m), exact default-interval parity before/after:

|Seed|Gap s|Gyro bias|Accel bias|Both biases|Both sample noises|All|
|---:|---:|---|---|---|---|---|
|900|30|1.960 / 5.055|5.293 / 11.052|7.243 / 16.155|2.286 / 5.412|9.458 / 21.390|
|900|60|15.949 / 42.210|24.636 / 55.605|40.545 / 98.045|3.224 / 6.437|42.694 / 101.889|
|109001|30|1.952 / 5.044|8.063 / 17.681|9.782 / 22.216|1.499 / 3.046|9.069 / 21.043|
|109001|60|16.612 / 43.223|32.580 / 66.038|44.987 / 99.539|4.160 / 8.419|47.899 / 106.058|
|109002|30|2.536 / 6.533|1.903 / 4.065|3.662 / 8.812|0.196 / 0.233|3.535 / 8.599|
|109002|60|19.496 / 50.039|8.814 / 18.001|27.833 / 67.383|3.857 / 7.134|25.092 / 63.632|

These uncalibrated bias ablations do not establish the attribution of residual online errors after GNSS/NHC bias estimation.

## Physical interpretation and realistic targets

Resolved units from code, not guesses: sim.py:179-184 converts30deg/h to0.000145444rad/s,5mg to0.04905m/s². sim.py:49-58 converts ARW0.3deg/sqrt(h) to8.72665e-5rad/sqrt(s), VRW0.1m/s/sqrt(h) to0.00166667m/s/sqrt(s). Defaults also include gyro bias RW3deg/h/sqrt(h), accel bias RW0.05mg/sqrt(s), speed-dependent vibration0.02m/s² per m/s and0.002deg/s per m/s. sim.py:185-188 adds base and vibration standard deviations before a single draw; those two terms are not independent quadrature sources.

One horizontal-axis endpoint scales for30/60s, zero initial error, flat straight linearization: turn-on accel bias bT²/2 =22.073/88.290m; gyro bias gravity coupling gbT³/6 =6.421/51.365m; ARW gqT^2.5/sqrt20 =0.944/5.338m; VRW qT^1.5/sqrt3 =0.158/0.447m. These omit vibration, random walks, maneuvers and aided bias estimation and cannot predict online NHC accuracy. The measured pure-INS components supply those realized distinctions.

GNSS sigma_h1m is per axis, sigma_v2m, MP sigma0.5m (configs/example_sim.yaml:18-20): stationary radial H sqrt(2*(1+.25))=1.581m, V sqrt(4+.25)=2.062m. Finite runs differ; MP starts at zero. Achievable planning goals: available H around1m (roughly0.7–1.5m on nominal trajectories), V roughly0.2–1.5m;30s gap H a few metres nominally,60s H a few to tens of metres, with metre-scale vertical performance when NHC is valid. These are provisional goals, not validated bounds or DETECTION gates: prior independent104002 already has30s H RMS17.142m and peak31.025m (existing /workspace/ccgins-v08/results/cc_scheme_audit/segment_ensemble/report.md). These three draws cannot establish tail acceptance. At20m/s60s spans1.2km: reasonable stress test for this MEMS grade, unsupported sub-metre online requirement without independent aiding/better calibration. This round's actual mean speeds are tabulated rather than assuming20m/s.

## End-to-end correctness re-examination

Historical bugs and their original line citations remain in ../report.md; subsequent review reports supersede closed reset/discretization/observability items. This round re-inspected:

- Truth/IMU: sim.py:1-5,158-169 and filters.py:32-45 use ENU navigation, FLU body, body-to-nav rotation matrices, old-R specific-force rotation, second-order position then velocity and right SO(3) increment. Truth inverts this same step; ideal-component replay checks closure. No quaternion ordering exists in this path. Fixed gravity[0,0,-9.81] and no default Earth rotation match both paths. filters.py:21-29 optional Earth model is local tangent with Coriolis, no transport; simulation cannot validate full Earth geodesy. This is scope, not an evidenced sign bug.
- GNSS: sim.py:191-217 and filters.py:122-124 agree on p+Rlever, FLU lever[.5,0,1.2], positive antenna offset and actual rounded-epoch multipath evolution. pipeline.py:256-260,416-420 propagates to k+1 before same-index GNSS, ignores invalid epochs. Colored MP is not explicitly estimated; nominal GNSS R omits its colored covariance. Prior colored/serial/calibration reports establish modeling limitations, not justification to retune one sigma here.
- InEKF: filters.py:233-255 includes body-frame additive bias coupling, invariant F/G;282-291 uses matching antenna and body-velocity Jacobians. NHC's direct invariant attitude column cancels algebraically; absence alone is not a defect. filters.py:91-118 uses first-order Phi/Qd, Joseph update, finite injection reset and symmetry. Higher-order alternatives/observability were previously measured, and current regression covers them. This round does not claim calibrated covariance or full bias observability from straight motion.
- Alignment/covariance: pipeline.py:119-132 uses truth-relative random R/v/p initialization and zero bias mean, optimistic relative to field alignment; filters.py:303-319 maps nav covariance to the filter convention. Maneuvers are needed for bias observability; NHC only measures lateral/vertical velocity. Causal sensor-only alignment with excitation/refusal and uncertainty remains open after previous startup feasibility refusals.
- Constraints/outages: pipeline.py:263-294 applies causal optional ZUPT/odometry; default example has neither enabled, uses NHC. Slip and vertical body motion in sim.py:152-156 violate exact zero body y/z and need joint uncertainty calibration. Outages keep propagation and NHC; no fabricated GNSS updates. pipeline.py:528-540 logs decimated same-time posterior states; t_start30 and terminal180 fix are explicit. No RTS smoother exists in this run; no offline-smoothed accuracy is claimed. RTS would need separate metric/report and cannot be represented as an online fix.

## New evidenced diagnostic bug and fix

At parent917e449, scripts/audit_ins_components.py:33-37 rounded both outage bounds and assumed t[0]=0. At dt.03, requested[.331,.601] became[.33,.60], integrating a sample before the outage. It could also silently accept slightly out-of-domain intervals or fail inconsistently on NaN/inf. This corrupts generalized component comparisons; default on-grid example values are unaffected.

Fix: share contained_steps in scripts/audit_scheme.py:56-70, use it in component integrate and publish actual_start/actual_end/propagated_duration/n_steps per replay. No partial samples invented. The example uses exactly the prior3000/6000steps. New tests in tests/test_scheme_audit.py exercise fractional and floating-grid boundaries with poisoned outside samples, verify closed-form constant-acceleration position and budget parity, reject invalid/empty/outside intervals. No production bug was demonstrated in this bounded continuation; no tuning, motion sensor or smoother was added without attribution evidence.

## Validation and residual plan

Reproduce using /tmp/ccgins-accuracy-venv/bin/python with PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1: -m scripts.audit_scheme --seeds 900 109001 109002 --out <before-or-after>; -m scripts.audit_ins_components with the same seeds; -m pytest -q. Parent component replay source was git show HEAD:scripts/audit_ins_components.py, preserved as /tmp/ccgins_components_parent.py during execution. Raw long stdout stays outside git. Initial command used unavailable python; corrected to the existing project venv before measurements.

Focused regression:38 passed. Full regression:288 passed in75.19s. Independent code/evidence review results are recorded in pytest.txt and /workspace/ccgins-v08/reviews/round_1795826951114844199_independent.md. Before/after signed histories, all segment dictionaries, GNSS/raw-INS metrics and component metrics are compared in paired_check.json. Independent review must pass before commit/push.

Residual plan: implement deployable sensor-only alignment with explicit uncertainty/refusal; use controlled signed replay to separate entry attitude/bias errors from in-gap NHC model mismatch; jointly calibrate colored GNSS, slip and process noise using frozen splits; independently vary trajectory/sensors/alignment for segment tails. Prior Q factorial and negative trials are closed evidence. This is a mid-audit evidence stop, not whole-scheme completion, with no concrete external blocker. DETECTION remains FAIL, confirmed_v08=false.
