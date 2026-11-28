# Causal coarse alignment feasibility and current whole-scheme budget

Candidate `scheme_coarse_alignment`, parent e9f1045. New bounded diagnostic; whole-scheme closure remains incomplete. No production changes, tuning, or new accuracy claim. Config configs/example_sim.yaml, seed 900 and fresh held-out 115001/115002, 180 s, scoring t_start=30 s. Shared synthetic init_seed=0 remains a limitation. Evidence root: /workspace/ccgins-v08/results/cc_scheme_audit/coarse_alignment/.

The error budget was run before implementing the new diagnostic. Before and after use identical online code and sensor seeds. Full budget.json files contain every interval, vertical RMSE, max/end error, 10 s bins, GNSS-only error and truth-initialized pure INS. Signed histories are errors_SEED.csv. No smoother is used.

| Seed | Segment | Before H/V RMSE m | After H/V RMSE m | H SSE share |
|---:|---|---:|---:|---:|
|900|available_30_60|0.989350/1.185786|0.989350/1.185786|0.397%|
|900|outage_60_90|1.488475/0.676636|1.488475/0.676636|0.899%|
|900|available_90_120|0.767795/0.474157|0.767795/0.474157|0.239%|
|900|outage_120_180|11.016203/0.640390|11.016203/0.640390|98.462%|
|900|terminal_available|1.386857/0.378029|1.386857/0.378029|0.003%|
|900|overall|7.019110/0.762560|7.019110/0.762560|100.000%|
|115001|available_30_60|0.751708/0.394607|0.751708/0.394607|1.747%|
|115001|outage_60_90|1.759361/0.382044|1.759361/0.382044|9.570%|
|115001|available_90_120|0.878645/0.254318|0.878645/0.254318|2.387%|
|115001|outage_120_180|3.735684/0.149768|3.735684/0.149768|86.289%|
|115001|terminal_available|0.885136/0.177223|0.885136/0.177223|0.008%|
|115001|overall|2.542603/0.286720|2.542603/0.286720|100.000%|
|115002|available_30_60|0.848907/0.307888|0.848907/0.307888|0.514%|
|115002|outage_60_90|2.610985/0.238425|2.610985/0.238425|4.865%|
|115002|available_90_120|1.041060/0.312570|1.041060/0.312570|0.773%|
|115002|outage_120_180|8.108257/0.322832|8.108257/0.322832|93.840%|
|115002|terminal_available|1.708793/0.555521|1.708793/0.555521|0.007%|
|115002|overall|5.291989/0.302822|5.291989/0.302822|100.000%|

After values are unchanged production behavior; byte equality is checked separately in validation.json before delivery. The historical 7.173489 m overall / 28.257610 m long-gap max baseline belongs to cfad161, not this parent. Current seed900 overall H=7.019110 m, long-gap H=11.016203 m, max=27.684186 m, SSE share=98.462%. This confirms long-gap dominance for seed900, not a universal dominance theorem. Both fresh held-out seeds also show long-gap dominance. All three aggregate available H RMSE values are below 1 m, while held-out115002 recovery interval is 1.041060 m. Do not turn a single seed's <=1 m observation into a universal gate.

| Seed | GNSS-only H/V m | Pure INS 30s H RMS/end m | Pure INS 60s H RMS/end m | Mean gap speed 30/60 m/s |
|---:|---:|---:|---:|---:|
|900|1.427/2.125|9.458/21.390|42.694/101.889|10.131/8.954|
|115001|1.368/2.403|7.209/14.712|40.709/74.644|8.601/15.112|
|115002|1.719/2.319|21.094/48.237|103.371/240.912|5.947/5.312|

Units resolved in ccgins/sim.py:43-58,179-185: gyro turn-on sigma is 30 deg/h (multiply pi/180/3600), accel turn-on sigma 5 mg (multiply .001*9.81). ARW=.3 deg/sqrt(h) converts to rad/sqrt(s) via pi/180/60; VRW=.1 m/s/sqrt(h) divides by60. Per-axis constant accel bias displacement scale .5*sigma_a*T² is22.073/88.290m at30/60s. Gyro bias gravity coupling g*sigma_bg*T³/6 is6.421/51.365m. ARW-only integrated gravity endpoint sigma g*sigma_ARW*T^2.5/sqrt(20) is.944/5.338m. These are unaided one-axis scales, not summed aided-filter forecasts. Additional random walk/vibration and actual turns affect measured INS. GNSS sigma_h=1m per axis, sigma_v=2m, correlated mp_sigma=.5m imply stationary radial H RMS1.581m and V2.062m; the realization starts multipath at zero.

## New alignment finding

The sampled start epochs in the first five seconds have zero translation, but are not a fixed attitude interval: simulator pitch and roll evolve at ccgins/sim.py:146-151. Using mean gyro as turn-on bias would absorb real angular motion. A causal mean-specific-force gravity direction uses only samples t<5s and is scored against up at5s. This provides tilt information only. A navigation yaw rotation leaves R.T*up unchanged (regression proves this symmetry); no heading estimate or covariance is claimed. The oracle bias-removed column is a diagnostic, never available to an online initializer.

| Seed | Max true rate deg/s | Noiseless tilt error deg | Measured tilt error deg | Oracle debiased tilt error deg |
|---:|---:|---:|---:|---:|
|900|0.520781|0.331113|0.343396|0.334511|
|115001|0.711146|0.075311|0.297310|0.068901|
|115002|0.793027|0.252662|0.286247|0.249529|

The 4.99s IMU sample spans the acceleration onset at5s; therefore the noiseless tilt error combines attitude averaging and that boundary acceleration step. It is not attributed solely to attitude motion. Earlier startup feasibility work (manifest scheme_audit_causal_zupt_alignment_feasibility) measured GNSS stop uncertainty and combined tilt error; this new probe separates noiseless motion and oracle bias-removed effects and tests heading/bias ambiguity.

This is an evidenced scenario limitation, not a simulator bug: stopped vehicles may pitch. The new diagnostic intentionally does not initialize or rerun the filter with truth-assisted yaw. Production truth-relative initialization at pipeline.py:119-132 remains open. No newly confirmed production bug was found, so no filter fix or noise tweak was justified. Prior confirmed bugs and original file/line evidence are preserved in /workspace/ccgins-v08/results/cc_scheme_audit/report.md and docs/scheme_recovery_checkpoints_audit.md.

## End-to-end audit disposition

- Truth/IMU: sim.py:152-168 uses ENU/FLU, body-to-nav rotation matrices, no quaternions. Accel is inverted using old attitude. filters.py:33-45 updates position/velocity with old attitude then right-multiplies rotation. Default constant gravity [0,0,-9.81] and absent Earth rotation agree. Optional fixed Earth rate/Coriolis is local, without geodetic transport; exact inverse simulation is optimistic for physical sensor validation.
- GNSS: sim.py:191-212 and filters.py:122-124 use positive p+R*lever in the same body frame. Multipath is correlated and continues through missing epochs. GNSS covariance does not model temporal correlation. pipeline.py:417-420 consumes the matching valid epoch after propagation. Antenna residuals and IMU-origin scoring are distinct and aligned.
- InEKF: filters.py:233-255 carries additive biases and invariant Jacobians; GNSS/body-velocity Jacobians are282-291; covariance reset is114-118,267-280 and initial transform303-319. First-order Phi/Q at97-101 and nominal-speed vibration55-66 remain approximations with prior experimental evidence, not newly proven dominant bugs. Maneuvers are necessary to separate tilt, bias and heading; stationary gravity alone cannot do so.
- Constraints/outages: pipeline.py:315-317 applies lateral/vertical NHC while sim.py:154-157 permits slip. Default ZUPT/odometry are off. Gaps are [60,90) and [120,180); IMU/NHC continue, missing GNSS does not update. A180s fix is terminal recovery, reported separately from last in-gap179.9s. Logging is same-epoch posterior at pipeline.py:528-540. No online smoothing exists; any future RTS result must be offline and separately scored.

Provisional engineering expectations: available H roughly0.7-1.5m, V roughly0.2-1.5m; 30s NHC-aided H a few metres, 60s metres to tens of metres. Prior ensembles exceeded narrow ranges, so these are not acceptance/tail bounds. A60s outage at20m/s spans1.2km and is a reasonable stress test, with no evidence supporting sub-metre online performance for this IMU grade. Actual speeds are tabulated above. Scenario realism (synthetic alignment, exact inverse IMU, no geodetic transport) must be separated from code correctness.

## Residual plan and honest stop

1. Specify a causal multi-stage initializer: gravity tilt with angular-motion handling, GNSS-aided heading only after sufficient baseline/excitation, joint bias uncertainty, and explicit unobservable refusal. Do not assign mean gyro to bias during this moving-attitude stop.
2. Validate independent start/alignment draws, low motion and turns before integration; quantify start-time latency and preserve t_start fairness.
3. Jointly evaluate colored GNSS, speed-dependent IMU noise and NHC slip with frozen splits; establish per-segment coverage/tail targets with larger fresh ensembles.
4. Optional manifold RTS remains separate post-processing, accounting for the endpoint fix.

No external blocker. This bounded round stops at the diagnostic evidence limit; whole-scheme validation is incomplete. DETECTION remains FAIL, confirmed_v08=false. No V0.8 claim.
