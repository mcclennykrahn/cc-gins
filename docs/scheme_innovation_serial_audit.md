# Whole-scheme audit continuation: GNSS innovation serial moments

Candidate `scheme_innovation_serial`; parent e012bf7. New read-only diagnostic, no production estimator/config change and no accuracy improvement claimed. Honest mid-audit: DETECTION FAIL; confirmed_v08=false. The whole-scheme residual plan remains open.

## State and error budget first

Read pending_handoff.md and project review.json before coding. Parent reviewed/pushed, no unfinished tracked work; excluded .dsh-prompts remains untouched. Closed negative trials were not reopened. Stage3 and prior audit evidence preserved.

Fresh paired baseline and replay use configs/example_sim.yaml, duration180s, seed900 and held-out99001/99002, t_start30s, default init_seed0. Before/after entire budget JSON and signed position histories are byte-identical (validation.json). Metrics score IMU origin against same-epoch truth, not antenna; GNSS-only residuals subtract true antenna p+Rlever. Outages are half-open60–90/120–180s; the180s corrected sample is available and separately reported. Before/after directories retain 10s bins, H/V RMSE, horizontal SSE fractions, peaks/endpoints and full signed histories.

Metres; every row is identical before and after.

|Seed|Segment|Before H|After H|Before V|After V|H SSE share|
|---:|---|---:|---:|---:|---:|---:|
|900|outage_60_90|1.488475|1.488475|0.676636|0.676636|0.899%|
|900|outage_120_180|11.016203|11.016203|0.640390|0.640390|98.462%|
|900|available|0.886598|0.886598|0.902407|0.902407|0.639%|
|900|overall|7.019110|7.019110|0.762560|0.762560|100.000%|
|900|available_30_60|0.989350|0.989350|1.185786|1.185786|0.397%|
|900|available_90_120|0.767795|0.767795|0.474157|0.474157|0.239%|
|900|terminal_available|1.386857|1.386857|0.378029|0.378029|0.003%|
|99001|outage_60_90|1.284531|1.284531|0.178708|0.178708|6.704%|
|99001|outage_120_180|3.162900|3.162900|0.409742|0.409742|81.296%|
|99001|available|1.214161|1.214161|0.394642|0.394642|12.000%|
|99001|overall|2.217872|2.217872|0.368582|0.368582|100.000%|
|99001|available_30_60|1.139420|1.139420|0.543045|0.543045|5.275%|
|99001|available_90_120|1.279376|1.279376|0.130573|0.130573|6.651%|
|99001|terminal_available|2.337296|2.337296|0.130676|0.130676|0.074%|
|99002|outage_60_90|1.590614|1.590614|0.278407|0.278407|7.949%|
|99002|outage_120_180|3.684514|3.684514|0.574164|0.574164|85.304%|
|99002|available|1.035330|1.035330|0.432858|0.432858|6.747%|
|99002|overall|2.522203|2.522203|0.471477|0.471477|100.000%|
|99002|available_30_60|1.141333|1.141333|0.346758|0.346758|4.093%|
|99002|available_90_120|0.918212|0.918212|0.504436|0.504436|2.649%|
|99002|terminal_available|0.700321|0.700321|0.444936|0.444936|0.005%|

Seed900 long gap contributes98.462% of horizontal squared error, confirming the original dominance hypothesis for that seed. Historical cfad161 overall7.173489m and long-gap peak28.257610m are reproduced in the original report; they are not this round's baseline. Available-segment H is of order1m; both fresh held-out seeds exceed1m. No universal <=1m target follows.

## Physics, GNSS and fairness

YAML confirms GNSS per-axis sigma_h1m/sigma_v2m, MP sigma.5m/tau30s. Expected stationary GNSS-only H=sqrt(2*(1+.25))=1.581m, V=sqrt(4+.25)=2.062m; zero-start MP and finite sampling change realized RMS.

|Seed|GNSS-only H|GNSS-only V|Pure INS30s H RMS/end|Pure INS60s H RMS/end|Actual mean gap speeds30/60s|
|---:|---:|---:|---|---|---|
|900|1.427352|2.124529|9.458/21.390|42.694/101.889|10.131/8.954m/s|
|99001|1.575756|2.095041|21.408/40.979|94.043/186.354|7.944/10.852m/s|
|99002|1.563831|2.174668|3.134/7.822|24.861/62.425|12.247/11.830m/s|

Pure INS diagnostic initializes at exact truth at each gap, then integrates raw unknown-bias IMU without aiding through the full endpoint before reacquisition. It is not the online filter. sim.py:179-180 resolves bias units:30deg/h=1.45444e-4rad/s;5mg=.04905m/s². ARW .3deg/sqrt(h) converts to .3*pi/180/60rad/sqrt(s), VRW .1/60m/s/sqrt(s) (sim.py:43-49). Additional default bias RW and speed-dependent vibration exist (sim.py:38-41,181-185).

One-axis flat straight-motion endpoint scales at30/60s: accel turn-on bias22.073/88.290m; gyro-bias gravity coupling6.421/51.365m; ARW.944/5.338m; VRW.158/.447m. These exclude residual bias estimation, initial errors, vibration and maneuvers and cannot be summed as realized online errors. Prior paired INS component ablations show cancellation.

A60s outage at20m/s spans1.2km, a reasonable MEMS stress scenario with NHC, but sub-metre online performance is unsupported without independent speed aiding or better bias calibration. vmax20 is a ceiling, not cruise speed (actual means above). Provisional planning scales remain available H roughly.7–1.5m/V.2–1.5m; NHC-aided30s H1–5m and60s H5–15m/V.2–2m. These are neither guaranteed targets nor statistical bounds: prior held-outs already exceed short-gap5m and available1.5m. Tail targets need a separate evaluation split and larger independent study. GNSS-only targets roughly1.6m H/2.1m V are sensor-model expectations.

## New measurement: pre-update innovations

scripts/audit_innovation_serial.py captures residual r=z-h and full S=HPH^T+R immediately before update_gnss; whitened u=chol(S)^-1r. The actual update executes unchanged once. Each trace verifies exact native-versus-instrumented t/p/v/P_pos equality. Capture retains rejected/forced updates, uses valid-GNSS time ordering and asserts fix count. Scope is a single unadapted vector-GNSS filter with monitor none. A unit regression uses correlated S and a state-changing fake update to verify pre-update capture and method restoration; another checks half-open windows, rejected fixes and no cross-gap pairs.

Eight independent trajectory/sensor seeds99101–99108 each use independent sampled truth-relative alignment seed=seed+300000. Each has paired nominal and no-MP runs retaining trajectory, IMU, white GNSS, initialization and outage validity. There are16 paired-model traces, not16 independent runs. Separate30–60 and90–120 intervals each contain30 fixes and29 adjacent1s pairs per run. No terminal fix or outage-crossing pair enters these summaries. No burn-in is removed from90s reacquisition, so that interval includes recovery. All diagnostics include the raw residual/S/u, update status, segment budgets and per-run moments in ensemble/innovations.json.

Uncentered second moment E[uu^T] has ideal I; lag E[u_k u_(k+1)^T] has ideal0, assuming a calibrated innovation model. Values below average per-run matrices across8 runs. Components are Cholesky-whitened coordinates in ENU ordering, not raw ENU metre errors. Serial dependence, estimated biases and recovery invalidate simple iid significance; there is no whiteness p-value, confidence bound or acceptance gate here.

|Model|Interval s|Diagonal second moment|Diagonal lag-one cross moment|Fixes/pairs|Rejected/forced|
|---|---|---|---|---:|---:|
|nominal|30_60|0.9772/1.1106/0.8530|0.0902/0.0274/-0.0459|240/232|0/0|
|nominal|90_120|0.9232/1.0918/1.0004|0.0042/0.0419/0.1674|240/232|0/0|
|no_multipath|30_60|0.8990/1.0819/0.8483|0.0272/0.0044/-0.0448|240/232|0/0|
|no_multipath|90_120|0.8853/1.0266/0.9563|-0.0295/-0.0172/0.1237|240/232|0/0|

Full means and3x3 matrices are in serial_summary.json. Nominal marginal second moments are near1 while the preceding independent checkpoint audit showed overconfident state covariance. Marginal innovation power alone is insufficient to establish state consistency. Removing MP reduces horizontal diagonal lag moments in these summaries, but finite-sample lag remains nonzero without MP (including vertical.1237 in90–120s). These measurements do not prove a universal correlation sign, isolate every bias/slip effect, or justify scalar R inflation. No new production bug is established and no filter change is promoted.

## End-to-end correctness re-examination and bug ledger

Original report ../report.md plus subsequent covariance-reset/discretization/observability reports remain the detailed historical bug ledger. Current anchors checked this round:

- sim.py:150-169 and filters.py:33-45 use body-to-nav ENU/FLU matrices, old-attitude specific-force integration, second-order position and then attitude increment. geometry.py:48-56 defines ZYX Euler. No quaternion convention applies to this path. sim.py:15 fixes gravity at[0,0,-9.81]. Synthetic truth/default filter omit Earth rotation together; optional fixed-tangent Earth/Coriolis is not full geodetic transport. Exact inversion of the filter integrator is an optimistic scenario assumption, not an independent physical validation.
- sim.py:190-210 validates GNSS rate, rounds to grid, evolves MP through outages with actual period, and adds the same p+Rlever sign/frame used at filters.py:122-124. pipeline.py:251-259,416-419 propagates to matching GNSS index and skips invalid fixes. No timing/lever scoring defect found. pipeline.py:179 uses white-only R: colored MP mismatch is evidenced, not a new sign/unit bug.
- filters.py:93-118 uses first-order Phi/Qd, Joseph update and finite-injection covariance reset. InEKF invariant gravity/bias/noise Jacobians at233-255 and additive biases at257-265 agree with existing finite-difference tests; GNSS at282-286 and body-velocity attitude cancellation at288-291 follow that convention. Nominal12m/s process noise at55-66 remains approximate; earlier oracle/prior-state schedules were unpromoted because they did not consistently improve held-outs. Observability depends on maneuver history; NHC cannot measure forward speed independently.
- pipeline.py:119-133 initializes relative to truth with sampled alignment and zero bias means. This is an identified scenario limitation; causal alignment is still missing. pipeline.py:265-294,315-325 implements optional ZUPT/default NHC, while sim.py:152-156 deliberately has slip/vertical motion. ZUPT is off and no odometer is supplied in this YAML. Motion constraints are approximate; no tiny sigma tweak made.
- Outages preserve propagation/NHC; terminal180s GNSS correction is excluded from the120–180s outage. No online smoother or RTS metric exists. A manifold RTS implementation would require cross-covariance logging and a separately labelled offline comparison; one terminal fix is little future aiding and does not validate smoothed outage targets.

Previously fixed bugs, with historical/current source locations: noise density incorrectly combined summed sample standard deviations and assumed dt=.01 (original filters.py:55-60, corrected55-66); outage metrics ignored t_start/collided repeated durations/empty intervals (original metrics.py:30-36, corrected30-42); GNSS MP used requested rather than rounded sampling period (original sim.py:191-200, corrected198-203), followed by requested-rate validation at190-194; diagnostic consistency hard-coded outage windows (parent9ac3be8 scripts/audit_calibration.py:19-26, now shared masks at15-25). They are closed evidence, not new fixes in this candidate. Historical review artifacts retain corresponding before/after and regression. No additional correctness defect found in this bounded continuation.

## Residual plan and stop

1. Joint colored-GNSS/slip/residual-bias calibration: compare a specified observable nuisance-state model with scalar R inflation using prior checkpoint evidence and these innovations; freeze training/evaluation splits before tuning. These8 runs do not establish a safe state parameterization.
2. Specify and implement causal initial alignment, including low-excitation failure behavior. Sampled truth-relative alignment is not a deployment solution.
3. Complete pre-gap/in-gap Q factorial and independent trajectory/sensor/alignment tail study. No new segment gate until uncertainty and tails are validated.
4. Any structural promotion requires same-config baseline/held-out segment budgets, independent review and regression. DETECTION fix remains separate and open; no V0.8 claim.

This is a structured evidence-limited mid-audit, not whole-scheme completion. No external blocker. New code supplies measurement instrumentation only. Independent review and full regression are recorded beside this report; commit provenance is in the delivery/manifest.

Reproduction: PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python scripts/audit_scheme.py --seeds 900 99001 99002 --out <dir> (pass the three seeds as separate arguments); scripts/audit_innovation_serial.py --out <ensemble-dir> uses the eight default seeds; python -m pytest -q runs regression. Before and after use identical production code. Stage root: /workspace/ccgins-v08/results/cc_scheme_audit/innovation_serial/.

Full regression:231 passed in85.11s (0:01:25), pytest.txt. Targeted instrumentation regressions:2 passed. No production positioning change.
