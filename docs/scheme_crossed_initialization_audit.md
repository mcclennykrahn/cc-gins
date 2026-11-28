# Crossed initialization sensitivity: bounded whole-scheme audit continuation

Candidate scheme_crossed_initialization; base e5c69f0. MID_AUDIT; new_work=true. No production changes, no online improvement claimed. DETECTION=FAIL; confirmed_v08=false.

Read pending_handoff and project review.json before coding. HEAD matched origin/main; only .dsh-prompts/ untracked. Latest deliveries and recorded negative trials are closed. No incomplete patch to resume. Prior evidence preserved.

## Error budget FIRST

Fresh pre-edit and post-edit runs: configs/example_sim.yaml, seed900 plus frozen held-out112001/112002; duration180s, scoring t_start30s, native init_seed0. Each table cell below is before=after; budgets and signed CSV histories exact (paired_check.json). Same-time posterior at IMU origin,10Hz logs; half-open gaps[60,90),[120,180),180s reacquisition separate. Ten-second error budgets and signed histories in before/ and after/.

|Seed|Segment|H RMSE m|V RMSE m|H SSE share %|
|---:|---|---:|---:|---:|
|900|available_30_60|0.989350|1.185786|0.3971|
|900|outage_60_90|1.488475|0.676636|0.8988|
|900|available_90_120|0.767795|0.474157|0.2391|
|900|outage_120_180|11.016203|0.640390|98.4624|
|900|terminal_available|1.386857|0.378029|0.0026|
|900|available|0.886598|0.902407|0.6388|
|900|overall|7.019110|0.762560|100.0000|
|112001|available_30_60|1.076354|0.614176|0.5616|
|112001|outage_60_90|1.060895|0.434585|0.5455|
|112001|available_90_120|0.716282|0.240180|0.2487|
|112001|outage_120_180|10.087064|0.455171|98.6387|
|112001|terminal_available|1.847435|0.407096|0.0055|
|112001|available|0.916563|0.466222|0.8158|
|112001|overall|6.421351|0.455628|100.0000|
|112002|available_30_60|0.682162|0.502906|1.8407|
|112002|outage_60_90|4.109478|0.139475|66.8001|
|112002|available_90_120|1.007563|0.962200|4.0156|
|112002|outage_120_180|1.858975|0.072301|27.3389|
|112002|terminal_available|0.597409|0.082775|0.0047|
|112002|available|0.860015|0.767074|5.8610|
|112002|overall|2.247855|0.491502|100.0000|

Seed900 long gap dominates98.4624% of H SSE; current overall7.019110m and gap peak27.684186m. Historical7.173m/28m are earlier-revision evidence, not invented current metrics. Available H is below1m on these three draws, not a universal bound (prior ensemble counterexamples remain).

|Seed|GNSS-only H/V RMSE m|Raw INS30s H RMSE/end m|Raw INS60s H RMSE/end m|Mean gap speed30/60s m/s|
|---:|---|---|---|---|
|900|1.427/2.125|9.458/21.390|42.694/101.889|10.131/8.954|
|112001|1.656/2.484|11.434/25.468|91.909/234.678|5.269/15.873|
|112002|1.518/1.961|4.183/9.309|31.601/72.312|18.359/10.522|

Raw INS resets to exact truth R/v/p at each gap, propagates every raw IMU step without bias subtraction/aiding and includes the endpoint before GNSS. It is a physical diagnostic, not an online baseline. GNSS-only scores antenna p+Rlever including multipath.

## New crossed experiment

Freeze trajectory, IMU and GNSS within each scenario; vary only cfg.init_seed across0/112101/112102. This contrasts initialization while retaining native covariance and ordinary filtering throughout; no privileged outage-entry substitutions. Truth-relative initial_state is still optimistic. Three scenario draws and three shared initialization perturbations form nine dependent cells, not nine independent validation runs. Ranges are conditional descriptive values; no confidence limits, variance attribution, tail gate or sensor-only alignment claim.

|Scenario seed|Init seed|Available H m|30s gap H m|60s gap H m|
|---:|---:|---:|---:|---:|
|900|0|0.886598|1.488475|11.016203|
|900|112101|0.873631|1.580675|11.136993|
|900|112102|0.874468|1.549312|11.096573|
|112001|0|0.916563|1.060895|10.087064|
|112001|112101|0.921267|1.042903|9.534790|
|112001|112102|0.919176|1.039992|10.236684|
|112002|0|0.860015|4.109478|1.858975|
|112002|112101|0.889281|2.926765|1.794860|
|112002|112102|0.899251|2.279049|2.169989|

Initialization changes seed112002 short-gap H RMSE4.109→2.279m; seed900 long-gap range11.016–11.137m and112001 long-gap9.535–10.237m. Initial perturbations can persist through the first30s scoring exclusion and subsequent aiding, but this small grid establishes neither general dominance nor an additive initialization fraction. It does not justify tuning initial covariance or constraints.

## Physics, realistic targets and audit ledger

Configuration: GNSS sigma_h1m per horizontal axis, sigma_v2m, mp_sigma0.5m, tau30s; asymptotic radial H sqrt(2*1.25)=1.581m, V sqrt(4.25)=2.062m. Multipath starts at zero; finite-run values differ. Configured vmax20m/s is a ceiling, not actual constant speed. At20m/s a60s gap spans1.2km: reasonable stress test, unsupported sub-metre online requirement. Available H roughly0.7–1.5m is a planning goal; nominal30s gaps aim for a few metres but prior17.142m RMS counterexample precludes a tight guarantee.60s errors can be metres to tens of metres (prior45.892m RMS). No validated tail bound.

Units resolved in ccgins/sim.py:48-51,181-188: gyro bias30deg/h=1.45444e-4rad/s; accel bias5mg=0.04905m/s²; ARW0.3deg/sqrt(h)=8.72665e-5rad/sqrt(s); VRW0.1m/s/sqrt(h)=0.00166667m/s/sqrt(s). Straight, flat single-horizontal-axis unaided endpoint scales30/60s: accel bias22.073/88.290m; gyro-bias gravity coupling6.421/51.365m; ARW gravity coupling0.944/5.338m; VRW0.158/0.447m. Not additive online RMSE contributions; maneuvers, calibration, bias RW and vibration matter.

No new production bug proved this round. No speculative fix promoted. Current audit findings and unresolved model limitations:

- Truth/IMU: sim.py:152-168 matches filters.py:33-45: body FLU→local ENU rotation matrices, relative SO(3) increments, old-attitude specific force, second-order position. No quaternion convention. Constant gravity[0,0,-9.81]; default omits Earth rotation consistently. Optional fixed-local Earth model filters.py:21-44 neglects transport; real-navigation scope limitation.
- Noise synthesis sim.py:181-188 adds base/vibration sample standard deviations; filters.py:55-67 folds vibration at a fixed nominal speed. This mismatch across speed is a calibration limitation, not evidence another scalar tuning solves drift.
- GNSS sim.py:198-215 and filters.py:122-124 share positive body lever p+Rlever; correlated multipath uses rounded actual epoch period. Nominal measurement covariance does not model temporal correlation. Pipeline propagates before consuming same-index fix (pipeline.py:256-260,416-420). Scoring uses the correct IMU origin and GNSS-only antenna origin.
- InEKF filters.py:233-255 includes bias couplings/noise;282-291 has invariant GNSS/body-velocity Jacobians. NHC attitude cancellation is intentional. First-order Phi/Qd filters.py:94-103 remains approximation; no new proof of dominant discretization bug. Joseph update and finite covariance reset110-118,257-280 retain prior fixes. Bias observability needs excitation and calibrated uncertainty, not merely low position RMSE.
- Constraints pipeline.py:315-317 impose zero lateral/vertical vehicle velocity; truth sim.py:154-157 includes small slip. ZUPT/odometry disabled in this config; previously fixed trusted-GNSS stop evidence retained. No evidence warrants a tiny NHC-sigma trial.
- Initialization pipeline.py:119-132 uses perturbed truth R/v/p, zero biases and prior covariance. filters.py:303-319 transforms nav covariance into invariant coordinates. New crossed experiment quantifies sensitivity within that optimistic model; causal sensor-only initialization remains unvalidated.
- Missing GNSS leaves IMU/NHC running. Posterior log pipeline.py:528-540 is same epoch; terminal reacquisition separate. No RTS smoother exists or is claimed. Any optional smoother must be scored separately as offline, explicitly exposing terminal future fix.

Prior fixed bugs and reviews remain in archived reports, including docs/scheme_entry_orientation_split_audit.md and the linked scheme reports. They are not reopened or counted as new fixes here.

## Reproduction, review and residual work

Use PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python:

```
-m scripts.audit_scheme --seeds 900 112001 112002 --out <before-or-after>
-m scripts.audit_crossed_initialization --out <crossed>
-m pytest -q
```

Full regression:307 passed in77.24s. Independent diagnostic review PASS;44 focused tests and independent recomputation of all nine cells passed. See /workspace/ccgins-v08/reviews/crossed_initialization.md and results/cc_scheme_audit/crossed_initialization/independent_check.json. Final report review recorded with delivery. Residual plan: validate causal sensor-only initialization with uncertainty/refusal; study joint entry state/bias interactions with explicit covariance handling; jointly calibrate colored GNSS, speed-dependent IMU uncertainty and NHC slip on frozen splits; predeclare independent segment-tail validation. This bounded continuation does not close the whole audit. No external blocker; available evidence limits completion. DETECTION remains FAIL and confirmed_v08=false.
