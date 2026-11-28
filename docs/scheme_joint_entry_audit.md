# Joint entry attitude/bias audit completion

Candidate: scheme_joint_entry_completion. Base 274330c. MID_AUDIT; new_work=true; DETECTION=FAIL; confirmed_v08=false.

Read pending_handoff and current project review.json before coding. HEAD equals origin/main. Prior deliveries and negative trials are closed. Resumed existing uncommitted simulator fix and joint factorial; preserved all prior evidence and excluded .dsh-prompts. No other repo touched.

## Error budget first

Fresh baseline from detached HEAD, then candidate on identical configs/example_sim.yaml and seeds 900/113001/113002; duration 180 s, t_start 30 s. Each value below is before=after: budget.json and all signed error CSVs byte-identical (paired_check.json). Same-time posterior at IMU origin, 10 Hz logs; half-open outages [60,90), [120,180); terminal 180 s GNSS reacquisition separately recorded. Ten-second budgets are in budget.json.

|Seed|Segment|H RMSE m|V RMSE m|H SSE share %|
|---:|---|---:|---:|---:|
|900|outage_60_90|1.488475|0.676636|0.8988|
|900|outage_120_180|11.016203|0.640390|98.4624|
|900|available|0.886598|0.902407|0.6388|
|900|overall|7.019110|0.762560|100.0000|
|900|available_30_60|0.989350|1.185786|0.3971|
|900|available_90_120|0.767795|0.474157|0.2391|
|900|terminal_available|1.386857|0.378029|0.0026|
|113001|outage_60_90|2.376185|0.350110|21.0272|
|113001|outage_120_180|3.189468|2.212633|75.7684|
|113001|available|0.655361|0.596514|3.2043|
|113001|overall|2.316644|1.457384|100.0000|
|113001|available_30_60|0.616826|0.452172|1.4169|
|113001|available_90_120|0.680146|0.696001|1.7228|
|113001|terminal_available|2.281709|2.681391|0.0646|
|113002|outage_60_90|3.033541|0.925693|38.3193|
|113002|outage_120_180|2.458898|0.298384|50.3534|
|113002|available|1.165270|0.529034|11.3272|
|113002|overall|2.190843|0.564729|100.0000|
|113002|available_30_60|1.219070|0.649760|6.1884|
|113002|available_90_120|1.109622|0.371386|5.1271|
|113002|terminal_available|0.923486|0.413723|0.0118|

Seed 900 long outage dominates 98.4624% of horizontal SSE. Current overall 7.019110 m, long-gap peak 27.684186 m; historical 7.173 m/28 m describe an earlier revision. Held-out 113002 available H RMSE exceeds 1 m, so <=1 m is not universal.

|Seed|GNSS-only H/V RMS m|Raw INS 30s H RMS/end m|Raw INS 60s H RMS/end m|Mean gap speed 30/60s m/s|
|---:|---|---|---|---|
|900|1.427/2.125|9.458/21.390|42.694/101.889|10.131/8.954|
|113001|1.579/2.181|6.160/14.238|32.599/82.331|13.151/15.137|
|113002|1.593/1.902|22.984/52.019|92.652/214.242|7.722/2.306|

Raw INS starts at exact truth R/v/p per gap, propagates all noisy IMU steps without bias subtraction/aiding, includes endpoint before GNSS; it is a physical diagnostic, not online accuracy. GNSS-only scores antenna p+Rlever and includes multipath.

## Confirmed bug and fix

ccgins/sim.py:121 (parent and candidate): rng.uniform(4,vmax) rejects valid low positive vmax such as 1 m/s after initial stop. Replaced lower bound by min(4,vmax). This enables slow-motion scenarios; default vmax20 sampling is unchanged. tests/test_cli.py:21-27 covers 1 and 4 m/s through _build_inputs, including the ceiling. No accuracy improvement claimed.

Pending diagnostic arithmetic had already been corrected before this resumed round: scripts/audit_joint_entry.py:31 converts arrays to float before squaring, preventing int16 overflow; tests/test_joint_entry.py covers it. This is not counted as a new navigation bug. The new biases replay arm allows all four cells of the attitude/bias factorial.

## Joint factorial scope

Frozen scenario/IMU/GNSS and init_seed0. Per gap, fresh runs replace attitude, both biases, or both groups once before the first in-gap IMU step. Native run is the fourth cell. All retain native covariance numerically without recentering or conditioning: this is privileged conditional sensitivity, not a consistent oracle posterior or deployable filter. Prefix equality is asserted. Cells/gaps share noise and are dependent; three seeds do not validate tails.

SSE contrast = SSE_AB - SSE_A - SSE_B + SSE_native. Even additive vector changes produce nonzero contrast through cross products. Vector interaction RMS of e_AB-e_A-e_B+e_native is reported separately. Neither statistic is an additive causal error fraction. No covariance/process noise/NHC tuning is justified by this experiment alone.


|Seed|Gap start s|Native/A/B/AB H RMS m|H MSE contrast m²|H vector interaction RMS m|
|---:|---:|---|---:|---:|
|900|60|1.488475 / 1.397379 / 1.034185 / 0.966454|0.127383|0.001244|
|900|120|11.016203 / 12.708062 / 2.960657 / 3.974061|-33.110428|0.044087|
|113001|60|2.376185 / 2.192261 / 2.046488 / 2.045401|0.835797|0.001745|
|113001|120|3.189468 / 1.908803 / 2.477375 / 1.399279|2.349768|0.002808|
|113002|60|3.033541 / 4.997158 / 2.087218 / 1.167184|-18.763371|0.006001|
|113002|120|2.458898 / 3.975357 / 2.141187 / 3.641198|-1.083641|0.005464|

A=truth attitude substitution; B=truth gyro and accel bias substitution. Seed 900 long-gap large negative SSE contrast coexists with only 0.044087 m vector interaction; squared-error cancellation alone must not be called strong dynamic coupling. Full vertical contrasts and signed traces are in factorial/.

## Physics, targets and end-to-end audit ledger


Configuration: GNSS sigma_h1m per horizontal axis, sigma_v2m, mp_sigma0.5m, tau30s; asymptotic radial H sqrt(2*1.25)=1.581m, V sqrt(4.25)=2.062m. Multipath starts at zero; finite-run values differ. Configured vmax20m/s is a ceiling, not actual constant speed. At20m/s a60s gap spans1.2km: reasonable stress test, unsupported sub-metre online requirement. Available H roughly0.7–1.5m is a planning goal; nominal30s gaps aim for a few metres but prior17.142m RMS counterexample precludes a tight guarantee.60s errors can be metres to tens of metres (prior45.892m RMS). No validated tail bound.

Units resolved in ccgins/sim.py:48-51,181-188: gyro bias30deg/h=1.45444e-4rad/s; accel bias5mg=0.04905m/s²; ARW0.3deg/sqrt(h)=8.72665e-5rad/sqrt(s); VRW0.1m/s/sqrt(h)=0.00166667m/s/sqrt(s). Straight, flat single-horizontal-axis unaided endpoint scales30/60s: accel bias22.073/88.290m; gyro-bias gravity coupling6.421/51.365m; ARW gravity coupling0.944/5.338m; VRW0.158/0.447m. Not additive online RMSE contributions; maneuvers, calibration, bias RW and vibration matter.

The low-speed simulator bug is fixed above. No filter accuracy fix or speculative tuning promoted. Current audit findings and unresolved model limitations:

- Truth/IMU: sim.py:152-168 matches filters.py:33-45: body FLU→local ENU rotation matrices, relative SO(3) increments, old-attitude specific force, second-order position. No quaternion convention. Constant gravity[0,0,-9.81]; default omits Earth rotation consistently. Optional fixed-local Earth model filters.py:21-44 neglects transport; real-navigation scope limitation.
- Noise synthesis sim.py:181-188 adds base/vibration sample standard deviations; filters.py:55-67 folds vibration at a fixed nominal speed. This mismatch across speed is a calibration limitation, not evidence another scalar tuning solves drift.
- GNSS sim.py:198-215 and filters.py:122-124 share positive body lever p+Rlever; correlated multipath uses rounded actual epoch period. Nominal measurement covariance does not model temporal correlation. Pipeline propagates before consuming same-index fix (pipeline.py:256-260,416-420). Scoring uses the correct IMU origin and GNSS-only antenna origin.
- InEKF filters.py:233-255 includes bias couplings/noise;282-291 has invariant GNSS/body-velocity Jacobians. NHC attitude cancellation is intentional. First-order Phi/Qd filters.py:94-103 remains approximation; no new proof of dominant discretization bug. Joseph update and finite covariance reset110-118,257-280 retain prior fixes. Bias observability needs excitation and calibrated uncertainty, not merely low position RMSE.
- Constraints pipeline.py:315-317 impose zero lateral/vertical vehicle velocity; truth sim.py:154-157 includes small slip. ZUPT/odometry disabled in this config; previously fixed trusted-GNSS stop evidence retained. No evidence warrants a tiny NHC-sigma trial.
- Initialization pipeline.py:119-132 uses perturbed truth R/v/p, zero biases and prior covariance. filters.py:303-319 transforms nav covariance into invariant coordinates. Prior crossed initialization and current entry interventions quantify sensitivity within that optimistic model; causal sensor-only initialization remains unvalidated.
- Missing GNSS leaves IMU/NHC running. Posterior log pipeline.py:528-540 is same epoch; terminal reacquisition separate. No RTS smoother exists or is claimed. Any optional smoother must be scored separately as offline, explicitly exposing terminal future fix.

Prior fixed bugs and reviews remain in archived reports, including docs/scheme_entry_orientation_split_audit.md and the linked scheme reports. They are not reopened or counted as new fixes here.


## Validation and residual plan

Fresh full regression: 319 passed in 77.19s. Independent review evidence is recorded beside this report and under reviews/joint_entry_completion.md. Reproduce with PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python -m scripts.audit_scheme --seeds 900 113001 113002 --out <budget>; -m scripts.audit_joint_entry --out <factorial>; -m pytest -q.

Whole-scheme audit remains incomplete. Next: sensor-only initialization with uncertainty/refusal across independent starts; joint entry/bias covariance treatment; colored GNSS, speed-dependent IMU and NHC slip calibration on frozen splits; predeclared independent segment-tail validation. No external blocker. Evidence does not support validated per-segment tail guarantees or a V0.8 claim. No optional RTS implemented; any future smoother must be reported separately from online filtering and disclose terminal future fix use.
