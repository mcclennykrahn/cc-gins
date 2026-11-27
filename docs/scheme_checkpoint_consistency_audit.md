# Online checkpoint consistency and whole-scheme audit continuation

Candidate `scheme_checkpoint_consistency`; parent `9ac3be8`. New diagnostic bug fix and online covariance evidence. No production filter/config change or positioning improvement claimed. Structured mid-audit delivery: DETECTION=FAIL, confirmed_v08=false.

## State and baseline first

Handoff and project review manifest read before editing. Parent reviewed and pushed, no incomplete tracked changes; only excluded `.dsh-prompts/` dirt. Closed trials remain closed. Stage3/v0.8 evidence preserved. Fresh baseline uses exactly configs/example_sim.yaml, seed900 plus held-out98001/98002, duration180s, t_start30s, default init_seed0. After is a replay after diagnostic edits with identical navigation algorithm, not a new estimator. Both budget JSON and all signed position CSVs are exactly equal (`validation.json`).

Same-epoch IMU-origin scoring; GNSS-only scoring subtracts the true antenna p+Rlever. Half-open outages60–90 and120–180, with the terminal180s reacquisition reported separately. 10s bins, horizontal peaks/endpoints and vertical errors are retained in before/ and after/. Historical overall7.173m/max long-gap~28m belongs to cfad161 (see ../report.md), not this new baseline.

Metres; before and after are identical in every row.

|Seed|Segment|Before H|After H|Before V|After V|H SSE share|
|---:|---|---:|---:|---:|---:|---:|
|900|outage_60_90|1.488475|1.488475|0.676636|0.676636|0.899%|
|900|outage_120_180|11.016203|11.016203|0.640390|0.640390|98.462%|
|900|available|0.886598|0.886598|0.902407|0.902407|0.639%|
|900|overall|7.019110|7.019110|0.762560|0.762560|100.000%|
|900|available_30_60|0.989350|0.989350|1.185786|1.185786|0.397%|
|900|available_90_120|0.767795|0.767795|0.474157|0.474157|0.239%|
|900|terminal_available|1.386857|1.386857|0.378029|0.378029|0.003%|
|98001|outage_60_90|1.688509|1.688509|0.173213|0.173213|17.382%|
|98001|outage_120_180|2.384215|2.384215|0.448349|0.448349|69.311%|
|98001|available|1.043835|1.043835|0.241578|0.241578|13.308%|
|98001|overall|1.810630|1.810630|0.331236|0.331236|100.000%|
|98001|available_30_60|1.220392|1.220392|0.176883|0.176883|9.080%|
|98001|available_90_120|0.818687|0.818687|0.291167|0.291167|4.086%|
|98001|terminal_available|2.638716|2.638716|0.504657|0.504657|0.141%|
|98002|outage_60_90|6.273171|6.273171|1.629334|1.629334|64.867%|
|98002|outage_120_180|3.088785|3.088785|0.212126|0.212126|31.453%|
|98002|available|1.055667|1.055667|0.838331|0.838331|3.680%|
|98002|overall|3.482126|3.482126|0.911032|0.911032|100.000%|
|98002|available_30_60|0.837115|0.837115|1.067373|1.067373|1.155%|
|98002|available_90_120|1.236970|1.236970|0.518274|0.518274|2.522%|
|98002|terminal_available|0.719160|0.719160|0.120365|0.120365|0.003%|

The60s gap contributes98.462% of seed900 horizontal SSE, confirming the stated dominance hypothesis for this seed. Held-out98002 instead has30s H RMS6.273m versus60s3.089m; dominance is not universal. Available H .887/1.044/1.056m is of order1m, not a universal <=1m guarantee.

## Bias units, physical budget and scenario fairness

Code resolves gyro_bias_sigma30 as deg/h and accel_bias_sigma5 as mg: `ccgins/sim.py:179-180` gives1.45444e-4rad/s and.04905m/s². `sim.py:43-49` converts .3deg/sqrt(h) ARW and.1m/s/sqrt(h) VRW by dividing by60 (plus radians for gyro). Defaults also include gyro/accel bias RW and speed-dependent vibration (`sim.py:38-41,181-185`), so YAML ARW/VRW alone understate the scenario noise.

Pure INS below starts at exact truth at each gap, integrates raw IMU with unknown biases and no aiding, and scores the full endpoint before any reacquisition. It is a diagnostic, not the online filter.

|Seed|Gap s|Mean speed m/s|Pure INS H RMS|H endpoint|V endpoint|
|---:|---:|---:|---:|---:|---:|
|900|30|10.131|9.458|21.390|37.583|
|900|60|8.954|42.694|101.889|160.128|
|98001|30|11.935|18.128|40.465|35.354|
|98001|60|0.952|80.932|179.215|145.264|
|98002|30|16.364|16.026|34.764|6.833|
|98002|60|15.520|49.222|103.941|55.063|

One-horizontal-axis straight-line endpoint scales at30/60s: accel turn-on bias bT²/2=22.073/88.290m; gyro-bias gravity coupling gbT³/6=6.421/51.365m; ARW gqT^2.5/sqrt20=.944/5.338m; VRW qT^1.5/sqrt3=.158/.447m. They exclude vibration, maneuvers, RW and estimation and are not additive realized errors. Prior component ablations (../ins_components/report.md) show cancellation; raw INS does not identify residual online error causes.

GNSS sigma_h1m is per axis, sigma_v2m, MP sigma.5m/tau30s. Stationary GNSS-only radial H expectation sqrt(2*(1²+.5²))=1.581m and V sqrt(2²+.5²)=2.062m; zero-start MP and finite traces vary.

|Seed|GNSS-only H RMS|GNSS-only V RMS|
|---:|---:|---:|
|900|1.427352|2.124529|
|98001|1.580566|1.923610|
|98002|1.568356|1.956789|

A60s gap at20m/s spans1.2km: reasonable MEMS stress test; unsupported as a universal sub-metre online test without independent forward-speed aiding or stronger calibration. Actual seed900 long-gap speed8.954m/s and98001 .952m/s show vmax20 is a ceiling rather than a prescribed cruise speed.

Provisional segment planning scales: available H roughly.7–1.5m/V.2–1.5m, NHC-aided30s H1–5m and60s H5–15m/V.2–2m. They are engineering ranges, not bounds, gates, or confidence intervals; this round's98002 short gap already exceeds5m and98001 long gap is below5m. No validated tail target is claimed.

## New bug fixed

At parent9ac3be8, `scripts/audit_calibration.py:19-26` accepted an outage list but hard-coded four example segment masks, ignored configurable scoring start, and omitted consistency metrics for overall/available/terminal segments. Changed schedules could raise KeyError or omit/misassign consistency rows. This was a diagnostic correctness bug; it did not alter production navigation or historical example schedule results. Current `scripts/audit_scheme.py:11-45` shares segment masks with `scripts/audit_calibration.py:15-25`, which now accepts t_start and computes consistency only for nonempty matching masks. Regression changes the schedule to40–60s and scoring start20s and checks exact NEES membership. Existing example test still passes. New independent-run aggregation test checks correlated horizontal covariance, count, second moments and Gaussian reference degrees of freedom.

## New joint online uncertainty evidence

Twelve fresh independent trajectory/sensor seeds98101–98112 each use a different alignment seed=seed+200000. Each is run with nominal MP and paired mp_sigma0; IMU, trajectory, white GNSS draws, validity and initial state remain paired. Alignment remains sampled relative to truth, not a causal estimator. `scripts/audit_checkpoint_consistency.py` retains all run/segment metrics and per-checkpoint ENU error/full navigation position covariance in ensemble/checkpoints.json.

At each fixed epoch, horizontal NEES is e_h^T P_h^-1 e_h /2; vertical is e_u²/P_uu. Averaging is across12 runs, never correlated time samples. Paired models are dependent, and the five epochs reuse the same runs: do not treat them as24 independent runs or60 independent observations. Under an ideal zero-mean Gaussian calibrated model the mean horizontal NEES/dim reference95% interval is [.517,1.640]; this is a diagnostic reference whose assumptions are being tested, not proof of consistency. Small empirical coverage counts do not establish tail gates; no significance across repeated epochs is claimed.

|Epoch s|Model|H RMS m|V RMS m|H NEES/dim|V NEES|H 95% coverage count|V 95% coverage count|
|---:|---|---:|---:|---:|---:|---:|---:|
|30.0|nominal|0.981|0.553|2.345|1.294|7/12|11/12|
|30.0|no_multipath|0.716|0.355|1.302|0.640|11/12|11/12|
|59.9|nominal|0.963|0.707|2.100|2.467|8/12|8/12|
|59.9|no_multipath|0.923|0.449|1.776|1.129|10/12|11/12|
|89.9|nominal|5.117|0.601|1.444|1.077|10/12|12/12|
|89.9|no_multipath|4.119|0.395|0.667|0.425|11/12|12/12|
|119.9|nominal|1.031|0.609|3.026|3.027|7/12|10/12|
|119.9|no_multipath|0.576|0.445|0.787|1.564|12/12|10/12|
|179.9|nominal|7.243|0.749|1.058|1.127|11/12|11/12|
|179.9|no_multipath|5.377|0.651|0.553|0.723|12/12|12/12|

Nominal horizontal NEES/dim exceeds the ideal reference upper value at30/59.9/119.9s; removing MP reduces it at all five checkpoints but leaves59.9s at1.776. Nominal179.9s H NEES/dim1.058 lies inside that reference despite7.243m H RMS. Accuracy and uncertainty calibration are different questions. The paired MP ablation supplies causal evidence that generated MP affects this finite ensemble's online errors/covariance ratios, not only measurement-mean uncertainty. It does not prove MP dominates all outages, validate a new state model, or isolate remaining NHC/slip/bias/process-noise effects. All point summaries and second moments are retained to prevent selecting only favorable checkpoints. No filter promotion follows from12 runs.

## End-to-end code re-examination

No new production equation bug established. Historical bugs/fixes and original source references remain in ../report.md; follow-up covariance-reset/discretization/observability evidence supersedes that report's old open items. Current inspected anchors:

- Truth/IMU: `sim.py:150-169` and `filters.py:33-45` use body-to-nav ENU/FLU rotation matrices, old-R acceleration, second-order position and then right rotation increment. `geometry.py:48-56` defines ZYX Euler. No quaternion in this path. Gravity fixed[0,0,-9.81] (`sim.py:15`), default truth/filter omit Earth rotation; optional tangent Earth/Coriolis is not full geodetic transport. Inverting the same integrator is excellent closure but optimistic validation of numerical realism.
- GNSS: `sim.py:190-210` validates rate, rounds to the IMU grid, evolves zero-start AR(1) MP through outages at actual epoch spacing and uses p+Rlever. `filters.py:122-124,282-286` uses the same antenna sign/frame. `pipeline.py:179` supplies white-only nominal R; colored MP is a demonstrated model mismatch, not a newly found sign/unit bug. `pipeline.py:251-259,416-419` propagates to matching GNSS index and skips invalid fixes. Scoring remains IMU origin at the same epoch.
- InEKF: `filters.py:93-118` uses first-order Phi and Qd, Joseph covariance and finite-correction reset. `filters.py:233-255` has invariant gravity/bias/noise Jacobians; body biases are additive (`257-265`), GNSS H invariant (`282-286`), body-velocity attitude cancellation (`288-291`) consistent with its error convention. First-order discretization and nominal-speed noise are approximations, not a new proven defect. Maneuvers determine bias observability; raw bias magnitude is not posterior uncertainty.
- Initialization: `pipeline.py:119-133` uses sampled truth-relative position/velocity/attitude and zero bias means, with larger initial covariance. Varying seeds improves coverage of that synthetic assumption; deployable causal alignment remains open.
- Constraints/outages: `pipeline.py:265-294,315-325` applies optional ZUPT and default NHC; ZUPT off here. Slip/vertical motion (`sim.py:152-156`) makes NHC approximate. No forward-speed measurement in this config; no new odometer. Propagation/NHC continue through outages. Half-open terminal fix cannot be counted as long-gap accuracy. No RTS implemented or reported; any future manifold RTS must be offline and separate, with little future aiding in the terminal gap.

## Residual plan and honest stop

1. Design an observable colored-GNSS nuisance model or correlation-aware update; compare it against scalar R inflation without claiming independent-fix equivalence. First check innovation histories and joint slip/bias effects; the12-run evidence does not establish a safe parameterization.
2. Implement separately specified causal initial alignment and test low-excitation failures; do not use truth-relative seed variation as a substitute.
3. Complete pre-gap/in-gap Q timing factorial and independent joint uncertainty/tail validation before setting acceptance bounds. Avoid tuning on these held-out seeds; use a new evaluation split.
4. Only promote structural improvements after paired baseline/held-out segment budgets, independent review and regression. Optional RTS remains a separate offline benchmark.

This round closes a diagnostic correctness defect and advances measurement-only uncertainty evidence to online independent-run checkpoints. Whole-scheme calibration and causal alignment remain incomplete at this bounded evidence stop; no external blocker. DETECTION remains FAIL, confirmed_v08=false. Review/regression and commit provenance are recorded alongside this report.

## Reproduction and validation

Use PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 with /tmp/ccgins-accuracy-venv/bin/python. Run `scripts/audit_scheme.py --seeds 900 98001 98002 --out <directory>` for before/ then after/. Run scripts/audit_checkpoint_consistency.py --out ensemble/ (default seeds98101–98112). Run python -m pytest -q. Evidence root is /workspace/ccgins-v08/results/cc_scheme_audit/checkpoint_consistency/; independent review /workspace/ccgins-v08/reviews/checkpoint_consistency_independent.md.

Full regression:229 passed in100.65s (0:01:40), `pytest.txt`; exact before/after JSON and signed histories: `validation.json`. Independent code/numeric/final report verdict is recorded in the review artifact.
