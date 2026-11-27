# Whole-scheme continuation: process-noise phase factorial

Candidate `scheme_noise_phase_factorial`, parent53cc0d8. New diagnostic code, no production change or accuracy promotion. Honest MID_AUDIT; DETECTION FAIL, confirmed_v08=false. Read pending handoff/current manifest first: parent reviewed and pushed, no unfinished tracked changes. Closed trials left closed; .dsh-prompts untouched. Stage3 and earlier evidence preserved.

## Error budget first

Fresh runs use configs/example_sim.yaml, duration180s, seed900 and held-out100001/100002, init_seed0, t_start30s. Same-epoch IMU-origin error; GNSS residual subtracts true p+Rlever at antenna. Outages half-open60–90/120–180s; terminal180s reacquisition separate. Before/after JSON and complete signed histories are byte-identical (validation.json). All rows below have identical before/after H and V; no navigation gain claimed. Ten-second histories/bins are in both budget.json files.

|Seed|Segment|Before/after H m|Before/after V m|H SSE share %|
|---:|---|---:|---:|---:|
|900|outage_60_90|1.488475|0.676636|0.899|
|900|outage_120_180|11.016203|0.640390|98.462|
|900|available|0.886598|0.902407|0.639|
|900|overall|7.019110|0.762560|100.000|
|900|available_30_60|0.989350|1.185786|0.397|
|900|available_90_120|0.767795|0.474157|0.239|
|900|terminal_available|1.386857|0.378029|0.003|
|100001|outage_60_90|3.627089|0.722026|34.085|
|100001|outage_120_180|3.415039|0.329566|60.433|
|100001|available|1.027727|0.209409|5.482|
|100001|overall|2.777445|0.406411|100.000|
|100001|available_30_60|0.924310|0.220680|2.214|
|100001|available_90_120|1.119203|0.195274|3.245|
|100001|terminal_available|1.643440|0.552953|0.023|
|100002|outage_60_90|3.824016|0.484162|14.801|
|100002|outage_120_180|6.431340|0.216721|83.730|
|100002|available|0.851031|0.380860|1.469|
|100002|overall|4.443703|0.351719|100.000|
|100002|available_30_60|0.791674|0.397353|0.634|
|100002|available_90_120|0.901585|0.364258|0.823|
|100002|terminal_available|1.842814|0.075633|0.011|

Seed900 long-gap H RMSE11.016203m contributes98.462% of H SSE, confirming long-gap dominance for this seed. Historical cfad161 overall7.173489m/long-gap peak28.257610m remains the reproduced original baseline in ../report.md; it is not this round's baseline. Current seed900 available H.886598m is on the order of1m, not a universal <=1m guarantee.

## Physical budget and scenario fairness

YAML fixes GNSS sigma_h1m per axis, sigma_v2m, mp_sigma.5m/tau30s. Stationary GNSS-only expectation H=sqrt(2*(1+.25))=1.581m, V=sqrt(4+.25)=2.062m. MP starts at zero; finite-run realized RMS differs.

|Seed|GNSS H/V m|Pure INS30s H RMS/end m|Pure INS60s H RMS/end m|Mean speed30s/60s m/s|
|---:|---|---|---|---|
|900|1.427/2.125|9.458/21.390|42.694/101.889|10.131/8.954|
|100001|1.536/1.985|26.373/58.699|92.071/206.211|9.889/8.414|
|100002|1.537/2.103|12.013/24.960|44.997/98.506|8.873/5.664|

Code resolves units: sim.py:179-180 converts30deg/h to1.45444e-4rad/s and5mg to.04905m/s². ARW.3deg/sqrt(h) becomes.3*pi/180/60rad/sqrt(s), VRW.1/60m/s/sqrt(s) (sim.py:43-49). Bias RW and speed-dependent white vibration also exist (38-41,181-185). One-axis flat-motion endpoint scales at30/60s: accel bias22.073/88.290m; gyro bias through gravity6.421/51.365m; ARW.944/5.338m; VRW.158/.447m. These are component scales, not realized online errors or quantities to sum. Pure INS above starts at exact gap truth, then integrates unknown-bias raw IMU without aiding through endpoint before reacquisition; it is separate from the online filter.

A60s outage at20m/s covers1.2km: a reasonable MEMS stress test, but a universal sub-metre online target is unsupported. vmax20 is a ceiling, not actual cruise speed. Planning scales from accumulated evidence: available H roughly.7–1.5m, V.2–1.5m; NHC30s H1–5m,60s H5–15m, V.2–2m. Prior held-outs exceed these scales; they are neither gates nor confidence bounds. Better speed aiding/bias calibration may help; larger independent tail validation is required for achievable guaranteed targets. GNSS-only model expectations above are defensible sensor scales.

## New 2x2 timing evidence

scripts/audit_noise_phase.py selects process noise using prior-state absolute forward body speed, as in the earlier unpromoted diagnostic. Factor A enables scheduling in all non-outage propagation intervals; factor B enables it inside the two known outage windows. Disabled intervals use nominal12m/s. This is offline known-window instrumentation, not an outage detector or deployable policy. Boundaries use interval START t[k] because IMU[k] drives t[k] to t[k+1]. The switch at60s affects60–60.01, not59.99–60. Available includes0–30s calibration and90–120s recovery. Long-gap effects include the entire earlier history, including first-gap changes; this is not isolation of just pre-120 and in-120 behavior.

Seed900 uses init0; held-outs use independent sampled truth-relative alignment init=seed+400000. Their factorial baselines therefore differ from the default-init before/after table. Compare each cell against its own00 baseline. Sensors/trajectory/alignment are identical across four cells within each seed. Native replay exactly matches cell00 t/p/v/P_pos for all three runs. Complete signed errors, covariance and per-IMU phase/speed schedules are in factorial/trace_*.npz. Checkpoint covariance diagnostics and10s bins are in factorial/metrics.json. Three seeds are descriptive examples, not tail validation, and the four cells are dependent.

|Seed|A available|B outage|Available H|30s H|60s H|Overall H|Overall V|
|---:|---:|---:|---:|---:|---:|---:|---:|
|900|0|0|0.886598|1.488475|11.016203|7.019110|0.762560|
|900|1|0|0.957162|1.626898|12.010850|7.652555|0.780716|
|900|0|1|0.889399|1.619937|10.789266|6.882841|0.758734|
|900|1|1|0.960088|1.760456|12.066813|7.693692|0.778688|
|100001|0|0|0.934795|1.899018|2.975399|2.146975|0.361070|
|100001|1|0|0.953261|2.222352|2.518986|1.971645|0.426648|
|100001|0|1|0.933790|1.793643|2.852087|2.060031|0.334855|
|100001|1|1|0.953586|2.007516|2.359631|1.842618|0.403892|
|100002|0|0|0.847233|3.968968|6.218672|4.346757|0.350472|
|100002|1|0|0.844186|3.793675|8.019954|5.373318|0.325264|
|100002|0|1|0.847915|4.089122|3.668802|3.001713|0.354982|
|100002|1|1|0.844626|3.913462|4.564871|3.417061|0.331488|

Each cell retains per-segment V, peaks, endpoints, covariance diagnostics and H SSE fractions in metrics.json. Signed factorial effects operate on H MSE in m², not an additive RMSE budget: available-only=y10-y00, outage-only=y01-y00, joint=y11-y00, interaction=y11-y10-y01+y00.

|Seed|60s available-only delta|60s outage-only delta|60s joint delta|60s interaction|
|---:|---:|---:|---:|---:|
|900|22.903793|-4.948463|24.251255|6.295925|
|100001|-2.507709|-0.718598|-3.285142|-0.058835|
|100002|25.647778|-25.211769|-17.833830|-18.269840|

Outage-only scheduling lowers overall H RMSE in all three examples, whereas available-only scheduling worsens900 and100002 and helps100001. Joint behavior is non-additive. This shows why whole-trajectory Q scheduling cannot be interpreted as purely in-gap diffusion correction. Three cases do not justify selectively lowering Q only during outages, or a production change. The hypothesis that a tiny NHC-sigma tweak is the dominant defect has no supporting evidence here.

## End-to-end re-examination and bug ledger

No new production correctness bug established in this bounded continuation. No bug fix claimed. Historical bug ledger and before/after are in ../report.md and the covariance_reset/discretization/observability reports. Current inspected anchors:

- Simulator truth/IMU: ccgins/sim.py:150-169, filters.py:33-45 use ENU nav/FLU body, body-to-nav matrices, old-attitude acceleration, second-order position then right attitude increment. geometry.py:48-56 specifies ZYX Euler; no quaternion convention applies to this path. sim.py:15 uses constant gravity. Default simulator/filter both omit Earth rotation; optional fixed-tangent Earth/Coriolis is not full geodetic transport. Truth inverts the same integrator, an optimistic scenario assumption rather than independent physical validation. Units and sample noise conversions are resolved above.
- GNSS: sim.py:190-210 validates rate, rounds to grid, propagates zero-start AR(1) MP through gaps using actual spacing, then adds p+Rlever. filters.py:122-124,282-286 matches lever sign/frame. pipeline.py:256-260,416-419 aligns propagation/update indices and skips invalid fixes. Same-epoch IMU-origin scoring is fair. pipeline.py:179 uses white-only R despite colored MP: demonstrated model mismatch, not new timing/sign bug.
- InEKF: filters.py:93-118 uses first-order Phi/Qd, Joseph update and finite-injection covariance reset. Invariant F/G at233-255, additive body bias injection257-265, GNSS H282-286 and body-velocity attitude cancellation288-291 match the tested convention. Nominal-speed white noise55-66 remains approximate. Earlier finite-difference/discretization tests exist; this round isolates when that noise approximation matters rather than changing Jacobians. Maneuvers determine bias observability; NHC does not independently observe forward speed.
- Initialization: pipeline.py:119-133 samples truth-relative attitude/velocity/position and zero bias means, with larger initial covariance. This remains a scenario limitation, not causal deployment alignment. Independent alignment seeds only test that assumption.
- Motion/outages: pipeline.py:263-294,313-325 supports guarded ZUPT and default NHC; ZUPT off here and no odometer supplied. Simulated slip/vertical motion(sim.py:152-156) makes NHC approximate. Propagation/NHC continue through gaps. Terminal180s correction is available, excluded from gap. No online/RTS smoother exists or is claimed. A future manifold RTS needs cross-covariance logging and separately labelled offline metrics; terminal-gap future aiding is limited.

Closed historical bugs with source anchors: original filters.py:55-60 noise density incorrectly combined sample standard deviations/assumed dt=.01 (current55-66 corrected); original metrics.py:30-36 ignored t_start/repeated-duration collisions/empty outages(current30-42 corrected); original sim.py:191-200 used requested MP period rather than rounded grid(current198-203 corrected, rate validation190-194); parent9ac3be8 scripts/audit_calibration.py:19-26 hard-coded segment masks(current15-25 shared masks). Finite-injection covariance reset was corrected in prior covariance_reset delivery (current filters.py:110-117,267-280). These fixes and evidence remain closed, not new work here.

## Validation and residual work

Six new regressions verify exact phase boundaries, pre-propagation body speed, four phase combinations, exception restoration, signed MSE interactions and rejection of duplicate cells. Full regression:237 passed in95.42s. Independent code/report review is recorded in /workspace/ccgins-v08/reviews/noise_phase_factorial_independent.md. Before/after complete budget and history equality are in validation.json. Reproduce with PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python scripts/audit_scheme.py --seeds 900 100001 100002 --out <dir>; scripts/audit_noise_phase.py --out <dir>; python -m pytest -q.

This closes the bounded pre-gap/in-gap noise-factorial measurement, not full calibration. Remaining work: specify causal initial alignment and low-excitation failure behavior; design observable joint colored-GNSS/slip/residual-bias uncertainty model using a frozen training/evaluation split; validate independent joint sensor/trajectory/alignment tails before target gates. A larger replication must test whether outage-only scheduling benefit persists, without tuning on these examples. DETECTION remains separately open. No external blocker; stopping at a structured evidence boundary. No V0.8 claim.
