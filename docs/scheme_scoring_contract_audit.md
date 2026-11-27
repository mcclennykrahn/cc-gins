# Positioning scheme audit: scoring contract

Candidate `scheme_scoring_contract`, parent `49a32e5`. Evidence-relative paths below resolve from `/workspace/ccgins-v08/results/cc_scheme_audit/scoring_contract/`. Honest mid-audit: no production filter change or accuracy improvement. DETECTION remains FAIL; confirmed_v08=false.

## Error budget first

Fresh before/after runs: configs/example_sim.yaml, seed900 and held-out102001/102002, duration180s, scoring t_start30s, default init seed. Navigation errors use IMU origin at the same logged epoch (every tenth IMU step); GNSS-only uses true antenna at valid fix epochs. Half-open gaps [60,90),[120,180); the terminal180s reacquisition is separate. Full segment dictionaries, maxima/endpoints, 10s bins and signed histories are in before/ and after/. validation.json proves exact equality of all original metrics and byte-identical histories. New INS metadata exposes actual propagation bounds.

Each H/V value below is metres and identical before/after.

| Segment | 900 H/V | 102001 H/V | 102002 H/V |
|---|---:|---:|---:|
|available|0.886598/0.902407|1.046433/0.320123|1.032230/0.201107|
|available_30_60|0.989350/1.185786|0.797446/0.424746|1.283845/0.249328|
|outage_60_90|1.488475/0.676636|1.755167/0.238486|1.263661/0.169744|
|available_90_120|0.767795/0.474157|1.243708/0.144644|0.693673/0.137172|
|outage_120_180|11.016203/0.640390|13.892463/0.263667|2.646929/0.171087|
|terminal_available|1.386857/0.378029|1.813730/1.091087|1.237822/0.112096|
|overall|7.019110/0.762560|8.843243/0.283178|1.883191/0.183447|
|GNSS only|1.427352/2.124529|1.618798/2.075687|1.564238/1.880736|

Long-gap H SSE fractions: 900: 98.462%, 102001: 98.652%, 102002: 78.971%. Seed900 confirms the dominance hypothesis; held-outs available H slightly exceeds1m, so <=1m is not a universal gate. Historical7.173m overall and about28m peak remain historical, not newly measured baseline values.

Seed900 H/V RMSE by 10s interval:

| Start | H | V |
|---:|---:|---:|
|30|1.342|1.468|
|40|0.793|1.259|
|50|0.712|0.692|
|60|0.268|0.674|
|70|1.581|0.686|
|80|2.019|0.670|
|90|0.895|0.504|
|100|0.775|0.519|
|110|0.606|0.389|
|120|0.861|0.440|
|130|2.153|0.484|
|140|3.957|0.604|
|150|7.093|0.754|
|160|12.995|0.786|
|170|22.089|0.693|

## INS scales, scenario fairness and targets

Code resolves bias units: sim.py:181 multiplies gyro_bias_sigma by pi/180/3600: 30deg/h =1.4544e-4rad/s. sim.py:182 multiplies accel_bias_sigma by1e-3*9.81:5mg=.04905m/s². sim.py:43-51 converts ARW.3deg/sqrt(h) and VRW.1m/s/sqrt(h) to per-sqrt(second) by /60; sim.py:183-187 applies bias RW sqrt(dt), white density/sqrt(dt), plus speed-dependent sample vibration. Single-axis uncalibrated endpoint scales at30/60s: accel bias22.073/88.290m; gyro bias gravity coupling6.421/51.365m; ARW gravity coupling.944/5.338m; VRW.158/.447m. These omit vibration and are component scales, not aided-filter predictions.

Exact-truth-initialized raw-IMU INS, without bias correction or aiding:

| Seed | 30s H RMS / H end / V end | 60s H RMS / H end / V end | Mean speed30/60 |
|---:|---:|---:|---:|
|900|9.458/21.390/37.583|42.694/101.889/160.128|10.131/8.954|
|102001|13.371/29.497/9.330|49.469/107.107/45.934|0.227/12.081|
|102002|13.440/31.224/57.799|47.858/105.690/224.195|0.874/8.044|

GNSS1m horizontal is per axis; with stationary0.5m MP the radial H expectation is1.581m and V2.062m. MP begins at zero; finite samples differ. GNSS-available online H roughly0.7–1.5m and V0.2–1.5m are plausible provisional goals. Aided30s H1–5m and60s H5–15m are starting engineering ranges, not probabilistic limits; prior held-out101001 already exceeded the30s range. No tail claim is supported. At20m/s,60s covers1.2km: a fair stress test, not a fair sub-metre requirement for this grade without independent speed aiding. These trajectories do not all maintain20m/s, and held-out first gaps here are mostly stopped. GNSS-only accuracy and raw INS scales cannot be substituted for aided-filter drift.

## Correctness re-examination and changes

The whole-scheme findings and historical bug citations remain in ../report.md and ../sensor_grid/report.md; closed trials are evidence, not unfinished work. Re-inspected relevant paths this round:

- sim.py:160-170 inverts filters.py:33-45 discrete mechanics: old-attitude acceleration, second-order position, then rotation. ENU/FLU body-to-nav matrices (sim.py:1-5), fixed gravity -9.81 and no Earth rate are consistent synthetic assumptions; rotation matrices avoid quaternion ordering ambiguity. Optional Earth/Coriolis path is a fixed local tangent approximation, not geodetic truth validation.
- sim.py:198-212 and filters.py:122-124 agree on antenna p+Rlever, body FLU lever[.5,0,1.2], actual sampled MP timing and half-open availability. Colored MP is not represented as an online nuisance state. No newly demonstrated lever/frame/sign bug.
- filters.py:233-255 right-invariant process Jacobians retain additive body bias states and state-dependent noise mapping; filters.py:282-291 GNSS/body velocity Jacobians match that convention. filters.py:97-101 still uses first-order Phi/Q; this is an approximation whose alternatives were already measured, not newly proven dominant defect. Joseph covariance plus finite injection reset is present at114-118; historical missing-reset finding is closed. Regression includes finite differences/reset/earth tests.
- pipeline.py:119-132 truth-relative random alignment is not deployable causal alignment; covariance conversion is filters.py:303-319. Maneuvers aid bias observability, but straight/stopped periods do not establish all biases. NHC constrains lateral/vertical velocity only; sim.py:155-156 has nonzero slip. Default ZUPT is off; optional odometry exists but no new sensor or tuning was introduced. Prior observability/slip/colored-GNSS reports remain relevant limits.
- pipeline.py:522-535 logs every tenth propagated epoch after updates at the same truth time. Outage skipping retains propagation/NHC; terminal180s receives a fix. Scoring point and t_start are consistent for the required example. No RTS smoother is present; any future smoother needs separate offline scoring, particularly because the terminal outage has little future aiding.

New diagnostic bugs, cited against parent49a32e5:

1. scripts/audit_scheme.py:59-67,83-87 hardcoded config/scoring30s and bins ending180s; GNSS scoring used direct float comparison. A config whose metrics.t_start differed silently produced a mismatched budget. Current lines89-115 accept --config, honor its t_start and actual trajectory end, use shared roundoff comparisons, reject invalid/empty scoring starts and represent no scored GNSS as null. Main-level nondefault config test exercises the actual entry point.
2. scripts/audit_scheme.py:70 rounded outage bounds to nearest indices without reporting the duration used. At dt.03, requested[.331,.601] previously propagated[.33,.60], starting before the outage. Current pure_ins_budget at52-84 narrows to contained complete steps [.36,.60], reports actual start/end/duration/count and rejects unsupported/empty/out-of-trajectory intervals. It never invents fractional sensor samples. Default on-grid30/60s values are unchanged. Analytical constant-acceleration regressions verify the endpoint and gravity cancellation.

A first new CLI test incorrectly assumed every IMU epoch was logged; it failed and was corrected to the documented every-tenth sampling. This exposed a bin endpoint issue for non-multiple-of-ten trajectory lengths: bins now use truth end, so the last logged interior epoch is retained. Final full regression and independent review are recorded separately; no production accuracy claim.

## Residual plan and reproducibility

Commands: PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python scripts/audit_scheme.py --seeds 900 102001 102002 --out <directory>; same interpreter -m pytest -q. Before ran on unmodified parent script; after default runs preceded only final start-validation/bin-end changes, which do not affect this default180s case; full tests cover final code.

Remaining substantive work: specify causal alignment with low-excitation failure handling; compare observable joint colored-GNSS/slip/residual-bias models using frozen splits; independently vary sensor/trajectory/alignment across enough trials before tail targets. Pre-gap/in-gap Q factorial is already closed evidence, not a pending task. Optional manifold RTS is a separate offline benchmark. DETECTION remains an independent unresolved gate. No concrete external blocker; this is a bounded evidence stop, not completed whole-scheme validation.

Final verification: 253 passed in 90.93s (0:01:30); focused final suite29 passed. Independent code review PASS (reviews/scoring_contract_independent.md under the project v08 folder).
