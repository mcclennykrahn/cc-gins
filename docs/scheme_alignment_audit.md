# Alignment sensitivity and fresh segment budget

Candidate `scheme_alignment_budget`, parent `36a6a09`. No production filter changes or parameter tuning. Config `configs/example_sim.yaml`, duration 180 s, t_start=30 s. Sensor/trajectory seeds 900, 95001, 95002; crossed initialization seeds 0, 95011, 95012. Same sensor inputs within each trajectory. Nine traces are correlated comparisons, not nine independent Monte Carlo samples.

Evidence paths in this report are relative to `/workspace/ccgins-v08/results/cc_scheme_audit/alignment_budget/`. Reproduce from the repo root with `PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python scripts/audit_alignment_budget.py --out <evidence-directory>`; the interpreter requires project dependencies. The fresh baseline command uses `scripts/audit_scheme.py --seeds 900 95001 95002 --out <baseline-directory>`.

## Before/after evidence

Fresh parent-behavior baseline uses `scripts/audit_scheme.py`. Candidate diagnostic uses the unchanged online filter. Complete segment dictionaries and signed position histories match bit for bit for init_seed=0 on all three sensor seeds. Other initialization draws assess sensitivity, not an algorithm improvement.

| Sensor | Init | Available H/V | 30s gap H/V | 60s gap H/V | Overall H/V | 60s H SSE share |
|---:|---:|---:|---:|---:|---:|---:|
|900|0|0.886598 / 0.902407|1.488475 / 0.676636|11.016203 / 0.640390|7.019110 / 0.762560|98.462%|
|900|95011|0.878442 / 0.852929|1.497056 / 0.654017|11.086532 / 0.636915|7.063188 / 0.734120|98.483%|
|900|95012|0.871990 / 0.865332|1.490296 / 0.663306|11.029242 / 0.639472|7.026636 / 0.742440|98.484%|
|95001|0|0.988703 / 0.731574|3.263573 / 1.036549|2.292548 / 1.001790|2.149669 / 0.911155|45.464%|
|95001|95011|0.989451 / 0.742291|3.207856 / 1.009296|2.285999 / 1.006935|2.130169 / 0.910778|46.036%|
|95001|95012|0.987149 / 0.748452|3.269083 / 1.038921|2.285939 / 1.004050|2.148242 / 0.918151|45.262%|
|95002|0|0.567419 / 0.332176|1.491550 / 0.609358|2.639613 / 0.242672|1.832683 / 0.376742|82.923%|
|95002|95011|0.570785 / 0.368938|1.543935 / 0.620739|2.625097 / 0.239604|1.833438 / 0.393016|81.946%|
|95002|95012|0.566886 / 0.355501|1.440212 / 0.622606|2.571518 / 0.241648|1.785093 / 0.389128|82.952%|

All RMSE values are metres. metrics.json includes both available intervals, terminal reacquisition separately, max/end errors, initial errors and 10 s bins. trace_*.npz contains same-epoch signed ENU position errors. The terminal 180 s fix is available; outage masks are [60,90), [120,180). Scoring is the IMU origin, not antenna.

## GNSS-only and pure INS comparison

Fresh same-sensor measurements below use true antenna residuals (GNSS-only) and exact-truth initialization at each gap (pure INS). Pure INS endpoint includes the full gap before any reacquisition; it is not an online result.

| Sensor | GNSS H/V RMS | INS 30s H RMS / endpoint | INS 60s H RMS / endpoint | Mean speed 30s / 60s |
|---:|---:|---:|---:|---:|
|900|1.427 / 2.125|9.458 / 21.390|42.694 / 101.889|10.131 / 8.954|
|95001|1.654 / 2.059|21.380 / 50.121|57.841 / 131.442|16.283 / 14.190|
|95002|1.254 / 2.237|16.955 / 37.753|66.815 / 156.913|8.408 / 10.588|

## Interpretation and whole-scheme status

The original cfad161 seed900 baseline H RMSE 7.173489 m, long-gap max 28.257610 m and 98.468% long-gap H SSE remain documented in ../report.md. Current init0 seed900 H RMSE is 7.019110 m; long gap H RMS 11.016203 m and SSE share 98.462%. This round does not claim a new filter improvement. Held-out95001 has a larger 30s-gap RMS than 60s-gap RMS; gap length alone does not predict drift.

Alignment evidence remains limited: pipeline.py:119-132 samples 0.5/0.5/3 deg attitude errors, 0.1 m/s velocity errors, and 1.5/1.5/3 m position errors around simulator truth. Covariance uses broader 1/1/5 deg, 0.3 m/s, 2/2/4 m priors and zero bias means. Truth-relative initialization is an optimistic scenario assumption, not a newly discovered implementation bug. No actual GNSS/static/kinematic alignment is implemented by this diagnostic.

Unit and frame evidence remains sim.py:43-58,179-185: 30 deg/h gyro bias, 5 mg accelerometer bias; ARW divides by60 after degrees-to-radians and VRW divides by60. Fixed local ENU/FLU gravity and rotation-matrix conventions remain internally consistent. One-axis pure-INS endpoint scales at30/60s: accelerometer bias22.073/88.290m; gyro-bias gravity coupling6.421/51.365m; ARW0.944/5.338m. These are uncalibrated bias/noise scales, not aided-filter predictions. GNSS radial stationary H expectation1.581m and V2.062m include0.5m MP; sigma_h=1m is per axis.

Whole-scheme audit and historical bugs with original file/line references are in ../report.md; subsequent fixes and evidence supersede its open items: ../covariance_reset/ (posterior reset now filters.py:114-118), ../discretization/, ../maneuver_observability/, ../multipath_calibration/, ../nhc_timing_factorial/, ../speed_noise_budget/, ../noise_schedule_isolation/, ../gnss_grid_rate/, ../grid_completion_fresh/. GNSS requested-rate validation is now sim.py:190-194. No new production bug was demonstrated in this round, so no additional filter change is justified. First-order Q, colored GNSS, approximate slip/NHC and realistic alignment remain modeling limits requiring quantitative evidence. No RTS smoothing result is claimed.

Realistic provisional goals remain GNSS-available H around0.7–1.5m (not a universal <=1m gate), V roughly0.2–1.5m; nominal NHC-aided30s H roughly1–5m and60s H roughly5–15m are engineering starting points rather than bounds. Prior noise-schedule trials already exceeded the30s range, so neither range is an acceptance gate. At20m/s a60s gap covers1.2km; sub-metre online accuracy throughout is unsupported with these biases and no forward-speed aiding. Seed900 actually averages8.954m/s in the long gap. Current samples cannot establish stochastic tail targets.

## Residual plan / honest stop

1. Replace truth-relative initialization with a separately specified causal alignment procedure and test its failures.
2. Complete pre-gap/in-gap Q factorial and joint colored GNSS/slip/residual-bias consistency analysis before promoting a noise model.
3. Use independent sensor/trajectory and alignment draws with adequate sample counts before declaring tail gates; current crossed draws cannot supply those bounds.
4. Any optional manifold RTS benchmark remains offline and separately scored; terminal outage has very little future aiding.

DETECTION remains FAIL and confirmed_v08=false. This is new bounded evidence, not full audit closure. No external blocker; stopping at the stated evidence limit. Independent review and regression recorded separately.
