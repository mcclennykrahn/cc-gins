# Process-noise schedule isolation: mid-audit evidence

Candidate `scheme_noise_schedule_isolation`; parent 247da3b. New bounded experiment, no production navigation change and no promoted candidate. All runs use configs/example_sim.yaml, seed900 plus preselected comparison seeds92001–92004, duration180s, dt.01s, t_start30s; paired init seeds0/1701. Sensor/trajectory arrays, GNSS, initial state, NHC, gates and nominal bias random walks are identical within each triple. Comparison seeds are not used for tuning. Two alignments per sensor seed are dependent repetitions, not ten independent draws.

## Question and method

Does the fixed12m/s white-noise approximation explain drift or undercoverage? The preceding speed-noise audit demonstrated a mismatch, not its navigation impact. `fixed12` replays the current online filter exactly. `oracle` replaces only gyro/accel noise densities at each pre-propagation epoch with NoiseParams.from_spec using abs truth body-forward speed. This is offline truth-fed covariance instrumentation, not an achievable online result or an accuracy bound. `prior_state` uses abs((R.T @ v)[0]) from the previous posterior before reading the next IMU sample. It is causal but ignores speed uncertainty and slip/noise correlation; it is an uncalibrated diagnostic. Both retain the simulator legacy sum-of-standard-deviations noise law. Nothing is tuned and no production configuration or stream interface changes.

The instrumentation patches NavFilter.propagate inside a context manager, changes only filt.noise, and restores the original method on exit, including exceptions. This is a single-filter monitor=none offline tool; it must not run concurrently in a shared Python process. Each run checks18000 schedule steps. All ten fixed12 runs assert bitwise native equality for times, positions, velocities and position covariances. No RTS smoother exists or is measured.

## Same-input horizontal / vertical RMSE, metres

Each cell is H / V. These are before/after diagnostic comparisons, not a deployed algorithm improvement. Both outage masks are half-open; GNSS at180s is a separate available sample. Overall and available include it, so no terminal correction contaminates the long-outage metric.

|Seed|Init|Schedule|Available30–60|Outage60–90|Available90–120|Outage120–180|Overall|
|---|---:|---|---:|---:|---:|---:|---:|
|900|0|fixed12|0.989 / 1.186|1.488 / 0.677|0.768 / 0.474|11.016 / 0.640|7.019 / 0.763|
|900|0|oracle|1.109 / 1.239|1.761 / 0.704|0.781 / 0.465|12.016 / 0.624|7.662 / 0.778|
|900|0|prior_state|1.109 / 1.240|1.760 / 0.704|0.781 / 0.465|12.067 / 0.626|7.694 / 0.779|
|900|1701|fixed12|0.972 / 1.211|1.412 / 0.636|0.773 / 0.456|10.701 / 0.616|6.818 / 0.753|
|900|1701|oracle|0.960 / 1.236|1.639 / 0.663|0.773 / 0.466|11.833 / 0.633|7.537 / 0.772|
|900|1701|prior_state|0.960 / 1.236|1.635 / 0.663|0.773 / 0.467|11.872 / 0.634|7.562 / 0.773|
|92001|0|fixed12|0.648 / 0.736|1.295 / 0.318|1.524 / 0.236|2.648 / 0.454|1.920 / 0.471|
|92001|0|oracle|0.674 / 0.749|3.064 / 0.409|1.498 / 0.239|2.835 / 0.413|2.373 / 0.474|
|92001|0|prior_state|0.675 / 0.748|3.097 / 0.405|1.497 / 0.240|2.836 / 0.417|2.382 / 0.475|
|92001|1701|fixed12|0.657 / 0.672|0.895 / 0.239|1.501 / 0.248|2.647 / 0.501|1.870 / 0.463|
|92001|1701|oracle|0.676 / 0.676|2.558 / 0.325|1.478 / 0.253|2.847 / 0.460|2.253 / 0.458|
|92001|1701|prior_state|0.676 / 0.674|2.584 / 0.321|1.477 / 0.255|2.845 / 0.463|2.258 / 0.459|
|92002|0|fixed12|1.293 / 0.345|10.330 / 0.414|1.524 / 0.249|1.577 / 0.075|4.808 / 0.270|
|92002|0|oracle|1.298 / 0.323|8.706 / 0.389|1.510 / 0.257|2.057 / 0.098|4.199 / 0.261|
|92002|0|prior_state|1.298 / 0.323|8.787 / 0.388|1.509 / 0.254|2.024 / 0.095|4.226 / 0.260|
|92002|1701|fixed12|1.280 / 0.356|10.812 / 0.454|1.516 / 0.237|1.844 / 0.071|5.051 / 0.283|
|92002|1701|oracle|1.287 / 0.337|9.372 / 0.455|1.501 / 0.299|2.370 / 0.120|4.537 / 0.296|
|92002|1701|prior_state|1.287 / 0.336|9.469 / 0.456|1.501 / 0.295|2.333 / 0.116|4.569 / 0.295|
|92003|0|fixed12|0.775 / 0.883|5.959 / 0.191|0.992 / 0.771|17.749 / 0.359|11.547 / 0.578|
|92003|0|oracle|0.732 / 0.862|5.576 / 0.121|0.994 / 0.723|17.390 / 0.407|11.287 / 0.568|
|92003|0|prior_state|0.732 / 0.863|5.590 / 0.122|0.993 / 0.723|17.377 / 0.406|11.281 / 0.568|
|92003|1701|fixed12|0.776 / 0.835|5.963 / 0.165|0.972 / 0.749|18.110 / 0.367|11.770 / 0.558|
|92003|1701|oracle|0.735 / 0.816|5.596 / 0.109|0.979 / 0.711|17.507 / 0.395|11.361 / 0.547|
|92003|1701|prior_state|0.735 / 0.816|5.609 / 0.108|0.979 / 0.710|17.468 / 0.393|11.339 / 0.546|
|92004|0|fixed12|0.762 / 0.286|3.385 / 0.242|1.056 / 0.165|2.327 / 0.492|2.190 / 0.361|
|92004|0|oracle|0.755 / 0.252|3.046 / 0.204|1.055 / 0.200|2.464 / 0.576|2.150 / 0.402|
|92004|0|prior_state|0.754 / 0.252|3.040 / 0.205|1.055 / 0.199|2.464 / 0.574|2.148 / 0.401|
|92004|1701|fixed12|0.776 / 0.325|3.263 / 0.255|1.058 / 0.150|2.189 / 0.469|2.095 / 0.356|
|92004|1701|oracle|0.774 / 0.277|2.831 / 0.205|1.059 / 0.192|2.313 / 0.560|2.022 / 0.396|
|92004|1701|prior_state|0.773 / 0.278|2.827 / 0.205|1.059 / 0.191|2.312 / 0.558|2.020 / 0.394|

## Measured interpretation and consistency

oracle: overall H improves in 6/10 pairs, 30s gap in 6/10, and60s gap in 2/10. Alignment pairs share sensor draws; this is descriptive counting only.
prior_state: overall H improves in 6/10 pairs, 30s gap in 6/10, and60s gap in 2/10. Alignment pairs share sensor draws; this is descriptive counting only.

|Seed900 init0 segment|Fixed12 NEES/3|Oracle NEES/3|Prior-state NEES/3|Fixed12 95% 3D coverage|Oracle coverage|Prior-state coverage|
|---|---:|---:|---:|---:|---:|---:|
|available_30_60|3.034|3.408|3.416|0.433|0.313|0.307|
|outage_60_90|1.118|1.266|1.267|1.000|1.000|1.000|
|available_90_120|2.582|2.692|2.694|0.390|0.400|0.397|
|outage_120_180|2.108|2.473|2.475|1.000|0.738|0.738|

NEES/3 uses full ENU position covariance; coverage is the3D chi-square threshold7.814728, not horizontal coverage. Samples within trajectories are correlated and these frequencies are not calibrated confidence statements. Oracle variance matching worsens seed900 final-gap RMSE and NEES; fixed-speed white noise is not a sufficient explanation for this draw. Changing Q also changes prior bias estimates, GNSS gating and NHC gains, so this experiment does not attribute the residual solely to any one source. No general benefit or isolated bias effect is established.

## Historical budget, units, correctness and targets

The original whole-scheme report ../report.md records the reproduced seed900 baseline H7.173489m overall, available H.881984m, outage30 H1.544370m, outage60 H11.258836m, long-gap peak28.257610m and98.468% horizontal SSE. Current fixed12 reproduces H7.019110m overall and11.016203m in the60s gap. These are separate historical/current baselines; no additional original baseline metrics are inferred. GNSS-only antenna residuals including MP were measured at H1.427352m/V2.124529m for seed900. White sigma_h1m is per axis; stationary multipath expectation is H1.581139m/V2.061553m, with finite-track variation.

Code audit evidence: ccgins/sim.py:43–58 and179–185 resolves gyro bias30deg/h ->30*pi/180/3600rad/s and accel bias5mg ->.04905m/s²; ARW.3deg/sqrt(h) ->.3*pi/180/60 and VRW.1m/s/sqrt(h) ->.1/60. Body FLU to nav ENU uses rotation matrices, no quaternion order; sim.py:158–169 inverts filters.py:32–44 old-attitude force, second-order position then rotation. Both synthetic truth and default filter omit Earth rotation and use fixed9.81 gravity. This is internally consistent local simulation, not real-world Earth validation.

GNSS lever arm +R@[.5,0,1.2] matches sim.py:204 and filters.py:123–125. GNSS multipath remains colored while the nominal R is white-only (pipeline.py:179); outage timing [60,90),[120,180) is sim.py:207–208, and pipeline.py:253–260 propagates IMU[k] before the matching epoch update. Position scoring is IMU-origin at the same logged time, not antenna. InEKF right-invariant error/additive body-bias Jacobians and Joseph/reset covariance were reviewed in ../report.md and ../covariance_reset/report.md; first-order Phi/Qd at filters.py:94–103 is still an approximation. Previous bugs and exact historical source references remain in those reports: noise dt/addition, outage masks/keys/empty windows, grid-period multipath, posterior reset, and detailed CLI scoring. No new production bug is demonstrated in this continuation.

NHC constrains lateral/vertical speed, not forward speed; simulated slip at sim.py:152–156 is nonzero. Initial alignment remains truth-perturbed at pipeline.py:119–132, not a field alignment algorithm. The maneuver/bias analysis in ../maneuver_observability/report.md and NHC causal isolation in ../nhc_timing_factorial/report.md remain applicable. No odometer or smoothing is enabled. These limitations must not be labeled fixed by a Q schedule.

Pure-INS one-axis endpoint scales from the original report: accel turn-on bias22.073m/88.290m at30/60s; gyro turn-on bias gravity coupling6.421m/51.365m; base VRW.158m/.447m; base ARW.944m/5.338m. These are unaided component scales before estimating bias, not additive bounds for the NHC filter. Measured seed900 truth-initialized raw-INS H endpoints21.390m/101.889m. The current scenario is a useful stress test;60s at20m/s spans1.2km, but seed900 average final-gap speed is8.954m/s. Submetre60s online accuracy is not a defensible target for this grade/aiding.

Broader comparison draws widen provisional targets. Current fixed12 available H spans.648–1.524m across the two interior segments; <=1m is plausible, not universal. Fixed12 30s-gap H spans.895–10.812m and60s-gap H1.577–18.110m here. These supersede narrow earlier three-seed descriptive ranges. Practical planning allowances should cover order1–2m aided H, order1–12m30s-gap H RMSE and order2–20m60s-gap H RMSE for these synthetic trajectories, with larger possible peaks. They are not acceptance guarantees or tail bounds; vertical numbers are in the table. GNSS-only remains order1.6m H/2.1m V, conditional on specified noise. Better calibrated bias/forward aiding or better IMU grade is needed for much tighter long-gap targets.

## Residual plan and honest stopping point

No production change is justified by these mixed results; do not promote the causal diagnostic or tune NHC sigma. Next isolate Q schedule during pre-gap calibration versus in-gap propagation, holding entry state/covariance fixed. Then separate colored GNSS, correlated slip and bias random-walk/model effects across more independent sensor/trajectory draws and realistic alignment. A calibrated causal speed model must propagate speed uncertainty and preserve batch/stream parity. Optional manifold RTS requires cross-covariance logging and a separately labeled offline benchmark; terminal outage has little future support. Whole-scheme investigation remains open, with no external blocker. DETECTION FAIL, confirmed_v08=false; no V0.8 claim.

Reproduce: OPENBLAS_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python -m scripts.audit_noise_schedule --out OUTPUT. metrics.json stores all segments, time bins, NEES/coverage and checkpoint covariance; trace_*.npz stores signed ENU errors, position covariance and noise schedules. Independent review and regression are required before safe intended-path commit.
