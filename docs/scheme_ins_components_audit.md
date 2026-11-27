# Pure INS component audit and fresh online error budget

Candidate `scheme_ins_components`, parent `7522117`. This is new diagnostic evidence, with no production filter or configuration change. The whole-scheme audit remains incomplete. DETECTION remains FAIL; confirmed_v08=false.

Evidence paths below resolve from `/workspace/ccgins-v08/results/cc_scheme_audit/ins_components/`. Config is exactly `configs/example_sim.yaml`: seed900 plus fresh held-out96001/96002, duration180s, dt0.01s, t_start30s. Baseline error budget was measured before implementing the diagnostic. Reproduce using `PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python scripts/audit_scheme.py --seeds 900 96001 96002 --out <baseline>` and `scripts/audit_ins_components.py --out <ablations>` with the same environment. No model, sigma or noise schedule was tuned.

## Measured online baseline and replay

Metres; H is radial horizontal RMSE, V is vertical RMSE. `baseline/budget.json` contains GNSS-only metrics, pure INS, maxima/endpoints and 10s bins; signed online ENU histories are `baseline/errors_<seed>.csv`. `after/` is a fresh online replay after adding the diagnostic, not a different filter. All segment dictionaries and signed histories match exactly across all three seeds; independent validation is recorded in `/workspace/ccgins-v08/reviews/ins_components_independent.md`.

|Seed|Segment|H RMSE|V RMSE|H SSE share|
|---:|---|---:|---:|---:|
|900|available_30_60|0.989350|1.185786|0.397%|
|900|outage_60_90|1.488475|0.676636|0.899%|
|900|available_90_120|0.767795|0.474157|0.239%|
|900|outage_120_180|11.016203|0.640390|98.462%|
|900|terminal_available|1.386857|0.378029|0.003%|
|900|available|0.886598|0.902407|0.639%|
|900|overall|7.019110|0.762560|100.000%|
|96001|available_30_60|1.528492|0.643493|4.246%|
|96001|outage_60_90|3.168657|0.495208|18.247%|
|96001|available_90_120|0.607639|0.525474|0.671%|
|96001|outage_120_180|4.597548|0.152344|76.831%|
|96001|terminal_available|0.878067|0.050080|0.005%|
|96001|available|1.162664|0.586969|4.922%|
|96001|overall|3.316224|0.442992|100.000%|
|96002|available_30_60|1.173952|0.323745|6.174%|
|96002|outage_60_90|2.430599|0.527649|26.467%|
|96002|available_90_120|0.763610|0.207594|2.612%|
|96002|outage_120_180|2.686331|0.148760|64.659%|
|96002|terminal_available|2.412957|0.272659|0.087%|
|96002|available|0.994328|0.271944|8.873%|
|96002|overall|2.112172|0.306761|100.000%|

Seed900 confirms long-gap domination (98.462% H SSE), but a universal <=1m available-segment target fails on held-out96001 (first available interval1.528m). The historical cfad161 overall7.173489m / long-gap max28.257610m remains historical, not a newly measured metric for this parent. Current seed900 overall7.019110m reflects previously reviewed fixes, not an improvement from this diagnostic.

## Pure INS component ablations

Each gap starts at exact truth R/v/p, propagates original IMU indices through the full 30/60s, and scores the endpoint before reacquisition. These are offline uncalibrated INS diagnostics, not online filter variants. Each bias path includes turn-on plus random walk. Sample noise includes base white noise and speed-dependent vibration; the simulator adds their standard deviations before drawing a single sample, so they are not separately independent sources. Removed components use simulator truth only in this diagnostic and never enter an online update.

Each cell is H RMSE / H endpoint in metres. Full V metrics, signed endpoints and sample counts are in `ablations/components.json`; every signed time history is in `ablations/trace_<seed>_<start>.npz`.

|Seed|Gap|Gyro bias|Accel bias|Gyro sample noise|Accel sample noise|Both biases|Both sample noises|All|
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|900|30s|1.960 / 5.055|5.293 / 11.052|0.955 / 2.302|2.109 / 4.847|7.243 / 16.155|2.286 / 5.412|9.458 / 21.390|
|900|60s|15.949 / 42.210|24.636 / 55.605|0.957 / 2.491|3.297 / 5.158|40.545 / 98.045|3.224 / 6.437|42.694 / 101.889|
|96001|30s|1.358 / 3.503|12.368 / 26.819|0.777 / 1.869|0.209 / 0.588|12.042 / 25.838|0.963 / 2.425|12.054 / 25.812|
|96001|60s|11.521 / 30.232|50.801 / 110.515|3.269 / 8.002|4.736 / 9.618|48.912 / 106.693|6.521 / 13.412|47.329 / 102.745|
|96002|30s|2.788 / 7.370|4.151 / 9.265|0.200 / 0.411|0.182 / 0.408|1.576 / 2.305|0.319 / 0.743|1.403 / 2.257|
|96002|60s|21.592 / 56.473|14.085 / 31.305|4.269 / 10.402|2.956 / 4.862|8.599 / 26.185|1.686 / 5.894|9.634 / 30.117|

The ideal-IMU case tests discrete truth/mechanization closure: maximum absolute position discrepancy is below1.151e-10m. Independent SciPy rotation recurrence agrees within1.337e-10m; see `validation.json`. The all-component case uses original raw IMU arrays and must reproduce baseline pure INS RMS/endpoints exactly.

For seed900, combined bias H RMS40.545m versus noise-only3.224m in the60s gap supports bias-dominated *uncalibrated pure INS* drift. It does not prove that residual online drift is bias dominated after GNSS/NHC estimation. Held-out96002 illustrates cancellation: gyro-bias-only60s H RMS21.592m and accel-bias-only14.085m combine to8.599m. Norms and RMS values cannot be added or assigned percentage shares; nonlinear rotation also couples components. Three draws are not tail statistics. This evidence alone does not justify a new process-noise model, tighter NHC or smoothing.

## End-to-end code audit status and units

No new production bug was demonstrated by this bounded round. Historical bugs, original file/line evidence, fixes and paired measurements are in `../report.md`; subsequent reviews supersede its former open covariance-reset, discretization and observability items. See `../alignment_budget/report.md` for the current residual plan and links to those completed investigations.

Current inspected evidence:

- `ccgins/sim.py:43-58,179-185`: gyro bias30deg/h ->30*pi/180/3600rad/s; accel bias5mg ->0.04905m/s². ARW0.3deg/sqrt(h) divides by60 after radians conversion, VRW0.1m/s/sqrt(h) divides by60. Defaults also include bias RW3deg/h/sqrt(h) and0.05mg/sqrt(s), and speed-dependent vibration. These are not merely the YAML white noise specs.
- `ccgins/sim.py:158-169` and `ccgins/filters.py:32-44`: truth inverts the same discrete step: old-R acceleration, second-order position, velocity then right rotation increment. `sim.py:1-5,18` defines local ENU/FLU, fixed gravity; `geometry.py:48-56` defines ZYX Euler/body-to-nav matrices. No quaternion convention in this path. Earth rotation is absent from both default truth and default mechanization; optional fixed tangent Earth/Coriolis path does not model transport. This synthetic test cannot validate full geodetic mechanization.
- `ccgins/sim.py:190-217`, `ccgins/filters.py:122-124,282-286`: GNSS antenna p+Rlever, same FLU lever sign/frame. MP evolves on actual rounded epochs, starts at zero; GNSS R models nominal white errors and does not explicitly estimate colored MP. MP variance is a modeling/calibration limitation, not a new independent white source. `pipeline.py:251-259,416-419` propagates k to k+1 before same-index GNSS, skipping invalid fixes. Half-open outages allow terminal180s reacquisition; scoring uses logged IMU origin, not antenna.
- `ccgins/filters.py:91-118,233-295`: first-order Phi/Qd, right-invariant SE_2(3) Jacobians with additive body biases, Joseph update and explicit reset. Body-velocity H has invariant attitude cancellation; absence of a direct attitude column is not evidence of a bug. Existing discretization/reset/observability regressions cover these equations; this round does not claim exact covariance consistency.
- `ccgins/pipeline.py:119-133`: truth-relative sampled initialization with zero bias means remains optimistic, not deployable causal alignment. `pipeline.py:265-294,315-325` gates optional ZUPT and applies NHC after causal history; default config uses NHC, no ZUPT or wheel speed. Slip in `sim.py:152-156` makes zero lateral/vertical speed approximate. No forward-speed observation or RTS smoother is present in this run.

## Physical expectations and segment targets

One-horizontal-axis endpoint scales with zero initial errors: accel turn-on bias bT²/2=22.073/88.290m at30/60s; gyro bias gravity coupling gbT³/6=6.421/51.365m; white ARW gqT^2.5/sqrt20=0.944/5.338m; VRW qT^1.5/sqrt3=0.158/0.447m. These straight-line linear scales exclude maneuvers, vibration, bias random walk and estimation; they are not additive bounds. Pure-INS component runs quantify realized effects rather than asserting these scales predict online accuracy.

GNSS sigma_h1m is per-axis, sigma_v2m, MP sigma0.5m: stationary GNSS-only radial H RMS expectation1.581m and V2.062m. Measured true-antenna residuals including MP:

|Seed|GNSS H RMS|GNSS V RMS|
|---:|---:|---:|
|900|1.427352|2.124529|
|96001|1.605443|1.821070|
|96002|1.506647|1.889414|

Provisional engineering ranges: GNSS-aided H roughly0.7–1.5m with recovery excursions (this round already exceeds1.5m slightly), V roughly0.2–1.5m; nominal NHC-aided30s H RMS1–5m and60s5–15m, V roughly0.2–2m. These are planning ranges, not gates or guaranteed bounds; some existing trajectories fall outside them. At20m/s a60s gap covers1.2km. It is a useful MEMS stress test, but sub-metre online positioning throughout is unsupported without better calibration or independent forward-speed/external aiding. Seed900 long-gap mean speed is8.954m/s, not20m/s. Optional RTS must be separately reported offline; no smoother result is claimed.

## Residual plan and delivery status

This is a structured mid-audit stop at a bounded evidence limit, with no external blocker. Complete causal alignment and the pre-gap/in-gap Q factorial, then jointly calibrate colored GNSS, slip and residual bias uncertainty. Independent trajectory/sensor/alignment Monte Carlo is needed for tail gates; these three samples cannot supply them. Any structural fix must follow demonstrated online error attribution and paired held-out regression. Independent review and full regression are recorded alongside this report. DETECTION remains FAIL, confirmed_v08=false; no V0.8 claim.

Regression: full suite `227 passed in 98.62s (0:01:38)`; see `pytest.txt`. Separate implementation, numerical and report review PASS: `/workspace/ccgins-v08/reviews/ins_components_independent.md`.
