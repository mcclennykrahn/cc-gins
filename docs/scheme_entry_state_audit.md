# Whole-scheme audit continuation: outage entry state sensitivity

Candidate `scheme_entry_state_oracle`, base `2943a37`. New diagnostic work; structured mid-audit delivery. No production code/config change, no accuracy improvement promoted. DETECTION=FAIL; confirmed_v08=false. No external blocker.

## State check and measured budget first

Read pending_handoff.md and project review.json before edits. HEAD equaled origin/main; only .dsh-prompts/ was untracked. Latest reviewed/pushed component replay delivery is CLOSED. Earlier negative trials and Stage3/v0.8 evidence were preserved. No genuinely incomplete patch remained.

Fresh baseline ran before edits: configs/example_sim.yaml, duration180s, seed900 and new held-out110001/110002, dt.01s, t_start30s, init_seed0. After the diagnostic addition, all budget dictionaries and signed CSV bytes match exactly; native replay traces also match. Values below are both before and after. Units metres. GNSS outages are half-open [60,90), [120,180); terminal180s reacquisition is a separate single epoch. Every-tenth-step posterior at the IMU origin is scored against same-time truth. GNSS-only residuals use the true antenna position.

|Seed|Segment|Before=after H RMSE|Before=after V RMSE|H SSE share %|
|---:|---|---:|---:|---:|
|900|available_30_60|0.989350|1.185786|0.3971|
|900|outage_60_90|1.488475|0.676636|0.8988|
|900|available_90_120|0.767795|0.474157|0.2391|
|900|outage_120_180|11.016203|0.640390|98.4624|
|900|terminal_available|1.386857|0.378029|0.0026|
|900|available|0.886598|0.902407|0.6388|
|900|overall|7.019110|0.762560|100.0000|
|110001|available_30_60|0.809882|0.758997|2.4531|
|110001|outage_60_90|4.106247|0.177879|63.0615|
|110001|available_90_120|0.947116|0.426999|3.3549|
|110001|outage_120_180|2.039469|0.838519|31.1128|
|110001|terminal_available|1.191119|0.644424|0.0177|
|110001|available|0.881781|0.615843|5.8257|
|110001|overall|2.311709|0.662751|100.0000|
|110002|available_30_60|0.692663|0.267968|0.0113|
|110002|outage_60_90|6.414199|0.678311|0.9671|
|110002|available_90_120|0.623007|0.748255|0.0091|
|110002|outage_120_180|45.891669|1.079677|99.0123|
|110002|terminal_available|1.697825|2.112005|0.0002|
|110002|available|0.661842|0.568104|0.0206|
|110002|overall|29.159135|0.828949|100.0000|

Seed900 long gap dominates (98.462% H SSE): current overall7.019110m, long-gap11.016203m, peak27.684186m. Historical overall7.173488835m / peak28.257610207m are archived in results/cc_scheme_audit/independent_review.json; prior reviewed fixes explain the different revision. This round claims zero online change. Seed110001 is dominated instead by the short gap (63.06% H SSE). Seed110002 long-gap peak107.543m is a new counterexample to a tight online outage target, despite available H0.662m. These are three fixed-init draws, not a tail distribution.

|Seed|GNSS-only H/V RMSE|Pure INS30s H RMS / endpoint|Pure INS60s H RMS / endpoint|Mean speed30s /60s m/s|
|---:|---|---|---|---|
|900|1.427 / 2.125|9.458 / 21.390|42.694 / 101.889|10.131 / 8.954|
|110001|1.605 / 1.867|21.636 / 49.866|74.763 / 160.102|17.408 / 16.994|
|110002|1.466 / 2.068|19.075 / 41.696|60.291 / 125.216|0.000 / 16.349|

Pure INS starts from exact truth R/v/p at each outage, propagates all3000/6000 raw IMU steps without bias subtraction or aiding, and scores its full interval including pre-reacquisition endpoint. It is a physical diagnostic, not the online filter. Ten-second online H/V budgets and maxima are in before/budget.json and after/budget.json; signed histories in errors_<seed>.csv and replay/*.npz. Zero mean speed in110002 short gap is a simulated stop; default ZUPT is disabled.

## Units, physical budget and realistic targets

Code resolves gyro_bias_sigma30 as deg/h, accel_bias_sigma5 as mg (ccgins/sim.py:180-183): 1.45444e-4rad/s and0.04905m/s². ARW0.3deg/sqrt(h) and VRW0.1m/s/sqrt(h) become8.72665e-5rad/sqrt(s),0.00166667m/s/sqrt(s) (sim.py:45-51). Bias RW is3deg/h/sqrt(h) and0.05mg/sqrt(s). Speed vibration is extra:0.02m/s²/(m/s),0.002deg/s/(m/s); simulation adds vibration/base sample standard deviations before drawing a single Gaussian (sim.py:185-188). This is not a quadrature sum of independent sources.

One-horizontal-axis unaided endpoint scales at30/60s for independent zero-initial-error flat straight approximations: accel bias bT²/2=22.073/88.290m; gyro gravity coupling gbT³/6=6.421/51.365m; ARW gqT^2.5/sqrt20=0.944/5.338m; VRW qT^1.5/sqrt3=0.158/0.447m. These omit maneuvers/vibration/RW and aided bias calibration; they cannot be summed into observed online RMSE. Measured pure INS above includes those effects for each realized draw.

GNSS sigma_h1m per axis, sigma_v2m, mp_sigma0.5m are verified in configs/example_sim.yaml:18-20 and sim.py:201-209. Stationary theoretical radial H sqrt(2*(1+.25))=1.581m, V sqrt(4+.25)=2.062m; multipath begins at zero and finite-run values vary. Available H around1m (nominal planning range roughly0.7–1.5m) and V around0.2–1.5m are plausible goals, not guarantees; prior independent seeds exceed1m. For30s, plan a few metres nominally with known17m RMS counterexamples; for60s, a few to tens of metres with this round showing45.9m RMS/107.5m peak. No validated upper bound exists. At20m/s the60s gap spans1.2km: fair MEMS stress test, unsupported sub-metre online requirement without additional aiding/calibration. Actual speeds above differ from vmax20m/s.

## Controlled entry-state experiment

New scripts/audit_entry_state.py wraps the native filter factory only within a synchronous offline run. Each arm reruns the entire native trajectory, changes selected state values ONCE immediately before propagate for IMU index6000 or12000, after the logged posterior at60/120s. Gyro and accel biases are taken from the actual synthesized IMU bias at that index. Position, velocity and internal covariance remain unchanged at intervention. Native NHC, GNSS, process noise and later bias updates continue normally; truth biases are not clamped over the gap. Prefix t<=entry position, velocity and position covariance arrays must match bit-for-bit. All signed traces are saved.

This is privileged-information sensitivity with fixed internal covariance, not a consistent oracle posterior. Especially for an invariant filter, changing R without transforming covariance changes its physical interpretation; that choice is held fixed to examine the native update response. Effects include NHC feedback, correlations and cancellation. Full attitude changes yaw as well as roll/pitch; this experiment does not isolate tilt or identify additive causal error fractions. Four arms are individual R/bg/ba and joint R+bg+ba. They do not resolve all interactions or uncertainty calibration.

|Seed|Gap s|Online H/V|Offline R H/V|Offline bg H/V|Offline ba H/V|Offline joint H/V|
|---:|---:|---|---|---|---|---|
|900|30|1.488 / 0.677|1.397 / 0.710|1.117 / 0.621|1.411 / 0.701|0.966 / 0.678|
|900|60|11.016 / 0.640|12.708 / 0.676|6.983 / 0.438|6.617 / 0.618|3.974 / 0.451|
|110001|30|4.106 / 0.178|3.518 / 0.184|4.088 / 0.105|4.509 / 0.230|3.953 / 0.146|
|110001|60|2.039 / 0.839|1.859 / 0.775|2.044 / 0.831|1.905 / 0.864|1.714 / 0.794|
|110002|30|6.414 / 0.678|12.603 / 0.418|5.706 / 0.661|23.226 / 1.076|3.601 / 0.595|
|110002|60|45.892 / 1.080|43.375 / 1.814|48.593 / 1.377|25.018 / 2.132|24.767 / 0.685|

Joint substitution reduces H RMS in all six gap pairs, but individual changes can substantially worsen it (110002 short-gap accel-bias replacement6.414→23.226m;900 long-gap R11.016→12.708m). Thus tighter NHC alone is not justified. Entry attitude/bias calibration is consequential on these draws, yet joint substitution still leaves24.767m RMS on110002 long gap and3.953m on110001 short gap. Remaining entry velocity/position errors, later IMU noise/bias evolution, model mismatch and fixed covariance can contribute; this study does not separate them. The previous in-gap NHC-truth diagnostic is a different conditional intervention and cannot be subtracted from these numbers.

## End-to-end correctness re-examination

- Truth/IMU frames and order: sim.py:3,152-168 uses local ENU, body FLU, body-to-nav R; specific force uses old R, p uses second-order acceleration, and relative SO(3) log inverts filters.py:33-45. No quaternion ordering is used. Gravity is constant[0,0,-9.81]; both simulator and default filter neglect Earth rate. Optional filters.py:21-30,38-44 local tangent Earth/Coriolis lacks transport, a scope limitation for long real drives rather than a new default sign bug.
- GNSS lever sign/frame and timing: sim.py:198-212 and filters.py:122-124 use p+Rlever. Positive[.5,0,1.2] is body FLU antenna offset. Multipath follows actual rounded grid period. pipeline.py:256-260 propagates to k+1 then uses same-index valid GNSS at416-420. No missing GNSS is fabricated. Colored MP is not estimated and nominal R omits it; previous calibration/serial reports remain relevant.
- InEKF: filters.py:94-103 uses first-order Phi/Qd. F/G at233-255 contain body additive bias coupling, invariant GNSS/NHC Jacobians at282-291; NHC attitude column cancels by invariant coordinates. Joseph covariance/reset at110-118 and finite injection/reset at257-280 preserve the previously reviewed fix. Maneuver-dependent bias observability is not proved by small position error. No new production equation bug evidenced.
- Constraints/alignment/outages: pipeline.py:315-317 zero lateral/vertical NHC approximates slip/w at sim.py:154-157. Default ZUPT/odometry are disabled. initial_state at pipeline.py:119-132 uses truth-relative random attitude/position/velocity and zero bias, an optimistic benchmark initialization; make_filter at filters.py:303-319 transforms covariance to invariant coordinates. Native GNSS continues estimating biases during available motion; a field alignment with uncertainty/refusal remains open. Outage propagation/NHC persists; logging at pipeline.py:528-540 scores same-time posterior IMU origin. No RTS smoother exists here and no smoothed metric is claimed.

No new production bug was confirmed in this bounded continuation. No production tuning or structural estimator change was made. The new evidence rejects treating residual drift as a single NHC-sigma issue and motivates deployable alignment/bias estimation with calibrated uncertainty. Prior bugs and fixes remain documented in the archived reports; they were not reopened as unfinished work.

## Verification and residual plan

Reproduce with the existing project environment:

```sh
export PYTHONPATH=. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
/tmp/ccgins-accuracy-venv/bin/python -m scripts.audit_scheme --seeds 900 110001 110002 --out /tmp/entry_budget
/tmp/ccgins-accuracy-venv/bin/python -m scripts.audit_entry_state --out /tmp/entry_interventions
/tmp/ccgins-accuracy-venv/bin/python -m pytest -q
```

New tests check entry bias indexing, one-time rather than continuous replacement, unmodified p/v/P, prefix timing, patch restoration and config/boundary refusals. Focused tests:5 passed in1.57s. Full regression:293 passed in82.55s. Independent targeted regression:43 passed in1.97s. Independent reviewer recomputed every segment for all24 intervention traces, verified prefix and before/after parity, and recorded independent_metric_check.json. The original code and metric review is in /workspace/ccgins-v08/reviews/entry_state_oracle.md. The follow-up prose and outage-half review is recorded in /workspace/ccgins-v08/reviews/outage_halves.md.

Residual plan: (1) isolate roll/pitch versus heading, entry velocity and bias interactions with explicitly specified covariance treatment; (2) implement and validate causal sensor-only alignment, rejecting unobservable starts with honest uncertainty; (3) jointly calibrate colored GNSS, process vibration and NHC slip on frozen training/validation splits; (4) predeclare independent trajectory/sensor/alignment tail tests and realistic per-segment targets. Optional RTS needs a separately reported offline comparison: the terminal180s reacquisition IS future information for earlier gap epochs, so smoothing that uses this fix must be distinguished from a scenario without reacquisition. No measurements after180s exist here. Three seeds and fixed init_seed0 cannot establish scheme completion. Audit remains incomplete because of these evidence limits, not an external blocker. DETECTION remains FAIL and confirmed_v08=false.
