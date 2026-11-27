# Whole-scheme audit and measured error budget

Baseline cfad161. Config configs/example_sim.yaml: seed900, duration180s, dt.01, t_start30s; held-out trajectory/sensor seeds49501/49502. All share CLI init_seed=0. No tuning on held-out seeds. Same inputs before/after; simulator default outputs unchanged. Online filter only. No RTS metric claimed.

## Measured per-segment horizontal / vertical RMSE (m)

| Seed | Segment | Before H | After H | Before V | After V |
|---|---|---:|---:|---:|---:|
|900|available_30_60|0.985386|0.986797|1.147169|1.162778|
|900|outage_60_90|1.544370|1.512239|0.651745|0.674476|
|900|available_90_120|0.762215|0.768319|0.471964|0.472049|
|900|outage_120_180|11.258836|11.040119|0.665695|0.643410|
|900|terminal_available|1.388767|1.367129|0.430497|0.380544|
|900|overall|7.173489|7.035064|0.754790|0.755822|
|49501|available_30_60|1.029707|1.034644|0.523134|0.521995|
|49501|outage_60_90|0.799606|0.824900|0.431717|0.419057|
|49501|available_90_120|1.169598|1.161532|0.579325|0.606407|
|49501|outage_120_180|5.392396|5.200525|0.739915|0.710263|
|49501|terminal_available|1.977128|1.859527|0.818049|0.882089|
|49501|overall|3.498449|3.381255|0.615075|0.604341|
|49502|available_30_60|0.652343|0.660441|0.152116|0.158120|
|49502|outage_60_90|3.003114|3.431730|0.145088|0.148243|
|49502|available_90_120|1.149407|1.143458|0.231822|0.261305|
|49502|outage_120_180|4.882219|4.840543|0.524838|0.511794|
|49502|terminal_available|2.761432|2.743795|0.432011|0.332493|
|49502|overall|3.418303|3.474680|0.360286|0.357510|

Seed900 long outage accounts for 98.468% of baseline horizontal SSE; available segments combined H=.881984m, outage30=1.544370m, outage60=11.258836m. Max outage60=28.257610m before and 27.795255m after. The earlier ~28m claim is reproduced, not a separate invented baseline. Small correction does not solve long-outage drift. Seed49502 overall and 30s-outage regress.

### GNSS-only, measured antenna residual including multipath

| Seed | H RMSE | V RMSE |
|---|---:|---:|
|900|1.427352|2.124529|
|49501|1.521003|2.088486|
|49502|1.636482|1.915685|

GNSS sigma_h=1m is per horizontal axis, not radial RMSE. Stationary-multipath expectation H=sqrt(2*(1²+.5²))=1.581139m, V=sqrt(2²+.5²)=2.061553m; short realized tracks vary and MP begins at zero. GNSS available epoch residuals use true antenna p+R lever; filter metrics use IMU origin p at identical logged epochs.

### Pure INS diagnostic: exact truth initialization, unknown biases, no aiding

Not an online-filter result: reinitialize to exact truth at each outage start and integrate the same raw IMU for the full 30/60s. Actual endpoint at90/180s precedes any GNSS correction in this diagnostic.

| Seed | Start | Duration | Mean speed m/s | H RMSE | H endpoint | V endpoint |
|---|---:|---:|---:|---:|---:|---:|
|900|60.0|30.0|10.131|9.458|21.390|37.583|
|900|120.0|60.0|8.954|42.694|101.889|160.128|
|49501|60.0|30.0|11.237|4.288|9.524|37.797|
|49501|120.0|60.0|19.407|29.754|61.272|144.633|
|49502|60.0|30.0|9.933|18.393|41.267|15.998|
|49502|120.0|60.0|16.294|93.112|210.119|66.094|

### Analytical endpoint scales (one horizontal axis; metres)

Linearized flat straight motion, zero initial position/velocity/tilt errors. Independent components are scales, not additive bounds or a prediction for a maneuvering NHC filter.

| Source | 30s | 60s |
|---|---:|---:|
|Turn-on accel bias .04905m/s²: bT²/2|22.073|88.290|
|Turn-on gyro bias 30deg/h: gbT³/6|6.421|51.365|
|VRW .1/60: qT^1.5/sqrt3|0.158|0.447|
|ARW .3deg/60: gqT^2.5/sqrt20|0.944|5.338|
|Accel bias RW .05mg/sqrt(s): qT^2.5/sqrt20|0.541|3.058|

Two independent horizontal axes increase radial RMS scales by sqrt2. These ignore speed-dependent vibration (at20m/s accel sample std .416667m/s², equivalent density .041667 at100Hz), initial errors and bias estimation. The filter can estimate biases during preceding GNSS-aided motion; their residual errors were not logged in this round, so raw turn-on scales should not be assigned to the aided filter.

## Audit findings, fixes and boundaries

1. **Fixed noise conversion bug:** old filters.py:55-60 used hypot(base density,vibration density) despite sim.py:184-185 adding sample standard deviations, and hardcoded dt=.01. Corrected filters.py:55-66 sums the density terms at nominal12m/s using actual dt. Batch pipeline.py:161, stream.py:121, stream_learning.py:91 and stream_replay.py:20 now supply dt. Statistical tests at5/20ms verify against simulator samples. Nominal speed remains an approximation; no truth speed feeds the filter. Additive simulator noise is a documented legacy model, not two independent noise draws.

2. **Fixed outage scoring bugs:** old metrics.py:30-36 ignored t_start for outage metrics, overwrote equal-duration outage keys, and failed on empty intervals. metrics.py:30-42 now intersects the scoring mask, distinguishes repeated integer-duration keys by start, and reports null for unscored intervals. Unique-duration example keys stay compatible.

3. **Fixed GNSS multipath timing bug:** old sim.py:191-200 rounded sampling to the IMU grid but evolved MP at requested period. sim.py:191-203 uses the actual rounded period and rejects step<1. Non-grid rates tested; default1Hz unchanged.

4. Truth/IMU: sim.py:158-169 inverts the exact discrete mechanization in filters.py:32-44 (old-attitude acceleration, second-order position, then attitude). Units sim.py:43-58,179-185: ARW .3*pi/180/60 rad/sqrt(s); VRW .1/60m/s/sqrt(s); gyro bias30*pi/180/3600 rad/s; accel bias5e-3*9.81m/s². Gyro RW3deg/h/sqrt(h), accel RW.05mg/sqrt(s) are additional defaults not listed in YAML. No unit-conversion error found.

5. Frames/gravity/rotation: sim.py:1-5,18 and geometry.py:48-56 use ENU/FLU, body-to-nav matrices, ZYX Euler. This path uses rotation matrices, no quaternion scalar-order convention. Gravity fixed [0,0,-9.81]; Earth rotation absent in both synthetic truth and default filter, internally consistent but not geodetic realism. filters.py:20-44 optional Earth/Coriolis uses fixed tangent ENU and neglects transport. Real-data Earth path separately tested, not validated by synthetic audit.

6. GNSS: sim.py:197-208 and filters.py:119-121 use antenna p+Rlever with FLU lever [.5,0,1.2], same sign/frame. MP AR(1) begins at0 (nonstationary early variance); filter R contains nominal white sigma only, not colored MP covariance. That is model approximation, not independent-noise calibration. pipeline.py:247-260 propagates IMU[k] to k+1, then updates matching GNSS index (pipeline.py:413-416); logs pipeline.py:519-535 store same-epoch truth and IMU origin after updates.

7. InEKF: filters.py:217-271 implements right-invariant SE_2(3) error with additive body biases; cross products in bias/noise Jacobians are consistent with that convention. ESKF has body attitude errors (filters.py:172-215). Measurement and propagation finite-difference tests cover both plus Earth path. Initial covariance is mapped to invariant coordinates by filters.py:277-293. No sign bug found in inspected default equations. Euler Phi=I+Fdt and Qd=GQcGᵀdt (filters.py:93-102) omit higher-order within-step coupling; this is first-order discretization, not an exact process covariance. Updates use Joseph form but no explicit post-injection error-coordinate reset (filters.py:105-117); the finite-correction consistency impact remains an open quantitative audit item.

8. Alignment/observability: pipeline.py:119-132 uses truth plus sampled alignment errors and zero initial biases, not a deployable alignment procedure. Fixed init seed makes paired tests fair but underexplores alignment robustness. GNSS position over maneuvers constrains velocity/attitude and biases indirectly; straight constant-speed motion leaves heading/forward drift weakly observed. NHC observes lateral/vertical body velocity only; InEKF H body velocity has no direct attitude column (filters.py:262-265), consistent with invariant cancellation, not missing yaw measurement. No forward speed observation in this YAML. Formal trajectory-dependent observability singular-value study remains open; do not claim all biases observable.

9. Constraints: pipeline.py:310-320 applies fixed NHC at10Hz after1s, with sigma.1m/s; sim.py:152-156 contains actual slip and vertical velocity, so constraints are approximate, not exact truth. ZUPT is off by default (pipeline.py:84); causal stop confirmation and optional independent odometry exist in pipeline.py:265-294 and zupt.py. No odometer introduced; NHC does not replace it. No tiny NHC tuning performed.

10. Outages/recovery: sim.py:207-208 uses [start,end), pipeline.py:416 skips invalid fixes but continues propagation and NHC. Terminal180s is valid GNSS and receives an online update. Last logged outage error is at179.9s, not terminal corrected state. Forced reacquisition policy after reject limit is a declared heuristic (pipeline.py:452-464), not evidence of fault-tolerant integrity. No smoothing in run. An RTS extension would need manifold error/cross covariance logging; report separately if built. Terminal outage has almost no future data, so smoothing cannot be assumed to restore it.

## Scenario fairness and achievable targets

At20m/s a60s outage spans1.2km: reasonable stress test for this MEMS grade, unreasonable to require sub-metre unaided online positioning throughout. Actual outage speeds appear above and differ by seed. Current grade has significant turn-on biases and added road vibration, not just quoted ARW/VRW.

Provisional engineering targets, not acceptance guarantees: GNSS-only H~1.6m/V~2.1m epoch RMS including MP; GNSS-aided segments H~.7–1.5m and V~.2–1.5m after convergence (<=1m is plausible, not universal). For current NHC/grade and these limited trajectories,30s outage H RMSE~1–5m with endpoint excursions up to~10m is realistic;60s H RMSE~5–15m and peaks/endpoints~10–40m. Vertical NHC-aided RMS~.2–2m is plausible here, not for unconstrained airborne/rough motion. Three seeds do not establish tail bounds. Submetre60s requires independently justified forward-speed aiding, better grade/bias calibration, stronger excitation or external measurements; optional offline smoothing is a different metric.

## Residual plan / honest scope

Bounded whole-scheme code audit and measured budget completed; a full consistency/observability investigation is not complete. Next: quantify finite-injection covariance reset and higher-order Q on long-range invariant coordinates; compute maneuver-conditioned bias observability with independent alignment seeds; expand Monte Carlo before setting targets as gates; calibrate colored GNSS and causal speed-dependent noise without truth leakage. Fix any resulting demonstrated errors with same-input paired regressions. Optional RTS needs an explicit offline benchmark and no online conflation. DETECTION remains FAIL, confirmed_v08=false; no V0.8 claim.

Evidence: before/after budget.json contain H/V, peaks, SSE shares and10s bins; errors_*.csv give time histories. Regression and independent re-review recorded under this stage and reviews/.
