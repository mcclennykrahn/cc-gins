# Whole-scheme audit: configured Earth-model consistency

Candidate `scheme_earth_model`; parent `94fd71b`. The handoff and project manifest were read first. Parent was already pushed; the older half-turn delivery was closed. Four inherited modified files contained genuinely unfinished Earth-model work, completed here. This is a structured MID_AUDIT delivery, not acceptance. DETECTION remains FAIL; confirmed_v08=false.

## Error budget first

Fresh parent and candidate runs use configs/example_sim.yaml, seed900 and held-out118001/118002, duration180s, dt0.01s, scoring t>=30s. All three default signed histories are byte-identical. H/V RMSE in metres follows; each entry is both before and after.

|Seed|Available30–60|Outage60–90|Available90–120|Outage120–180|Terminal180|All available|Overall|
|---|---|---|---|---|---|---|---|
|900|0.989/1.186|1.488/0.677|0.768/0.474|11.016/0.640|1.387/0.378|0.887/0.902|7.019/0.763|
|118001|1.019/0.797|4.325/1.127|1.041/0.302|1.209/0.550|0.288/0.604|1.029/0.603|2.179/0.721|
|118002|1.205/0.625|2.597/0.553|1.197/0.269|5.330/0.729|2.428/1.540|1.204/0.485|3.645/0.606|

|Seed|Long-gap H SSE share|GNSS-only H/V|Pure INS30s H RMS/end|Pure INS60s H RMS/end|Mean speed30/60s|
|---|---|---|---|---|---|
|900|98.46%|1.427/2.125|9.458/21.390|42.694/101.889|10.131/8.954|
|118001|12.31%|1.447/2.354|12.094/26.092|22.286/31.407|12.312/7.712|
|118002|85.49%|1.617/1.931|5.807/11.489|16.111/26.973|7.857/15.225|

Seed900 ten-second horizontal RMS bins starting30s: 1.342, 0.793, 0.712, 0.268, 1.581, 2.019, 0.895, 0.775, 0.606, 0.861, 2.153, 3.957, 7.093, 12.995, 22.089 m. Full per-seed vertical bins, max/end errors and signed traces are in default_before/ and default_after/.

The measured seed900 long-gap dominance hypothesis is confirmed (98.46% SSE), but the held-out short gap can dominate. The historical7.173m overall / approximately28m long-gap maximum are supplied historical evidence, not fresh metrics. Current seed900 overall7.019m reflects prior fixes; this round changes no default navigation output.

The configured sigmas are1m per horizontal axis,2m vertical,0.5m correlated multipath with30s time constant. Stationary GNSS-only H RMS expectation sqrt(2*(1+0.25))=1.581m and V=sqrt(4+0.25)=2.062m; the simulated multipath starts at zero, so finite-run numbers differ. Pure INS above is exactly truth-initialized at gap entry, without bias correction or NHC, and is a diagnostic, not online performance.

Units resolved from sim.py:43–58,198–205: gyro bias30deg/h=1.45444e-4rad/s; accelerometer bias5mg=0.04905m/s². ARW0.3deg/sqrt(h) becomes8.72665e-5rad/sqrt(s); VRW0.1m/s/sqrt(h) becomes0.00166667m/s/sqrt(s). Noise samples divide density by sqrt(dt), bias increments multiply by sqrt(dt). One-axis constant-bias endpoint scales at30/60s are22.07/88.29m (accelerometer) and6.42/51.37m (gyro tilt via gravity). White ARW gravity coupling is0.94/5.34m; VRW0.16/0.45m. These small-angle fixed-attitude scales exclude vibration, rotation and random walk and are not aided-filter bounds.

## Demonstrated bugs and fixes

At parent94fd71b, cli.py:136 ignored configured Earth fields when generating truth IMU; sim.py:171–172 inverted only zero Earth rate and fixed9.81 gravity, whereas pipeline.py:173–176 passed configured gravity/rotation to the filter. Thus identical noiseless truth-state mechanization with explicit Earth settings could not close. Parent scripts/audit_scheme.py:85–86 also hardcoded the default model in pure-INS diagnostics. This is a correctness defect confined to explicitly configured simulation Earth models, not evidence of a default drift cause.

Fix: Config.earth_model (pipeline.py:93–103) resolves and validates the explicit model once for both CLI synthesis (cli.py:136–138) and pipeline filtering (pipeline.py:185). sim.py:165–183 inverts the actual discrete left Earth rotation and Coriolis: Exp(w_ie dt) premultiplies the next truth attitude before recovering body rotation; +2w_ie×v cancels mechanization's negative Coriolis; configured downward gravity is removed in specific force. audit_scheme.py:74–89,126–127 propagates its pure-INS diagnostic with the same model. No transport-rate/geodetic model is implied.

verification.json measures noiseless full180s closure under config.yaml: parent position error664.683m, candidate1.401e-8m; rotation Frobenius error0.01856 to7.75e-13. This closure test starts exactly at truth and does not represent online accuracy. Tests cover gravity alone, rotation plus gravity, finite input validation, moving60s closure and configured pure-INS closure. The independent reviewer separately tests held-out moving closure.

Explicit-Earth before/after H/V RMSE follows. This is a physically consistent input correction with changed IMU samples, not an accuracy comparison on identical sensor records. Same config, seeds, truth motion and stochastic draws are retained. Mixed-model before results sometimes benefit from cancellation; larger after errors must not be sold as improvement.

|Seed|State|Available|30s gap|60s gap|Overall|
|---|---|---|---|---|---|
|900|before|0.986/0.849|1.031/0.157|6.087/0.149|3.926/0.550|
|118001|before|1.088/0.683|5.414/1.618|1.379/0.401|2.663/0.880|
|118002|before|1.213/0.481|4.436/0.477|2.503/1.640|2.650/1.102|
|900|after|0.887/0.903|1.487/0.676|10.951/0.641|6.978/0.763|
|118001|after|1.029/0.603|4.329/1.129|1.208/0.550|2.180/0.722|
|118002|after|1.204/0.485|2.607/0.555|5.302/0.731|3.630/0.608|

The old pure-INS diagnostic used the default propagation model, so its configured-Earth numbers are superseded, not a matched-physics comparison. All explicit-Earth disjoint segments, per-time bins, pure-INS and GNSS metrics are retained in verified_before/budget.json and after/budget.json. The original inherited before directory is retained; verified_before is a fresh rerun from the parent checkout.

## End-to-end re-examination and scenario fairness

Truth generation at sim.py:150–183 is body-to-nav rotation matrices, local ENU navigation and FLU body (not NED or an ambiguous quaternion order). The propagation at filters.py:33–46 uses old attitude for acceleration and second-order position before rotation, matching synthesis. Default gravity and Earth-neglect remain internally consistent. The optional Earth path is fixed local rotation/Coriolis, omitting geographic transport and gravity gradients; that approximation limits long-range real replay, rather than explaining the default synthetic drift. Existing half-turn fix and tests remain intact; no new rotation defect established.

GNSS at sim.py:210–230 uses antenna p+R lever, same positive body lever frame as filters.py:122–124,282–286. Multipath evolves through outages on actual GNSS grid elapsed time. Pipeline propagates to the measurement epoch before update and skips invalid fixes (pipeline.py:238,428). Audit errors score simultaneous IMU origins, GNSS-only scores true antenna residuals, and t_start30 is honored. Outages are half-open[60,90),[120,180); the single180s recovery observation is separately exposed, not credited to outage accuracy. The last logged in-gap epoch is179.9s; pure INS propagates a full60s to180 before recovery.

InEKF filters.py:233–254 uses right-invariant navigation errors and additive body biases, gravity coupling, Earth terms and bias cross-coupling. GNSS/body-velocity Jacobians at282–296 and initial coordinate transform at303–319 remain covered by existing finite-difference/parity tests. First-order Phi=I+Fdt and Qd=GQcGᵀdt at93–102 are nominal100Hz approximations, not exact continuous discretization. Joseph update and reset at105–118 retain covariance consistency after injection. No new equation defect demonstrated here. Existing rejected discretization/noise trials remain closed; they do not justify tuning Q blindly.

Initial state pipeline.py:131–144 uses truth-relative attitude, velocity and position draws and broad covariance with zero bias means. It is an optimistic warm start, not causal alignment. Bias/tilt information depends on maneuvers; gravity alone cannot identify yaw and straight-line motion cannot cleanly separate tilt/accelerometer bias. Prior maneuver and entry-state studies remain relevant; no bias observability claim follows from finite covariance alone.

Online NHC uses lateral/vertical body velocity with approximate zero-slip assumptions (pipeline.py constraint branches; sim.py:160–162 has nonzero slip). Optional ZUPT (pipeline.py:288–293) requires stop evidence and is disabled in the example. Forward speed is unmeasured, no odometer is enabled, and IMU/NHC continue through gaps. No reset to truth or smoother was added. Offline RTS would require manifold-consistent cross-covariance and separately labelled future-data metrics; a terminal gap has only the endpoint fix, so no offline gain is assumed.

At20m/s a60s outage spans1.2km. It is a reasonable stress test for this IMU grade, but a universal sub-metre online outage target is unsupported. Actual mean speeds are in the table. Provisional planning targets: available H roughly0.7–1.5m (<=1m is not universal), V roughly0.2–1.5m; NHC-aided30s H a few metres and60s H metres to tens of metres. Previous32-run exploratory evidence in docs/scheme_so3_half_turn_audit.md reaches24.308m60s H RMSE and53.371m peak. These ranges are descriptive scales, not validated acceptance bounds. Separate scenario limitations from the explicit-Earth bug: that bug does not explain the default outage tail.

## Residual plan and evidence stop

This delivery finishes inherited Earth-model work and provides fresh budgets. It does not close the whole scheme. Remaining work: implement and independently validate causal initialization with heading refusal and startup latency; jointly calibrate colored GNSS, speed-dependent IMU uncertainty, NHC slip and entry-state/bias covariance on frozen splits; specify and validate independent outage-tail targets on physical replay. No further structural filter change is justified by this bounded evidence. No external blocker is asserted; the stop is an evidence/scope limit. Independent review and full regression are recorded in reviews/earth_model_independent.* and pytest.txt. Keep DETECTION FAIL and confirmed_v08=false.

Reproduce: OPENBLAS_NUM_THREADS=1 /tmp/ccgins-accuracy-venv/bin/python -m scripts.audit_scheme --out <dir> --seeds 900 118001 118002; add --config /workspace/ccgins-v08/results/cc_scheme_audit/earth_model/config.yaml for explicit Earth. Run results/cc_scheme_audit/earth_model/verify.py with the parent checkout at /tmp/ccgins-earth-parent for paired trace and closure checks.
