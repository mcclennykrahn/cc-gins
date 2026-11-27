# Positioning scheme audit — when motion constraints help

The original example simulation measured **7.173489 m horizontal RMSE** and a **28.257610 m peak** in the 60 s outage. That outage accounts for **98.468%** of the original horizontal sum of squared errors. After the previously pushed IMU noise and covariance reset fixes, fresh seed 900 replay measures **7.019110 m** overall, **11.016203 m** in the 60 s gap and **27.684186 m** peak. The gap still accounts for **98.462%** of horizontal squared error. Combined GNSS availability before any fixes scored **0.881984 m horizontal RMSE**. The results support both original hypotheses.

## Error budget and scoring

All new cases use configs/example_sim.yaml, 180 s at 100 Hz, and score at t ≥ 30 s. The filter logs the IMU-origin estimate against the same-epoch truth every 0.1 s. GNSS observes the antenna using p + R lever, with lever [0.5, 0, 1.2] m in FLU. GNSS updates fall after propagation and the 90 and 180 s fixes are available. The half-open outage windows exclude these fixes. The 180 s fix is reported separately. See metrics.json and the signed traces for 10 s H/V RMSE bins, peaks, end errors and covariance.

| Seed 900, alignment 0 | Original H/V RMSE (m) | Current H/V RMSE (m) |
|---|---:|---:|
| GNSS available, 30–60 s | 0.985 / 1.147 | 0.989 / 1.186 |
| Outage, 60–90 s | 1.544 / 0.652 | 1.488 / 0.677 |
| GNSS available, 90–120 s | 0.762 / 0.472 | 0.768 / 0.474 |
| Outage, 120–180 s | 11.259 / 0.666 | 11.016 / 0.640 |
| 180 s reacquisition | 1.389 / 0.430 | 1.387 / 0.378 |
| All scored samples | 7.173 / 0.755 | 7.019 / 0.763 |

Prior measured GNSS-only antenna residual for seed 900 is **1.427 m H / 2.125 m V**, including multipath. The stationary expectation is **1.581 m H / 2.062 m V** from 1 m per horizontal axis, 2 m vertical and 0.5 m correlated multipath. The initialized multipath transient and finite sample explain why the measured number need not equal the stationary expectation.

Prior raw-INS runs with exact truth initialization and unknown biases measured seed 900 horizontal endpoint drift of **21.39 m over 30 s** and **101.89 m over 60 s**. One-axis flat-motion endpoint scales from the configured IMU are 22.07 / 88.29 m for 5 mg accelerometer bias, 6.42 / 51.36 m for 30 deg/h gyro bias through gravity, 0.944 / 5.338 m for 0.3 deg/√h ARW and 0.158 / 0.447 m for 0.1 m/s/√h VRW. Simulator bias conversion is at ccgins/sim.py:179–180 and density conversion at ccgins/sim.py:43–52. The aided filter estimates some bias during prior GNSS exposure, and NHC continues during the gaps.

At 20 m/s the long gap spans 1.2 km, a reasonable stress case for this IMU grade. Seed 900 actually averaged 8.95 m/s across that gap. A planning scale is about 1 m H during GNSS availability, 1–9 m H RMSE during 30 s loss, and roughly 1.4–13 m H RMSE during 60 s loss across the nominal comparison runs linked here. Typical vertical errors with NHC fall around 0.1–2 m in these runs. The ranges are descriptive and should not become tail gates. Sub-metre online performance through 60 s would require substantially stronger independent aiding, bias calibration or a better IMU.

## Paired pre-entry and in-gap constraint ablation

The inherited targeted replay completed 18 rows. The new factorial reuses those cases and adds complementary schedules and two sensor draws. Seed 900 and comparison seeds 80301–80304 each use alignment seeds 0 and 1701, resulting in 70 traces. Each comparison uses the same truth, IMU and GNSS samples. Alignment variants share sensor draws. The sensor seeds therefore provide five draws total, four beyond the example. No tuning used those seeds.

The four cells for each gap are full NHC, NHC before entry only, NHC during the gap only, and neither. The pre-entry history covers all epochs from initialization through the last epoch before the gap, including the earlier outage when analyzing the long gap. Each control resumes nominal NHC at gap end. The experiment replays zero lateral/vertical measurements with fixed nominal variance using the precomputed pseudo-measurement interface (scripts/audit_nhc_outage.py:21–34 and ccgins/pipeline.py:313–325). It does not consume simulator truth measurements. A native fixed-NHC run matches the full replay bit for bit in logged position, velocity and position covariance. Each in-gap comparison shares an exact logged pre-gap prefix with the paired history.

| Seed 900 gap, alignment 0 | Full NHC | Before only | During only | Neither |
|---|---:|---:|---:|---:|
| 60–90 s H/V RMSE (m) | 1.488 / 0.677 | 4.872 / 3.273 | 0.924 / 1.163 | 12.682 / 2.043 |
| 120–180 s H/V RMSE (m) | 11.016 / 0.640 | 17.061 / 8.864 | 12.260 / 0.741 | 24.731 / 8.288 |

In-gap NHC reduced long-gap H RMSE in **10/10** seed/alignment pairs with pre-entry NHC and **10/10** without it. At fixed in-gap NHC, pre-entry NHC helped in **8/10** pairs. Short-gap in-gap NHC helped in **8/10** with pre-entry NHC and **6/10** without it. Short-gap horizontal benefit is less consistent. This establishes a causal benefit from long-gap motion updates in the simulator, beyond any effect on entry estimates. The ablation does not identify a slip model or imply additional accuracy from changing NHC variance.

## Correctness fixes and review

- **New scoring bug.** The parent at f2e5a2f, ccgins/cli.py:228, built detailed outage rows without the scoring mask. As a result, CLI JSON and the Markdown report could disagree with the shared summary in ccgins/metrics.py:28–37. The boundary fixture reported 100 m in the detailed row and 3 m in the summary. The fix at ccgins/cli.py:228 intersects both windows, yielding a 3 m detailed peak and omitting entirely unscored rows. At ccgins/cli.py:207–210 the existing empty-window error now precedes reductions. The fixture also excludes an artificially large reacquisition error at the outage end.
- **Same-configuration before/after.** All scored outputs match exactly for parent vs proposed CLI on all 70 traces. Both nominal example outages start after 30 s, and this patch changes reporting only. Full nominal results, per-segment before/after equality, and the regression fixture are recorded beside this report. There is no proposed online accuracy gain.
- **Inherited fixes.** 8251fe0 corrected IMU density/sample-interval handling at ccgins/filters.py:55–66, rounded GNSS multipath timing at ccgins/sim.py:191–203 and outage selection/naming at ccgins/metrics.py:28–37. 70f21b4 added post-injection covariance transport. The original and continuation reports linked below capture the original file/line evidence and paired seed results. Their other review artifacts and the DETECTION criteria remain preserved.
- **Audit coverage.** The original report covers truth mechanization and integration order, ENU/FLU and rotation matrices, fixed synthetic gravity and omitted Earth rotation, antenna lever arm, timing, bias conventions, InEKF process noise and Jacobians, initialization, NHC/ZUPT, outage and reacquisition behavior. Subsequent reports quantify reset/discretization, maneuver-conditioned bias observability, and the effect of colored multipath on covariance. ZUPT stays disabled in this configuration. Truth-perturbed alignment, planar motion and open-loop drift in unobserved directions limit transfer to field tests. No smoother was added. An optional manifold RTS benchmark would use future measurements, report offline separately, and has little future support in this terminal outage.
- **Independent numerical checks.** A separate calculation reconstructed all 70 traces, including covariance consistency, time bins, update counts and causal prefixes. The initial verifier confused a duration with an end timestamp. It was corrected and the full check passed. A separate boundary oracle checked the CLI fix. The previously unfinished 20-row NHC/multipath evidence also passed its numerical verification. Full final regression passed with 217 tests in 81.52 s. A separate implementation reviewer has not signed off on the pending scripts and CLI patch.

## Handoff and residual work

The current repository and origin/main are at f2e5a2f840ae868749500d4251a01dd197a3b8c1. The candidate edits are local pending independent implementation sign-off. Review the CLI diff and the two diagnostics, then commit only the intended paths if review and regression pass. The main structural hypothesis remaining is a colored GNSS state using continuous multipath propagation through outages and position cross-covariance, with analytic Jacobians and batch/stream parity before paired nominal tests. Short-gap variation also merits comparing causal slip-state modeling and forward-speed observability with existing odometer inputs if representative data are available. Independent sensor draws are needed before specifying tail gates. DETECTION remains FAIL and confirmed_v08=false.

- [Original budget and full-path audit](../report.md)
- [Discretization](../discretization/report.md), [covariance reset](../covariance_reset/report.md), and [maneuver observability](../maneuver_observability/report.md)
- [GNSS multipath calibration](../multipath_calibration/report.md) and [NHC/multipath factorial](../nhc_model/report.md)

Reproduce the new runs from the repository root with `OPENBLAS_NUM_THREADS=1 python -m scripts.audit_nhc_outage --factorial --seeds 900 80301 80302 80303 80304 --out OUTPUT`. Generated files include raw signed errors at the logged cadence and complete per-segment H/V scores for each held-out pairing.

## Completion addendum — 2026-11-27

The historical pending implementation sign-off above is now closed: independent review PASS in `/workspace/ccgins-v08/reviews/cc_scheme_nhc_completion_review.json`. All 70 factorial traces, 20 model traces and 70 parent CLI comparisons passed independent verification. Final regression: 217 passed in 95.91 s. Intended source and report files are committed in the delivery recorded by review.json. The broader structural audit remains open; DETECTION FAIL, confirmed_v08=false.
