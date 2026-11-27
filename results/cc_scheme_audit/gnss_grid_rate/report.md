# Whole-scheme follow-up: GNSS grid validation and fixed-entry outage noise isolation

Parent and remote checked at `8da5e8ed303d980053880a43183a47c4a852b99e`. The previous process-noise result is reviewed and pushed. The working tree initially contained only the untracked prompt directory. This round starts with a fresh example-sim run at the parent, then repeats after the rate validation fix. Sensor/trajectory seeds are 900 and new draws 93001–93004. Main replay uses CLI alignment seed 0, duration 180 s and scoring from 30 s.

## Error budget

Each entry is horizontal / vertical RMSE in metres, identical before and after on all five seeds. The signed time histories and the complete JSON budgets are byte-identical in the before and after directories. Each row is a single realization.

| Seed | Available combined | 30–60 s | Gap 60–90 s | 90–120 s | Gap 120–180 s | 180 s reacquisition | Overall |
|---|---:|---:|---:|---:|---:|---:|---:|
| 900 | 0.887 / 0.902 | 0.989 / 1.186 | 1.488 / 0.677 | 0.768 / 0.474 | 11.016 / 0.640 | 1.387 / 0.378 | 7.019 / 0.763 |
| 93001 | 0.880 / 0.229 | 0.642 / 0.279 | 2.286 / 0.407 | 1.066 / 0.165 | 2.240 / 0.523 | 1.028 / 0.355 | 1.833 / 0.405 |
| 93002 | 0.728 / 0.390 | 0.902 / 0.491 | 2.954 / 0.275 | 0.494 / 0.254 | 1.690 / 0.200 | 1.207 / 0.055 | 1.760 / 0.304 |
| 93003 | 1.136 / 0.390 | 0.845 / 0.507 | 0.464 / 0.240 | 1.363 / 0.219 | 4.764 / 0.440 | 2.037 / 0.358 | 3.104 / 0.387 |
| 93004 | 0.677 / 0.638 | 0.495 / 0.623 | 1.969 / 0.476 | 0.815 / 0.653 | 1.886 / 0.206 | 1.612 / 0.282 | 1.543 / 0.474 |

Seed 900 long gap contributes 98.462% of horizontal SSE. Its successive 10 s horizontal RMSE values are 0.861, 2.153, 3.957, 7.093, 12.995 and 22.089 m. The earlier 7.173489 m overall and 28.257610 m long-gap peak were measured at the original cfad161 baseline. See the [initial audit](../report.md) for its experiment and [covariance reset](../covariance_reset/report.md) for the subsequent paired changes. The 60 s gap dominates seed 900, but the 30 s gap has higher RMSE than the 60 s gap on three of the four new seeds.

GNSS-only seed 900 at the 61 available scored epochs gives 1.427 m H and 2.125 m V. With stationary 0.5 m Gauss–Markov multipath and independent 1 m horizontal axis / 2 m vertical white noise, the model implies 1.581 m H and 2.062 m V. The simulation starts multipath at zero. Raw INS restarted with exact truth and the same noisy IMU at each gap entry gives 9.458 / 42.694 m H RMSE over 30 / 60 s. Its endpoint H errors are 21.390 / 101.889 m, and endpoint V errors are 37.583 / 160.128 m. Mean simulated speeds are 10.131 / 8.954 m/s. At the configured 20 m/s cap a 60 s gap spans 1.2 km, a reasonable MEMS stress case. The actual seed 900 60 s gap covers about 537 m.

For a flat straight trajectory with zero initial pose/velocity error, one-axis 30 / 60 s endpoint scales are 22.073 / 88.290 m from constant accelerometer bias, 6.421 / 51.365 m from constant gyro bias through tilt and gravity, 0.158 / 0.447 m from VRW and 0.944 / 5.338 m from ARW. Those scales omit vibration, bias random walk, motion and aiding. Bias values are simulator turn-on standard deviations. Using the larger previously reported campaign alongside these draws, metre-class available H RMSE (roughly 0.5–1.5 m in the sampled segments), several metres in a 30 s gap with observed cases reaching about 11 m, and low-single to low-double-digit metres in a 60 s gap are reasonable engineering expectations. These are descriptive, with tail targets still open. Vertical aided errors in this table are roughly 0.2–1.2 m excluding the isolated reacquisition epoch.

## Code and scenario audit

- **Fixed GNSS grid validation.** Parent `ccgins/sim.py:189–194` validated the rounded step. At IMU dt 0.1 s it silently mapped 10.1 and 15 Hz to the 10 Hz grid, despite rejecting some higher requests. Zero and nonfinite requests also failed inconsistently. Candidate `ccgins/sim.py:189–196` validates the requested rate before rounding. Exact grid rate and legacy rounding of legal off-grid rates are preserved. Multipath continues to evolve at the actual rounded spacing. Regression in `tests/test_scheme_audit.py`.
- **Truth, synthesis and reference frames.** `ccgins/sim.py:100–169` creates a locally flat ENU trajectory with body FLU and ZYX body-to-navigation rotation matrices (`ccgins/geometry.py:48–56`). This path uses no quaternion ordering. The inverse synthesis uses old-attitude specific force, trapezoidal position equivalent to constant acceleration per step, and the relative rotation log. These agree with `ccgins/filters.py:33–45`. Default constant 9.81 m/s² gravity and no Earth rotation match in both; optional Earth rotation/Coriolis and fixed tangent frame are real-data features with transport neglected.
- **IMU and filter covariance.** `ccgins/sim.py:43–51,179–185` divides ARW and VRW by 60 after converting gyro degrees to radians, divides gyro turn-on bias by 3600 after converting degrees to radians, and maps accelerometer bias with 0.001 × 9.81. The requested values are 0.3 deg/√h, 0.1 m/s/√h, 30 deg/h and 5 mg. `ccgins/filters.py:55–66,93–118,229–295` matches amplitude conventions at nominal speed, uses first-order transition and process covariance, and transports posterior covariance through the injection reset. The right-invariant state carries body-frame additive biases. See [bias observability](../maneuver_observability/report.md) for the discrete tangent results. Forward drift and heading can remain weakly determined given the trajectory and NHC.
- **GNSS and score timing.** `ccgins/sim.py:199–210` and `ccgins/filters.py:122–124` use antenna position p + R × FLU lever arm. `ccgins/pipeline.py:250–260,417–422,522–532` advances IMU to the next epoch, then applies matching valid GNSS and logs position and truth at the IMU origin. Scoring starts at 30 s, outages are half-open, and 180 s is a separate available sample. GNSS measurement covariance uses white sigma; colored multipath is a model approximation tracked in [multipath calibration](../multipath_calibration/report.md).
- **Alignment, constraints and smoothing.** `ccgins/pipeline.py:119–132` samples around truth and uses a broader initial covariance, with optional bias variance overrides. It is an alignment proxy. `ccgins/pipeline.py:84–85,263–294,312–317` provides optional causal stop confirmation and NHC, with ZUPT disabled for this config. Lean, slip and vertical motion make NHC approximate. The filter continues IMU/NHC through invalid GNSS epochs. Forward-speed aiding is absent in this YAML. The scored output is the forward online pass. An RTS implementation would require a separately reported offline metric, and the final outage leaves only the terminal fix as future information.

## Fixed-entry process-noise experiment

The prior [whole-run comparison](../noise_schedule_isolation/report.md) varied the noise schedule during both GNSS-aided alignment and outages. Here the native schedule runs through the 120 s epoch, then oracle or estimated forward speed sets Q from the IMU interval beginning at 120 s. The script asserts bit-for-bit equality of attitude, velocity, position, both biases and full covariance at the first altered propagation, plus identical logged position errors through 120 s. The oracle is an offline diagnostic and prior-state uses the pre-propagation forward speed estimate. Both paths still use NHC throughout. Terminal GNSS is eligible at 180 s and outside the scored outage.

| Sensor seed | Alignment seed | Fixed H | Oracle H | Prior-state H |
|---|---:|---:|---:|---:|
| 900 | 0 | 11.016203 | 10.538171 | 10.569502 |
| 900 | 1701 | 10.700931 | 10.217376 | 10.248390 |
| 93001 | 0 | 2.239727 | 2.213543 | 2.215475 |
| 93001 | 1701 | 2.190729 | 2.161727 | 2.163799 |
| 93002 | 0 | 1.689772 | 1.708788 | 1.706651 |
| 93002 | 1701 | 1.762266 | 1.782395 | 1.780298 |
| 93003 | 0 | 4.764470 | 5.113220 | 5.098256 |
| 93003 | 1701 | 4.791727 | 5.119540 | 5.104557 |
| 93004 | 0 | 1.885556 | 1.894429 | 1.896032 |
| 93004 | 1701 | 1.882954 | 1.887856 | 1.889805 |

Each altered schedule reduces H RMSE in four of ten pairs, with median changes +0.006888 m (oracle) and +0.008664 m (prior-state). The two alignments reuse each sensor/trajectory draw. On seed 900 with alignment 0, the previous whole-run oracle / prior-state values were 12.016261 / 12.066813 m, so the sign reverses when pre-gap history is controlled. The comparison identifies a dependence on pre-gap scheduling and its interaction with in-gap updates. A pre-gap-only arm would separate these two effects. No estimator change is justified by these results.

## Remaining work

Use a full pre-gap/in-gap factorial with independent sensor and more realistic alignment draws. Separate colored multipath, true slip and residual bias contributions to calibration, and use held-out Monte Carlo before fixing tail gates. Any promoted causal schedule needs speed-uncertainty treatment and batch/stream parity. Optional smoothing remains a separate offline research track. DETECTION stays FAIL and confirmed_v08 stays false.
