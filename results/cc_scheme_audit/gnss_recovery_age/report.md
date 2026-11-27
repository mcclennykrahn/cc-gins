# Positioning audit and GNSS recovery age correction

The seed-900 60 s outage contributes 98.46% of horizontal squared error. The held-out seed 106002 instead gets 68.17% of its squared error in the 30 s outage. The fix corrects the logged age of the accepted fix at the same epoch as the posterior state. All paired position histories and error budgets match exactly.

## Measured online error budget

Paired runs used configs/example_sim.yaml, duration 180 s, dt 0.01 s, t_start 30 s and init_seed 0. Seeds 106001 and 106002 are held out. Each cell is horizontal / vertical RMSE in metres, before = after.

| Segment | 900 | 106001 | 106002 |
|---|---:|---:|---:|
| Available 30–60 s | 0.989 / 1.186 | 0.725 / 0.328 | 0.935 / 0.381 |
| 30 s outage | 1.488 / 0.677 | 1.259 / 0.176 | 7.746 / 0.556 |
| Available 90–120 s | 0.768 / 0.474 | 1.558 / 0.660 | 0.826 / 0.593 |
| 60 s outage | 11.016 / 0.640 | 2.306 / 0.947 | 3.636 / 0.390 |
| Terminal fix at 180 s | 1.387 / 0.378 | 0.572 / 0.708 | 2.205 / 0.396 |
| All GNSS-available | 0.887 / 0.902 | 1.215 / 0.522 | 0.886 / 0.498 |
| Overall | 7.019 / 0.763 | 1.742 / 0.688 | 4.194 / 0.471 |

The original cfad161 evidence is 7.173489 m overall and 28.257610 m long-gap peak. The current seed-900 baseline is 7.019110 m overall with 27.684186 m long-gap peak, reflecting previously reviewed changes. Seed 900 long-gap H RMSE climbs from 0.861 m in [120,130) to 22.089 m in [170,180). All 10 s H/V bins and signed position CSVs are in before/ and after/.

## GNSS-only and pure INS

| Seed | GNSS-only H/V RMSE | 30 s raw INS H RMSE / H endpoint | 60 s raw INS H RMSE / H endpoint |
|---:|---:|---:|---:|
| 900 | 1.427 / 2.125 | 9.458 / 21.390 | 42.694 / 101.889 |
| 106001 | 1.799 / 1.935 | 11.626 / 19.991 | 49.414 / 78.299 |
| 106002 | 1.727 / 1.791 | 9.989 / 23.991 | 49.131 / 127.249 |

GNSS uses 1 m horizontal noise per axis, 2 m vertical and 0.5 m correlated multipath. The stationary GNSS-only expectation is 1.581 m radial H and 2.062 m V. Multipath starts at zero and the finite samples vary. GNSS-only scores the antenna, while online scores the IMU origin against truth at the logged epoch. The 60–90 s and 120–180 s intervals are half-open. The 180 s accepted fix is scored separately and included in the overall and available aggregates.

Pure INS uses the same raw IMU from exact truth alignment at each outage boundary, with no corrections. At 30/60 s, analytic one-axis endpoint scales are 22.073/88.290 m for unestimated 5 mg accelerometer bias, 6.421/51.365 m for 30 deg/h gyro bias through gravity, 0.158/0.447 m for VRW and 0.944/5.338 m for ARW. These scales omit vibration, random-walk bias and alignment errors. Seed 900's realized mean speeds are 10.13 and 8.95 m/s. At the configured 20 m/s cap a 60 s gap spans 1.2 km, a reasonable online stress case for this grade. Aided available accuracy near 1 m is plausible, with larger recovery excursions. The earlier held-out 104002 case reached 17.142 m RMS and 31.025 m peak in the short gap. Tens of metres of horizontal peak error are credible for these stress cases. These draws do not establish tail guarantees or a sub-metre outage target.

## Correctness findings

- **Fixed GNSS recovery age.** At parent 6023acc, ccgins/pipeline.py:261 sampled age before applying GNSS at lines 416–494. Line 533 recorded that age alongside position and covariance from after the update. Accepted seed-900 fixes at 90 s and 180 s therefore reported ages of 31 s and 61 s. Lines 533–534 now log the age after the update. Invalid and rejected fixes still leave age running. The motion update still uses pre-update age because it runs before the GNSS update. The regression covers accepted, missing and valid-but-rejected epochs. The probe and matched budgets verify the diagnostic-only effect.
- **Truth and IMU.** ccgins/sim.py:158–169 inverts ccgins/filters.py:33–45, using old-attitude acceleration, second-order position, then rotation. The default uses ENU navigation and FLU body axes with body-to-navigation rotation matrices (ccgins/geometry.py:48–56); there is no quaternion path. Gravity is [0,0,-9.81] and Earth rotation is absent in default synthetic truth and navigation. Optional Earth/Coriolis uses a fixed tangent plane and omits transport. ccgins/sim.py:45–58 converts ARW and VRW from root-hours by dividing by 60. Lines 181–184 convert gyro bias from deg/h to rad/s and accel bias from mg to m/s² and add body bias random walks. Speed-varying vibration remains a model approximation when represented by nominal process noise.
- **GNSS.** ccgins/sim.py:191–212 uses the actual rounded IMU grid cadence for multipath and p+R·lever for the antenna, consistent with ccgins/filters.py:122–124 and 282–286. Simulator MP is colored and the default filter uses white R. Default monitoring is disabled and accepted fixes still use the innovation gate. Availability masks describe simulated signal availability, while the corrected age describes accepted aiding.
- **InEKF.** ccgins/filters.py:93–118 uses first-order propagation and Joseph covariance update followed by reset. Lines 233–268 use the right-invariant Jacobians with additive body biases. Prior finite-difference, discretization and observability evidence is preserved in the earlier scheme reports. Maneuvers affect bias observability, with forward drift weakly constrained by NHC alone. This round establishes no new equation or process-noise correction.
- **Constraints and alignment.** ccgins/pipeline.py:312–317 applies lateral/vertical NHC. Simulated slip and vertical motion at ccgins/sim.py:152–156 limit the zero-component approximation. ZUPT is disabled here; when enabled the causal dwell and stop guard are in ccgins/pipeline.py:264–295. No odometry input is supplied. ccgins/pipeline.py:119–132 initializes around truth with a fixed covariance and zero initial bias estimate, mapped into filter coordinates by ccgins/filters.py:303–319. This is a limitation of deploying the tested configuration.
- **Outages and smoothing.** IMU and NHC continue through invalid GNSS epochs (ccgins/pipeline.py:250–261,312–317,416–419); scoring uses same-epoch post-update state (522–534) and half-open windows (ccgins/metrics.py:18–33). There is no RTS smoother in the online path. A separately scored offline smoother would need manifold transition and cross-covariance bookkeeping; the terminal gap has only one future GNSS epoch.

## Scope and remaining work

Pending entry-transport work was reviewed, regression-tested and delivered as 6023acc. Its signed decomposition shows seed 900's long-gap H RMSE of 11.016 m compared with 0.986 m of entry position-plus-constant-velocity transport and 10.612 m of remainder. Subsequent motion constraints, dynamics and correlated cross terms are part of that remainder. The result supports investigating entry attitude and bias together with subsequent constraint errors; it does not identify a single dominant causal term.

This is a mid-audit delivery. Remaining work is sensor-only alignment with uncertainty/refusal; controlled attitude/bias/NHC interventions that isolate causes under the same inputs; joint noise and bias calibration; and a predeclared independent tail study. No structural filter change was justified by this timing finding. DETECTION remains FAIL and confirmed_v08 remains false.

## Reproduction

From the repository root with PYTHONPATH=. and one BLAS/OpenMP thread, run scripts/audit_scheme.py --seeds 900 106001 106002 --out OUTPUT and the evidence directory's probe.py before and after the code patch. The compact summary contains the probe and segment measurements. The review record and full pytest summary are under /workspace/ccgins-v08.
