# Maneuver-conditioned bias observability

Candidate `scheme_maneuver_observability` extends the whole-scheme inspection at parent `70f21b4`. The [original audit](../report.md) covers truth synthesis and integration order, IMU unit conversions, ENU/FLU and attitude conventions, gravity and Earth rotation, GNSS antenna sign and timing, filter equations, alignment, constraints and outage handling. The [covariance-reset continuation](../covariance_reset/report.md) updates the error-coordinate and discretization findings.

## Error budget

Fresh paired runs used `configs/example_sim.yaml:4–39`, 180 s, 100 Hz, 1 Hz GNSS, scoring from 30 s and initialization seed 0. Seeds 68433–68436 were newly drawn comparison seeds. Available windows are [30,60) and [90,120) plus the 180 s reacquisition. Outages are [60,90) and [120,180). The filter is scored at the IMU origin at the same post-update epoch. GNSS is scored separately at the true antenna location. All before/after budgets and signed error CSVs are byte-identical. Each table cell gives the common before and after H/V RMSE in metres.

| Seed | 30–60 s | 60–90 s outage | 90–120 s | 120–180 s outage | 180 s fix | Available pooled | Overall |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 900 | 0.989/1.186 | 1.488/0.677 | 0.768/0.474 | 11.016/0.640 | 1.387/0.378 | 0.887/0.902 | 7.019/0.763 |
| 68433 | 1.138/0.945 | 2.397/2.055 | 0.825/0.281 | 5.303/0.266 | 0.832/0.047 | 0.993/0.697 | 3.576/1.033 |
| 68434 | 0.985/0.583 | 2.965/0.404 | 0.900/0.330 | 2.588/0.376 | 1.650/0.337 | 0.945/0.473 | 2.189/0.423 |
| 68435 | 0.878/0.806 | 2.212/0.736 | 0.789/0.306 | 4.016/0.149 | 0.823/0.031 | 0.835/0.609 | 2.775/0.516 |
| 68436 | 1.119/0.505 | 1.678/0.261 | 1.169/0.372 | 6.102/0.988 | 1.492/0.748 | 1.145/0.445 | 3.996/0.695 |

The original seed-900 baseline at `cfad161` measured 7.173489 m horizontal RMSE and a 28.257610 m final-outage peak. At the current parent and after the diagnostic fix these are 7.019110 m and 27.684186 m. The final outage contributes 98.462% of seed-900 horizontal squared error. In seed 68434 the shorter outage contributes more squared error than the final outage. The new comparisons average between 2.15 and 19.02 m/s within individual outage windows. At the configured 20 m/s ceiling a 60 s outage spans 1.2 km, a reasonable online stress test for this grade.

The GNSS model uses white sigmas of 1 m per horizontal axis and 2 m vertical, plus 0.5 m correlated multipath per axis. Seed 900 raw GNSS measured 1.427/2.125 m H/V RMS, versus stationary-model expectations of 1.581/2.062 m. The simulator converts 30 deg/h turn-on gyro bias to rad/s and 5 mg accelerometer bias to m/s² at `ccgins/sim.py:178–185`. ARW 0.3 deg/√h and VRW 0.1 m/s/√h use density conversions in `ccgins/sim.py:43–52`.

With exact state at outage entry and no bias correction, seed 900 raw-IMU propagation measured horizontal RMS 9.46 and 42.69 m at 30 and 60 s, and horizontal endpoints 21.39 and 101.89 m. Vertical endpoints were 37.58 and 160.13 m. One-axis constant-bias endpoint scales are 22.1/88.3 m from acceleration and 6.4/51.4 m from gyro-induced tilt. The prior audit gives corresponding ARW, VRW and bias-walk terms. With GNSS and existing NHC, provisional H RMSE planning ranges are 0.7–1.5 m available, 1–5 m for 30 s and 3–15 m for 60 s. Around 0.2–1.5 m vertical while available and 0.2–2 m during outages are plausible here. These ranges allow occasional results outside them and carry no statistical guarantee.

### Seed 900 time evolution

| Ten-second interval start (s) | H RMSE (m) | V RMSE (m) |
|---:|---:|---:|
| 30 | 1.342 | 1.468 |
| 40 | 0.793 | 1.259 |
| 50 | 0.712 | 0.692 |
| 60 | 0.268 | 0.674 |
| 70 | 1.581 | 0.686 |
| 80 | 2.019 | 0.670 |
| 90 | 0.895 | 0.504 |
| 100 | 0.775 | 0.519 |
| 110 | 0.606 | 0.389 |
| 120 | 0.861 | 0.440 |
| 130 | 2.153 | 0.484 |
| 140 | 3.957 | 0.604 |
| 150 | 7.093 | 0.754 |
| 160 | 12.995 | 0.786 |
| 170 | 22.089 | 0.693 |

## Bias observability

The offline diagnostic propagates the native ESKF physical-error tangent using the derivative of the discrete mechanization. It evaluates the GNSS antenna and lateral/vertical body-velocity Jacobians along ideal inputs at scheduled measurement epochs. The online example uses InEKF. Physical perturbations in either coordinate system describe the same local model to first order. For each independent window, initial attitude, velocity and position are free nuisance parameters projected out by SVD. Bias columns are scaled by 30 deg/h and 5 mg. The weakest singular value is the minimum Euclidean norm of the stacked whitened residual induced by a one-unit scaled constant-bias combination, after optimizing the nuisance state. Actual and level straight controls share scheduled GNSS/NHC epochs and mean forward speed. The control changes speed variation, attitude and small constraint residuals together, so the comparison does not isolate turning alone.

| Seed 900 interval (s) | Integrated absolute body-z rotation (deg) | Weakest bias sensitivity | Straight control |
|---:|---:|---:|---:|
| 30–60 | 214.7 | 0.1574 | 1.51e-12 |
| 60–90 | 170.9 | 0.04022 | 1.44e-32 |
| 90–120 | 280.9 | 0.1818 | 1.51e-12 |
| 120–180 | 30.7 | 0.1347 | 9.03e-30 |

All straight NHC-only controls show two appreciable bias directions and four near-zero directions. On the simulated trajectories the weakest sensitivity was at most 0.226 in every outage window. The seed-900 lateral NHC truth RMS is 0.0163 and 0.0064 m/s in the two outages, versus a nominal 0.1 m/s constraint sigma. These nonzero, maneuver-correlated residuals matter when interpreting repeated updates. Whitening uses nominal sigmas and ignores process noise, colored multipath and temporal correlation in the constraint residuals. The result is local deterministic sensitivity and provides no posterior bias variance, global identifiability or measured gain from changing the trajectory. It gives no basis for another NHC sigma or cadence promotion.

## Verified diagnostic bug

Initial independent review found a spurious yaw direction in the Euler sensitivity design. Seed 900 at 60–90 s moved from nuisance rank 6 to rank 5 after using the exact discrete tangent in `scripts/audit_observability.py:28–40`. This fixes the offline diagnostic. A turning-motion regression checks the yaw gauge and compares all tangent columns against finite differences. The bias-direction JSON now records its standardized coordinates and SI scales.

`70f21b4:scripts/audit_scheme.py:40` divided by zero for exact horizontal replay. That emitted NaN, which is invalid standard JSON and could break ingestion of noiseless controls. The current `scripts/audit_scheme.py:40` records null for the undefined fraction and preserves valid zero RMSE. A targeted regression exercises the denominator-zero case. The two other new tests check nuisance projection and the straight NHC-only null modes. Navigation evaluation is unchanged.

## Remaining work

The next causal check is available-window covariance calibration across independent alignments and trajectories, separating colored GNSS residuals from serially correlated NHC mismatch. For maneuver design, compare matched trajectories with one excitation changed at a time and carry priors, process noise and colored measurements into a covariance or information calculation. Consider an independently measured forward-speed input if a lower outage target is required. An optional manifold RTS smoother remains an offline experiment with separate metrics and only one GNSS reacquisition epoch after the terminal gap. DETECTION remains FAIL and `confirmed_v08=false`.

Reproduce with `OPENBLAS_NUM_THREADS=1 PYTHONPATH=. python scripts/audit_observability.py --out OUTPUT.json`. Run `scripts/audit_scheme.py --out OUTPUT --seeds 900 68433 68434 68435 68436` at each revision for the online comparison. Delivery evidence includes both budget JSONs, the signed ENU traces and 40 sensitivity spectra. The evidence directory is `/workspace/ccgins-v08/results/cc_scheme_audit/maneuver_observability`.
