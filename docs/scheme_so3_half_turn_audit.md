# Positioning scheme audit — half-turn mechanization and outage budget

Candidate `scheme_so3_half_turn`. Parent `1e7b1e6621c4076e4935f3124c5115f1cab1c6cd` matched the project manifest and live origin. The only initial dirt was the untracked prompt directory. The previous reviewed delivery and unpromoted trials were closed. This delivery is mid-audit. DETECTION remains FAIL and `confirmed_v08=false`.

## Measured error budget

Reference run used `configs/example_sim.yaml`, 180 s, 100 Hz IMU, 1 Hz GNSS and scoring at or after 30 s. Seeds 117001 and 117002 are held out. The table gives horizontal/vertical RMSE in metres. The largest absolute before/after position trace difference was 3.64×10⁻¹² m and the largest segment RMSE difference was 8.88×10⁻¹³ m. These differences are floating-point effects.

| Seed | Available 30–60 s | 30 s outage | Available 90–120 s | 60 s outage | 180 s recovery | All available | Overall |
|---|---:|---:|---:|---:|---:|---:|---:|
|900|0.989/1.186|1.488/0.677|0.768/0.474|11.016/0.640|1.387/0.378|0.887/0.902|7.019/0.763|
|117001|0.763/1.007|3.697/0.071|0.692/0.139|5.468/0.290|1.407/0.506|0.730/0.719|3.860/0.491|
|117002|1.010/0.624|2.291/0.946|0.741/0.877|3.210/0.963|1.387/0.789|0.887/0.761|2.341/0.884|

Long-gap shares of horizontal squared error were 98.46%, 80.23% and 75.12%. Seed 900 peaked at 27.684 m in the long gap. The earlier 7.173 m overall and approximately 28 m peak refer to the historical version. Seed 900 horizontal RMS in successive ten-second bins from 30 s was 1.34, 0.79, 0.71, 0.27, 1.58, 2.02, 0.89, 0.77, 0.61, 0.86, 2.15, 3.96, 7.09, 12.99 and 22.09 m. All per-seed H/V bins and signed errors for the final source are in `current_before/` and `current_after/` in the delivery evidence.

| Seed | GNSS-only H/V RMS, m | Raw INS 30 s H RMS / end, m | Raw INS 60 s H RMS / end, m | Mean gap speed 30/60 s, m/s |
|---|---:|---:|---:|---:|
|900|1.427/2.125|9.458/21.390|42.694/101.889|10.131/8.954|
|117001|1.385/2.136|13.889/31.463|49.625/98.897|0.000/17.188|
|117002|1.489/2.138|13.308/31.924|54.695/114.044|11.738/7.259|

Raw INS restarts at exact truth, then propagates with uncompensated noisy IMU. Fixed-attitude, small-angle one-axis endpoint scales at 30/60 s are 22.07/88.29 m for accelerometer bias (σₐT²/2), 6.42/51.37 m for gyro bias through gravity (gσᵦT³/6), 0.94/5.34 m for gyro ARW through gravity (gσ_ARW T^(5/2)/√20), and 0.16/0.45 m for accelerometer VRW (σ_VRW T^(3/2)/√3). These exclude rotation, vibration and bias random walk. GNSS-only stationary antenna RMS from 1 m horizontal per-axis white, 2 m vertical white and 0.5 m stationary multipath is 1.581 m horizontal and 2.062 m vertical. In simulation the 30 s correlation process starts at zero.

The half-open gaps are [60,90) and [120,180). The 180 s recovery fix is scored separately from the last in-gap log at 179.9 s. Online errors compare same-epoch IMU origins and GNSS-only errors compare antenna positions. The 60 s gap occupies the final third of the commanded scenario and about 40% of the scored span. It is a useful stress case for the simulated IMU, with 1.2 km of travel if speed were held at 20 m/s. The actual long-gap means varied widely. Provisional online targets are roughly 1 m H when available, a few metres over 30 s and metres to tens of metres over 60 s. These are planning scales and depend on the entry solution, vehicle motion and aiding. A universal sub-metre 60 s target has no support here.

## Correctness audit

- **New simulator-mechanization bug.** Parent `ccgins/geometry.py:27–32` used a nonsymmetric column of (R+I)/2 and forced the dominant axis component positive. For an angle just below π the sign of the principal rotation matters. The trace-only angle also loses precision near π. In `ccgins/sim.py:171`, the defective logarithm supplied the gyro increment; `ccgins/filters.py:43` then reconstructed the wrong rotation. On simulated seed 3506 at dt=25 s, step 4, the one-step noiseless rotation Frobenius error fell from 1.029×10⁻⁵ to 9.09×10⁻¹⁶. The fix at `ccgins/geometry.py:24–35` derives the unsigned axis from the symmetric Rodrigues term, determines its sign from the skew, and evaluates the angle by atan2. At exactly π either sign reconstructs the same matrix. The geometry regressions cover the old near-π cutoff, the new branch transition and exact π. The revised calculation is stable on either side of the new transition. The coarse-grid case tests numerical closure and is outside realistic IMU sampling.
- **Truth and IMU.** `ccgins/sim.py:150–175` uses body-to-navigation rotation matrices with ENU navigation, FLU body axes, fixed downward 9.81 m/s² and old-attitude specific force. Translation uses the start velocity and second-order position term before the right attitude increment, matching `ccgins/filters.py:33–46`. This path uses rotation matrices. Default simulation omits Earth rate; the optional fixed-local Earth rate and Coriolis omit geodetic transport. Discrete inverse synthesis under the same model tests algebraic closure. At `sim.py:43–56,184–192`, 30 deg/h converts to 1.4544×10⁻⁴ rad/s and 5 mg converts to 0.04905 m/s². ARW 0.3 deg/√h and VRW 0.1 m/s/√h divide by 60 for √seconds, with gyro degrees converted to radians. White samples scale with 1/√dt and bias increments with √dt. Vibration in simulation uses true speed; filter Q uses a nominal 12 m/s in `filters.py:55–66`.
- **GNSS model and outage clock.** `sim.py:195–220` uses antenna p+R·lever and evolves multipath on the actual sampled clock through invalid fixes. `filters.py:122–124,282–286` uses the same lever sign and frame. `pipeline.py:256–260,416–422` propagates to the measurement epoch and skips invalid fixes. Nominal measurement covariance is diagonal white at `pipeline.py:179`; correlation can affect slow position offsets and calibration. Previous colored-budget and paired calibration results remain applicable.
- **InEKF and initialization.** `filters.py:91–118,233–296,303–319` implements first-order Φ/Q, right-invariant position and body-velocity Jacobians, additive body biases, Joseph covariance and reset, and the initial covariance coordinate transform. These approximations suit the nominal grid and require separate validation for larger sample periods. Combined GNSS, inertial and motion updates provide maneuver-dependent attitude and bias information. Gravity alone leaves heading unobservable, and near-constant velocity makes tilt and horizontal accelerometer bias difficult to distinguish. The initial attitude, position and velocity draws at `pipeline.py:119–132` are relative to truth and paired with broader priors. The warm start does not represent causal acquisition.
- **Constraints and smoothing.** `pipeline.py:263–279,315–317` applies causal optional ZUPT and online lateral/vertical NHC. Realized lateral slip in `sim.py:154–162` makes zero lateral velocity approximate. Forward speed remains unmeasured in this example and ZUPT and odometry are disabled. During gaps the pipeline continues IMU and NHC. No smoother was used. An optional RTS result belongs in a separate offline comparison with explicit future-fix availability, particularly the single endpoint fix at 180 s.

## Residual plan

Validate a causal initialization that refuses heading when motion evidence is insufficient, with startup latency and independently varied trajectories. Evaluate a joint colored-fix, inertial process-noise and slip model using paired, frozen splits and position consistency, innovation and outage-tail metrics. Confirm performance on physical replay before operational acceptance. This round promotes only the demonstrated numerical correctness fix.

## Independent scenario ensemble

Seeds 117101–117132 were frozen before the runs. Each scenario and sensor seed has its own initialization draw (`init_seed=seed+1000000`). This inherited exploratory ensemble extends the benchmark to more motion and bias realizations and retains its optimistic truth-relative warm start. The final source was replayed on the three paired seeds above.

| Segment | H RMSE min / median / max, m | V RMSE min / median / max, m |
|---|---:|---:|
| Available | 0.443 / 0.973 / 1.367 | 0.172 / 0.523 / 1.055 |
| 30 s outage | 0.726 / 2.097 / 8.856 | 0.116 / 0.470 / 1.502 |
| 60 s outage | 1.126 / 3.647 / 24.308 | 0.110 / 0.395 / 1.476 |

In 24/32 runs the long gap contributed more than half of scored horizontal squared error. Available H RMSE was at most 1 m on 16/32 and at most 1.5 m on all 32. Four short gaps exceeded 5 m H RMSE and six long gaps exceeded 10 m H RMSE. The largest instantaneous long-gap H error was 53.371 m. With only 32 independent synthetic runs these are descriptive frequencies and maxima. They do not establish an operational tail bound.

Reproduce the paired run at the parent and candidate with `python -m scripts.audit_scheme --out <directory> --seeds 900 117001 117002` from the repository root. Reproduce the ensemble with `python -m scripts.audit_segment_ensemble --out <directory> --seeds $(seq 117101 117132)`. The inherited ensemble partitions use consecutive blocks of eight. `results/cc_scheme_audit/so3_near_pi/consolidate.py` checks the paired traces and summarizes the inherited partitions. Use the same numerical environment for comparisons.
