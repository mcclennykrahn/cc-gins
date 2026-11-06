# Gate-blocked follow-up: robust navigation before another threshold sweep

Stage 2 does not authorize a detector promotion. The declared validation campaign
already rejects every candidate. A three-point grid cannot prove that no CUSUM
setting can ever generalize. It demonstrates that sensitivity alone is inadequate
for this configuration: k=0/h=20 alarms on all four clean trajectories, while
k=3/h=50 and k=6/h=100 both lose seeds 501 and 503 before ramp onset. Higher
thresholds delay those clean alarms rather than eliminating them in this grid.
Do not interpret an early latched alarm as successful ramp detection.

The next proposed robust-nav fix is an opt-in, shared batch/stream guard-reference
health state, paired with the existing hybrid-to-fixed-NHC fallback. This is a
proposal, not an implemented improvement or accepted detection method:

1. Record reference eligibility across the *whole* DR reset interval (accepted,
   fresh, finite GNSS statistics, no forced acceptance, no alarm/recovery epoch).
   A healthy final fix alone must not erase earlier suspicious reference history.
   Missing/rejected fixes invalidate the continuous-health interval.
2. Hold bias-monitor/ACI reference updates when this reference is unhealthy;
   retain lateral/vertical fixed NHC and drop learned forward aiding using an
   explicit recorded reason. Never reseed DR during alarm or use growing guard
   covariance as source authentication. Bound and report elapsed reset age and
   positional covariance as diagnostic validity indicators.
3. Add shared-state unit tests for interruptions, forced accepts, stale epochs,
   alarm/recovery boundaries and all-window health. Add loaded-checkpoint
   batch/stream parity and clean/outage fallback tests with historical defaults
   unchanged. Record resets, fallback epochs and reason codes to distinguish a
   detector statistic failure from contaminated reference or motion-model bias.
4. Use a new preregistered validation-only experiment before any further tuning;
   keep 900–909 diagnostic outcomes out of implementation/threshold selection.
   Report clean/alarm exposure, all default-noise ramp delays/misses, navigation
   error, kappa saturation and healthy/outage coverage separately. Repeated use of
   these known held-out seeds is development evidence; require fresh independent
   seeds for a subsequent generalization claim.

Whole-window reset health may prevent contaminated reference propagation, but may
also lengthen DR drift and increase false alarms; it is not asserted to close
DETECTION. A permanently GNSS-free guard cannot eliminate IMU/model uncertainty.
This failure analysis advances reproducibility and specifies the next falsifiable
robust-nav change without shipping an unvalidated navigation behavior.
