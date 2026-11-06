# Stage 4 frozen diagnostics

No detector, noise, attack, checkpoint or navigation tuning. Reuse Stage3 BASE,
validation seeds 500–503 and previously exposed diagnostic seeds 900–909, rates
0/0.2/0.5/1 m/s, 600 s runs, onset 300 s. These are NOT fresh held-out evidence.
Run parity and both flag settings. DETECTION remains FAIL; confirmed_v08=false;
TCN and CUSUM remain unpromoted.

`reason_events` counts calls to invalidate per reason over the object's lifetime.
Repeated IMU freshness checks and repeated boundary observations count again;
these are bookkeeping occurrences, not independent GNSS failures or durations.
`reason_intervals` counts each reason at most once in each assessment interval,
including the current unfinished interval. Reasons overlap. Both counters survive
begin(); existing interval-local reason_codes and closed-interval history remain.
Disabled flag counters are absent/empty, not proof of zero unhealthy events.

Misses include preattack latched alarms and no-alarm runs; explicitly report both.
An alarm exactly at onset is detected with zero delay. Clean alarms count runs
with an alarm, not alarm transitions. Exposure is nominal simulation duration.
No confidence/generalization claim. Coverage/kappa/reseed-age/DR covariance
remain unmeasured. Preserve all historical Stage3 evidence.
