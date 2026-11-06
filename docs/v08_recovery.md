# Opt-in integrity recovery

Use `Config(monitor='dual', exclude_on_alarm=True, recovery_epochs=3,
recovery_q=3.0)` in `pipeline.run` or `LearnedStreamingNavigator.from_truth`.
The loaded primary remains `load('results/motionnet.pt')` (legacy CNN). For a
minimal runnable loaded-checkpoint batch/stream example see
`results/v08_stage1/evaluate.py`; stream arrival contracts remain in
[stream_offgrid_semantics.md](stream_offgrid_semantics.md).

`recovery_epochs=0` preserves permanent latching. With recovery enabled, only
received GNSS fixes with finite nonnegative guard q at most recovery_q advance
the streak. A larger q resets it. Missing epochs neither advance nor reset it;
M fixes therefore need not be consecutive wall-clock seconds. Invalid q resets
the streak and raises ValueError. The completing fix remains excluded from main
GNSS update/calibration, and the next fix is eligible for ordinary gating. The
CUSUM accumulator clears on recovery and can alarm again. `alarm_time` retains
the first alarm. Batch `extra['recovery_times']` and stream GNSS diagnostics
`recoveries`/`healthy_streak` expose recovery. Existing `exclude_on_alarm=False`
is a diagnostic mode and does not enact exclusion.

Main and DR continue motion constraints during exclusion. DR is not reseeded
while alarmed. Batch motion ACI eligibility now uses current alarm state so it
can resume after a new healthy accepted reference and warmup, as stream already
does. This does not establish whole-window healthy DR reset semantics.

These settings are explicit engineering mechanism settings, not validation-tuned
CUSUM thresholds. Large DR covariance can make agreement misleading; repeated
agreement is not proof an attack ended. Long-alarm drift may prevent recovery.
No authentication, false-alarm/hour guarantee, default-noise ramp reliability,
HPL, or nominal coverage follows from this feature. Detection remains a separate
failing V0.8 gate. Do not enable this automatically in historical primary configs.

ACI uses clipped alpha, finite sliding reference buffers, bounded kappa, correlated
GNSS-constrained references and outage inflation. It does not turn the
pre-existing fifth-batch coverage 0.691387 into nominal 0.95. That locked-protocol
under-coverage remains the explicit research budget until a validation-selected
change improves independent held-out measurements.
