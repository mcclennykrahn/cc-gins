# Learned streaming event and arrival semantics

`LearnedStreamingNavigator.from_truth(..., model=load('results/motionnet.pt'),
delay=1.01, max_buffer=4096, callback=handler)` initializes the simulation protocol.
Direct construction accepts R/v/p/P/noise and fixed positive `dt`; it does not need
truth. The default primary checkpoint is legacy CNN. No TCN promotion.

Push fixed-rate IMU in increasing event time. Acceleration/gyro at t are held over
[t,next_t). `delay` holds incoming IMU by event-time seconds; `get_state()` returns
the processed epoch, not arrival time. GNSS must arrive strictly before its epoch
has been processed. The base epoch-comparison tolerance is 1e-9 s; a fix
within that tolerance of a grid epoch is treated as coincident. Delayed and out-of-order arrivals are sorted by measurement
epoch; duplicate pending epochs, current/past processed fixes, nonfinite data,
nonpositive covariance and exhausted buffers raise explicit errors. There is no
rewind of already emitted states. Increase delay to accommodate known arrival lag.
Do not confuse offline replay from initialization with a fixed-lag smoother.

Off-grid GNSS splits the previous held IMU interval at its measurement epoch.
Main and DR filters both propagate those same subintervals; only main consumes
GNSS. Learned feature caches append exactly one original IMU and pre-interval
attitude per nominal increment. Splits never create network samples. Learned
updates stay on the nominal cadence. At a nominal epoch ordering is propagation,
wheel, learned pseudo, GNSS. Off-grid fixes precede later nominal motion updates.
Whole nominal intervals retain `dt` arithmetic; split intervals use actual elapsed
time. Exact parity evidence applies to matched grid schedules only. Splitting can
change discretization and historical attitudes, so off-grid covariance equality
with the index-based batch estimator is neither expected nor asserted.

Hybrid uses the existing forward ACI/correlation inflation plus fixed mounted NHC,
with standard-deviation/kappa/bias fallback. Wheel data use `push_odometry` before
its future nominal IMU epoch, explicit speed/sigma/mount/lever/validity; no holding
or interpolation. Wheels update both filters before learned inference. Learned
ZUPT remains rejected: GNSS stop-confirmation state is not yet integrated in this
subclass. Baseline streaming retains that supported path.

`max_buffer` bounds each IMU/GNSS/wheel queue and retained pseudo/GNSS diagnostic
history. It must cover ceil(delay/dt)+1. Histories retain the most recent events;
callbacks provide full processed history to a caller-managed sink. IMU callbacks
contain `epoch`, latest IMU `arrival_time`, `status='processed'`, and copied state.
GNSS callbacks contain measurement `t`, optional caller-supplied `arrival_time`,
`processed_arrival_time`, status, NIS, DR statistic, CUSUM, alarm and acceptance.
If GNSS arrival metadata is omitted it records the latest received IMU epoch
(or None before any IMU). Arrival metadata does not schedule processing. Callbacks
run synchronously after actuation; exceptions propagate and do not roll back the
processed epoch. Caller-retained callback data is outside the navigator's bound.

Call `flush()` only after all tail GNSS arrivals. It drains the remaining IMU,
without fabricating a later IMU clock. Future GNSS beyond the final IMU cannot be
applied. These APIs are single-threaded and do not promise hardware I/O.

# One-file offline replay

`ccgins.stream_replay.save_artifact(path, scenario, cfg)` writes NPZ schema v1:
full IMU t/accel/gyro, valid GNSS epoch/arrival/position/full covariance,
initial R/v/p/P, antenna lever, and JSON config/noise/dt/delay/source metadata.
No pickle is used. Epoch-zero GNSS is omitted to match the batch protocol.
The exporter uses nominal synthetic GNSS sigma; it is not a general receiver log
converter. Wheels are not included in v1. Replay loads all sensors from one file;
checkpoint weights are a separately selected model dependency.

```
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.stream_replay \
  results/proposal_alignment_stream_fifth/seed_900.npz
```

This is offline artifact replay only, with no live hardware or receiver support.
