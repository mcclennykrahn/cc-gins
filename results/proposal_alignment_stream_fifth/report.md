# Fifth batch: learned streaming, offline replay and original test seeds (2026-11-06)

Baseline: `40b412d65e44aa80e3fa0b8417d3771e0b88abbd`; local Linux, gpt-6.1-sol. All new navigation data are simulated IMU and **synthetic GNSS**. No retraining, remote agents or hardware I/O.

## Implemented API and parity scope

LearnedStreamingNavigator now accepts bounded delayed, out-of-order and off-grid GNSS. Shared base propagation splits held IMU intervals; main and DR follow the same subintervals. Network caches retain one original IMU/pre-interval attitude per nominal sample. Nominal-grid arithmetic and pseudo cadence are retained. Hybrid forward ACI + mounted fixed NHC, correlation inflation, std/kappa/bias fallback and mounted wheel updates are supported. Wheels precede learned inference and update both filters. Learned ZUPT remains explicitly rejected; the baseline path supports it.

[Builder/constructor/push semantics](../../docs/stream_offgrid_semantics.md) specify epoch ordering, delay, errors and cache behavior. IMU/GNSS/wheel queues and diagnostic histories are bounded by max_buffer. Histories retain recent records; synchronous processed callbacks expose arrival metadata, event time, status, copied states and GNSS NIS/DR/CUSUM. Overflow raises BufferError; malformed or too-late input raises an explicit exception. Already processed epochs are never rewound.

Tests retain the pre-existing 1e-10 matched-grid learned comparisons and unchanged base parity tests. New checks cover explicit off-grid main/guard subinterval propagation, delayed/OOO arrival equivalence, hybrid/wheel batch parity, post-warmup mounting/bias fallback, buffer errors, callbacks and one-file replay. No tolerances were loosened. The initial hybrid grace-period mismatch was fixed in code.

## NIS / dual DR-CUSUM ramp evidence

`spoof_probe.py` uses a stationary synthetic, low-noise mechanism fixture with the canonical legacy model in learned_nhc mode. A 1 m/s east ramp starts at event t=5 s and is present through t=13 s (maximum 8 m); afterward measurements return to clean zero. Every GNSS arrives exactly 100 IMU samples (1 s) after its event, with processing holdback 1.01 s. CUSUM k=.5, h=7; noise densities and initial covariance are explicit in the script. This is a constructed detector mechanism check, not realistic sensor-distribution validation.

Clean alarm: None; attacked alarm event: 8.0 s. Event-time delay from ramp onset: 3.0 s; arrival-clock label available at approximately 9.01 s. No earlier alarm is observed. All clean-control fixes and all pre/post-attack fixes in the attacked diagnostic run are accepted. NIS, DR q, CUSUM and alarm flags are retained in spoof.json.

This probe uses exclude_on_alarm=False and a deliberately open main NIS gate to isolate monitoring and clean post-attack acceptance. Thus alarm/fake labeling does not mean automatic exclusion or recovery. A second ramp pass with exclude_on_alarm=True verifies triggering-fix exclusion at the same event time; the retained gross-injection test also remains as an extreme rejection regression. Post-attack fixes stay excluded in the latched exclusion pass, so only the diagnostic pass demonstrates clean post-attack acceptance. Latching, healthy-window reset and recovery are not repaired. An exploratory 1 m/s ramp with default noisy-IMU scenario 713 did not alarm (h=15, 25 s); reducing declared GNSS sigma without changing its simulated noise produced clean alarms. Neither is presented as success. No calibrated FA/hour or general ramp-detection guarantee is claimed.

## Offline one-file replay

`seed_900.npz` contains the full 600 s IMU and valid GNSS plus arrival epochs, covariances, initialization, noise and JSON config. SHA256 is in metrics.json. It is committed as the representative full artifact; seeds 901–909 are reproducible local artifacts, excluded by the repository NPZ rule. The checkpoint is a separate model dependency. Schema v1 intentionally has no wheel/hardware decoder. Arrays are decompressed once before timing.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.stream_replay results/proposal_alignment_stream_fifth/seed_900.npz
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_stream_fifth/validate.py
```

The restored short demo is `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_stream_fifth/demo.py`: canonical legacy + hybrid + ACI/dual monitor, seed 715, 40 s, outage 34–39 s. It writes short_demo.npz/short_demo.json; this supplemental demo is not the headline validation.

**Offline artifact replay only; no hardware I/O promise.** Replay starts from stored initialization and never revises processed epochs. Future GNSS beyond the final IMU is an explicit replay error.

## Actual original 10-test-seed validation

Canonical checkpoint: `results/motionnet.pt`, SHA256 `e2495238df1cf3093155f26cec929d2341f03abb489f04b8acb9dd6a9f515445`. It was read only. Filter settings come from the pre-existing `configs/proposal_alignment_full.yaml`: hybrid, ACI, corr_scale=[40,40,40], Huber 1.5, dual DR-CUSUM h=50/k=3, GNSS ACI disabled. No test-based tuning. Each seed uses 600 s at 100 Hz, original outages (150,30), (280,60), (420,120) s and initial seed 0. Metrics start at 30 s. GNSS epoch zero is excluded in both paths. Arrival lag is 1 s; holdback 1.01 s. This historical canonical model is not substituted with the later third-batch smooth legacy checkpoint; prior published values are not reused as current measurements.

| Seed | Stream RMSE_h m | Batch RMSE_h m | Stream coverage95_h | Batch coverage95_h | Mean ms | p95 ms | Hz |
|---|---:|---:|---:|---:|---:|---:|---:|
| 900 | 22.507075 | 22.507075 | 0.833187 | 0.833187 | 0.2939 | 1.1780 | 3391.3 |
| 901 | 1.728970 | 1.728970 | 0.500263 | 0.500263 | 0.3129 | 1.2536 | 3186.1 |
| 902 | 37.812479 | 37.812479 | 0.398351 | 0.398351 | 0.3011 | 1.2177 | 3311.0 |
| 903 | 13.217814 | 13.217814 | 0.954745 | 0.954745 | 0.3075 | 1.2364 | 3241.8 |
| 904 | 2.897335 | 2.897335 | 0.777758 | 0.777758 | 0.3164 | 1.2592 | 3150.7 |
| 905 | 1.581675 | 1.581675 | 0.670409 | 0.670409 | 0.3154 | 1.2529 | 3161.2 |
| 906 | 3.924896 | 3.924896 | 0.790037 | 0.790037 | 0.3140 | 1.2785 | 3174.4 |
| 907 | 2.583001 | 2.583001 | 0.764427 | 0.764427 | 0.3113 | 1.2439 | 3202.6 |
| 908 | 12.578399 | 12.578399 | 0.661287 | 0.661287 | 0.2965 | 1.2029 | 3362.6 |
| 909 | 2.836805 | 2.836805 | 0.563410 | 0.563410 | 0.3180 | 1.2585 | 3134.7 |

Completed seeds: 10. Arithmetic mean RMSE_h stream/batch: 10.166845 / 10.166845 m; coverage95_h: 0.691387 / 0.691387. These are measured coverage fractions, not nominal-coverage guarantees.

| All logged epochs: maximum absolute stream/batch difference | Value |
|---|---:|
| position_max_abs | 0 |
| velocity_max_abs | 0 |
| covariance_max_abs | 0 |
| position_covariance_max_abs | 0 |

Mean per-IMU arrival-call latency across equal-size seeds: 0.3087 ms. Worst per-seed p95: 1.2785 ms (not a pooled p95). Total-sample/total-wall throughput: 3229.3 Hz, 32.29 times the 100 Hz compute target. Each table p95 is measured over all IMU arrival calls in that seed, including due GNSS, inference, logging and state callbacks. Tail flush is included in throughput but separately recorded in metrics.json, not distributed into per-call quantiles. Loading/decompression is excluded. OpenBLAS/OMP/Torch threads=1; local tests and other runs shared the box during this measurement, with no CPU isolation or hard-real-time deadline guarantee. The intentional 1.01 s buffering delay is in addition to compute time, so throughput does not imply low end-to-end availability latency.

The major-state comparison logs position, velocity, full internal 15x15 covariance and transformed position covariance at every batch logging epoch. All numeric discrepancies are reported directly; no adjusted equality claim is made. Short matched-grid tests prove their own parity contract only. Off-grid updates intentionally change discrete propagation and have no index-based batch equality claim.

## Limits and bookkeeping

No HPL, failsafe/recovery guarantee, automatic spoof recovery, calibrated FA/hour, nominal closed-loop coverage, live hardware replay, real wheel validation, or independent real-drive generalization is established. M3/M4/M5 remain partial. TCN was not changed or retrained in this batch; **legacy remains primary, TCN is not promoted**. No GNSS kappa risk expansion or default GNSS ACI change.

Matrix counts remain {'complete': 5, 'partial': 17, 'experiment_substitute': 2, 'unimplemented': 4}.

Full pytest: `149 passed in 70.26s (0:01:10)`. Historical tables/checkpoints and prior smoke/next/full/stream metrics were not rewritten. New evidence is confined to this fifth directory.
