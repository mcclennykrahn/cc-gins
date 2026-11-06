# Next paired evaluation — reduced train (2026-11-06)

**Reduced-train / not full M3 acceptance. All 10 test seeds 900–909 completed for both architectures and NHC; full nominal 30/60/120 s outages.** All IMU and GNSS data here are synthetic simulation. Historical published legacy tables/checkpoints were not rewritten.

Baseline: `58b3dc976046e4f2107b84e5eee8fff9a9bfe59a`. See [debug findings](../../docs/proposal_tcn_debug.md), [raw smoke rerun](../proposal_alignment_debug/smoke/metrics.json), and [per-channel probe](../proposal_alignment_debug/probe.json).

## Precommitted assessment

`configs/proposal_alignment_next.yaml` was locked before training: train 100–103 × 600 s, val 500–503, test 900–909 × 600 s. Calibration 700–705 untouched. Outages are (start, duration) = (150,30), (280,60), (420,120) s. Identical Adam lr .002, cosine schedule, 12 epochs, batch 128, stride 20, initialization seed 0, one Torch/BLAS thread per process. Three independent local navigation processes; completed runs retained across a scheduling restart. This reduces the train pool from 20 to 4, not test coverage or outage duration.

Config SHA256: `3b74b3dbf23bfeeb3f57073a206c5f04645b0bce3baf6ea4a66c30ea306dba57`. Only hard/smooth variance activation was selected, by mean validation Gaussian NLL pooled equally over architectures and validation seeds. Selected **smooth** before test navigation; scores: {'hard': -1.600951339120371, 'smooth': -1.7108198261316874}. Same selected hyperparameters and ACI for both architectures. No test-based tuning or ACI/correlation sweep. Model hashes, environment, resolved filters and every seed are in metrics.json.

Network diagnostics are reference-conditioned (TCN uses reference roll/pitch and true simulated biases; legacy raw IMU). Navigation uses live estimated filter attitude/current estimated biases, separately for A and DR B. Truth only initializes the shared existing protocol and scores errors. Network NLL is Gaussian NLL including .5 log(2π), averaged across three channels and windows; negative continuous-density NLL is valid. Navigation RMSE/coverage start at 30 s; outage coverage includes all outage epochs. Table values are unweighted means of seed metrics.

Paired learned settings: InEKF, hybrid fixed lateral/up NHC σ=.1, ACI α=.05/γ=.01/window=300/outage β=2/T_ref=60, corr_scale=[40,40,40], Huber=1.5, dual CUSUM k=3/h=50, reset=30 s. These inherited smoke thresholds are **not calibrated integrity thresholds**. NHC is a supplemental control with the same nominal sensor/filter/noise settings but ACI/monitor disabled and no learned forward channel.

| Method | RMSE_h m | Network NLL | Nav coverage | Outage coverage | notes |
|---|---:|---:|---:|---:|---|
| legacy_cnn (selected) | 4.137701 | -1.849681 | 0.718102 | 0.791095 | smooth; 10 seeds |
| proposal_tcn (selected) | 13.242062 | -1.669093 | 0.709718 | 0.810619 | smooth; 10 seeds |
| nhc (control) | 3.269130 | — | 0.712752 | 0.828000 | control, no ACI/dual |

Validation navigation means (architecture assessment only; no new settings selected):

| Method | Val RMSE_h m | Test seed RMSE_h std m |
|---|---:|---:|
| legacy_cnn | 11.925570 | 2.414972 |
| proposal_tcn | 6.495384 | 16.992875 |

Validation-ranked architecture: **proposal_tcn**. Test-reported TCN minus legacy RMSE_h: **+9.104361 m**.
**Legacy remains the validated primary working method; it has the lower test RMSE among the paired learned configurations with validation-selected settings. TCN is not established as an improvement / not primary.** Both the stacked legacy CNN and proposal TCN remain available. A better network likelihood does not establish a better closed-loop navigation method.

## Debug evidence and limits

The reproduced smoke aggregates exactly equal the frozen prior smoke metrics. `probe.py` shows TCN seed 900 NLL channels 4.919387 / 0.420892 / 35.277800, with vertical variance reaching exp(-12). Seed 901 vertical NLL is 24.418560. This explains the catastrophic standalone NLL tail; reference/serve mismatch cannot explain those reference-conditioned measurements.

Hard log-variance clipping has a zero-gradient region outside [-12,4]. The regression explicitly asserts zero hard-head gradients at raw -13/5 and restoring smooth-head gradients. Added optional smooth clipping with checkpoint metadata, preserving original hard-clipped models and old loaders. No objective/target/normalization redesign. The six-epoch isolated validation pilot changes TCN NLL 1.363208→1.340219; this is a pilot, not primary evidence.

| Reduced-train validation pilot | Mean network NLL |
|---|---:|
| legacy_cnn, hard | -1.715645 |
| legacy_cnn, smooth | -1.758699 |
| proposal_tcn, hard | -1.486257 |
| proposal_tcn, smooth | -1.662941 |

More training reduces the severe raw-NLL failure even with hard clipping. The pre-clamp tail probe finds no vertical lower-bound crossing on either smoke training seed, but 3.286979% on test 900; test 901 has large NLL without crossing -12. Thus lower-clamp dead gradients during training are not established as the cause of vertical collapse. Forward upper-clamp saturation is present on 82.932996%/96.080910% of smoke training windows. The evidence supports held-out tail overconfidence and a separate demonstrated clamp-gradient defect, not an assertion that smooth clipping alone cures TCN. β-NLL constants/channel averaging, bias ordering, ZYX roll/pitch, independent channel scaling and exclusive-window labels were checked without finding the suspected formula/preprocessing bug. TCN receptive field is 29 samples before global pooling; same padding uses only the supplied trailing window and is not pointwise causal.

Hybrid navigation consumes learned forward speed/variance only; its fixed lateral/up NHC does not consume the vertical variance responsible for the raw-NLL tail. A separate validation-only diagnostic retains a difficult seed 501, toggles monitor only, and scores live-input NLL. It is not used to change the locked primary settings:

| Pilot/debug seed 501 | Monitor | RMSE_h m | Alarm time s | Live NLL | Reference NLL |
|---|---|---:|---:|---:|---:|
| legacy_cnn | dual | 24.247199 | 121.0 | -1.464472 | -1.464473 |
| legacy_cnn | none | 1.372363 | None | -1.464472 | -1.464473 |
| proposal_tcn | dual | 9.787959 | 124.0 | -1.516533 | -1.512363 |
| proposal_tcn | none | 2.578018 | None | -1.513976 | -1.512363 |

See [nav_pilot.json](../proposal_alignment_debug/nav_pilot.json) for per-channel live diagnostics and final κ. False alarms in a no-attack simulation are failures, not attack-detection successes; a latched alarm excludes later GNSS and can dominate full-run RMSE. This pilot isolates monitor effects only on one validation seed. It does not calibrate CUSUM thresholds, establish a domain-gap remedy, or replace the paired table. No bias-subtraction/attitude-removal/filter-conditioned-training sweep was selected on tests.

## Per-seed primary results

| Seed | Legacy RMSE_h | TCN RMSE_h | NHC RMSE_h |
|---|---:|---:|---:|
| 900 | 3.052672 | 19.725385 | 3.539593 |
| 901 | 1.770391 | 1.882649 | 2.325729 |
| 902 | 2.588377 | 35.743197 | 3.817212 |
| 903 | 2.793989 | 10.855931 | 3.785946 |
| 904 | 6.444951 | 2.924641 | 3.234764 |
| 905 | 1.600333 | 1.626451 | 1.625731 |
| 906 | 5.360306 | 4.527306 | 4.427595 |
| 907 | 2.418846 | 2.889139 | 4.125125 |
| 908 | 8.051570 | 50.200119 | 1.489760 |
| 909 | 7.295573 | 2.045802 | 4.319843 |

## Remaining gaps

Full-chain **streaming learning+ACI+integrity is still incomplete**. This batch completes the ten-seed batch TCN attitude/navigation chain only; it does not establish proposal acceptance. Outstanding: full 20×600 s training, independent real-drive TCN/KITTI/i2Nav validation, reference/filter conditioning gap, calibrated correlation/CUSUM thresholds and recovery, GNSS κ_g, online mounting/18-state, HPL/exceedance rates, ≥20-run acceptance, B5 Student-t/VB, UrbanNav raw-pseudorange TC, smoothing and online learning. Existing robust GNSS/ACI/dual-monitor paths and historical published results remain intact.

## Reproduce and tests

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_next
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_debug/nav_pilot.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_next/build_report.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q
```

Full pytest: 124 passed in 64.17s (0:01:04). Six additional regression cases; original tests retained. `run.log` captures training, selection and every completed navigation run; the initial scheduling interrupt is retained. Historical tables/checkpoints have no git diff.
