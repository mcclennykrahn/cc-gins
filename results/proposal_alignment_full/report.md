# Original 20-train paired evaluation (2026-11-06)

**Original training budget resolved: YES. Both architectures completed training on all 20 seeds 100–119 × 600 s.** This restores the original simulation training budget; it is not full M3 acceptance. All 10 paired tests 900–909 and validations 500–503 completed. All IMU and GNSS data are synthetic.

Baseline `e1562135aacf67ce2f06076b3c202fa855da0791`; protocol/harness commit `f0c8e55`. Historical published tables and checkpoints were not rewritten.

## Locked protocol

The YAML was locked before training. Equal Adam lr .002, cosine schedule, 12 epochs, batch 128, stride 20, initialization seed 0, one Torch/BLAS thread per process. Both networks receive the same windows/optimizer steps. Smooth variance was preselected in e156213 validation; no new activation or test-based configuration tuning. TCN has bias-compensated IMU plus reference roll/pitch during training; legacy has raw six-channel IMU. This compares those input/architecture packages. Runtime need not be equal for differently sized networks.

Config SHA256: `68cf4613a017b5a3c15d785e46a0f2bac11015425c39b2297b9f221855d3be33`. Frozen protocol and per-architecture completion/model hashes are in `training/`. Nominal outages (150,30), (280,60), (420,120) s match the prior locked batch. Calibration seeds 700–705 are separate and never used for network training/architecture selection.

Both learned methods use the same inherited hybrid fixed lateral/up NHC, learned ACI, corr_scale=[40,40,40], Huber and DR-CUSUM settings. These are fixed research settings, not calibrated integrity thresholds. GNSS κ_g is disabled in this primary table. NHC is a supplemental control with ACI/monitor disabled and no learned forward channel.

Network NLL includes the Gaussian constant, averaged over channels/windows, using reference-conditioned inputs. Navigation uses live estimated attitude/bias for each filter separately. Truth initializes the existing shared protocol and scores errors. Navigation metrics start at 30 s; outage coverage includes outage epochs. Values below are unweighted means across seed metrics.

| Method | RMSE_h m | Network NLL | Nav coverage | Outage coverage | notes |
|---|---:|---:|---:|---:|---|
| legacy_cnn (selected) | 5.153636 | -2.694522 | 0.739151 | 0.789619 | smooth, locked settings |
| proposal_tcn (selected) | 31.536755 | -2.940682 | 0.703280 | 0.756095 | smooth, locked settings |
| nhc (control) | 3.269130 | — | 0.712752 | 0.828000 | no learned ACI/monitor |

## Validation versus held-out test

| Architecture | Validation RMSE_h m | Test RMSE_h std m |
|---|---:|---:|
| legacy_cnn | 4.060636 | 4.025420 |
| proposal_tcn | 4.042560 | 30.014913 |

Validation-ranked architecture: **proposal_tcn**. Lower paired learned-method test RMSE: **legacy_cnn**. Test TCN minus legacy: **+26.383119 m**.

**Legacy remains the validated working primary. TCN was not promoted.** Validation ranking does not establish universal superiority; neither network is a full navigation integrity guarantee. NHC control is reported honestly and has different monitor/ACI behavior.

## Per-seed paired tests

| Seed | Legacy RMSE_h m | TCN RMSE_h m | NHC RMSE_h m |
|---|---:|---:|---:|
| 900 | 2.824461 | 3.100519 | 3.539593 |
| 901 | 1.745458 | 25.676659 | 2.325729 |
| 902 | 3.195904 | 61.612114 | 3.817212 |
| 903 | 14.226699 | 75.039624 | 3.785946 |
| 904 | 2.674717 | 2.788557 | 3.234764 |
| 905 | 10.709279 | 2.265838 | 1.625731 |
| 906 | 3.318102 | 17.665066 | 4.427595 |
| 907 | 2.585665 | 2.815730 | 4.125125 |
| 908 | 7.564319 | 42.355860 | 1.489760 |
| 909 | 2.691755 | 82.047582 | 4.319843 |

## Calibration, GNSS κ_g, and limits

See [implementation and scope](../../docs/proposal_alignment_full.md), `grouped_calibration.json` and `gnss_aci_pilot.json`. Grouped n_eff diagnostics use separate calibration seeds, with within-seed centering/lag pairs and equal-seed averaging. They do not retune the locked correlation factor or repair the previously isolated dual false-alarm/latch issue. GNSS κ_g is implemented and tested as an opt-in vector-NIS, next-fix scale in the actual batch pipeline. The calibration-only pilot is separate from the paired primary results. No HPL, attack-recovery, nominal-coverage or full-chain streaming guarantee is claimed.

Remaining gaps: streaming MotionNet/ACI/DR-CUSUM and covariance equality for that full chain, independent real-drive TCN validation, reference/filter input conditioning, calibrated integrity thresholds, online mounting/18-state, healthy-window reset/recovery, HPL/exceedance, B5 VB/Student-t, ≥20-test-run acceptance and UrbanNav raw-pseudorange TC. Original training-budget restoration alone does not close M3.

## Commands run this session

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u -m ccgins.alignment_full
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_full/supplement.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q
.venv/bin/python results/proposal_alignment_full/build_report.py
```

Training/navigation output is in `run.log`; all seed measurements in `metrics.json`. Full pytest output is in `pytest.log`. All reported new measurements were generated by these local commands.

## Separate calibration measurements

| Architecture | Grouped corr_scale forward / lateral / up |
|---|---|
| legacy_cnn | 43.684626 / 45.810802 / 25.910023 |
| proposal_tcn | 21.168829 / 34.916304 / 25.504519 |

These reference-conditioned seed 700–705 diagnostics were not substituted for the locked primary corr_scale. Checkpoint class, activation, parameter count and SHA256 are recorded with them.

| Calibration seed 700, 120 s | Monitor | κ_g enabled | RMSE_h m | Eligible κ_g samples | Final κ_g |
|---|---|---|---:|---:|---:|
| outage 80–100 s | none | False | 1.272512 | 0 | 1.000000 |
| outage 80–100 s | none | True | 1.271768 | 69 | 0.686436 |
| outage 80–100 s | dual | False | 1.272512 | 0 | 1.000000 |
| outage 80–100 s | dual | True | 1.271768 | 69 | 0.686436 |

This short calibration-only pilot demonstrates actual batch wiring, not navigation improvement or attack/integrity acceptance. Disabled κ_g has no calibration buffer; final scale one denotes no adaptation.

Full pytest: **130 passed in 56.48s**. Original tests retained; no regression-masking edits.
