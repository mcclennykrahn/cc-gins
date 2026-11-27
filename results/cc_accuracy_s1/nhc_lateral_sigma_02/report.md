# Lateral NHC uncertainty trial

NOT_IMPROVED; unpromoted. Defaults unchanged. Self-check: 16 unique finite paired rows; development baseline metrics match prior campaign except runtime_us.

| Split | Baseline mean RMSE (m) | Candidate mean | Baseline pooled | Candidate pooled | Worse seeds |
|---|---:|---:|---:|---:|---|
| development | 5.350608 | 5.557925 | 5.587421 | 5.782360 | [900, 901, 902] |
| held_out | 6.044694 | 6.073293 | 8.467377 | 8.439114 | [39302, 39303] |

| Seed | Baseline horizontal RMSE (m) | Candidate |
|---|---:|---:|
| 900 | 7.173489 | 7.435359 |
| 901 | 5.619521 | 5.702798 |
| 902 | 3.258814 | 3.535617 |
| 39301 | 2.441497 | 2.170648 |
| 39302 | 2.126946 | 2.391493 |
| 39303 | 6.859417 | 7.341302 |
| 39304 | 1.514010 | 1.442867 |
| 39305 | 17.281602 | 17.020155 |

Both development and held-out mean RMSE worsen. Held-out pooled RMSE slightly improves but does not meet the predeclared combined criteria. Synthetic-only, five held-out seeds, no real-data or detection validation. No retuning. DETECTION remains FAIL; confirmed_v08=false. See paired_metrics.json for coverage and ANEES.
