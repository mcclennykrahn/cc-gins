# Adaptive GNSS covariance accuracy trial

NOT_IMPROVED; unpromoted. Only adaptive_r changed; production defaults preserved.

| Split | Baseline mean RMSE (m) | Candidate | Baseline pooled | Candidate pooled | Worse seeds |
|---|---:|---:|---:|---:|---|
| development | 5.350608 | 5.217436 | 5.587421 | 5.474076 | [900] |
| held_out | 3.104006 | 3.302775 | 3.129586 | 3.349018 | [49401, 49402, 49403] |

| Seed | Baseline horizontal RMSE (m) | Candidate |
|---|---:|---:|
| 900 | 7.173489 | 7.174571 |
| 901 | 5.619521 | 5.353776 |
| 902 | 3.258814 | 3.123960 |
| 49401 | 3.567377 | 4.236559 |
| 49402 | 2.632007 | 2.787946 |
| 49403 | 2.857480 | 3.150324 |
| 49404 | 3.596593 | 3.583190 |
| 49405 | 2.866574 | 2.755855 |

Example seed worsens slightly; held-out mean also worsens. Development mean improvement does not satisfy the predeclared combined criterion. Five held-out synthetic seeds in one scenario provide limited generalization evidence; no real-data or new detection validation. Existing covariance matching uses each innovation before its state update, and cannot isolate time-correlated multipath from model error. No retuning or algorithm changes. Full outage, coverage and ANEES metrics are in paired_metrics.json and summary.json. DETECTION remains FAIL; confirmed_v08=false.

Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. /tmp/ccgins-accuracy-venv/bin/python scripts/evaluate_gnss_adaptive_accuracy.py`.
