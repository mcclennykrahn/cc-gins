# Fresh GNSS grid completion budget

Parent 8da5e8e and candidate use configs/example_sim.yaml, 180 s, t_start=30 s, alignment seed 0. Complete budgets and signed CSV histories match exactly for all three seeds. No accuracy improvement attributed to rate validation.

| Seed | Segment | H RMSE before=after (m) | V RMSE before=after (m) |
|---|---|---:|---:|
|900|available_30_60|0.989350|1.185786|
|900|outage_60_90|1.488475|0.676636|
|900|available_90_120|0.767795|0.474157|
|900|outage_120_180|11.016203|0.640390|
|900|terminal_available|1.386857|0.378029|
|900|available|0.886598|0.902407|
|900|overall|7.019110|0.762560|
|94001|available_30_60|1.157889|1.041641|
|94001|outage_60_90|3.316451|0.951517|
|94001|available_90_120|0.739974|0.359104|
|94001|outage_120_180|2.588359|0.095524|
|94001|terminal_available|2.112902|0.186521|
|94001|available|0.974675|0.778482|
|94001|overall|2.292757|0.653651|
|94002|available_30_60|1.060875|0.270729|
|94002|outage_60_90|1.380315|0.144792|
|94002|available_90_120|1.149593|0.901273|
|94002|outage_120_180|1.859111|0.504815|
|94002|terminal_available|0.250442|0.477786|
|94002|available|1.105250|0.665159|
|94002|overall|1.500505|0.532174|

The seed900 long outage contributes 98.462% of H SSE, confirming the original baseline diagnosis. Held-out seed94002 available H RMSE is 1.105m: <=1m is not a universal gate. Seed94001 short-gap RMSE exceeds long-gap RMSE. These single realizations do not establish tail bounds.

Original baseline and complete unit/frame/filter/constraint/scoring audit: ../report.md. Current GNSS validation and controlled-entry Q evidence: ../gnss_grid_rate/report.md. Pure INS, GNSS-only and 10-second bin values are in fresh_budget.json. Signed error CSVs and parent budget are external evidence under /workspace/ccgins-v08/results/cc_scheme_audit/grid_completion_fresh/.

Remaining audit: pre-gap/in-gap Q factorial; joint colored-multipath/slip/bias covariance calibration; more realistic alignment; independent Monte Carlo before adopting tail gates. Optional RTS stays a separate offline track. No external blocker; full scheme sign-off remains incomplete. DETECTION FAIL, confirmed_v08=false.
