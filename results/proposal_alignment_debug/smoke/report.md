# Proposal TCN paired smoke (measured this session)

Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_experiment`.
Frozen config: `configs/proposal_alignment_smoke.yaml`; SHA256 `f9b62f7a9af63ac342ef9153c8fb03198c7443460caeccc1ce97e6580da263e5`. Resolved defaults are in metrics.json.
All data are synthetic simulation IMU + synthetic GNSS. Train seeds 100/101, validation 500, paired tests 900/901; 80 s, outage 45–65 s. Six epochs, Adam lr .002, cosine schedule, beta-NLL beta=.5, batch 128, stride 20, initialization seed 0, one CPU thread. No tuning after observing validation/test.
Network diagnostics use reference roll/pitch and true simulated biases for TCN; legacy uses raw IMU. This compares the proposal input/architecture package, not an isolated dilation ablation. Training uses the same reference-conditioned TCN inputs.
Navigation uses only live filter roll/pitch and current estimated biases, with trailing IMU windows; the DR guard infers separately from its own attitude/bias estimates; no test reference enters learning inference. Reference initializes the shared existing protocol and scores errors. Both networks use identical hybrid/ACI/outage-inflation/Huber/dual-monitor settings. Fixed correlation factor 40 and CUSUM h=50 are smoke settings, not calibrated integrity thresholds. NHC is a supplemental fixed-NHC control without ACI/dual monitoring.
Network NLL includes Gaussian constant and averages channels/epochs. Network coverage is scalar marginal raw-variance coverage. Navigation coverage is the 2D position ellipse; these are different quantities.

| Method | Navigation horizontal RMSE m | Position coverage | Outage coverage | Outage max m | Network NLL |
|---|---:|---:|---:|---:|---:|
| legacy_cnn | 3.001493 | 0.795409 | 0.775000 | 7.788132 | 1.938337 |
| proposal_tcn | 3.848627 | 0.681637 | 0.500000 | 10.899140 | 11.595938 |
| nhc | 3.963107 | 0.674651 | 0.500000 | 11.213616 | nan |

| Network | Forward/lateral/up RMSE m/s | Forward/lateral/up raw coverage |
|---|---|---|
| legacy_cnn | [11.67122406907983, 0.25518722814017125, 0.11867518930654758] | [0.8767383059418458, 1.0, 1.0] |
| proposal_tcn | [13.010410045623921, 0.08082143112986973, 0.05234950445642868] | [0.8938053097345133, 0.9292035398230089, 0.9058154235145386] |

Every seed and validation result is retained in metrics.json. This tiny undertrained smoke verifies a reproducible primary-path integration; it is not the full 10-seed table, proposal ≥20-run acceptance, independent road validation, or evidence that TCN improves accuracy. Historical published tables remain unchanged and use legacy CNN. GNSS κ_g, mounting state, HPL/recovery and streaming learned integration remain missing.
