# Original-budget evaluation and GNSS calibration (2026-11-06)

The authoritative starting HEAD is `e1562135aacf67ce2f06076b3c202fa855da0791`.
See the [full-protocol measured report](../results/proposal_alignment_full/report.md)
for completion status, paired results and validation versus test decisions.
The protocol was locked before training in `configs/proposal_alignment_full.yaml`.
All data in this experiment are simulated IMU and **synthetic GNSS**. Historical
simulation/KITTI/i2Nav tables and checkpoints are preserved.

## Protocol and grouping

The original simulation training split is seeds 100–119, each 600 s; validation
500–503; tests 900–909; separate calibration 700–705. Nominal outages retain the
previous batch's starts/durations (150,30), (280,60), (420,120) s. Both networks
receive the same twelve epochs, Adam lr .002, cosine schedule, batch 128,
stride 20, initialization seed zero and one Torch/BLAS thread per process.
Smooth log variance was selected by the previous validation-only comparison
(e156213); this batch performs no new activation sweep. Checkpoint metadata
preserves architecture and activation; completion records include model hashes.
TCN training/standalone diagnostics use reference roll/pitch and simulated true
biases; deployment uses each filter's own estimated inputs. This remains a gap.

`learning.grouped_correlation_factor` computes `1 + 2 sum rho(lag)` separately
within each independent seed, centering each group's normalized errors independently
and stopping at its first non-positive correlation or the lag cap. It averages
those factors equally across drives, as in the existing simulation training script.
No cross-drive lag pairs are created. The legacy variable name `n_eff` denotes a
variance inflation / integrated correlation time, **not** an independent sample
count; that count is approximately N/factor. No formula change to the historical
estimator is justified by the latch pilot. Constant channels return factor one
in the new grouped API. The regression demonstrates that large between-drive mean
offsets do not inflate its estimate, unlike naive concatenation.

`grouped_calibration.json` uses only calibration seeds 700–705, reference-conditioned
network errors at the actual pseudo-observation cadence. It is an offline diagnostic,
not deployed GNSS-reference calibration or a proof of sufficient correlation correction.
The locked primary table retains corr_scale=[40,40,40] for both architectures;
no test-driven retuning. Group means are recorded separately because demeaning
correlation estimates does not remove predictive bias from navigation.

The prior no-attack dual-monitor false-alarm/latch pilot remains separate evidence.
A CUSUM alarm can exclude future GNSS and change the entire navigation trajectory;
this does not establish a correlation formula error or an atmospheric/bias-model fix.
No atmospheric or bias latch fix is claimed here. Those previous pilot results are
pre-existing; this batch does not rewrite them or tune thresholds on tests.

## Actual GNSS κ_g mapping

`GNSSACICalibrator` stores pre-update innovation vectors r, nominal full measurement
covariances R and projected state covariances C=HPHᵀ. A single scalar κ_g solves

`quantile[ rᵀ (C + κ_g R)⁻¹ r, 1−alpha_t ] = chi2_dim(1−alpha)`.

For GNSS position dim=3; dividing both sides by three gives the proposal's NIS/3
version. This is not three independent scalar χ²₁ calibrators, and it does not
multiply the full innovation covariance by κ_g. ACI updates alpha_t and solves by
bounded log-space bisection. Defaults match the learned calibrator's minimum
sample count and solve cadence. Full cross-covariances are retained.

Enable with `Config(gnss_aci=True)` (also accepted in CLI filter YAML). Before each
GNSS update, the previous κ_g scales the nominal R. Both DR/single monitor and
main-filter update consume this same scaled R. Only then can an accepted, unforced,
not-alarmed fix after the existing warmup enter the calibrator. Its newly solved
κ_g applies to the next fix. Missing/invalid, rejected, forced-reacquisition and
alarm-excluded fixes do not train the scale; outages freeze it. Epoch-sigma R is
used when configured. Sage-Husa adaptive R and scalar range coupling combinations
raise explicit errors. Huber remains subsequent robustification; stored NIS scores
are computed from the original pre-update innovation and C, not robustified NIS.

The calibration-seed pilot separately exercises the actual loaded legacy MotionNet
+ learned ACI + Huber path with monitor off and dual monitor on, with κ_g off/on.
It is exploratory and is not included in or selected using the primary test table.
The primary comparison keeps κ_g disabled to preserve the precommitted protocol.
Acceptance gating censors calibration scores; serial correlation, state errors,
monitor/calibrator feedback and alpha/κ bounds prevent any claim of exact nominal
coverage, attack robustness or closed-loop integrity. Increasing R can reduce a
monitor statistic, so attack/false-alarm threshold studies remain necessary.

## Scope

This batch chooses full training plus tested GNSS κ_g over an unfinished streaming
integration. Existing deterministic streaming fixed NHC/ZUPT/wheel parity remains
available. Streaming learning/ACI/DR-CUSUM, including GNSS κ_g, is explicitly
unsupported; there is no new full-chain batch↔stream covariance-equality claim.
Remaining proposal gaps include independent real-drive TCN validation, online
mounting/18-state estimation, healthy-window guard resets, alarm recovery, HPL and
exceedance reporting, B5 Student-t/VB, ≥20-test-run acceptance, and UrbanNav raw
pseudorange tightly coupled processing. M3/M4/M5 acceptance is not inferred from
restoring the simulation training budget.
