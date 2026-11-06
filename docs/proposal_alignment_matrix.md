# Proposal alignment audit

Proposal evidence: `docs/project_proposal.md` §§3–9; improvement/literature documents describe intent, never implementation. Baseline audit committed as ecb70c8 before code changes; this version updates network evidence after the pullback. Counts are bookkeeping, not a completion percentage.

| Proposal item | Expected modules / behavior | Current reality | Status | Evidence |
|---|---|---|---|---|
| geometry §3.1/4 M0 | SE₂(3), Lie tools, numerical Jacobians | SO(3) Exp/Log/Jacobians and SE₂(3) injection; no standalone full SE₂(3) API. | partial | `ccgins/geometry.py`, `ccgins/filters.py`, `tests/test_geometry.py`, `tests/test_filters.py` |
| filters §3.2 M1/M2 | InEKF and ESKF shared mechanization | Both 15-state filters runnable; Joseph updates and navigation covariance conversion tested. Large 90-degree convergence acceptance and KF-GINS cross-validation absent. | partial | `ccgins/filters.py`, `ccgins/comparison.py`, `tests/test_filters.py`, `results/inekf_vs_eskf/report.md` |
| mount §3.1/3.4 | 18-state online δm mounting estimation | 15-state only; i2Nav opposite-half offline mounting fixed during evaluation. | experiment_substitute | `ccgins/filters.py`, `ccgins/i2nav_scenarios.py`, `results/i2nav/mount_calibration.json`, `tests/test_i2nav.py` |
| noise §3.2/4 | GM biases, second-order discretization, IMU YAML profiles | Random-walk biases, first-order Phi; dataclass noise and optional Earth correction. | partial | `ccgins/filters.py`, `ccgins/sim.py`, `tests/test_earth.py`, `configs/example_sim.yaml` |
| gnss §3.3 | LC position/optional velocity and lever arm | Position LC with lever arm supported/tested; optional GNSS velocity absent. | partial | `ccgins/filters.py`, `tests/test_filters.py`, `tests/test_robust.py` |
| robust §3.3 | Gate and robust Huber/Student-t GNSS | Huber plus chi-square gate wired via Config/CLI; forced reacquisition after repeated rejects; default robust off. | complete | `ccgins/pipeline.py`, `ccgins/filters.py`, `ccgins/cli.py`, `tests/test_robust.py`, `tests/test_cli.py` |
| network §3.4 M3 | 3-layer dilated TCN, roll/pitch, bias-compensated IMU, β-NLL heads | Baseline: plain six-channel stride CNN, no attitude/dilation. Pullback: MotionNetTCN 1/2/4 dilations, eight normalized IMU+roll/pitch channels, beta-NLL, live filter attitude/current-bias correction in batch; explicit sim-training and CLI architecture selection; legacy_cnn training default. Legacy weights still load. Original simulation training budget **resolved**: both arches trained on 100–119 × 600 s, same 12 epochs and smooth variance; full-protocol val/test evidence in the new report. Independent real-drive validation and M3 acceptance remain missing. | partial | `ccgins/learning.py`, `scripts/train_motionnet.py`, `tests/test_learning_pipeline.py`, `results/motionnet_val.json`, `results/kitti/motionnet_val.json`, `results/i2nav/motionnet_val.json`, `tests/test_proposal_tcn.py`, `ccgins/alignment_experiment.py`, `configs/proposal_alignment_smoke.yaml`, `results/proposal_alignment/report.md`, `configs/proposal_alignment_full.yaml`, `results/proposal_alignment_full/report.md` |
| datasets §3.4/5 | Sliding windows and real body-velocity supervision | Simulation/KITTI/i2Nav windows and labels; KITTI body velocity corrected using R-transformed ENU rather than horizontal vf/vl. | complete | `ccgins/learning.py`, `ccgins/data/kitti.py`, `ccgins/data/i2nav.py`, `scripts/train_motionnet_kitti.py`, `scripts/train_motionnet_i2nav.py`, `tests/test_learning_pipeline.py`, `tests/test_i2nav.py` |
| nhc §3.4/6 B1-B4 | Fixed NHC, learned NHC and learned full velocity | All batch modes selectable; hybrid/v2/v3 add forward rejection and fixed lateral/vertical NHC. | complete | `ccgins/pipeline.py`, `ccgins/cli.py`, `tests/test_hybrid_kitti.py`, `tests/test_learning_pipeline.py`, `results/outage_table.md` |
| aci §3.5 M4 | Per-learning-channel online κ and outage inflation | learned_vel calibrates three channels; hybrid/v2/v3 only forward; pre-pseudo reference is GNSS-constrained prior, t>30 and recent accepted GNSS. Clipped alpha/sliding window/bounded κ; correlated closed-loop reference. | partial | `ccgins/aci.py`, `ccgins/pipeline.py`, `tests/test_aci.py`, `tests/test_learning_pipeline.py`, `results/outage_table.md`, `results/outage_shift_table.md` |
| gnss_aci §3.5 | Online GNSS κ_g from vector NIS | Opt-in joint vector-NIS κ_g, pre-update C+κ_g R, next-fix scale, healthy accepted unforced references; tested batch path/calibration pilot. Integrity threshold/attack acceptance remains open; AdaptiveR is a separate Sage-Husa path. | partial | `ccgins/aci.py`, `ccgins/pipeline.py`, `tests/test_gnss_aci.py`, `results/proposal_alignment_full/gnss_aci_pilot.json` |
| coverage §3.5/7 M4 | Nominal long-run coverage and consistent closed loop | Published v3 full/outage simulation 0.699/0.833; shift 0.730/0.876; KITTI FT outage 0.681; i2Nav mounted v3 0.574, correlation-NHC 0.887. ACI is NOT a mathematical closed-loop integrity guarantee. | partial | `results/outage_table.md`, `results/outage_shift_table.md`, `results/kitti/outage_table.md`, `results/i2nav/outage_table.md` |
| dual §3.6 M5 | GNSS-free DR guard, CUSUM, healthy-window reset | Batch dual guard and single-channel comparison present; reset uses current health rather than proving entire-window health; alarms latch without recovery M epochs. | partial | `ccgins/integrity.py`, `ccgins/pipeline.py`, `tests/test_learning_pipeline.py`, `results/spoof_table.md`, `results/i2nav/spoof_table.md` |
| hpl §3.6/7 | HPL and HPE>HPL reporting | No HPL output or exceedance metric; NEES coverage is different. | unimplemented | `ccgins/metrics.py`, `ccgins/pipeline.py`, `ccgins/integrity.py` |
| stream §4 | Sensor→learning→ACI→integrity online assembly | Baseline parity retained. Learned stream adds off-grid/delayed/OOO GNSS, hybrid + wheel layering, bounded queues/history, processed callbacks and one-file offline replay. NIS/dual CUSUM retained; HPL/recovery and learned ZUPT remain gaps. | partial | `ccgins/stream.py`, `ccgins/stream_learning.py`, `tests/test_stream_parity.py`, `tests/test_stream_learning.py`, `results/proposal_alignment_stream_fifth/report.md` |
| cli §4 M6 | Configuration end-to-end assembly and reports | Simulation CLI supports learned checkpoint, ACI, dual monitor and robust GNSS via Config; scripts handle real datasets. | complete | `ccgins/cli.py`, `configs/example_sim.yaml`, `tests/test_cli.py` |
| simulation §5.1 M0 | Two MEMS grades; outages, jumps, colored noise, ramp attacks | Simulation noise/bias/vibration, colored GNSS and configurable outages/ramp; no comprehensive two-grade jump/NLOS benchmark. | partial | `ccgins/sim.py`, `ccgins/scenarios.py`, `tests/test_filters.py`, `results/outage_table.md`, `results/spoof_table.md` |
| kitti §5.2 | Disjoint drives; real IMU plus synthetic GNSS | Drive splits and test segments implemented; GNSS truth+noise/outage/spoof, NOT real received GNSS. | complete | `ccgins/kitti_scenarios.py`, `ccgins/data/kitti.py`, `scripts/run_kitti_experiment.py`, `tests/test_hybrid_kitti.py`, `results/kitti/outage_table.md` |
| i2nav §5.3 | Real MEMS/RTK cross-device, KF-GINS validation | Real RTK, four co-mounted IMUs, opposite time halves of ONE drive; no independent roads or KF-GINS comparison. | partial | `ccgins/i2nav_scenarios.py`, `scripts/run_i2nav_experiment.py`, `tests/test_i2nav.py`, `results/i2nav/outage_paired.md` |
| baselines §6 | B0–B5/Ours, A1–A4 | B0–B4 variants and ACI/inflation/filter/single-monitor ablations recorded. B5 VB/Student-t not present; Sage-Husa/Huber are different. No unified full Ours benchmark with GNSS κ_g/HPL. | partial | `scripts/run_outage_experiment.py`, `scripts/run_spoof_experiment.py`, `results/outage_table.md`, `results/spoof_table.md`, `ccgins/adaptive.py` |
| protocol §5/7 M6 | Locked splits, paired baselines, ≥20 MC, mean/std | Simulation 100–119 train,500–503 val,900–909 test,700–705 calibration. Only 10 test seeds; KITTI held-out drives; i2Nav same-drive temporal folds. Original 20×600 s training restored for both arches in the full protocol; ten paired test seeds, not ≥20-test-run acceptance. | partial | `ccgins/scenarios.py`, `ccgins/kitti_scenarios.py`, `ccgins/i2nav_scenarios.py`, `scripts/run_outage_experiment.py`, `results/outage_metrics.json` |
| metrics §7 | RMSE, velocity/yaw, %DT, NEES/NIS fit, HPL, FAs/hour, efficiency | Position RMSE/outage endpoints/NEES/coverage/runtime; spoof delays and detection counts. Missing unified yaw/%DT/NIS-fit/HPL/memory/FA-per-hour suite. | partial | `ccgins/metrics.py`, `scripts/run_spoof_experiment.py`, `tests/test_comparison.py`, `results/inekf_vs_eskf/metrics.json` |
| reports §4/9 M6 | Automatic full report and plots | Separate experiment scripts and tables; no unified make_report.py full M6 acceptance. | partial | `scripts/run_outage_experiment.py`, `scripts/plot_i2nav_summary.py`, `results/i2nav/outage_table.md` |
| export §4 | TorchScript/ONNX export | State-dict persistence only; no export utility. | unimplemented | `ccgins/learning.py` |
| urbannav §3.7/5.4 M7 | UrbanNav raw mult satellite pseudorange TC + clock/NLOS | Single RTK baseline projected to rank-one scalar range replaces vector update; exploratory substitute, no satellites/clock/RINEX/UrbanNav. | experiment_substitute | `ccgins/range_coupling.py`, `ccgins/range_experiment.py`, `tests/test_range_coupling.py`, `results/range_coupling/report.md` |
| smoothing §3.7 M7 | Fixed-lag RTS 1–5s | No smoother. | unimplemented | `ccgins/pipeline.py` |
| online_train §3.7 M7 | Healthy GNSS last-layer SGD with rollback | Offline fine-tuning only. | unimplemented | `ccgins/learning.py`, `scripts/train_motionnet_i2nav.py` |
| motion_extras §3.1 optional odometer; post-proposal ZUPT | Optional wheel scale; later causal motion research | Mounted wheel/ZUPT integrated and parity tested; no wheel-scale state or real wheel data; synthetic wheel studies are exploratory. | partial | `ccgins/zupt.py`, `ccgins/stream.py`, `tests/test_zupt.py`, `tests/test_stream_motion.py`, `results/adaptive_motion/report.md`, `results/stream_motion/report.md` |

## Primary path vs exploratory path

Primary = M0–M6 spine: InEKF-LC + learned body velocity + ACI/outage inflation + robust GNSS + dual-channel integrity, assembled by `pipeline.run` and `ccgins.cli`. This is a runnable research spine, with proposal gaps above; no integrity guarantee.

Exploratory = `baseline_range_anchor`, Sage-Husa `adaptive_r`, stream-only extras, ZUPT/wheel research and synthetic-wheel reports. Preserve these; they do not replace primary-line validation.

## Evidence interpretation

All table numbers above are pre-existing, not re-run. Simulation GNSS is synthetic; KITTI uses real OXTS IMU/reference with synthetic GNSS; i2Nav uses real RTK with artificial masking/attacks. Historical tables use legacy CNN. Simulation spoof dual detection improves 0.2 m/s delay 85.9→53.4 s, but real-RTK results in `results/i2nav/spoof_table.md` do not establish general dual superiority. No empirical table implies closed-loop statistical or safety guarantees.

Legacy `results/motionnet.pt`, `results/kitti/motionnet_kitti.pt`, and `results/i2nav/motionnet_fold{A,B}.pt` are six-channel MotionNet state dictionaries; all four loaders were regression-tested successfully in tests/test_proposal_tcn.py. The TCN checkpoint uses architecture metadata; historical plain state dictionaries remain supported.

Current counts after third batch: {'complete': 5, 'partial': 17, 'experiment_substitute': 2, 'unimplemented': 4}

## Pullback outcome

The architecture row remains partial: the requested network and batch filter-input wiring now exist with focused tests and a measured locked paired smoke, but full drive/fold retraining and ≥20-run validation are outstanding. New network training uses reference attitude/true simulated biases; deployed inference uses filter estimates. This conditioning gap is explicitly reported.

See `results/proposal_alignment/report.md` and `metrics.json` for measured results and `configs/proposal_alignment_smoke.yaml` for frozen parameters. TCN did not outperform legacy CNN in this small undertrained test; no historical table was replaced. `configs/proposal_primary.yaml` runs the complete currently supported primary batch spine through CLI; GNSS κ_g/HPL/online mounting/recovery remain absent.

## 2026-11-06 next-batch measured update

[Next report](../results/proposal_alignment_next/report.md) and
[debug investigation](proposal_tcn_debug.md) supersede the mini-smoke-only assessment
above without replacing its frozen measurements. Four 600 s training seeds,
validation 500–503, and all ten test seeds 900–909 with nominal outages completed.
This is reduced training, not full M3 acceptance. Optional smooth variance bounds
address a demonstrated hard-clamp gradient defect; reference-conditioned vertical
variance extrapolation explains the earlier catastrophic NLL tail, but is not
established to originate from lower-bound saturation during training. Validation
preferred TCN; held-out navigation did not confirm it. Legacy remains the validated
working default. Fixed-threshold dual false alarms/latching were isolated in a
validation-only pilot. Full-chain streaming learning/ACI/integrity, GNSS κ_g,
18-state mounting, HPL/recovery and UrbanNav TC remain incomplete. No audit status
is upgraded to complete on the basis of this reduced-train simulation.

## 2026-11-06 third-batch original-budget update

**Original training budget resolved**: both legacy_cnn and proposal_tcn completed
all seeds 100–119 × 600 s with identical 12-epoch optimizer opportunity and smooth
log variance. See [full report](../results/proposal_alignment_full/report.md) for
paired validation/test results and architecture decision, and
[scope/API documentation](proposal_alignment_full.md). This supersedes the reduced
training limitation above; it does not erase earlier measured evidence or establish
M3 acceptance, independent real-drive validation or ≥20-test-run acceptance.

Separate calibration 700–705 is grouped within seed before equal-seed averaging;
its n_eff correlation inflation is not an independent sample count. No test-driven
corr_scale change or atmospheric/bias latch fix is claimed. The locked primary
comparison retains the prior equal correlation/monitor settings. Prior false-alarm
and latched GNSS exclusion findings remain distinct from correlation estimation.

GNSS κ_g now exists in the actual batch ACI/pipeline path, opt-in and tested with
joint χ²₃ NIS, full covariance, next-fix causality, outage freezing and rejection/
forced/alarm eligibility regressions. A separate loaded-network calibration pilot
is reported; it is not primary test tuning or a full navigation integrity guarantee.
Streaming learning/ACI/DR-CUSUM remains incomplete and explicitly rejected, as does
streaming GNSS κ_g. No new full-chain covariance-equality result is claimed.
Online mounting/18-state, HPL/recovery, real-drive TCN, B5, ≥20 tests and UrbanNav TC
remain gaps. Historical published tables/checkpoints were not rewritten.

Full-protocol measured decision: validation RMSE TCN 4.042560 m vs legacy 4.060636 m; test RMSE TCN 31.536755 m vs legacy 5.153636 m, NHC control 3.269130 m. TCN was not promoted; legacy remains the working primary. Full pytest: 130 passed in 56.48s.

## 2026-11-06 fourth-batch streaming update

The learned primary spine now has an opt-in streaming engine. See
[stream report](../results/proposal_alignment_stream/report.md) for the supported
API, causal arrival-delay contract, outage freshness grace, delayed batch semantic
comparison, gross-injection CUSUM exclusion and checkpoint runtime demo. Fixed
NHC/ZUPT/wheel baseline parity is preserved; hybrid/bias fallback and wheel/ZUPT
combinations remain batch/baseline-only. No replay of processed epochs or off-grid
learned GNSS is supported. Narrow matched-grid equality is measured, with no general
exact-covariance or HPL guarantee. M3/M4/M5 remain partial: coverage, integrity
thresholds, whole-window reset, recovery and independent real-drive acceptance
are unresolved. The earlier batch-only statements above describe historical states.

The frozen TCN remains available, not promoted. Original 20×600 training and
full-protocol validation/test numbers above are retained. Fourth-batch diagnostics
are in `results/proposal_tcn_diagnosis/`; no test-based selection or redesign.
Current bookkeeping counts: {'complete': 5, 'partial': 17,
'experiment_substitute': 2, 'unimplemented': 4}.

## 2026-11-06 fifth-batch streaming update

See [fifth stream report](../results/proposal_alignment_stream_fifth/report.md) and
[event-time builder semantics](stream_offgrid_semantics.md). Off-grid split
propagation shares the base path and applies to both main and DR. Hybrid forward
fallback and mounted wheels layer before GNSS. Bounded queue errors and processed
callbacks are explicit. One-file replay is offline only, not hardware integration.
The 10 original synthetic test seeds use the canonical historical checkpoint;
matched-grid comparisons do not establish off-grid batch equality.

The realistic ramp fixture is a low-noise stationary mechanism test, not a
calibrated false-alarm/hour result or recovery test. M3/M4/M5 remain partial; HPL
and failsafe guarantees remain unimplemented. TCN diagnosis was skipped this
batch; legacy_cnn remains primary and TCN is not promoted.

Current bookkeeping counts: {'complete': 5, 'partial': 17,
'experiment_substitute': 2, 'unimplemented': 4}. No row changed acceptance class.
