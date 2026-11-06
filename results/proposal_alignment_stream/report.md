# Fourth batch: causal streaming assembly (2026-11-06)

Implemented an **opt-in** learned streaming chain on this Linux box at baseline
`4e0a362217334c3b46b73528ebfda160f687da68`, model gpt-6.1-sol.
Data in all new runs are simulated IMU and **synthetic GNSS**. No live receiver,
external wheel schema, remote agent, retraining, or default-model change is involved.

## API and supported scope

```python
from ccgins.learning import load
from ccgins.pipeline import Config
from ccgins.stream_learning import LearnedStreamingNavigator
cfg = Config(pseudo='learned_vel', aci=True, gnss_aci=True,
             monitor='dual', corr_scale=(40., 40., 40.))
nav = LearnedStreamingNavigator.from_truth(
    truth, imu_spec, gnss_spec, cfg,
    model=load('results/proposal_alignment_full/legacy_cnn.pt'), delay=.05)
# In arrival order; enqueue each GNSS before processing its event-time epoch:
# nav.push_gnss(epoch, position, full_covariance)
# nav.push_imu(epoch, acceleration, angular_rate)
# nav.get_state() reports processed time, potentially behind arrival time.
# After final permitted GNSS arrivals: nav.flush()
```

`from_truth` is a simulation initializer; direct construction takes R/v/p/P/noise,
like the baseline. Explicit class selection is the opt-in. Legacy checkpoint is
the working primary; TCN remains selectable through its checkpoint metadata.
There is no CLI default change. Supported learned modes: full velocity and learned
NHC. Filter, mount (learned NHC), robust GNSS, Earth/noise settings use existing
models. Baseline `StreamingNavigator` remains unchanged with fixed NHC/ZUPT/wheel
and its strict parity/rejection tests.

Every processed increment appends the previous IMU sample and pre-propagation
attitude to bounded trailing windows. At a full window and configured cadence
(default 10 samples), existing `learning.predict` runs once per filter. TCN uses
each filter's own historical roll/pitch and current estimated biases; the DR guard
does not consume the main filter's attitude/bias. No repeated-feature bootstrap,
partial-window padding, repeated batch reruns or overlapping-window covariance
shortcut is used. Overlap still means correlated measurement errors: supplied
correlation inflation is a research setting, not proof of covariance consistency.

ACI reuses `ACICalibrator`; GNSS scale reuses `GNSSACICalibrator` with pre-update
vector residual/projected covariance and next-fix application. Accepted GNSS
freshness (<1.5 s), warmup (t>30 s), and no alarm gate the pseudo reference.
Before the first accepted fix, pseudo calibration is ineligible. During missing
GNSS, freshness has the inherited 1.5 s grace interval; after that both adaptation
states freeze and pseudo effective variance inflates. This is a stale-reference
policy, not instantaneous oracle knowledge that an outage has begun.

Dual integrity reuses `Cusum` and `dr_statistic`; the guard gets propagation and
its own learned motion updates, never a direct GNSS update. Existing healthy-now
reset conditions are retained. An alarm latches and excludes the triggering GNSS
before actuation, also preventing calibration from the excluded fix. Shared ACI
scales originate in GNSS-constrained references: guard is not statistically
independent of all past GNSS. Whole-window health and recovery remain missing.

GNSS delayed by less than configured processing delay can be inserted while its
IMU epoch is pending. Published decisions are never revised. IMU must be fixed
rate; GNSS must be on its grid, unique and arrive strictly before processing its
epoch. Older/current-processed fixes are rejected, with no replay or smoothing.
Rejects also include hybrid/bias-fallback, ZUPT/wheel combinations in this engine,
adaptive-R and scalar range coupling. Those baseline/batch capabilities remain
available separately. This is a runnable learned primary spine with explicit
scope gaps, not complete proposal M3/M4/M5 acceptance.

## Tests and matched semantics

`tests/test_stream_learning.py` adds ten cases: four architecture/mode delayed
batch comparisons, one future-invariance case, one joint outage-freeze case, one
precise dual-CUSUM malicious-exclusion case, one rejection-contract case, and
two arrival-buffer/duplicate/prequeued-grid cases. Original tests remain unchanged.
The initial focused baseline+learned run passed 37 cases; the final full-suite
result including the additional arrival-contract cases is in `pytest.log`.

The short comparisons use seed 710, 4 s, GNSS arrival lag 30 ms, processing delay
50 ms, full trailing window 100 samples, 31 pseudo epochs. Both architectures
and learned modes agree with batch final position/covariance and all integrity
statistics within 1e-10 absolute tolerance. Epoch-zero GNSS is disabled to match
batch (which starts updates after its first propagation). Tests use deterministic
untrained networks to test interfaces independently of checkpoint quality.

The causal test changes future IMU and GNSS after 3 s: all emitted decisions and
snapshot covariance remain unchanged; continuation does not mutate old logs.
Outage test seed 712 runs 39 s with missing GNSS 34–39 s: both calibrators have
observed references first, then observation counts, alpha and kappa stay exactly
frozen after the freshness grace period while effective pseudo variance grows.
Malicious test seed 713 injects +10000 m east from 3 s: clean control has no alarm;
CUSUM h=50 rejects precisely at 3 s and every subsequent malicious fix. Pre-attack
fixes remain accepted. DR state/covariance equals the missing-GNSS counterfactual.
This tests a gross injection, not calibrated false-alarm/ramp detection performance.

## Reproducible checkpoint demo

Run `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_stream/benchmark.py`.
`metrics.json`, `benchmark.log`, and per-architecture traces retain all measurements.
The demo uses seed 715, 40 s, outage 34–39 s, 30 ms GNSS arrival lag and 50 ms
processing delay, one Torch thread. It exercises learned ACI, GNSS ACI and a DR
reset; an intentionally huge monitor threshold isolates semantic comparison and
is **not** an integrity setting. Spoof rejection uses the separate test above.
Each loaded checkpoint makes 391 pseudo decisions and processes 35 GNSS fixes;
54 learned ACI references, five GNSS ACI references, and 34 frozen decisions are
recorded. Matched final position, covariance and monitor-statistic differences
are zero in this demo. Timing is recorded in metrics.json and summarized below.

The shared start/grid/window schedule permits this narrow numerical regression.
**No general exact covariance equality is claimed** for other starts, input timing,
off-grid updates or missing references. A trailing window predicts the last sample
(idx-1) and acts at idx, inheriting the batch 10 ms label-to-actuation lag; the
arrival buffer adds 50 ms availability delay, without changing event epochs.

No HPL guarantee, nominal closed-loop coverage, safe deployment claim, calibrated
false-alarm rate, automatic recovery, independent real-drive validation, 18-state
mount estimation or full proposal completion is established. Traces include raw
feature means, attitude means/current bias, predicted/effective variance, calibration
eligibility/scales, actuation acceptance, CUSUM statistic and exclusion. IMU caches
are bounded; retained diagnostic logs and GNSS epoch deduplication grow with run
length and should be drained/managed by a production host.

Secondary TCN diagnosis follows only after this primary chain and tests landed.
Prior full-protocol metrics are retained, and TCN is **not promoted**.

Measured local runtime (final checkpoint demo, includes diagnostic logging):

- legacy_cnn: 4001 samples, 1.181519 s wall, 3386.3 IMU samples/s.
- proposal_tcn: 4001 samples, 1.314392 s wall, 3044.0 IMU samples/s.

Secondary bounded diagnosis completed: [TCN report](../proposal_tcn_diagnosis/report.md).
Architecture, seed finiteness, opening/detrending, validation transforms/adjacency,
training curves and prior test-channel errors are recorded. Causal attribution of
the full navigation failure remains partial; no fix or promotion is claimed.

Final full suite: **140 passed in 47.10s** (`pytest.log`). All ten new chain/arrival
cases and the unchanged baseline parity/motion suites passed. Historical published
legacy tables, checkpoints and earlier full/smoke/next metrics were not rewritten.
