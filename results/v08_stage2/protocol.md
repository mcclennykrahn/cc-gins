# Stage 2 preregistered campaign (2026-11-06)

Written before validation completes. No changes to simulator, attack generation,
noise, checkpoint, runtime detector or defaults. Only seeds 500–503 are used for
selection. Held-out seeds 900–909 are run by a separate command after selection.json
is written and committed. Prior historical held-out evidence exists; this is a new
parameter campaign, not a claim those seed identities have never been seen.

Use the historical scripts/run_spoof_experiment.py 600 s scenario, default IMU/GNSS
noise, 300 s east ramp onset and rates 0.2, 0.5, 1.0 m/s. Clean runs have no attack
and no outages. Retain the script's v3 hybrid/ACI/bias fallback, canonical legacy
checkpoint and correlation scales. The original script's calibration seeds 700–705
are NOT used here. Candidate dual CUSUM (k,h): (0,20), (3,50), (6,100), fixed 30 s
DR reset and default permanent latch. No candidate recovery tuning is attempted;
ongoing ramps do not justify statistical reconnection. Preserve Stage 1 recovery
checks separately. This is the full default-noise ramp campaign; it is not a
recreation of the earlier exploratory seed-713 25 s probe or the low-noise fixture.

Campaign engineering acceptance: zero clean alarms and all 12 validation / 30
held-out attacks detected after onset and by the 600 s horizon, on each split.
This conservative finite-campaign rule is not an operational integrity standard,
latency bound or FA/hour certification. All individual delays are reported.
Rank passing candidates first, then lexicographically total clean+pre-onset alarms,
misses, and total detected delay; ties retain grid order. If none pass validation,
freeze the least-bad candidate for held-out diagnosis and prohibit promotion even
if it happens to pass held-out. No grid expansion after looking at held-out.
The limited grid cannot prove that every possible detector parameter fails.

Commands (from repository root):

```
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/v08_stage2/evaluate.py select > results/v08_stage2/selection.log 2>&1
# Commit selection.json, validation.json, evaluator and this protocol before test.
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/v08_stage2/evaluate.py test > results/v08_stage2/test.log 2>&1
.venv/bin/python -m pytest -q > results/v08_stage2/pytest.log 2>&1
```

Raw row alarm_time is the first alarm, delay_s is null for both misses and
pre-onset alarms, which are separately counted. Clean exposure is total simulated
trajectory time, including any post-alarm interval; clean_gnss_epochs counts actual
monitor evaluations. Latching makes at most one alarm event per trajectory.
