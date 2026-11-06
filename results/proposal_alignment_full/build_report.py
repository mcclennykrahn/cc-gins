"""Render measured results only after exact protocol completion checks."""
import json
from pathlib import Path
import numpy as np

out = Path('results/proposal_alignment_full')
p = json.loads((out/'metrics.json').read_text())
c = p['config']
assert c['train_seeds'] == list(range(100,120)) and c['duration'] == 600
assert c['test_seeds'] == list(range(900,910)) and c['val_seeds'] == list(range(500,504))
arches = ['legacy_cnn', 'proposal_tcn', 'nhc']
for arch in arches:
    assert sorted(r['seed'] for r in p['records'] if r['architecture']==arch and r['split']=='test') == c['test_seeds']
    if arch != 'nhc':
        done = json.loads((out/'training'/f'{arch}_smooth_completed.json').read_text())
        assert done['train_seeds'] == c['train_seeds'] and done['epochs'] == c['epochs']
val = {a:float(np.mean([r['navigation']['rmse_h'] for r in p['records'] if r['architecture']==a and r['split']=='val'])) for a in arches[:2]}
val_best = min(val, key=val.get)
a = p['aggregates']
test_best = min(arches[:2], key=lambda x:a[x]['rmse_h'])
selection = json.loads((out/'selection.json').read_text())
selection.update(architecture_validation_criterion='mean navigation horizontal RMSE over val 500–503 only',
    validation_rmse_h=val, validation_ranked_architecture=val_best,
    test_comparison_is_not_hyperparameter_selection=True)
(out/'architecture_selection.json').write_text(json.dumps(selection,indent=2)+'\n')
lines = ['# Original 20-train paired evaluation (2026-11-06)', '',
'**Original training budget resolved: YES. Both architectures completed training on all 20 seeds 100–119 × 600 s.** This restores the original simulation training budget; it is not full M3 acceptance. All 10 paired tests 900–909 and validations 500–503 completed. All IMU and GNSS data are synthetic.', '',
'Baseline `e1562135aacf67ce2f06076b3c202fa855da0791`; protocol/harness commit `f0c8e55`. Historical published tables and checkpoints were not rewritten.', '',
'## Locked protocol', '',
'The YAML was locked before training. Equal Adam lr .002, cosine schedule, 12 epochs, batch 128, stride 20, initialization seed 0, one Torch/BLAS thread per process. Both networks receive the same windows/optimizer steps. Smooth variance was preselected in e156213 validation; no new activation or test-based configuration tuning. TCN has bias-compensated IMU plus reference roll/pitch during training; legacy has raw six-channel IMU. This compares those input/architecture packages. Runtime need not be equal for differently sized networks.', '',
f"Config SHA256: `{selection['config_sha256']}`. Frozen protocol and per-architecture completion/model hashes are in `training/`. Nominal outages (150,30), (280,60), (420,120) s match the prior locked batch. Calibration seeds 700–705 are separate and never used for network training/architecture selection.", '',
'Both learned methods use the same inherited hybrid fixed lateral/up NHC, learned ACI, corr_scale=[40,40,40], Huber and DR-CUSUM settings. These are fixed research settings, not calibrated integrity thresholds. GNSS κ_g is disabled in this primary table. NHC is a supplemental control with ACI/monitor disabled and no learned forward channel.', '',
'Network NLL includes the Gaussian constant, averaged over channels/windows, using reference-conditioned inputs. Navigation uses live estimated attitude/bias for each filter separately. Truth initializes the existing shared protocol and scores errors. Navigation metrics start at 30 s; outage coverage includes outage epochs. Values below are unweighted means across seed metrics.', '',
'| Method | RMSE_h m | Network NLL | Nav coverage | Outage coverage | notes |',
'|---|---:|---:|---:|---:|---|']
for arch in arches:
    v=a[arch]; nll='—' if v['gaussian_nll'] is None else f"{v['gaussian_nll']:.6f}"
    lines.append(f"| {arch} {'(selected)' if arch!='nhc' else '(control)'} | {v['rmse_h']:.6f} | {nll} | {v['coverage95_h']:.6f} | {v['coverage95_h_outage']:.6f} | {'smooth, locked settings' if arch!='nhc' else 'no learned ACI/monitor'} |")
lines += ['', '## Validation versus held-out test', '', '| Architecture | Validation RMSE_h m | Test RMSE_h std m |', '|---|---:|---:|']
for arch in arches[:2]:
    std=np.std([r['navigation']['rmse_h'] for r in p['records'] if r['architecture']==arch and r['split']=='test'])
    lines.append(f'| {arch} | {val[arch]:.6f} | {std:.6f} |')
lines += ['', f'Validation-ranked architecture: **{val_best}**. Lower paired learned-method test RMSE: **{test_best}**. Test TCN minus legacy: **{a["proposal_tcn"]["rmse_h"]-a["legacy_cnn"]["rmse_h"]:+.6f} m**.', '']
if test_best=='legacy_cnn':
    lines += ['**Legacy remains the validated working primary. TCN was not promoted.** Validation ranking does not establish universal superiority; neither network is a full navigation integrity guarantee. NHC control is reported honestly and has different monitor/ACI behavior.']
else:
    lines += ['TCN has lower test RMSE in this locked comparison; this is simulation evidence only, not universal superiority. Historical primary checkpoints remain unchanged.']
lines += ['', '## Per-seed paired tests', '', '| Seed | Legacy RMSE_h m | TCN RMSE_h m | NHC RMSE_h m |', '|---|---:|---:|---:|']
for seed in c['test_seeds']:
    vals=[next(r['navigation']['rmse_h'] for r in p['records'] if r['architecture']==arch and r['split']=='test' and r['seed']==seed) for arch in arches]
    lines.append(f'| {seed} | '+' | '.join(f'{v:.6f}' for v in vals)+' |')
lines += ['', '## Calibration, GNSS κ_g, and limits', '',
'See [implementation and scope](../../docs/proposal_alignment_full.md), `grouped_calibration.json` and `gnss_aci_pilot.json`. Grouped n_eff diagnostics use separate calibration seeds, with within-seed centering/lag pairs and equal-seed averaging. They do not retune the locked correlation factor or repair the previously isolated dual false-alarm/latch issue. GNSS κ_g is implemented and tested as an opt-in vector-NIS, next-fix scale in the actual batch pipeline. The calibration-only pilot is separate from the paired primary results. No HPL, attack-recovery, nominal-coverage or full-chain streaming guarantee is claimed.', '',
'Remaining gaps: streaming MotionNet/ACI/DR-CUSUM and covariance equality for that full chain, independent real-drive TCN validation, reference/filter input conditioning, calibrated integrity thresholds, online mounting/18-state, healthy-window reset/recovery, HPL/exceedance, B5 VB/Student-t, ≥20-test-run acceptance and UrbanNav raw-pseudorange TC. Original training-budget restoration alone does not close M3.', '',
'## Commands run this session', '', '```bash',
'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u -m ccgins.alignment_full',
'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_full/supplement.py',
'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q',
'.venv/bin/python results/proposal_alignment_full/build_report.py', '```', '',
'Training/navigation output is in `run.log`; all seed measurements in `metrics.json`. Full pytest output is in `pytest.log`. All reported new measurements were generated by these local commands.']
cal = json.loads((out/'grouped_calibration.json').read_text())
pilot = json.loads((out/'gnss_aci_pilot.json').read_text())
lines += ['', '## Separate calibration measurements', '',
    '| Architecture | Grouped corr_scale forward / lateral / up |', '|---|---|']
for arch, record in cal.items():
    lines.append(f"| {arch} | "+' / '.join(f'{v:.6f}' for v in record['grouped_corr_scale'])+' |')
lines += ['', 'These reference-conditioned seed 700–705 diagnostics were not substituted for the locked primary corr_scale. Checkpoint class, activation, parameter count and SHA256 are recorded with them.', '',
    '| Calibration seed 700, 120 s | Monitor | κ_g enabled | RMSE_h m | Eligible κ_g samples | Final κ_g |',
    '|---|---|---|---:|---:|---:|']
for record in pilot['records']:
    trace = record['gnss_aci_trace']
    final = trace[-1]['kappa_next'] if trace else 1.
    lines.append(f"| outage 80–100 s | {record['monitor']} | {record['gnss_aci']} | {record['navigation']['rmse_h']:.6f} | {sum(r['eligible'] for r in trace)} | {final:.6f} |")
lines += ['', 'This short calibration-only pilot demonstrates actual batch wiring, not navigation improvement or attack/integrity acceptance. Disabled κ_g has no calibration buffer; final scale one denotes no adaptation.']
lines += ['', 'Full pytest: **'+(out/'pytest.log').read_text().strip().splitlines()[-1]+'**. Original tests retained; no regression-masking edits.']
(out/'report.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines[lines.index('| Method | RMSE_h m | Network NLL | Nav coverage | Outage coverage | notes |'):][:5]))
