"""Build a measured-only report after the locked experiment has finished."""
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from dataclasses import asdict
from ccgins.pipeline import Config

p=Path('results/proposal_alignment_next'); d=Path('results/proposal_alignment_debug')
m=json.loads((p/'metrics.json').read_text()); records=m['records']
for arch in ['legacy_cnn','proposal_tcn','nhc']:
    assert sorted(r['seed'] for r in records if r['architecture']==arch and r['split']=='test')==list(range(900,910))
assert len(records)==38
m['environment']={'torch':str(torch.__version__),'numpy':np.__version__,'device':'local CPU','navigation_workers':3}
m['checkpoint_sha256']={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in p.glob('*.pt')}
m['nhc_resolved_filter']=asdict(Config(**{**m['config']['filter'],'pseudo':'nhc','aci':False,'monitor':'none','corr_scale':None}))
val={a:float(np.mean([r['navigation']['rmse_h'] for r in records if r['architecture']==a and r['split']=='val'])) for a in ['legacy_cnn','proposal_tcn']}
m['validation_navigation_rmse_h']=val
winner=min(val,key=val.get)
m['validation_selected_architecture']=winner
m['validated_working_primary']='legacy_cnn'
m['completion']={'test_seeds_per_method':10,'validation_seeds_per_network':4,'full_nominal_outages':True,'full_M3_acceptance':False}
m['network_conditioning']='reference attitude/true simulated bias for TCN; raw IMU for legacy'
m['navigation_conditioning']='live filter attitude/current estimated bias; independent guard inputs'
(p/'metrics.json').write_text(json.dumps(m,indent=2)+'\n')
pilots=json.loads((d/'validation_pilots.json').read_text())
navpilot=json.loads((d/'nav_pilot.json').read_text())
lines=['# Next paired evaluation — reduced train (2026-11-06)','',
'**Reduced-train / not full M3 acceptance. All 10 test seeds 900–909 completed for both architectures and NHC; full nominal 30/60/120 s outages.** All IMU and GNSS data here are synthetic simulation. Historical published legacy tables/checkpoints were not rewritten.','',
'Baseline: `58b3dc976046e4f2107b84e5eee8fff9a9bfe59a`. See [debug findings](../../docs/proposal_tcn_debug.md), [raw smoke rerun](../proposal_alignment_debug/smoke/metrics.json), and [per-channel probe](../proposal_alignment_debug/probe.json).','',
'## Precommitted assessment','',
'`configs/proposal_alignment_next.yaml` was locked before training: train 100–103 × 600 s, val 500–503, test 900–909 × 600 s. Calibration 700–705 untouched. Outages are (start, duration) = (150,30), (280,60), (420,120) s. Identical Adam lr .002, cosine schedule, 12 epochs, batch 128, stride 20, initialization seed 0, one Torch/BLAS thread per process. Three independent local navigation processes; completed runs retained across a scheduling restart. This reduces the train pool from 20 to 4, not test coverage or outage duration.','',
f"Config SHA256: `{m['selection']['config_sha256']}`. Only hard/smooth variance activation was selected, by mean validation Gaussian NLL pooled equally over architectures and validation seeds. Selected **{m['selection']['activation']}** before test navigation; scores: {m['selection']['scores']}. Same selected hyperparameters and ACI for both architectures. No test-based tuning or ACI/correlation sweep. Model hashes, environment, resolved filters and every seed are in metrics.json.",'',
'Network diagnostics are reference-conditioned (TCN uses reference roll/pitch and true simulated biases; legacy raw IMU). Navigation uses live estimated filter attitude/current estimated biases, separately for A and DR B. Truth only initializes the shared existing protocol and scores errors. Network NLL is Gaussian NLL including .5 log(2π), averaged across three channels and windows; negative continuous-density NLL is valid. Navigation RMSE/coverage start at 30 s; outage coverage includes all outage epochs. Table values are unweighted means of seed metrics.','',
'Paired learned settings: InEKF, hybrid fixed lateral/up NHC σ=.1, ACI α=.05/γ=.01/window=300/outage β=2/T_ref=60, corr_scale=[40,40,40], Huber=1.5, dual CUSUM k=3/h=50, reset=30 s. These inherited smoke thresholds are **not calibrated integrity thresholds**. NHC is a supplemental control with the same nominal sensor/filter/noise settings but ACI/monitor disabled and no learned forward channel.','',
'| Method | RMSE_h m | Network NLL | Nav coverage | Outage coverage | notes |','|---|---:|---:|---:|---:|---|']
for a,v in m['aggregates'].items():
    n='—' if v['gaussian_nll'] is None else f"{v['gaussian_nll']:.6f}"
    lines.append(f"| {a + (' (control)' if a=='nhc' else ' (selected)')} | {v['rmse_h']:.6f} | {n} | {v['coverage95_h']:.6f} | {v['coverage95_h_outage']:.6f} | {'control, no ACI/dual' if a=='nhc' else 'smooth; 10 seeds'} |")
lines+=['','Validation navigation means (architecture assessment only; no new settings selected):','', '| Method | Val RMSE_h m | Test seed RMSE_h std m |','|---|---:|---:|']
for a in val:
    sd=np.std([r['navigation']['rmse_h'] for r in records if r['architecture']==a and r['split']=='test'],ddof=1)
    lines.append(f'| {a} | {val[a]:.6f} | {sd:.6f} |')
legacy=m['aggregates']['legacy_cnn']['rmse_h']; tcn=m['aggregates']['proposal_tcn']['rmse_h']
lines+=['',f'Validation-ranked architecture: **{winner}**. Test-reported TCN minus legacy RMSE_h: **{tcn-legacy:+.6f} m**.']
if winner=='legacy_cnn' or tcn>=legacy:
    lines+=['**Legacy remains the validated primary working method; it has the lower test RMSE among the paired learned configurations with validation-selected settings. TCN is not established as an improvement / not primary.** Both the stacked legacy CNN and proposal TCN remain available. A better network likelihood does not establish a better closed-loop navigation method.']
else:
    lines+=['TCN wins this reduced-train validation-selected/test-reported comparison, but full M3/real-drive acceptance remains open; the historical validated legacy working default is retained conservatively.']
lines+=['','## Debug evidence and limits','',
'The reproduced smoke aggregates exactly equal the frozen prior smoke metrics. `probe.py` shows TCN seed 900 NLL channels 4.919387 / 0.420892 / 35.277800, with vertical variance reaching exp(-12). Seed 901 vertical NLL is 24.418560. This explains the catastrophic standalone NLL tail; reference/serve mismatch cannot explain those reference-conditioned measurements.','',
'Hard log-variance clipping has a zero-gradient region outside [-12,4]. The regression explicitly asserts zero hard-head gradients at raw -13/5 and restoring smooth-head gradients. Added optional smooth clipping with checkpoint metadata, preserving original hard-clipped models and old loaders. No objective/target/normalization redesign. The six-epoch isolated validation pilot changes TCN NLL 1.363208→1.340219; this is a pilot, not primary evidence.','',
'| Reduced-train validation pilot | Mean network NLL |','|---|---:|']
for a in ['legacy_cnn','proposal_tcn']:
 for act in ['hard','smooth']:
    score=np.mean([r['network']['gaussian_nll'] for r in pilots if r['architecture']==a and r['activation']==act])
    lines.append(f'| {a}, {act} | {score:.6f} |')
lines+=['','More training reduces the severe raw-NLL failure even with hard clipping. The pre-clamp tail probe finds no vertical lower-bound crossing on either smoke training seed, but 3.286979% on test 900; test 901 has large NLL without crossing -12. Thus lower-clamp dead gradients during training are not established as the cause of vertical collapse. Forward upper-clamp saturation is present on 82.932996%/96.080910% of smoke training windows. The evidence supports held-out tail overconfidence and a separate demonstrated clamp-gradient defect, not an assertion that smooth clipping alone cures TCN. β-NLL constants/channel averaging, bias ordering, ZYX roll/pitch, independent channel scaling and exclusive-window labels were checked without finding the suspected formula/preprocessing bug. TCN receptive field is 29 samples before global pooling; same padding uses only the supplied trailing window and is not pointwise causal.','',
'Hybrid navigation consumes learned forward speed/variance only; its fixed lateral/up NHC does not consume the vertical variance responsible for the raw-NLL tail. A separate validation-only diagnostic retains a difficult seed 501, toggles monitor only, and scores live-input NLL. It is not used to change the locked primary settings:','',
'| Pilot/debug seed 501 | Monitor | RMSE_h m | Alarm time s | Live NLL | Reference NLL |','|---|---|---:|---:|---:|---:|']
for r in navpilot:
    lines.append(f"| {r['architecture']} | {r['monitor']} | {r['navigation']['rmse_h']:.6f} | {r['alarm_time']} | {r['live_network_nll']:.6f} | {r['reference_network']['gaussian_nll']:.6f} |")
lines+=['','See [nav_pilot.json](../proposal_alignment_debug/nav_pilot.json) for per-channel live diagnostics and final κ. False alarms in a no-attack simulation are failures, not attack-detection successes; a latched alarm excludes later GNSS and can dominate full-run RMSE. This pilot isolates monitor effects only on one validation seed. It does not calibrate CUSUM thresholds, establish a domain-gap remedy, or replace the paired table. No bias-subtraction/attitude-removal/filter-conditioned-training sweep was selected on tests.','',
'## Per-seed primary results','',
'| Seed | Legacy RMSE_h | TCN RMSE_h | NHC RMSE_h |','|---|---:|---:|---:|']
for seed in range(900,910):
    values=[next(r['navigation']['rmse_h'] for r in records if r['architecture']==a and r['split']=='test' and r['seed']==seed) for a in ['legacy_cnn','proposal_tcn','nhc']]
    lines.append(f'| {seed} | '+ ' | '.join(f'{v:.6f}' for v in values)+' |')
lines+=['','## Remaining gaps','',
'Full-chain **streaming learning+ACI+integrity is still incomplete**. This batch completes the ten-seed batch TCN attitude/navigation chain only; it does not establish proposal acceptance. Outstanding: full 20×600 s training, independent real-drive TCN/KITTI/i2Nav validation, reference/filter conditioning gap, calibrated correlation/CUSUM thresholds and recovery, GNSS κ_g, online mounting/18-state, HPL/exceedance rates, ≥20-run acceptance, B5 Student-t/VB, UrbanNav raw-pseudorange TC, smoothing and online learning. Existing robust GNSS/ACI/dual-monitor paths and historical published results remain intact.','',
'## Reproduce and tests','',
'```bash','OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_next',
'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_debug/nav_pilot.py',
'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=. .venv/bin/python results/proposal_alignment_next/build_report.py',
'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q','```','',
'Full pytest: '+(d/'pytest.log').read_text().strip().splitlines()[-1]+'. Six additional regression cases; original tests retained. `run.log` captures training, selection and every completed navigation run; the initial scheduling interrupt is retained. Historical tables/checkpoints have no git diff.']
(p/'report.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines[:35]))
