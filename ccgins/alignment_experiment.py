"""Frozen mini paired proposal pullback evaluation (all GNSS synthetic).

Reproduce: OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_experiment
"""
from pathlib import Path
import argparse
import json
import hashlib
from dataclasses import asdict
import numpy as np
import torch
import yaml
from scipy.stats import chi2
from . import learning
from .scenarios import make_scenario
from .pipeline import Config, run
from .metrics import summarize


def evaluate(config='configs/proposal_alignment_smoke.yaml', out='results/proposal_alignment'):
    p = Path(out)
    p.mkdir(parents=True, exist_ok=True)
    c = yaml.safe_load(Path(config).read_text())
    torch.set_num_threads(c['threads'])
    train = [make_scenario(s, duration=c['duration']) for s in c['train_seeds']]
    records = []
    for arch in ('legacy_cnn', 'proposal_tcn'):
        net = learning.train_motionnet([s.imu6() for s in train], [s.truth.v_body for s in train],
            epochs=c['epochs'], batch=c['batch'], lr=c['lr'], stride=c['stride'],
            seed=c['training_seed'], architecture=arch,
            attitude_list=[learning.roll_pitch(s.truth.R) for s in train] if arch == 'proposal_tcn' else None,
            bias_list=[np.c_[s.imu.gyro_bias, s.imu.accel_bias] for s in train] if arch == 'proposal_tcn' else None)
        learning.save(net, p / f'{arch}.pt')
        for split in ('val', 'test'):
            for seed in c[f'{split}_seeds']:
                sc = make_scenario(seed, duration=c['duration'], outages=c['outages'])
                idx = np.arange(learning.WINDOW, len(sc.truth.t), 10)
                # Standalone supervised diagnostic, explicitly reference-conditioned.
                mu, var = learning.predict(net, sc.imu6(), idx,
                    attitude=learning.roll_pitch(sc.truth.R), bias=np.c_[sc.imu.gyro_bias, sc.imu.accel_bias])
                e = mu - sc.truth.v_body[idx-1]
                raw = dict(rmse_fwd_lat_up=np.sqrt(np.mean(e**2,axis=0)).tolist(),
                    gaussian_nll=float(np.mean(.5*(np.log(2*np.pi*var)+e**2/var))),
                    coverage95_fwd_lat_up=np.mean(e**2/var <= chi2.ppf(.95,1),axis=0).tolist())
                cfg = Config(**c['filter'])
                log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, model=net)
                nav = summarize(log, c['outages'], t_start=30)
                nav['alarm_time'] = log.alarm_time
                nav['aci_updates'] = len(log.kappa_hist)
                records.append(dict(architecture=arch, split=split, seed=seed,
                    data_source='synthetic simulation IMU + synthetic GNSS',
                    reference_conditioned_network=raw, filter_conditioned_navigation=nav))
    for seed in c['test_seeds']:
        sc = make_scenario(seed, duration=c['duration'], outages=c['outages'])
        cfg = Config(**{**c['filter'], 'pseudo':'nhc', 'aci':False, 'monitor':'none', 'corr_scale':None})
        log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec,cfg)
        records.append(dict(architecture='nhc',split='test',seed=seed,
            data_source='synthetic simulation IMU + synthetic GNSS',
            filter_conditioned_navigation=summarize(log,c['outages'],t_start=30)))
    aggregates = {}
    for arch in ('legacy_cnn','proposal_tcn','nhc'):
        rs = [r for r in records if r['architecture']==arch and r['split']=='test']
        aggregates[arch] = {key:float(np.mean([r['filter_conditioned_navigation'][key] for r in rs]))
            for key in ('rmse_h','coverage95_h','coverage95_h_outage','max_err_outage_20s')}
        if arch != 'nhc':
            aggregates[arch]['gaussian_nll'] = float(np.mean([r['reference_conditioned_network']['gaussian_nll'] for r in rs]))
            for key in ('rmse_fwd_lat_up','coverage95_fwd_lat_up'):
                aggregates[arch][key] = np.mean([r['reference_conditioned_network'][key] for r in rs],axis=0).tolist()
    payload = dict(schema_version=1, config=c, resolved_filter=asdict(Config(**c['filter'])),
        config_sha256=hashlib.sha256(Path(config).read_bytes()).hexdigest(),
        torch_version=str(torch.__version__),numpy_version=np.__version__, records=records, aggregates=aggregates)
    (p/'metrics.json').write_text(json.dumps(payload,indent=2)+'\n')
    lines=['# Proposal TCN paired smoke (measured this session)','',
        'Reproduce: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_experiment`.',
        f'Frozen config: `{config}`; SHA256 `{payload["config_sha256"]}`. Resolved defaults are in metrics.json.',
        'All data are synthetic simulation IMU + synthetic GNSS. Train seeds 100/101, validation 500, paired tests 900/901; 80 s, outage 45–65 s. Six epochs, Adam lr .002, cosine schedule, beta-NLL beta=.5, batch 128, stride 20, initialization seed 0, one CPU thread. No tuning after observing validation/test.',
        'Network diagnostics use reference roll/pitch and true simulated biases for TCN; legacy uses raw IMU. This compares the proposal input/architecture package, not an isolated dilation ablation. Training uses the same reference-conditioned TCN inputs.',
        'Navigation uses only live filter roll/pitch and current estimated biases, with trailing IMU windows; the DR guard infers separately from its own attitude/bias estimates; no test reference enters learning inference. Reference initializes the shared existing protocol and scores errors. Both networks use identical hybrid/ACI/outage-inflation/Huber/dual-monitor settings. Fixed correlation factor 40 and CUSUM h=50 are smoke settings, not calibrated integrity thresholds. NHC is a supplemental fixed-NHC control without ACI/dual monitoring.',
        'Network NLL includes Gaussian constant and averages channels/epochs. Network coverage is scalar marginal raw-variance coverage. Navigation coverage is the 2D position ellipse; these are different quantities.', '',
        '| Method | Navigation horizontal RMSE m | Position coverage | Outage coverage | Outage max m | Network NLL |',
        '|---|---:|---:|---:|---:|---:|']
    for arch,a in aggregates.items():
        lines.append(f"| {arch} | {a['rmse_h']:.6f} | {a['coverage95_h']:.6f} | {a['coverage95_h_outage']:.6f} | {a['max_err_outage_20s']:.6f} | {a.get('gaussian_nll',float('nan')):.6f} |")
    lines+=['','| Network | Forward/lateral/up RMSE m/s | Forward/lateral/up raw coverage |','|---|---|---|']
    for arch in ('legacy_cnn','proposal_tcn'):
        a=aggregates[arch];lines.append(f"| {arch} | {a['rmse_fwd_lat_up']} | {a['coverage95_fwd_lat_up']} |")
    lines+=['','Every seed and validation result is retained in metrics.json. This tiny undertrained smoke verifies a reproducible primary-path integration; it is not the full 10-seed table, proposal ≥20-run acceptance, independent road validation, or evidence that TCN improves accuracy. Historical published tables remain unchanged and use legacy CNN. GNSS κ_g, mounting state, HPL/recovery and streaming learned integration remain missing.']
    (p/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(aggregates,indent=2))
    return payload


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',default='configs/proposal_alignment_smoke.yaml')
    ap.add_argument('--out',default='results/proposal_alignment')
    args=ap.parse_args()
    evaluate(args.config,args.out)

if __name__=='__main__':
    main()
