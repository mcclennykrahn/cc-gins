"""Validation-only CUSUM selection; separate frozen held-out evaluation commands."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from ccgins.learning import load, predict, WINDOW
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario

ROOT = Path(__file__).resolve().parent
CANDIDATES = [dict(cusum_k=0., cusum_h=20.), dict(cusum_k=3., cusum_h=50.),
              dict(cusum_k=6., cusum_h=100.)]
RATES = [0., .2, .5, 1.]
BASE = dict(filter='inekf', pseudo='hybrid', corr_scale=(38.02,61.09,39.1),
            aci=True, aci_gamma=.05, aci_window=100, hybrid_max_kappa=10.,
            hybrid_bias_z=4., hybrid_bias_latch=True, monitor='dual',
            dr_reset_period=30., recovery_epochs=0)


def summarize(rows):
    clean = [r for r in rows if r['rate'] == 0]
    return dict(clean_alarms=sum(r['alarm_time'] is not None for r in clean),
                clean_runs=len(clean), clean_exposure_s=600*len(clean),
                clean_gnss_epochs=sum(r['gnss_epochs'] for r in clean),
                attacks={str(rate): dict(runs=len(rr := [r for r in rows if r['rate']==rate]),
                    detected=sum(r['delay_s'] is not None for r in rr),
                    misses=sum(r['delay_s'] is None for r in rr),
                    preattack_alarms=sum(r['alarm_time'] is not None and r['alarm_time']<300 for r in rr),
                    delays_s=[r['delay_s'] for r in rr]) for rate in RATES[1:]})


def eligible(summary):
    # Engineering campaign acceptance, declared before validation; no FA-rate claim.
    return summary['clean_alarms']==0 and all(a['misses']==0 and a['preattack_alarms']==0
                                            for a in summary['attacks'].values())


def worker(args):
    seed, candidates = args
    torch.set_num_threads(1)
    net = load('results/motionnet.pt')
    rows = []
    for rate in RATES:
        sc = make_scenario(seed, spoof_start=300. if rate else None, spoof_rate=rate)
        idx = np.arange(WINDOW, len(sc.truth.t), 10)
        with torch.no_grad():
            mu, var = predict(net, sc.imu6(), idx)
        for candidate in candidates:
            cfg = Config(init_seed=seed, **BASE, **candidate)
            log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg, pred=(idx,mu,var))
            alarm = None if log.alarm_time is None else float(log.alarm_time)
            rows.append(dict(seed=seed, rate=rate, candidate=candidate, alarm_time=alarm,
                delay_s=alarm-300 if rate and alarm is not None and alarm>=300 else None,
                gnss_epochs=len(log.gnss_t), recoveries=len(log.extra['recovery_times']),
                end_horizontal_error_m=float(np.linalg.norm(log.p_est[-1,:2]-log.p_true[-1,:2]))))
    print(f'completed seed {seed}', flush=True)
    return rows


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('phase', choices=['select','test'])
    ap.add_argument('--workers', type=int, default=4)
    args = ap.parse_args()
    if args.phase == 'select':
        if (ROOT/'selection.json').exists():
            raise SystemExit('Selection already frozen; refusing overwrite')
        seeds, candidates = list(range(500,504)), CANDIDATES
    else:
        selection = json.loads((ROOT/'selection.json').read_text())
        seeds, candidates = list(range(900,910)), [selection['frozen_candidate']]
        if (ROOT/'metrics.json').exists():
            raise SystemExit('Held-out artifact already exists; refusing overwrite')
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = sum(pool.map(worker, [(s,candidates) for s in seeds]), [])
    if args.phase == 'select':
        summaries = [dict(candidate=c, summary=summarize([r for r in rows if r['candidate']==c])) for c in candidates]
        passing = [s for s in summaries if eligible(s['summary'])]
        # If all fail, still freeze the least-bad candidate for honest held-out diagnosis.
        def rank(s):
            su=s['summary']; attacks=list(su['attacks'].values())
            return (su['clean_alarms'] + sum(a['preattack_alarms'] for a in attacks),
                    sum(a['misses'] for a in attacks),
                    sum(d for a in attacks for d in a['delays_s'] if d is not None))
        chosen = min(passing or summaries, key=rank)
        write(ROOT/'validation.json', dict(rows=rows, candidates=summaries))
        write(ROOT/'selection.json', dict(validation_seeds=seeds, test_seeds=list(range(900,910)),
            candidates=candidates, base=BASE, duration_s=600, onset_s=300, rates=RATES,
            acceptance='zero clean alarms and all attacks detected after onset, each split separately',
            ranking='passing first; then total clean+pre-attack alarms, misses, total detected delay; stable grid order ties',
            validation_pass=bool(passing), frozen_candidate=chosen['candidate'],
            frozen_config=asdict(Config(**BASE, **chosen['candidate'])),
            checkpoint_sha256=hashlib.sha256(Path('results/motionnet.pt').read_bytes()).hexdigest(),
            evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            held_out_used_for_selection=False))
        print(json.dumps(summaries), flush=True)
    else:
        summary = summarize(rows)
        write(ROOT/'metrics.json', dict(selection_sha256=hashlib.sha256((ROOT/'selection.json').read_bytes()).hexdigest(),
            held_out_rows=rows, held_out_summary=summary,
            detection_pass=selection['validation_pass'] and eligible(summary)))
        print(json.dumps(summary), flush=True)

if __name__ == '__main__':
    main()
