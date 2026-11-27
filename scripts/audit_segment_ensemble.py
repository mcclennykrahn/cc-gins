"""Finite independent scenario/alignment draws; descriptive ranges, not tail gates."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.pipeline import run
from ccgins.timing import in_window
from scripts.audit_scheme import budget, pure_ins_budget


def summarize(rows):
    """Equal weight per run; never pool epochs as independent trials."""
    if not rows or len({r['seed'] for r in rows}) != len(rows):
        raise ValueError('require nonempty distinct scenario seeds')
    names = set.intersection(*(set(r['segments']) for r in rows))
    result = {}
    for name in sorted(names):
        result[name] = {}
        for metric in ('rmse_h', 'rmse_v', 'max_h', 'horizontal_sse_fraction'):
            values = np.asarray([r['segments'][name][metric] for r in rows], float)
            if not np.isfinite(values).all():
                raise ValueError('nonfinite segment metric')
            result[name][metric] = dict(zip(('min', 'median', 'max'),
                map(float, (values.min(), np.median(values), values.max()))))
    return dict(n_runs=len(rows), segments=result,
                long_outage_dominates_count=sum(
                    r['segments']['outage_120_180']['horizontal_sse_fraction'] > .5
                    for r in rows),
                available_h_at_most_1m_count=sum(
                    r['segments']['available']['rmse_h'] <= 1. for r in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=list(range(104101, 104117)))
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds):
        parser.error('duplicate scenario seeds')
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_segment_ensemble', config=raw, rows=[],
        production_changed=False,
        scope='Independent scenario seeds and init_seed=seed+1000000; truth-relative initialization remains optimistic. Min/median/max are descriptive, not confidence or tail bounds.')
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        cfg = replace(_filter_config(raw), init_seed=seed+1000000)
        log = run(truth, imu, gnss, spec, gs, cfg)
        error = log.p_est-log.p_true
        antenna = truth.p[gnss.idx] + np.einsum('nij,j->ni', truth.R[gnss.idx], gs.lever_arm)
        mask = gnss.valid & (gnss.t >= raw['metrics']['t_start'])
        row = dict(seed=seed, init_seed=cfg.init_seed,
            segments=budget(log.t, error, gs.outages, raw['metrics']['t_start'], scenario_end=truth.t[-1]),
            gnss_only=budget(gnss.t[mask], (gnss.pos-antenna)[mask], [], raw['metrics']['t_start'])['overall'],
            pure_ins=pure_ins_budget(truth, imu, gs.outages), bins=[])
        for start in np.arange(raw['metrics']['t_start'], truth.t[-1], 10.):
            mask = in_window(log.t, start, min(start+10., truth.t[-1]))
            row['bins'].append(dict(start=float(start), **budget(log.t[mask], error[mask], [], start)['overall']))
        np.savez_compressed(args.out/f'trace_{seed}.npz', t=log.t, error=error)
        result['rows'].append(row)
        result['summary'] = summarize(result['rows'])
        (args.out/'ensemble.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(seed, row['segments']['overall']['rmse_h'], flush=True)


if __name__ == '__main__':
    main()
