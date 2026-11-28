"""Cross initialization draws with fixed sensor/trajectory draws; descriptive only."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.pipeline import run
from scripts.audit_scheme import budget


def summarize_crossed(rows):
    """Conditional ranges within scenario; crossed cells are not independent runs."""
    keys = [(r['seed'], r['init_seed']) for r in rows]
    if not keys or len(set(keys)) != len(keys):
        raise ValueError('require nonempty unique scenario/initialization cells')
    seeds = sorted({s for s, _ in keys})
    inits = sorted({i for _, i in keys})
    if set(keys) != {(s, i) for s in seeds for i in inits}:
        raise ValueError('require a complete crossed grid')
    result = {}
    for seed in seeds:
        cells = [r for r in rows if r['seed'] == seed]
        names = set(cells[0]['segments'])
        if any(set(r['segments']) != names for r in cells):
            raise ValueError('inconsistent segment names')
        result[str(seed)] = {}
        for name in sorted(names):
            values = np.asarray([r['segments'][name]['rmse_h'] for r in cells])
            if not np.isfinite(values).all() or (values < 0).any():
                raise ValueError('invalid horizontal RMSE')
            result[str(seed)][name] = dict(min=float(values.min()),
                max=float(values.max()), span=float(np.ptp(values)))
    return dict(n_scenarios=len(seeds), n_initializations=len(inits),
                n_cells=len(rows), conditional_horizontal_rmse=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[900, 112001, 112002])
    parser.add_argument('--init-seeds', type=int, nargs='+', default=[0, 112101, 112102])
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds) or len(set(args.init_seeds)) != len(args.init_seeds):
        parser.error('duplicate seeds')
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_crossed_initialization', config=raw, rows=[],
        scope='Truth-relative initialization sensitivity, not sensor-only alignment. Cells share sensor draws within scenario and initialization draws across scenarios; no independent tail sample or additive attribution.')
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        for init in args.init_seeds:
            cfg = replace(_filter_config(raw), init_seed=init)
            log = run(truth, imu, gnss, spec, gs, cfg)
            error = log.p_est - log.p_true
            segments = budget(log.t, error, gs.outages, raw['metrics']['t_start'],
                              scenario_end=truth.t[-1])
            result['rows'].append(dict(seed=seed, init_seed=init, segments=segments))
            np.savez_compressed(args.out/f'trace_{seed}_{init}.npz', t=log.t, error=error)
            (args.out/'crossed.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            print(seed, init, segments['overall']['rmse_h'], flush=True)
    result['summary'] = summarize_crossed(result['rows'])
    (args.out/'crossed.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
