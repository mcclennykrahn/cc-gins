"""Conditional attitude/bias factorial; privileged offline diagnostic only.

All arms retain the native invariant covariance numerically, without conditioning
or recentering. This tests sensitivity of the specified intervention procedure,
not consistent oracle estimation, causal error fractions or an online algorithm.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.pipeline import run
from ccgins.timing import in_window, at_or_after
from scripts.audit_entry_state import replay, verify_prefix
from scripts.audit_scheme import budget


def interaction(t, errors, start, duration, t_start):
    """Signed SSE factorial contrast, keeping vector interaction separate.

    Even additive vector changes can yield nonzero SSE contrasts through their
    cross product. Never interpret the SSE contrast alone as dynamic coupling.
    """
    required = {'native', 'attitude', 'biases', 'attitude_and_biases'}
    t = np.asarray(t)
    if (set(errors) != required or t.ndim != 1 or not len(t)
            or not np.isfinite(t).all() or not np.all(np.diff(t) > 0)):
        raise ValueError('require increasing epochs and all four factorial cells')
    arrays = {k: np.asarray(v, dtype=float) for k, v in errors.items()}
    if any(v.shape != (len(t), 3) or not np.isfinite(v).all() for v in arrays.values()):
        raise ValueError('require finite aligned Nx3 errors')
    if not np.isfinite([start, duration, t_start]).all() or duration <= 0:
        raise ValueError('require finite bounds and positive duration')
    mask = in_window(t, start, start + duration) & at_or_after(t, t_start)
    if not mask.any():
        raise ValueError('no scored outage samples')
    result = dict(n=int(mask.sum()))
    for axis, sl in [('horizontal', slice(0, 2)), ('vertical', slice(2, 3))]:
        e = {k: v[mask, sl] for k, v in arrays.items()}
        sse = {k: float(np.sum(v*v)) for k, v in e.items()}
        vector = e['attitude_and_biases'] - e['attitude'] - e['biases'] + e['native']
        contrast = sse['attitude_and_biases'] - sse['attitude'] - sse['biases'] + sse['native']
        result[axis] = dict(sse=sse, sse_contrast=contrast,
            mse_contrast=contrast / mask.sum(),
            vector_interaction_rms=float(np.sqrt(np.mean(np.sum(vector*vector, axis=1)))))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[900, 113001, 113002])
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds):
        parser.error('duplicate scenario seeds')
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_joint_entry_bias', config=raw, rows=[],
        scope=__doc__, initialization='Native truth-relative init_seed=0; cells and gaps within each scenario are dependent.')
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        cfg = _filter_config(raw)
        native = run(truth, imu, gnss, spec, gs, cfg)
        scoring = dict(t_start=raw['metrics']['t_start'], scenario_end=truth.t[-1])
        for start, duration in gs.outages:
            errors = dict(native=native.p_est - native.p_true)
            entry = {}
            for arm in ('attitude', 'biases', 'attitude_and_biases'):
                altered, evidence = replay(truth, imu, gnss, spec, gs, cfg, start, duration, arm)
                evidence['identical_prefix_epochs'] = verify_prefix(native, altered, start)
                entry[arm] = evidence
                errors[arm] = altered.p_est - altered.p_true
            row = dict(seed=seed, start=start, duration=duration, entry=entry,
                segments={arm: budget(native.t, error, gs.outages, **scoring)
                          for arm, error in errors.items()},
                interaction=interaction(native.t, errors, start, duration, raw['metrics']['t_start']))
            result['rows'].append(row)
            np.savez_compressed(args.out / f'{seed}_{start:g}.npz', t=native.t, **errors)
            (args.out / 'factorial.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            print(seed, start, row['interaction'], flush=True)


if __name__ == '__main__':
    main()
