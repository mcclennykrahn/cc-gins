"""Factorial diagnostic of fixed motion constraints and synthetic GNSS multipath."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.learning import WINDOW
from ccgins.pipeline import run
from ccgins.sim import simulate_gnss
from scripts.audit_calibration import position_consistency
from scripts.audit_scheme import budget


def constraint_truth(truth, cfg, outages):
    """Measure model residuals at the fixed-NHC update epochs, offline only."""
    indices = np.arange(cfg.pseudo_every, len(truth.t), cfg.pseudo_every)
    indices = indices[indices >= WINDOW]
    times = truth.t[indices]
    velocity = truth.v_body[indices, 1:]
    rows = {}
    boundaries = [30., *(edge for start, duration in outages for edge in (start, start + duration))]
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        values = velocity[(times >= start) & (times < end)]
        centered = values - values.mean(axis=0)
        rows[f'{start:g}_{end:g}'] = {
            'n': len(values),
            'mean': values.mean(axis=0).tolist(),
            'rms': np.sqrt(np.mean(values**2, axis=0)).tolist(),
            'lag1_autocorrelation': [
                float(np.corrcoef(channel[:-1], channel[1:])[0, 1])
                if np.std(channel[:-1]) > 0 and np.std(channel[1:]) > 0 else None
                for channel in centered.T
            ],
        }
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', nargs='+', type=int, default=[900, 78201, 78202, 78203, 78204])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    cfg = _filter_config(raw)
    if cfg.pseudo != 'nhc' or cfg.mount_py is not None:
        raise ValueError('This diagnostic requires fixed NHC in the simulated body frame')
    findings = {'config': raw, 'rows': [], 'truth_residuals': {}}
    for seed in args.seeds:
        truth, imu, nominal, imu_spec, gnss_spec = _build_inputs(raw, seed_override=seed)
        outages = gnss_spec.outages
        findings['truth_residuals'][str(seed)] = constraint_truth(truth, cfg, outages)
        white = simulate_gnss(truth, replace(gnss_spec, mp_sigma=0.), seed=seed)
        for multipath, gnss in ((True, nominal), (False, white)):
            for pseudo in ('nhc', 'none'):
                log = run(truth, imu, gnss, imu_spec, gnss_spec, replace(cfg, pseudo=pseudo))
                error = log.p_est - log.p_true
                # Export signed errors and physical covariance for independent metric recomputation.
                np.savez_compressed(
                    args.out / f'trace_{seed}_{int(multipath)}_{pseudo}.npz',
                    t=log.t, error=error, position_covariance=log.P_pos,
                )
                row = dict(seed=seed, init_seed=cfg.init_seed, multipath=multipath,
                           pseudo=pseudo, **position_consistency(log, outages), bins=[])
                for start in range(30, 180, 10):
                    mask = (log.t >= start) & (log.t < start + 10)
                    row['bins'].append(dict(start=start, **budget(
                        log.t[mask], error[mask], [], start,
                    )['overall']))
                findings['rows'].append(row)
                (args.out / 'metrics.json').write_text(
                    json.dumps(findings, indent=2, allow_nan=False) + '\n',
                )
                print(seed, multipath, pseudo, row['segments']['overall']['rmse_h'], flush=True)


if __name__ == '__main__':
    main()
