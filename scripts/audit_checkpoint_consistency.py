"""Independent-run position checkpoints, paired multipath diagnostic; no tuning."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
from scipy.stats import chi2

from ccgins.cli import load_config, _build_inputs, _filter_config
from ccgins.pipeline import run
from ccgins.sim import simulate_gnss
from scripts.audit_calibration import position_consistency


def aggregate(samples):
    """Samples must be independent runs at one epoch, never successive epochs."""
    error = np.asarray([s['error_enu'] for s in samples])
    covariance = np.asarray([s['covariance'] for s in samples])
    h = np.einsum('ni,ni->n', error[:, :2],
                  np.linalg.solve(covariance[:, :2, :2], error[:, :2, None])[..., 0])
    v = error[:, 2]**2 / covariance[:, 2, 2]
    n = len(samples)
    return dict(n=n, rmse_h=float(np.sqrt(np.mean(np.sum(error[:, :2]**2, axis=1)))),
                rmse_v=float(np.sqrt(np.mean(error[:, 2]**2))),
                mean_error_enu=error.mean(axis=0).tolist(),
                mean_covariance=covariance.mean(axis=0).tolist(),
                mean_error_outer=(error.T @ error / n).tolist(),
                horizontal_nees_per_dim=float(h.mean()/2),
                vertical_nees=float(v.mean()),
                horizontal_95pct_coverage=float(np.mean(h <= chi2.ppf(.95, 2))),
                vertical_95pct_coverage=float(np.mean(v <= chi2.ppf(.95, 1))),
                ideal_gaussian_mean_h_nees_per_dim_95_interval=(chi2.ppf([.025, .975], 2*n)/(2*n)).tolist(),
                ideal_gaussian_mean_v_nees_95_interval=(chi2.ppf([.025, .975], n)/n).tolist())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seeds', nargs='+', type=int, default=list(range(98101, 98113)))
    args = ap.parse_args()
    if len(set(args.seeds)) != len(args.seeds):
        ap.error('duplicate seeds are not independent runs')
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_checkpoint_consistency', config=raw,
                  scope='Independent trajectory/sensor/alignment runs; paired models are dependent. Ideal Gaussian intervals are diagnostic references, not proof of consistency or tail gates.', rows=[])
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        for model in ('nominal', 'no_multipath'):
            cfg = _filter_config(raw)
            cfg.init_seed = seed + 200000
            measurements = gnss if model == 'nominal' else simulate_gnss(truth, replace(gs, mp_sigma=0.), seed=seed)
            log = run(truth, imu, measurements, spec, gs, cfg)
            result['rows'].append(dict(seed=seed, init_seed=cfg.init_seed, model=model,
                                      **position_consistency(log, gs.outages, raw['metrics']['t_start'],
                                              scenario_end=truth.t[-1])))
            (args.out/'checkpoints.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            print(seed, model, flush=True)
    result['ensemble'] = {}
    for model in ('nominal', 'no_multipath'):
        rows = [r for r in result['rows'] if r['model'] == model]
        result['ensemble'][model] = {str(t): aggregate([next(c for c in r['checkpoints'] if c['t'] == t) for r in rows])
                                     for t in (30., 59.9, 89.9, 119.9, 179.9)}
    (args.out/'checkpoints.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
