"""Offline GNSS residual covariance budget; no filter tuning or truth-fed updates.

The exact covariance includes the simulator's zero initial multipath state.
Mean uncertainty is for an unweighted position-residual mean, not a navigation
filter covariance or a measurement innovation covariance.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, load_config


def covariance(t, sigma, mp_sigma, tau):
    """Covariance at generated epochs, with MP fixed to zero at t[0]."""
    elapsed = np.asarray(t) - t[0]
    return (sigma**2 * np.eye(len(t)) + mp_sigma**2 *
            (np.exp(-np.abs(elapsed[:, None] - elapsed[None, :]) / tau)
             - np.exp(-(elapsed[:, None] + elapsed[None, :]) / tau)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seeds', type=int, nargs='+', default=[900, 97001, 97002])
    ap.add_argument('--draws', type=int, default=8192)
    args = ap.parse_args()
    if args.draws < 2:
        ap.error('--draws must be at least 2')
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(config=raw, draws=args.draws, monte_carlo_seed=97003,
                  scope='GNSS measurement residuals only; not innovations or online NEES', rows=[])
    for seed in args.seeds:
        truth, _, gnss, _, spec = _build_inputs(raw, seed_override=seed)
        antenna = truth.p[gnss.idx] + np.einsum('nij,j->ni', truth.R[gnss.idx], spec.lever_arm)
        residual = gnss.pos - antenna
        masks = {'available': gnss.valid & (gnss.t >= 30),
                 'available_30_60': (gnss.t >= 30) & (gnss.t < 60) & gnss.valid,
                 'available_90_120': (gnss.t >= 90) & (gnss.t < 120) & gnss.valid}
        # Independent vectorized recurrence verifies covariance predictions.
        # MP evolves through invalid epochs, matching the simulator.
        rng = np.random.default_rng(97003)
        mp = np.zeros((args.draws, len(gnss.t)))
        for k in range(1, len(gnss.t)):
            a = np.exp(-(gnss.t[k] - gnss.t[k-1]) / spec.mp_tau)
            mp[:, k] = a * mp[:, k-1] + np.sqrt(1-a*a) * spec.mp_sigma * rng.standard_normal(args.draws)
        draws = {axis: mp + rng.standard_normal(mp.shape) * sigma
                 for axis, sigma in [('h_axis', spec.sigma_h), ('v', spec.sigma_v)]}
        row = dict(seed=seed, segments={})
        for name, mask in masks.items():
            n = int(mask.sum())
            axes = {}
            for axis, sigma in [('h_axis', spec.sigma_h), ('v', spec.sigma_v)]:
                C = covariance(gnss.t, sigma, spec.mp_sigma, spec.mp_tau)[np.ix_(mask, mask)]
                mean_var = float(C.sum() / n**2)
                marginal = float(np.trace(C) / n)
                axes[axis] = dict(expected_marginal_variance=marginal,
                    expected_mean_variance=mean_var,
                    white_only_mean_variance=sigma**2/n,
                    independent_inflated_mean_variance=marginal/n,
                    effective_n=marginal/mean_var,
                    mc_marginal_variance=float(np.mean(draws[axis][:, mask]**2)),
                    mc_mean_variance=float(np.mean(draws[axis][:, mask].mean(axis=1)**2)))
            e = residual[mask]
            row['segments'][name] = dict(n=n, mean_enu=e.mean(axis=0).tolist(),
                measured_rmse_h=float(np.sqrt(np.mean(np.sum(e[:, :2]**2, axis=1)))),
                measured_rmse_v=float(np.sqrt(np.mean(e[:, 2]**2))), axes=axes)
        result['rows'].append(row)
        np.savez_compressed(args.out / f'residual_{seed}.npz', t=gnss.t,
                            residual=residual, valid=gnss.valid)
        (args.out / 'colored.json').write_text(json.dumps(result, indent=2) + '\n')
        print(seed, row['segments']['available'], flush=True)


if __name__ == '__main__':
    main()
