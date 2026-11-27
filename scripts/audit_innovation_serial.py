"""Read-only GNSS innovation serial-moment diagnostic for the example scheme."""
import argparse
from contextlib import contextmanager
from dataclasses import replace
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.filters import NavFilter
from ccgins.pipeline import run
from ccgins.sim import simulate_gnss
from scripts.audit_scheme import budget


@contextmanager
def capture_innovations(times):
    """Single-filter diagnostic only; callers verify valid-fix count and replay."""
    original = NavFilter.update_gnss
    rows = []

    def update(filt, z, lever, Rm, **kwargs):
        h, H = filt.gnss_model(lever)
        residual = z - h
        covariance = H @ filt.P @ H.T + Rm
        white = np.linalg.solve(np.linalg.cholesky(covariance), residual)
        info = original(filt, z, lever, Rm, **kwargs)
        rows.append(dict(t=float(times[len(rows)]), residual=residual.tolist(),
                         covariance=covariance.tolist(), whitened=white.tolist(),
                         nis=float(info.nis), accepted=bool(info.accepted),
                         forced=bool(kwargs.get('force', False))))
        return info

    with patch.object(NavFilter, 'update_gnss', update):
        yield rows
    if len(rows) != len(times):
        raise AssertionError('diagnostic must capture exactly all valid fixes')


def serial_moments(rows, start, end, period=1.):
    """Uncentered moments: calibrated innovations have mean0, covarianceI, lag0.

    No rejected-fix selection and no pairs across outages or interval boundaries.
    Time samples are dependent: no iid confidence interval or whiteness p-value.
    """
    selected = [r for r in rows if start <= r['t'] < end]
    if not selected:
        return dict(n=0, pairs=0)
    u = np.asarray([r['whitened'] for r in selected])
    t = np.asarray([r['t'] for r in selected])
    paired = np.isclose(np.diff(t), period, rtol=0., atol=1e-7)
    lag = u[:-1][paired].T @ u[1:][paired] / paired.sum() if paired.any() else None
    return dict(n=len(u), pairs=int(paired.sum()), mean=u.mean(axis=0).tolist(),
                second_moment=(u.T @ u / len(u)).tolist(),
                lag_one_cross_moment=None if lag is None else lag.tolist(),
                rejected=sum(not r['accepted'] for r in selected),
                forced=sum(r['forced'] for r in selected))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seeds', nargs='+', type=int, default=list(range(99101, 99109)))
    args = ap.parse_args()
    if len(set(args.seeds)) != len(args.seeds):
        ap.error('duplicate seeds')
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_innovation_serial', config=raw, rows=[],
                  scope='Paired MP ablation; uncentered serial moments, no iid significance or parameter promotion.')
    for seed in args.seeds:
        truth, imu, nominal, spec, gs = _build_inputs(raw, seed_override=seed)
        cfg = replace(_filter_config(raw), init_seed=seed + 300000)
        if cfg.monitor != 'none' or cfg.adaptive_r or cfg.gnss_aci or cfg.baseline_range_anchor is not None:
            raise ValueError('requires single unadapted vector-GNSS filter')
        for model in ('nominal', 'no_multipath'):
            gnss = nominal if model == 'nominal' else simulate_gnss(truth, replace(gs, mp_sigma=0.), seed=seed)
            native = run(truth, imu, gnss, spec, gs, cfg)
            with capture_innovations(gnss.t[gnss.valid]) as innovations:
                log = run(truth, imu, gnss, spec, gs, cfg)
            for field in ('t', 'p_est', 'v_est', 'P_pos'):
                np.testing.assert_array_equal(getattr(native, field), getattr(log, field))
            result['rows'].append(dict(seed=seed, init_seed=cfg.init_seed, model=model,
                exact_native_replay=True, innovations=innovations,
                segments=budget(log.t, log.p_est-log.p_true, gs.outages, raw['metrics']['t_start']),
                intervals={f'{a}_{b}': serial_moments(innovations, a, b)
                           for a, b in ((30, 60), (90, 120))}))
            (args.out/'innovations.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            print(seed, model, flush=True)


if __name__ == '__main__':
    main()
