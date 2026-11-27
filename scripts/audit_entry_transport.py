"""Offline signed outage decomposition; algebraic transport, not causal attribution."""
import argparse
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.pipeline import run
from ccgins.timing import at_or_after, in_window
from scripts.audit_scheme import budget


def decompose(t, position_error, velocity_error, start, duration, t_start=30.):
    """e(t) = e_p(anchor) + (t-anchor)*e_v(anchor) + remainder.

    Anchor is the last logged epoch at/before outage start, never a future
    sample. The remainder includes all subsequent dynamics AND measurement
    updates (including NHC); it is not an independent sensor-noise component.
    Signed cross terms are retained, so the SSE identity holds under cancellation.
    """
    t, ep, ev = map(np.asarray, (t, position_error, velocity_error))
    if (t.ndim != 1 or not len(t) or ep.shape != (len(t), 3)
            or ev.shape != ep.shape or not np.isfinite(t).all()
            or not np.isfinite(ep).all() or not np.isfinite(ev).all()
            or not np.all(np.diff(t) > 0)):
        raise ValueError('require finite increasing epochs and aligned Nx3 errors')
    if not np.isfinite([start, duration, t_start]).all() or duration <= 0:
        raise ValueError('require finite bounds and positive duration')
    upper_start = np.nextafter(np.nextafter(start, np.inf), np.inf)
    preceding = np.flatnonzero(t <= upper_start)
    mask = in_window(t, start, start + duration) & at_or_after(t, t_start)
    if not len(preceding) or not mask.any():
        raise ValueError('require a preceding anchor and scored outage epochs')
    anchor = preceding[-1]
    elapsed = t[mask] - t[anchor]
    p = np.broadcast_to(ep[anchor], (mask.sum(), 3))
    v = elapsed[:, None] * ev[anchor]
    residual = ep[mask] - p - v
    result = dict(start=float(start), duration=float(duration),
        anchor_t=float(t[anchor]), first_scored_t=float(t[mask][0]),
        last_scored_t=float(t[mask][-1]), n=int(mask.sum()),
        entry_position_error_enu=ep[anchor].tolist(),
        entry_velocity_error_enu=ev[anchor].tolist())
    for name, sl in [('horizontal', slice(0, 2)), ('vertical', slice(2, 3))]:
        a, b, c = (x[:, sl] for x in (p, v, residual))
        terms = dict(entry_position=float(np.sum(a*a)),
            entry_velocity_transport=float(np.sum(b*b)),
            remainder=float(np.sum(c*c)),
            cross_position_velocity=float(2*np.sum(a*b)),
            cross_position_remainder=float(2*np.sum(a*c)),
            cross_velocity_remainder=float(2*np.sum(b*c)))
        total = float(np.sum(ep[mask, sl]**2))
        result[name] = dict(sse=total, terms=terms,
            reconstructed_sse=sum(terms.values()),
            rmse=float(np.sqrt(total/mask.sum())),
            transport_only_rmse=float(np.sqrt(np.mean(np.sum((a+b)**2, axis=1)))),
            remainder_rmse=float(np.sqrt(np.mean(np.sum(c*c, axis=1)))),
            endpoint_error=ep[mask, sl][-1].tolist(),
            endpoint_transport=(a+b)[-1].tolist(),
            endpoint_remainder=c[-1].tolist())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--config', default='configs/example_sim.yaml')
    parser.add_argument('--seeds', nargs='+', type=int, default=[900, 105001, 105002])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config(args.config)
    t_start = float(raw.get('metrics', {}).get('t_start', 60.))
    result = dict(candidate_id='scheme_entry_transport', config=raw, rows=[],
        scope='Offline algebraic decomposition. Correlated signed cross terms must be retained; no causal fraction or counterfactual filter improvement is inferred.')
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        log = run(truth, imu, gnss, spec, gs, _filter_config(raw))
        ep, ev = log.p_est-log.p_true, log.v_est-log.v_true
        rows = [decompose(log.t, ep, ev, start, duration, t_start)
                for start, duration in gs.outages
                if (in_window(log.t, start, start+duration) & at_or_after(log.t, t_start)).any()]
        result['rows'].append(dict(seed=seed, outages=rows,
            segments=budget(log.t, ep, gs.outages, t_start, scenario_end=truth.t[-1])))
        np.savez_compressed(args.out/f'trace_{seed}.npz', t=log.t, ep=ep, ev=ev)
        (args.out/'decomposition.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(seed, flush=True)


if __name__ == '__main__':
    main()
