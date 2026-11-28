"""Reproducible segment error budget; no tuning or truth-fed filter updates."""
import argparse
import json
from pathlib import Path
import numpy as np
from ccgins.cli import load_config, _build_inputs, _filter_config
from ccgins.pipeline import run
from ccgins.filters import mechanize
from ccgins.sim import G_NAV
from ccgins.timing import at_or_after, in_window


def segment_masks(t, outages, t_start=30., *, scenario_end=None):
    """Half-open outages, with a separate row for a logged scenario endpoint."""
    end = float(t[-1]) if scenario_end is None else scenario_end
    selected = at_or_after(t, t_start)
    outage = np.zeros(len(t), dtype=bool)
    masks = {}
    for start, duration in outages:
        mask = in_window(t, start, start + duration)
        outage |= mask
        masks[f'outage_{start:g}_{start+duration:g}'] = mask & selected
    masks['available'] = selected & ~outage
    masks['overall'] = selected
    # Disjoint available intervals expose recovery transients.
    boundaries = sorted({t_start, end} | {
        float(x) for start, duration in outages for x in (start, start + duration)
        if t_start <= x <= end
    })
    for a, b in zip(boundaries[:-1], boundaries[1:]):
        mask = selected & ~outage & in_window(t, a, b)
        if mask.any():
            masks[f'available_{a:g}_{b:g}'] = mask
    terminal = masks['available'] & at_or_after(t, end)
    if terminal.any():
        masks['terminal_available'] = terminal
    return masks


def budget(t, error, outages, t_start=30., *, scenario_end=None):
    if not len(t) or not at_or_after(t, t_start).any():
        raise ValueError('no epochs in scoring window')
    masks = segment_masks(t, outages, t_start, scenario_end=scenario_end)
    total = np.sum(error[masks['overall'], :2]**2)
    rows = {}
    for name, mask in masks.items():
        e = error[mask]
        if not len(e):
            continue
        h2 = np.sum(e[:, :2]**2, axis=1)
        rows[name] = dict(n=len(e), rmse_h=float(np.sqrt(h2.mean())),
                          rmse_v=float(np.sqrt(np.mean(e[:, 2]**2))),
                          max_h=float(np.sqrt(h2.max())), end_h=float(np.sqrt(h2[-1])),
                          horizontal_sse_fraction=float(h2.sum()/total) if total else None)
    return rows


def contained_steps(truth, start, duration):
    """Indices bounding complete IMU steps contained in a requested interval."""
    end = start + duration
    if duration <= 0 or not np.isfinite([start, end]).all():
        raise ValueError("pure INS outage must have finite bounds and positive duration")
    if not at_or_after(start, truth.t[0]) or not at_or_after(truth.t[-1], end):
        raise ValueError("pure INS outage must lie within the truth trajectory")
    first = int(np.flatnonzero(at_or_after(truth.t, start))[0])
    last = int(np.flatnonzero(at_or_after(truth.t, end))[0])
    if not at_or_after(end, truth.t[last]):
        last -= 1
    if last <= first:
        raise ValueError("pure INS outage contains no complete IMU step")
    return first, last


def pure_ins_budget(truth, imu, outages, *, earth=None):
    """Propagate complete steps inside requested outages; report actual grid bounds.

    Narrow off-grid bounds rather than inventing fractional IMU samples.
    This is an exact-truth-initialized diagnostic, never an online result.
    """
    if earth is None:
        earth = {'w_ie': None, 'g': G_NAV}
    rows = []
    for start, duration in outages:
        first, last = contained_steps(truth, start, duration)
        R, v, p = truth.R[first].copy(), truth.v[first].copy(), truth.p[first].copy()
        errors = []
        for k in range(first, last):
            R, v, p = mechanize(R, v, p, imu.gyro[k], imu.accel[k], truth.dt,
                                earth=earth)
            errors.append(p - truth.p[k + 1])
        e = np.asarray(errors)
        rows.append(dict(start=start, duration=duration,
            actual_start=float(truth.t[first]), actual_end=float(truth.t[last]),
            propagated_duration=float(truth.t[last] - truth.t[first]), n_steps=last-first,
            rmse_h=float(np.sqrt(np.mean(np.sum(e[:, :2]**2, axis=1)))),
            end_h=float(np.linalg.norm(e[-1, :2])), end_v=float(abs(e[-1, 2])),
            mean_speed=float(np.mean(truth.v_body[first:last, 0]))))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--config', default='configs/example_sim.yaml')
    ap.add_argument('--seeds', type=int, nargs='+', default=[900, 49501, 49502])
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    raw = load_config(args.config)
    t_start = float(raw.get('metrics', {}).get('t_start', 60.))
    if not np.isfinite(t_start) or t_start < 0:
        raise ValueError('metrics.t_start must be finite and non-negative')
    result = dict(config=raw, scoring=f'IMU origin, same logged epoch; t>={t_start:g}; half-open outages', rows=[])
    cfg = _filter_config(raw)
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        log = run(truth, imu, gnss, spec, gs, cfg)
        error = log.p_est-log.p_true
        ant = truth.p[gnss.idx] + np.einsum('nij,j->ni', truth.R[gnss.idx], gs.lever_arm)
        ge = gnss.pos-ant
        gmask = gnss.valid & at_or_after(gnss.t, t_start)
        row = dict(seed=seed, segments=budget(log.t, error, gs.outages, t_start,
                                             scenario_end=truth.t[-1]),
                   gnss_only=(budget(gnss.t[gmask], ge[gmask], [], t_start)['overall']
                              if gmask.any() else None))
        row['pure_ins_truth_initialized'] = pure_ins_budget(truth, imu, gs.outages,
                                                           earth=cfg.earth_model())
        row['bins'] = []
        for s in np.arange(t_start, truth.t[-1], 10.):
            m = in_window(log.t, s, min(s + 10., truth.t[-1]))
            if m.any():
                row['bins'].append(dict(start=float(s), **budget(log.t[m], error[m], [], s)['overall']))
        result['rows'].append(row)
        np.savetxt(out/f'errors_{seed}.csv', np.column_stack([log.t,error]),
                   delimiter=',', header='t,error_e,error_n,error_u', comments='')
        (out/'budget.json').write_text(json.dumps(result, indent=2)+'\n')
        print(seed, row['segments'], flush=True)

if __name__ == '__main__':
    main()
