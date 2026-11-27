"""Reproducible segment error budget; no tuning or truth-fed filter updates."""
import argparse
import json
from pathlib import Path
import numpy as np
from ccgins.cli import load_config, _build_inputs, _filter_config
from ccgins.pipeline import run
from ccgins.filters import mechanize


def budget(t, error, outages, t_start=30.):
    """Half-open outages; terminal reacquisition is a separate available sample."""
    selected = t >= t_start
    outage = np.zeros(len(t), dtype=bool)
    masks = {}
    for start, duration in outages:
        mask = (t >= start) & (t < start + duration)
        outage |= mask
        masks[f'outage_{start:g}_{start+duration:g}'] = mask & selected
    masks['available'] = selected & ~outage
    masks['overall'] = selected
    # Disjoint available intervals expose recovery transients.
    boundaries = sorted({t_start, float(t[-1])} | {float(x) for sd in outages for x in (sd[0], sum(sd)) if t_start <= x <= t[-1]})
    for a, b in zip(boundaries[:-1], boundaries[1:]):
        mask = selected & ~outage & (t >= a) & (t < b)
        if mask.any():
            masks[f'available_{a:g}_{b:g}'] = mask
    if selected[-1] and not outage[-1]:
        masks['terminal_available'] = t == t[-1]
    total = np.sum(error[selected, :2]**2)
    rows = {}
    for name, mask in masks.items():
        e = error[mask]
        if not len(e):
            continue
        h2 = np.sum(e[:, :2]**2, axis=1)
        rows[name] = dict(n=len(e), rmse_h=float(np.sqrt(h2.mean())),
                          rmse_v=float(np.sqrt(np.mean(e[:, 2]**2))),
                          max_h=float(np.sqrt(h2.max())), end_h=float(np.sqrt(h2[-1])),
                          horizontal_sse_fraction=float(h2.sum()/total))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seeds', type=int, nargs='+', default=[900, 49501, 49502])
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(config=raw, scoring='IMU origin, same logged epoch; t>=30; half-open outages', rows=[])
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        log = run(truth, imu, gnss, spec, gs, _filter_config(raw))
        error = log.p_est-log.p_true
        ant = truth.p[gnss.idx] + np.einsum('nij,j->ni', truth.R[gnss.idx], gs.lever_arm)
        ge = gnss.pos-ant
        gmask = gnss.valid & (gnss.t >= 30.)
        row = dict(seed=seed, segments=budget(log.t, error, gs.outages),
                   gnss_only=budget(gnss.t[gmask], ge[gmask], [], 30.)['overall'])
        row['pure_ins_truth_initialized'] = []
        for start, duration in gs.outages:
            first, last = int(round(start/truth.dt)), int(round((start+duration)/truth.dt))
            R, v, p = truth.R[first].copy(), truth.v[first].copy(), truth.p[first].copy()
            errors = []
            for k in range(first, last):
                R, v, p = mechanize(R, v, p, imu.gyro[k], imu.accel[k], truth.dt,
                                    earth={'w_ie': None, 'g': np.array([0.,0.,-9.81])})
                errors.append(p-truth.p[k+1])
            e = np.asarray(errors)
            row['pure_ins_truth_initialized'].append(dict(start=start, duration=duration,
                rmse_h=float(np.sqrt(np.mean(np.sum(e[:,:2]**2,axis=1)))),
                end_h=float(np.linalg.norm(e[-1,:2])), end_v=float(abs(e[-1,2])),
                mean_speed=float(np.mean(truth.v_body[first:last,0]))))
        row['bins'] = []
        for s in range(30, 180, 10):
            m=(log.t >= s)&(log.t < s+10)
            row['bins'].append(dict(start=s, **budget(log.t[m], error[m], [], s)['overall']))
        result['rows'].append(row)
        np.savetxt(out/f'errors_{seed}.csv', np.column_stack([log.t,error]),
                   delimiter=',', header='t,error_e,error_n,error_u', comments='')
        (out/'budget.json').write_text(json.dumps(result, indent=2)+'\n')
        print(seed, row['segments'], flush=True)

if __name__ == '__main__':
    main()
