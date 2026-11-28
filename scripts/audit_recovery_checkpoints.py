"""Measure the effect of withholding the first fix after each example outage."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from ccgins.cli import load_config, _build_inputs, _filter_config
from ccgins.pipeline import run
from scripts.audit_checkpoint_consistency import aggregate

CHECKPOINTS = (89.9, 90., 91., 95., 179.9, 180.)
REACQUISITION = (90., 180.)


def exact_index(times, epoch):
    matches = np.flatnonzero(np.isclose(times, epoch, rtol=0., atol=1e-7))
    if len(matches) != 1:
        raise ValueError(f'expected exactly one sample at {epoch:g} s')
    return int(matches[0])


def checkpoints(log):
    rows = []
    for epoch in CHECKPOINTS:
        k = exact_index(log.t, epoch)
        rows.append(dict(t=epoch, error_enu=(log.p_est[k] - log.p_true[k]).tolist(),
                         covariance=log.P_pos[k].tolist(),
                         gnss_age=float(log.extra['since_gnss'][k])))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seeds', type=int, nargs='+', default=list(range(114101, 114113)))
    args = ap.parse_args()
    if len(set(args.seeds)) != len(args.seeds):
        ap.error('duplicate seeds are not independent runs')
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(config=raw, rows=[],
                  design='Each withheld checkpoint uses a replay with only its associated return fix omitted.')
    dest = args.out / 'recovery.json'
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        cfg = _filter_config(raw)
        cfg.init_seed = seed + 200000
        native = run(truth, imu, gnss, spec, gs, cfg)
        samples = dict(native=checkpoints(native), withheld=[])
        for epoch in REACQUISITION:
            k = exact_index(gnss.t, epoch)
            if not gnss.valid[k] or k == 0 or gnss.valid[k - 1]:
                raise ValueError(f'{epoch:g} s is not a GNSS return epoch')
            valid = gnss.valid.copy()
            valid[k] = False
            withheld = run(truth, imu, replace(gnss, valid=valid), spec, gs, cfg)
            prefix = native.t < epoch
            np.testing.assert_array_equal(native.t, withheld.t)
            np.testing.assert_array_equal(native.p_est[prefix], withheld.p_est[prefix])
            np.testing.assert_array_equal(native.P_pos[prefix], withheld.P_pos[prefix])
            i = CHECKPOINTS.index(epoch)
            if samples['native'][i]['gnss_age'] != 0:
                raise ValueError(f'native GNSS update missing at {epoch:g} s')
            rows = checkpoints(withheld)
            if rows[i]['gnss_age'] <= 0:
                raise ValueError(f'withheld GNSS update applied at {epoch:g} s')
            # Pair each return with a replay that differs at that return only.
            samples['withheld'].extend(row for row in rows if
                                       (row['t'] < 120.) == (epoch == REACQUISITION[0]))
        result['rows'].append(dict(seed=seed, init_seed=cfg.init_seed, **samples))
        dest.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        print(seed, flush=True)
    result['ensemble'] = {
        model: {str(epoch): aggregate([r[model][i] for r in result['rows']])
                for i, epoch in enumerate(CHECKPOINTS)}
        for model in ('native', 'withheld')
    }
    dest.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
