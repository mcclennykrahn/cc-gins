"""Paired colored-GNSS ablation; offline evidence, no navigation tuning."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from ccgins.cli import load_config, _build_inputs, _filter_config
from ccgins.pipeline import run
from ccgins.sim import simulate_gnss
from scripts.audit_scheme import budget, segment_masks


def position_consistency(log, outages, t_start=30., *, scenario_end=None):
    error = log.p_est - log.p_true
    nees = np.einsum('ni,ni->n', error,
                     np.linalg.solve(log.P_pos, error[..., None])[..., 0])
    rows = budget(log.t, error, outages, t_start, scenario_end=scenario_end)
    masks = segment_masks(log.t, outages, t_start, scenario_end=scenario_end)
    for name, mask in masks.items():
        if not mask.any():
            continue
        rows[name].update(position_nees_per_dim=float(nees[mask].mean()/3),
                          position_95pct_coverage=float(np.mean(nees[mask] <= 7.814727903251179)))
    checkpoints = []
    for t in (30., 59.9, 89.9, 119.9, 179.9):
        k = np.flatnonzero(np.isclose(log.t, t, rtol=0, atol=1e-7))
        if len(k):
            i = k[0]
            checkpoints.append(dict(t=t, position_nees_per_dim=float(nees[i]/3),
                                    error_enu=error[i].tolist(), covariance=log.P_pos[i].tolist()))
    return dict(segments=rows, checkpoints=checkpoints)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seeds', type=int, nargs='+', default=[900, 73101, 73102, 73103, 73104, 73105])
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_multipath_calibration', config=raw,
                  scope='Diagnostic model ablation, not a promoted filter; paired initializations reuse sensors; time samples correlated.', rows=[])
    for seed in args.seeds:
        truth, imu, nominal, spec, gs = _build_inputs(raw, seed_override=seed)
        white = simulate_gnss(truth, replace(gs, mp_sigma=0.), seed=seed)
        # Each trajectory has a fixed reference and an independently varied alignment.
        for init in (0, seed + 100):
            for label, gnss in [('nominal', nominal), ('no_multipath', white)]:
                cfg = _filter_config(raw)
                cfg.init_seed = init
                log = run(truth, imu, gnss, spec, gs, cfg)
                row = dict(seed=seed, init_seed=init, model=label,
                           **position_consistency(log, gs.outages, raw['metrics']['t_start'],
                                              scenario_end=truth.t[-1]))
                result['rows'].append(row)
                (out/'calibration.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
                print(seed, init, label, row['segments']['overall']['rmse_h'], flush=True)


if __name__ == '__main__':
    main()
