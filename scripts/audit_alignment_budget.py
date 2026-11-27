"""Independent alignment draws on paired sensor trajectories; offline audit only."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.pipeline import initial_state, run
from scripts.audit_scheme import budget


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[900, 95001, 95002])
    parser.add_argument('--init-seeds', type=int, nargs='+', default=[0, 95011, 95012])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_alignment_budget', config=raw,
                  scope='Paired sensor inputs; sampled truth-relative alignment, not a deployable alignment or tail bound.',
                  production_changed=False, rows=[])
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        for init in args.init_seeds:
            cfg = replace(_filter_config(raw), init_seed=init)
            R0, v0, p0, _ = initial_state(truth, init, cfg.p0_bias)
            log = run(truth, imu, gnss, spec, gs, cfg)
            error = log.p_est - log.p_true
            row = dict(seed=seed, init_seed=init,
                       initial_rotation_error_matrix=(R0 @ truth.R[0].T).tolist(),
                       initial_velocity_error=(v0-truth.v[0]).tolist(),
                       initial_position_error=(p0-truth.p[0]).tolist(),
                       segments=budget(log.t, error, gs.outages, raw['metrics']['t_start']), bins=[])
            for start in range(30, 180, 10):
                mask = (log.t >= start) & (log.t < start+10)
                row['bins'].append(dict(start=start, **budget(log.t[mask], error[mask], [], start)['overall']))
            np.savez_compressed(args.out / f'trace_{seed}_{init}.npz', t=log.t, error=error)
            result['rows'].append(row)
            (args.out / 'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            print(seed, init, row['segments']['overall']['rmse_h'], flush=True)


if __name__ == '__main__':
    main()
