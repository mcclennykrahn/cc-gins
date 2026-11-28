"""Offline process-noise schedule isolation, never an online-filter promotion.

The oracle uses truth speed solely to isolate the known noise mismatch. The
prior-state schedule uses only the pre-propagation estimate but ignores its
uncertainty; it is a diagnostic, not a calibrated deployable model.
"""
import argparse
from contextlib import contextmanager
from dataclasses import replace
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

from ccgins.cli import _build_inputs, _filter_config, load_config
from ccgins.filters import NavFilter, NoiseParams
from ccgins.pipeline import run
from scripts.audit_calibration import position_consistency
from scripts.audit_scheme import budget


@contextmanager
def noise_schedule(spec, dt, mode, truth_speed):
    """Scoped instrumentation of the single-filter, monitor=none diagnostic."""
    if mode not in ('fixed12', 'oracle', 'prior_state'):
        raise ValueError('unknown schedule')
    original = NavFilter.propagate
    steps = []

    def propagate(filt, gyro, accel, step_dt):
        if step_dt != dt:
            raise ValueError('schedule requires a fixed IMU dt')
        k = len(steps)
        if mode == 'fixed12':
            speed = 12.
        elif mode == 'oracle':
            speed = float(abs(truth_speed[k]))
        else:
            speed = float(abs((filt.R.T @ filt.v)[0]))
        filt.noise = NoiseParams.from_spec(spec, vmax_eff=speed, dt=dt)
        steps.append(speed)
        return original(filt, gyro, accel, step_dt)

    with patch.object(NavFilter, 'propagate', propagate):
        yield steps


def experiment(raw, seed, init_seed, out):
    truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
    cfg = replace(_filter_config(raw), init_seed=init_seed)
    if cfg.monitor != 'none' or cfg.filter != 'inekf':
        raise ValueError('requires a single InEKF without monitor')
    native = run(truth, imu, gnss, spec, gs, cfg)
    rows = []
    for mode in ('fixed12', 'oracle', 'prior_state'):
        with noise_schedule(spec, truth.dt, mode, truth.v_body[:, 0]) as speeds:
            log = run(truth, imu, gnss, spec, gs, cfg)
        if len(speeds) != len(truth.t) - 1:
            raise AssertionError('schedule did not cover exactly one trajectory')
        if mode == 'fixed12':
            for field in ('t', 'p_est', 'v_est', 'P_pos'):
                np.testing.assert_array_equal(getattr(native, field), getattr(log, field))
        error = log.p_est - log.p_true
        row = dict(seed=seed, init_seed=init_seed, mode=mode,
                   fixed12_exact_native_replay=(mode == 'fixed12'),
                   **position_consistency(log, gs.outages, raw['metrics']['t_start'],
                                              scenario_end=truth.t[-1]), bins=[])
        for start in range(30, 180, 10):
            mask = (log.t >= start) & (log.t < start + 10)
            row['bins'].append(dict(start=start, **budget(
                log.t[mask], error[mask], [], start)['overall']))
        np.savez_compressed(out / f'trace_{seed}_{init_seed}_{mode}.npz',
                            t=log.t, error=error, position_covariance=log.P_pos,
                            imu_t=truth.t[:-1], scheduled_speed=np.array(speeds),
                            truth_speed=truth.v_body[:-1, 0])
        rows.append(row)
        print(seed, init_seed, mode, row['segments']['overall']['rmse_h'], flush=True)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[900, 92001, 92002, 92003, 92004])
    parser.add_argument('--init-seeds', type=int, nargs='+', default=[0, 1701])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_noise_schedule_isolation', config=raw,
                  production_changed=False, oracle_is_offline=True,
                  scope='Descriptive paired diagnostic; prior-state speed uncertainty ignored; no tuning or promotion.',
                  rows=[])
    for seed in args.seeds:
        for init in args.init_seeds:
            result['rows'].extend(experiment(raw, seed, init, args.out))
            (args.out / 'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
