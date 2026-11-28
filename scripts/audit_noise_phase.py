"""Offline 2x2 process-noise timing diagnostic; no production change.

Known scenario windows select phases, so this is not an outage detector or a
deployable schedule. Speed itself uses only the pre-propagation filter state.
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
from ccgins.timing import in_window
from scripts.audit_calibration import position_consistency
from scripts.audit_scheme import budget


@contextmanager
def phase_schedule(spec, times, dt, outages, available, outage):
    """Select Q by propagation interval's *start*, including pre-scoring time."""
    original = NavFilter.propagate
    trace = []

    def propagate(filt, gyro, accel, step_dt):
        if step_dt != dt:
            raise ValueError('schedule requires a fixed IMU dt')
        t = float(times[len(trace)])
        in_gap = any(in_window(t, start, start + duration) for start, duration in outages)
        enabled = outage if in_gap else available
        speed = float(abs((filt.R.T @ filt.v)[0])) if enabled else 12.
        filt.noise = NoiseParams.from_spec(spec, vmax_eff=speed, dt=dt)
        trace.append((t, in_gap, enabled, speed))
        return original(filt, gyro, accel, step_dt)

    with patch.object(NavFilter, 'propagate', propagate):
        yield trace


def factorial_effects(rows):
    """Signed effects on horizontal MSE, not additive RMSE decompositions."""
    by_cell = {(r['available_schedule'], r['outage_schedule']): r for r in rows}
    if len(rows) != 4 or len(by_cell) != 4:
        raise ValueError('exactly four distinct factorial cells required')
    result = {}
    for segment in by_cell[False, False]['segments']:
        y = {cell: row['segments'][segment]['rmse_h']**2 for cell, row in by_cell.items()}
        result[segment] = dict(
            baseline_h_mse=y[False, False],
            available_only_delta_h_mse=y[True, False] - y[False, False],
            outage_only_delta_h_mse=y[False, True] - y[False, False],
            joint_delta_h_mse=y[True, True] - y[False, False],
            interaction_h_mse=y[True, True] - y[True, False] - y[False, True] + y[False, False])
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seeds', type=int, nargs='+', default=[900, 100001, 100002])
    args = ap.parse_args()
    if len(set(args.seeds)) != len(args.seeds):
        ap.error('duplicate trajectory/sensor seeds')
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config('configs/example_sim.yaml')
    result = dict(candidate_id='scheme_noise_phase_factorial', config=raw,
                  production_changed=False, scope='Offline known-window factorial. Paired cells are dependent. No tuning, independent-tail bound or promotion.',
                  rows=[], effects=[])
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        # Preserve the required seed900 baseline; vary alignment on held-outs.
        cfg = replace(_filter_config(raw), init_seed=0 if seed == 900 else seed + 400000)
        if cfg.monitor != 'none' or cfg.filter != 'inekf':
            raise ValueError('single unmonitored InEKF required')
        native = run(truth, imu, gnss, spec, gs, cfg)
        cells = []
        for available, outage in ((False, False), (True, False), (False, True), (True, True)):
            with phase_schedule(spec, truth.t[:-1], truth.dt, gs.outages, available, outage) as trace:
                log = run(truth, imu, gnss, spec, gs, cfg)
            if len(trace) != len(truth.t) - 1:
                raise AssertionError('instrumentation must cover exactly one trajectory')
            if not available and not outage:
                for field in ('t', 'p_est', 'v_est', 'P_pos'):
                    np.testing.assert_array_equal(getattr(native, field), getattr(log, field))
            error = log.p_est - log.p_true
            row = dict(seed=seed, init_seed=cfg.init_seed, available_schedule=available,
                       outage_schedule=outage, native_replay_exact=not available and not outage,
                       **position_consistency(log, gs.outages, raw['metrics']['t_start'],
                                              scenario_end=truth.t[-1]), bins=[])
            for start in np.arange(raw['metrics']['t_start'], log.t[-1], 10.):
                mask = (log.t >= start) & (log.t < start + 10.)
                if mask.any():
                    row['bins'].append(dict(start=float(start), **budget(log.t[mask], error[mask], [], start)['overall']))
            np.savez_compressed(args.out / f'trace_{seed}_{int(available)}{int(outage)}.npz',
                                t=log.t, error=error, position_covariance=log.P_pos,
                                schedule=np.asarray(trace))
            cells.append(row)
            result['rows'].append(row)
            (args.out / 'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            print(seed, available, outage, row['segments']['overall']['rmse_h'], flush=True)
        result['effects'].append(dict(seed=seed, init_seed=cfg.init_seed, segments=factorial_effects(cells)))
        (args.out / 'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
