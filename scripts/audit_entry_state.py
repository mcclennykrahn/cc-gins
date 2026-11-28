"""Offline entry-state interventions using truth; never an online accuracy result.

Each arm starts a fresh native run and changes one state group once, immediately
before the first IMU step in one outage. Covariance is deliberately held fixed
at intervention: these are conditional sensitivity experiments, not statistically
consistent oracle posteriors, additive error fractions or deployable algorithms.
"""
import argparse
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

from ccgins import pipeline
from ccgins.geometry import so3_exp, rot_to_yaw
from ccgins.cli import _build_inputs, _filter_config, load_config
from scripts.audit_scheme import budget, contained_steps


ARMS = {'attitude': ('R',), 'gyro_bias': ('bg',), 'accel_bias': ('ba',),
        'biases': ('bg', 'ba'),
        'attitude_and_biases': ('R', 'bg', 'ba'),
        'heading': ('R',), 'tilt': ('R',), 'velocity': ('v',)}


def partial_attitude(estimated, truth, arm):
    """ZYX split; replace yaw or yaw-free tilt, refusing vertical forward axes."""
    if arm not in ('heading', 'tilt'):
        raise ValueError('partial attitude requires heading or tilt')
    for R in (estimated, truth):
        if np.hypot(R[0, 0], R[1, 0]) < 1e-6:
            raise ValueError('heading undefined for vertical forward axis')
    delta = rot_to_yaw(truth) - rot_to_yaw(estimated)
    if arm == 'heading':
        return so3_exp(np.array([0., 0., delta])) @ estimated
    return so3_exp(np.array([0., 0., -delta])) @ truth


def replay(truth, imu, gnss, spec, gs, cfg, start, duration, arm):
    """Restrict to unaided-monitor NHC configuration; return log and entry evidence."""
    if arm not in ARMS:
        raise ValueError('unknown entry-state arm')
    if (cfg.monitor != 'none' or cfg.pseudo != 'nhc' or cfg.zupt
            or cfg.adaptive_r or cfg.aci or cfg.gnss_aci):
        raise ValueError('entry-state diagnostic requires plain online NHC, no monitor')
    first, _ = contained_steps(truth, start, duration)
    if not np.isclose(truth.t[first], start, rtol=0, atol=1e-10):
        raise ValueError('entry-state intervention requires an on-grid start')
    evidence = {}
    native_factory = pipeline.make_filter

    def factory(*args, **kwargs):
        filt = native_factory(*args, **kwargs)
        native_propagate = filt.propagate
        step = 0

        def propagate(gyro, accel, dt):
            nonlocal step
            if step == first:
                truth_state = dict(R=truth.R[first], bg=imu.gyro_bias[first],
                              ba=imu.accel_bias[first], v=truth.v[first])
                target = dict(truth_state)
                if arm in ('heading', 'tilt'):
                    target['R'] = partial_attitude(filt.R, truth.R[first], arm)
                covariance = filt.P.copy()
                evidence.update(epoch=float(truth.t[first]), imu_index=first,
                    fields=list(ARMS[arm]),
                    before={name: getattr(filt, name).tolist() for name in target},
                    truth={name: value.tolist() for name, value in truth_state.items()},
                    target={name: value.tolist() for name, value in target.items()})
                for name in ARMS[arm]:
                    setattr(filt, name, target[name].copy())
                np.testing.assert_array_equal(filt.P, covariance)
                evidence['after'] = {name: getattr(filt, name).tolist() for name in target}
                evidence['covariance_unchanged'] = True
            native_propagate(gyro, accel, dt)
            step += 1

        filt.propagate = propagate
        return filt

    # Patching is isolated to this synchronous diagnostic, never production code.
    with patch.object(pipeline, 'make_filter', factory):
        log = pipeline.run(truth, imu, gnss, spec, gs, cfg)
    if not evidence:
        raise RuntimeError('entry intervention was not reached')
    return log, evidence


def verify_prefix(native, altered, start):
    """The logged posterior AT start precedes the intervention on the next step."""
    np.testing.assert_array_equal(native.t, altered.t)
    prefix = native.t <= start
    for field in ('p_est', 'v_est', 'P_pos'):
        np.testing.assert_array_equal(getattr(native, field)[prefix],
                                      getattr(altered, field)[prefix])
    return int(prefix.sum())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--config', default='configs/example_sim.yaml')
    parser.add_argument('--seeds', type=int, nargs='+', default=[900, 110001, 110002])
    parser.add_argument('--arms', nargs='+', choices=list(ARMS), default=list(ARMS))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    raw = load_config(args.config)
    result = dict(candidate_id='scheme_entry_state_oracle', config=raw, rows=[],
        scope=__doc__, initialization='Native init_seed=0 shared across draws; not independent alignment validation.')
    for seed in args.seeds:
        truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=seed)
        cfg = _filter_config(raw)
        native = pipeline.run(truth, imu, gnss, spec, gs, cfg)
        scoring = dict(t_start=float(raw['metrics']['t_start']), scenario_end=truth.t[-1])
        online = budget(native.t, native.p_est-native.p_true, gs.outages, **scoring)
        np.savez_compressed(args.out / f'online_{seed}.npz', t=native.t,
                            error=native.p_est-native.p_true)
        for start, duration in gs.outages:
            for arm in args.arms:
                altered, entry = replay(truth, imu, gnss, spec, gs, cfg, start, duration, arm)
                n_prefix = verify_prefix(native, altered, start)
                result['rows'].append(dict(seed=seed, start=start, duration=duration,
                    arm=arm, entry=entry, identical_prefix_epochs=n_prefix,
                    online=online,
                    offline_intervention=budget(altered.t, altered.p_est-altered.p_true,
                                                gs.outages, **scoring)))
                np.savez_compressed(args.out / f'{seed}_{start:g}_{arm}.npz',
                                    t=altered.t, error=altered.p_est-altered.p_true)
                (args.out / 'interventions.json').write_text(
                    json.dumps(result, indent=2, allow_nan=False)+'\n')
                print(seed, start, arm, flush=True)


if __name__ == '__main__':
    main()
