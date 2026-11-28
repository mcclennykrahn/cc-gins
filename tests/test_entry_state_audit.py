"""Boundary and intervention controls for privileged offline sensitivity runs."""
from types import SimpleNamespace

import numpy as np
import pytest

from ccgins import pipeline
from ccgins.cli import _build_inputs, _filter_config, load_config
from scripts.audit_entry_state import ARMS, replay, verify_prefix


@pytest.mark.parametrize('arm', ARMS)
def test_entry_intervention_is_once_before_correct_sample(monkeypatch, arm):
    t = np.arange(5, dtype=float)
    truth = SimpleNamespace(t=t, R=np.tile(3.*np.eye(3), (5, 1, 1)))
    imu = SimpleNamespace(gyro_bias=np.full((5, 3), 20.),
                          accel_bias=np.full((5, 3), 40.))
    imu.gyro_bias[2] = 22.
    imu.accel_bias[2] = 42.
    covariance = np.diag(np.arange(1, 16, dtype=float))
    filt = SimpleNamespace(R=np.eye(3), bg=np.zeros(3), ba=np.ones(3),
                           p=np.full(3, 8.), v=np.full(3, 9.), P=covariance.copy())
    records = []

    def propagate(gyro, accel, dt):
        records.append({name: getattr(filt, name).copy() for name in ('R', 'bg', 'ba', 'p', 'v', 'P')})
        filt.bg += 1.
        filt.ba += 1.

    filt.propagate = propagate
    native_factory = lambda *a, **kw: filt
    monkeypatch.setattr(pipeline, 'make_filter', native_factory)

    def run(*args):
        f = pipeline.make_filter()
        for k in range(4):
            f.propagate(None, None, 1.)
        return 'log'

    monkeypatch.setattr(pipeline, 'run', run)
    log, entry = replay(truth, imu, None, None, None, pipeline.Config(pseudo='nhc'), 2., 2., arm)
    assert log == 'log'
    assert pipeline.make_filter is native_factory  # patch restored
    assert entry['imu_index'] == 2
    assert entry['epoch'] == 2.
    assert entry['covariance_unchanged'] is True
    np.testing.assert_array_equal(records[1]['bg'], np.ones(3))
    targets = dict(R=truth.R[2], bg=imu.gyro_bias[2], ba=imu.accel_bias[2])
    unchanged = dict(R=np.eye(3), bg=np.full(3, 2.), ba=np.full(3, 3.))
    for name in targets:
        np.testing.assert_array_equal(records[2][name], targets[name] if name in ARMS[arm] else unchanged[name])
    np.testing.assert_array_equal(records[3]['bg'], records[2]['bg']+1.)
    np.testing.assert_array_equal(records[3]['ba'], records[2]['ba']+1.)
    for record in records:
        np.testing.assert_array_equal(record['P'], covariance)
        np.testing.assert_array_equal(record['p'], np.full(3, 8.))
        np.testing.assert_array_equal(record['v'], np.full(3, 9.))


def test_entry_replay_native_prefix_and_configuration_guards():
    raw = load_config('configs/example_sim.yaml')
    raw['scenario'].update(duration=2.)
    raw['gnss']['outages'] = [dict(start=1., duration=1.)]
    truth, imu, gnss, spec, gs = _build_inputs(raw, seed_override=900)
    cfg = _filter_config(raw)
    native = pipeline.run(truth, imu, gnss, spec, gs, cfg)
    altered, _ = replay(truth, imu, gnss, spec, gs, cfg, 1., 1., 'attitude_and_biases')
    assert verify_prefix(native, altered, 1.) == 10
    assert not np.array_equal(native.p_est[-1], altered.p_est[-1])
    with pytest.raises(ValueError, match='on-grid'):
        replay(truth, imu, gnss, spec, gs, cfg, 1.001, .5, 'attitude')
    with pytest.raises(ValueError, match='unknown'):
        replay(truth, imu, gnss, spec, gs, cfg, 1., 1., 'none')
    cfg.monitor = 'dual'
    with pytest.raises(ValueError, match='plain online NHC'):
        replay(truth, imu, gnss, spec, gs, cfg, 1., 1., 'attitude')
