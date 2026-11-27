"""Boundary timing, causal speed selection and factorial interaction checks."""
from types import SimpleNamespace

import numpy as np
import pytest

from ccgins.filters import NavFilter, NoiseParams
from ccgins.sim import ImuSpec
from scripts.audit_noise_phase import factorial_effects, phase_schedule


@pytest.mark.parametrize('available,outage', [(False, False), (True, False), (False, True), (True, True)])
def test_phase_boundaries_and_prior_speed(monkeypatch, available, outage):
    seen = []
    def propagation(filt, gyro, accel, dt):
        seen.append(filt.noise)
        filt.v *= 2.
    monkeypatch.setattr(NavFilter, 'propagate', propagation)
    spec = ImuSpec()
    filt = SimpleNamespace(R=np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]),
                           v=np.array([40., -3., 50.]))
    times = [59.99, 60., 89.99, 90., 119.99, 120., 179.99, 180.]
    gaps = [False, True, True, False, False, True, True, False]
    with phase_schedule(spec, times, .01, [(60., 30.), (120., 60.)], available, outage) as trace:
        for _ in times:
            NavFilter.propagate(filt, None, None, .01)
    assert NavFilter.propagate is propagation
    for k, (_, gap, enabled, speed) in enumerate(trace):
        assert gap == gaps[k]
        assert enabled == (outage if gap else available)
        assert speed == (3. * 2**k if enabled else 12.)
        assert seen[k] == NoiseParams.from_spec(spec, vmax_eff=speed, dt=.01)


def test_phase_exception_restores_patch(monkeypatch):
    original = lambda *args: None
    monkeypatch.setattr(NavFilter, 'propagate', original)
    with pytest.raises(ValueError, match='fixed IMU dt'):
        with phase_schedule(ImuSpec(), [0.], .01, [], False, False):
            NavFilter.propagate(None, None, None, .02)
    assert NavFilter.propagate is original


def test_factorial_effects_use_mse_and_signed_interaction():
    rows = [dict(available_schedule=a, outage_schedule=b,
                 segments={'gap': {'rmse_h': r}})
            for a, b, r in [(False, False, 2.), (True, False, 3.), (False, True, 1.), (True, True, 4.)]]
    assert factorial_effects(rows)['gap'] == dict(baseline_h_mse=4., available_only_delta_h_mse=5.,
        outage_only_delta_h_mse=-3., joint_delta_h_mse=12., interaction_h_mse=10.)
    with pytest.raises(ValueError, match='four distinct'):
        factorial_effects(rows[:3] + rows[:1])
