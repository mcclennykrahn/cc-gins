"""Causality and instrumentation restoration for offline noise isolation."""
from types import SimpleNamespace

import numpy as np
import pytest

from ccgins.filters import NavFilter, NoiseParams
from ccgins.sim import ImuSpec
from scripts.audit_noise_schedule import noise_schedule


def test_prior_state_uses_prepropagation_body_forward_speed(monkeypatch):
    seen = []

    def propagation(filt, gyro, accel, dt):
        seen.append(filt.noise)
        filt.v = np.array([100., 200., 300.])

    monkeypatch.setattr(NavFilter, 'propagate', propagation)
    # Body forward points along nav +y; transverse and vertical speed excluded.
    filt = SimpleNamespace(R=np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]),
                           v=np.array([40., -3., 50.]))
    spec = ImuSpec()
    with noise_schedule(spec, .01, 'prior_state', None) as speeds:
        NavFilter.propagate(filt, None, None, .01)
    assert speeds == [3.]
    assert seen == [NoiseParams.from_spec(spec, vmax_eff=3., dt=.01)]
    assert NavFilter.propagate is propagation


def test_oracle_index_and_exception_restore(monkeypatch):
    original = lambda *args: None
    monkeypatch.setattr(NavFilter, 'propagate', original)
    filt = SimpleNamespace()
    with pytest.raises(ValueError, match='fixed IMU dt'):
        with noise_schedule(ImuSpec(), .01, 'oracle', np.array([-2., 7.])) as speeds:
            NavFilter.propagate(filt, None, None, .01)
            NavFilter.propagate(filt, None, None, .01)
            assert speeds == [2., 7.]
            NavFilter.propagate(filt, None, None, .02)
    assert NavFilter.propagate is original


def test_invalid_schedule_rejected():
    with pytest.raises(ValueError, match='unknown schedule'):
        with noise_schedule(ImuSpec(), .01, 'future_speed', None):
            pass
