"""Check posterior covariance in the tangent coordinates used after injection."""

import numpy as np
import pytest

from ccgins.filters import ESKF, InEKF, NoiseParams
from ccgins.geometry import euler_to_rot, so3_left_jacobian, so3_log


def state_error(truth, estimate):
    if isinstance(estimate, ESKF):
        attitude = so3_log(estimate.R.T @ truth.R)
        velocity = truth.v - estimate.v
        position = truth.p - estimate.p
    else:
        relative = truth.R @ estimate.R.T
        attitude = so3_log(relative)
        jacobian = so3_left_jacobian(attitude)
        velocity = np.linalg.solve(jacobian, truth.v - relative @ estimate.v)
        position = np.linalg.solve(jacobian, truth.p - relative @ estimate.p)
    return np.r_[attitude, velocity, position, truth.bg - estimate.bg, truth.ba - estimate.ba]


@pytest.mark.parametrize("filter_type", [ESKF, InEKF])
def test_update_transports_joseph_covariance_to_injected_state(filter_type):
    orientation = euler_to_rot(0.2, -0.3, 0.4)
    prior = filter_type(orientation, [17, -4, 1], [2000, 1000, 20],
                        np.eye(15), NoiseParams(0.01, 0.01, 0.01, 0.01))
    correction = np.r_[0.2, -0.17, 0.13, 2, -3, 0.5, 8, -5, 1, np.full(6, 0.01)]
    posterior = prior.copy()
    # With P=H=Rm=I, K=I/2 and the pre-reset Joseph covariance is I/2.
    posterior.update(2 * correction, np.eye(15), np.eye(15))

    step = 1e-5
    numerical = np.empty((15, 15))
    for column, offset in enumerate(step * np.eye(15)):
        plus, minus = prior.copy(), prior.copy()
        plus._inject(correction + offset)
        minus._inject(correction - offset)
        numerical[:, column] = (state_error(plus, posterior) - state_error(minus, posterior)) / (2 * step)

    np.testing.assert_allclose(state_error(posterior, posterior), 0, atol=1e-10)
    np.testing.assert_allclose(posterior.P, numerical @ numerical.T / 2, rtol=1e-6, atol=1e-7)
    assert np.linalg.norm(posterior.P - np.eye(15) / 2) > 1e-3
