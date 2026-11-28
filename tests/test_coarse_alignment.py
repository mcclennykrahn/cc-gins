import numpy as np
import pytest
from ccgins.geometry import euler_to_rot
from scripts.audit_coarse_alignment import gravity_direction, angle_deg


def test_gravity_heading_gauge_and_bias_ambiguity():
    up = np.array([0., 0., 1.])
    R = euler_to_rot(.1, -.2, .7)
    yaw = euler_to_rot(0., 0., 1.3)
    np.testing.assert_allclose(R.T @ up, (yaw @ R).T @ up)
    a = 9.81 * (R.T @ up)
    np.testing.assert_allclose(gravity_direction(np.tile(a, (500, 1))), R.T @ up)
    # A constant transverse bias is indistinguishable from tilt in this mean.
    shifted = gravity_direction(np.tile(a + np.array([.04905, 0., 0.]), (500, 1)))
    assert angle_deg(shifted, R.T @ up) > .2


@pytest.mark.parametrize('a', [[], [[0., 0., 0.]], [[np.nan, 0., 1.]], [[1., 2.]]])
def test_invalid_gravity_prefix(a):
    with pytest.raises(ValueError):
        gravity_direction(a)
