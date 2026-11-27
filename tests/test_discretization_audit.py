"""Independent analytic checks for offline covariance diagnostic."""
import numpy as np
from scripts.audit_discretization import exact_step, accumulated_covariance


def test_integrated_white_acceleration_matches_closed_form():
    F = np.array([[0.,1.],[0.,0.]])
    W = np.diag([0., 4.])
    phi, q = exact_step(F, W, .1)
    np.testing.assert_allclose(phi, [[1., .1],[0.,1.]], atol=1e-14)
    expected = 4*np.array([[.1**3/3, .1**2/2],[.1**2/2,.1]])
    np.testing.assert_allclose(q, expected, atol=1e-14)
    for steps in (0, 1, 3, 600):
        T = steps*.1
        np.testing.assert_allclose(accumulated_covariance(phi,q,steps),
            4*np.array([[T**3/3,T**2/2],[T**2/2,T]]), rtol=1e-12, atol=1e-12)


def test_exact_covariance_composes_for_coupled_stable_model():
    F = np.array([[-.3, .7],[-.2, -.4]])
    W = np.array([[2., .4],[.4,1.]])
    phi, q = exact_step(F,W,.025)
    _, expected = exact_step(F,W,2.)
    actual = accumulated_covariance(phi,q,80)
    np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
    assert np.linalg.eigvalsh(actual).min() > 0
