import numpy as np

from ccgins.geometry import skew, so3_exp, so3_left_jacobian, so3_log


def test_exp_log_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(100):
        phi = rng.standard_normal(3) * rng.uniform(0, 3.0)
        R = so3_exp(phi)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert np.allclose(so3_exp(so3_log(R)), R, atol=1e-9)


def test_left_jacobian_numeric():
    rng = np.random.default_rng(1)
    phi = rng.standard_normal(3)
    J = so3_left_jacobian(phi)
    eps = 1e-6
    Jn = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3)
        d[i] = eps
        # Exp(phi + d) ≈ Exp(J d) Exp(phi)
        Jn[:, i] = so3_log(so3_exp(phi + d) @ so3_exp(phi).T) / eps
    assert np.allclose(J, Jn, atol=1e-5)


def test_skew():
    a, b = np.array([1.0, 2, 3]), np.array([-1.0, 0.5, 2])
    assert np.allclose(skew(a) @ b, np.cross(a, b))
