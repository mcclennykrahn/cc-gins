import numpy as np
import pytest
from ccgins.filters import ESKF, InEKF, NoiseParams

@pytest.mark.parametrize('cls', [ESKF, InEKF])
@pytest.mark.parametrize('size', [0.5, 30.0])
def test_robust_fix(cls, size):
    f = cls(np.eye(3), np.zeros(3), np.zeros(3), np.eye(15)*0.01,
            NoiseParams(0, 0, 0, 0))
    plain = f.copy()
    z = np.array([size*np.sqrt(1.01), 0, 0])
    plain.update_gnss(z, np.zeros(3), np.eye(3))
    info = f.update_gnss(z, np.zeros(3), np.eye(3), robust_delta=1.5)
    assert info.nis == pytest.approx(size**2)
    if size == 30:
        assert np.linalg.norm(f.p) < 0.01*np.linalg.norm(plain.p)
        assert f.P[6,6] > plain.P[6,6]
    else:
        assert np.allclose(f.p, plain.p)
    assert np.linalg.eigvalsh(f.P).min() > 0

@pytest.mark.parametrize('cls', [ESKF, InEKF])
def test_robust_gate_and_force(cls):
    f = cls(np.eye(3), np.zeros(3), np.zeros(3), np.eye(15), NoiseParams(0,0,0,0))
    assert not f.update_gnss(np.ones(3)*30, np.zeros(3), np.eye(3), gate=10, robust_delta=1.5).accepted
    assert np.allclose(f.p, 0)
    assert f.update_gnss(np.zeros(3), np.zeros(3), np.eye(3), robust_delta=1.5).accepted

@pytest.mark.parametrize('delta', [0, -1, float('nan')])
def test_robust_invalid(delta):
    f = ESKF(np.eye(3), np.zeros(3), np.zeros(3), np.eye(15), NoiseParams(0,0,0,0))
    with pytest.raises(ValueError):
        f.update_gnss(np.zeros(3), np.zeros(3), np.eye(3), robust_delta=delta)
