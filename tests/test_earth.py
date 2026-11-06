import numpy as np
import pytest
from ccgins.filters import ESKF, InEKF, NoiseParams, set_earth
from ccgins.data.i2nav import earth_rate_enu, local_gravity
from ccgins.geometry import so3_log

@pytest.mark.parametrize('cls', [ESKF, InEKF])
def test_earth_covariance_linearization(cls):
    earth = dict(w_ie=earth_rate_enu(30), g=np.array([0,0,-local_gravity(30)]))
    a = cls(np.eye(3), np.array([10.,2,0]), np.array([20.,50,0]),
            np.eye(15), NoiseParams(0,0,0,0), earth=earth)
    dx = np.arange(15)*1e-7
    b = a.copy()
    b._inject(dx)
    w, f, dt = np.array([0.01,0.02,0.03]), np.array([0.1,0.2,9.8]), 1e-4
    F,_ = a._jacobians(w,f)
    a.propagate(w,f,dt); b.propagate(w,f,dt)
    if cls is ESKF:
        err = np.r_[so3_log(a.R.T@b.R), b.v-a.v,b.p-a.p,b.bg-a.bg,b.ba-a.ba]
    else:
        dR = b.R@a.R.T
        err = np.r_[so3_log(dR), b.v-dR@a.v,b.p-dR@a.p,b.bg-a.bg,b.ba-a.ba]
    assert np.linalg.norm(err-(np.eye(15)+F*dt)@dx) < 1e-10

@pytest.mark.parametrize('cls', [ESKF, InEKF])
def test_stationary_instance_earth_and_isolation(cls):
    w,g = earth_rate_enu(30),local_gravity(30)
    a = cls(np.eye(3), np.zeros(3),np.zeros(3), np.eye(15),NoiseParams(0,0,0,0),
            earth=dict(w_ie=w,g=np.array([0,0,-g])))
    set_earth()  # changing the legacy global must not change an existing filter
    for _ in range(1000):
        a.propagate(w,np.array([0,0,g]),0.1)
    assert np.linalg.norm(a.p) < 1e-7
    assert np.linalg.norm(so3_log(a.R)) < 1e-10
    assert np.isfinite(a.P).all()
