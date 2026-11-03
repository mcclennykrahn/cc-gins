import numpy as np
import pytest

from ccgins.filters import ESKF, InEKF, NoiseParams, mechanize
from ccgins.geometry import so3_exp, so3_log
from ccgins.sim import GnssSpec, ImuSpec, generate_trajectory, simulate_gnss, simulate_imu
from ccgins.pipeline import Config, run

NOISE = NoiseParams(1e-3, 1e-2, 1e-5, 1e-4)


def _rand_filter(cls, seed=0):
    rng = np.random.default_rng(seed)
    R = so3_exp(rng.standard_normal(3))
    v = rng.standard_normal(3) * 5
    p = rng.standard_normal(3) * 100
    f = cls(R, v, p, np.eye(15), NOISE, bg=rng.standard_normal(3) * 1e-3, ba=rng.standard_normal(3) * 1e-2)
    return f


def test_mechanization_reproduces_truth():
    tr = generate_trajectory(60.0, seed=3)
    R, v, p = tr.R[0], tr.v[0], tr.p[0]
    for k in range(len(tr.t) - 1):
        R, v, p = mechanize(R, v, p, tr.gyro[k], tr.accel[k], tr.dt)
    assert np.linalg.norm(p - tr.p[-1]) < 1e-6
    assert np.linalg.norm(so3_log(R.T @ tr.R[-1])) < 1e-9


@pytest.mark.parametrize("cls", [ESKF, InEKF])
@pytest.mark.parametrize("model", ["gnss", "bodyvel"])
def test_measurement_jacobians(cls, model):
    lever = np.array([0.5, -0.2, 1.2])
    f0 = _rand_filter(cls)
    h0, H = f0.gnss_model(lever) if model == "gnss" else f0.body_vel_model()
    eps = 1e-6
    Hn = np.zeros_like(H)
    for i in range(15):
        f = f0.copy()
        dx = np.zeros(15)
        dx[i] = eps
        f._inject(dx)
        h, _ = f.gnss_model(lever) if model == "gnss" else f.body_vel_model()
        Hn[:, i] = (h - h0) / eps
    assert np.allclose(H, Hn, atol=1e-4 * max(1, np.abs(H).max()))


def _error(cls, fa, fb):
    """error of fb relative to fa in cls's error convention (first order)."""
    if cls is ESKF:
        dth = so3_log(fa.R.T @ fb.R)
        return np.r_[dth, fb.v - fa.v, fb.p - fa.p, fb.bg - fa.bg, fb.ba - fa.ba]
    phi = so3_log(fb.R @ fa.R.T)
    dR = so3_exp(phi)
    return np.r_[phi, fb.v - dR @ fa.v, fb.p - dR @ fa.p, fb.bg - fa.bg, fb.ba - fa.ba]


@pytest.mark.parametrize("cls", [ESKF, InEKF])
def test_propagation_jacobian(cls):
    dt = 0.01
    rng = np.random.default_rng(5)
    gyro, accel = np.array([0.1, -0.05, 0.3]), np.array([0.5, 0.2, 9.9])
    f0 = _rand_filter(cls, 2)
    dx = rng.standard_normal(15) * 1e-5
    f1 = f0.copy()
    f1._inject(dx)
    F, _ = f0._jacobians(gyro - f0.bg, accel - f0.ba)
    Phi = np.eye(15) + F * dt
    a, b = f0.copy(), f1.copy()
    a.propagate(gyro, accel, dt)
    b.propagate(gyro, accel, dt)
    err = _error(cls, a, b)
    pred = Phi @ dx
    assert np.linalg.norm(err - pred) < 2e-3 * np.linalg.norm(dx) + 1e-9


@pytest.mark.parametrize("kind", ["eskf", "inekf"])
def test_noise_free_imu_converges(kind):
    tr = generate_trajectory(120.0, seed=4)
    isp = ImuSpec()
    imu = simulate_imu(tr, isp, noise=False)
    gs = GnssSpec(sigma_h=0.3, sigma_v=0.5, mp_sigma=0.0)
    gn = simulate_gnss(tr, gs, seed=4)
    log = run(tr, imu, gn, isp, gs, Config(filter=kind))
    e = np.linalg.norm(log.p_est - log.p_true, axis=1)
    assert e[log.t > 60].max() < 1.0
