import numpy as np
import pytest

from ccgins.filters import NoiseParams, make_filter
from ccgins.sim import DEG, G_NAV, ImuSpec
from ccgins.zupt import (detect_zero_velocity, odometry_update, sliding_variance,
                         velocity_update, zupt_update)

P0 = np.diag(np.r_[np.full(3, (1 * DEG) ** 2), np.full(3, 0.1**2), np.full(3, 0.5**2),
                   np.full(3, (10 * DEG / 3600) ** 2), np.full(3, (2e-3 * 9.81) ** 2)])


def _noise():
    d = ImuSpec().densities()
    return NoiseParams(d["gyro"], d["accel"], d["gyro_bias_rw"], d["accel_bias_rw"])


def _filter(kind, noise):
    return make_filter(kind, np.eye(3), np.zeros(3), np.zeros(3), P0, noise)


def test_sliding_variance_matches_numpy():
    rng = np.random.default_rng(0)
    x = rng.standard_normal((200, 3))
    var = sliding_variance(x, 21)
    assert var.shape == x.shape
    ref = x[90:111].var(axis=0)
    assert np.allclose(var[100], ref)


def test_detect_static_and_moving_segments():
    dt, duration = 0.01, 30.0
    n = int(duration / dt)
    t = np.arange(n) * dt
    rng = np.random.default_rng(1)
    d = ImuSpec().densities()

    # Stationary: gravity (a DC term, removed by the variance) plus MEMS white noise.
    gyro = rng.standard_normal((n, 3)) * d["gyro"] / np.sqrt(dt)
    accel = np.tile(-G_NAV, (n, 1)) + rng.standard_normal((n, 3)) * d["accel"] / np.sqrt(dt)
    moving = (t >= 10.0) & (t < 20.0)
    gyro[moving] += 0.2 * np.stack([np.sin(2 * np.pi * 1.5 * t[moving]),
                                    np.cos(2 * np.pi * 2.0 * t[moving]),
                                    np.sin(2 * np.pi * 1.0 * t[moving])], axis=1)
    accel[moving] += 0.5 * np.stack([np.sin(2 * np.pi * 0.7 * t[moving]),
                                     np.cos(2 * np.pi * 1.1 * t[moving]),
                                     np.sin(2 * np.pi * 1.7 * t[moving])], axis=1)

    mask, va, vg = detect_zero_velocity(gyro, accel, window=50, min_duration=50,
                                        return_variance=True)
    assert mask.shape == (n,)
    assert va.shape == vg.shape == (n,)
    # Ignore the window-width transition at the edges of each segment.
    assert mask[:900].all()                      # stationary, far from the transition
    assert not mask[1050:1950].any()             # moving
    assert mask[2050:].all()                     # stationary again
    assert mask[~moving].sum() > 0.95 * (~moving).sum()


@pytest.mark.parametrize("kind", ["eskf", "inekf"])
def test_zupt_reduces_velocity_covariance(kind):
    rng = np.random.default_rng(2)
    f = _filter(kind, _noise())
    f.v = np.array([0.5, -0.2, 0.1])
    f.P[3:6, 3:6] = np.diag([0.5, 0.5, 0.2])
    before = np.trace(f.P[3:6, 3:6])
    info = zupt_update(f, sigma=0.01)
    after = np.trace(f.P[3:6, 3:6])
    assert info.accepted and info.dof == 3
    assert after < before
    # Position/attitude covariance should not grow from a velocity-only update.
    assert np.trace(f.P[6:9, 6:9]) <= P0[6:9, 6:9].sum() + 1e-9
    assert np.linalg.norm(f.v) < 0.5


@pytest.mark.parametrize("kind", ["eskf", "inekf"])
def test_odometry_update_reduces_forward_velocity(kind):
    f = _filter(kind, _noise())
    f.v = np.array([3.0, 0.0, 0.0])
    info = odometry_update(f, speed=1.0, sigma=0.1)
    assert info.accepted and info.dof == 1
    assert f.v[0] < 3.0
    assert abs((f.R.T @ f.v)[0]) < 3.0


def _stationary_run(kind, use_zupt):
    dt, duration = 0.01, 60.0
    n = int(duration / dt)
    rng = np.random.default_rng(3)
    d = ImuSpec().densities()
    bg = np.array([0.5, -0.3, 0.4]) * DEG / 3600
    ba = np.array([5e-3, -4e-3, 3e-3])
    gyro = np.tile(bg, (n, 1)) + rng.standard_normal((n, 3)) * d["gyro"] / np.sqrt(dt)
    accel = np.tile(-G_NAV + ba, (n, 1)) + rng.standard_normal((n, 3)) * d["accel"] / np.sqrt(dt)
    mask = detect_zero_velocity(gyro, accel)
    assert mask.all()

    f = _filter(kind, _noise())
    for k in range(n):
        f.propagate(gyro[k], accel[k], dt)
        if use_zupt and mask[k]:
            zupt_update(f, sigma=0.01)
    return f


@pytest.mark.parametrize("kind", ["eskf", "inekf"])
def test_zupt_reduces_static_position_drift(kind):
    drift_zupt = np.linalg.norm(_stationary_run(kind, True).p)
    drift_free = np.linalg.norm(_stationary_run(kind, False).p)
    assert drift_free > 1.0
    assert drift_zupt < 0.1 * drift_free


def test_velocity_update_accepts_vector_sigma():
    f = _filter("eskf", _noise())
    info = velocity_update(f, np.array([0.1, 0.0, 0.0]), np.array([0.1, 0.1, 0.2]))
    assert info.accepted and info.dof == 3
