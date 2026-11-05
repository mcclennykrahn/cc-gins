import numpy as np

from ccgins.adaptive import AdaptiveR, gnss_update
from ccgins.filters import NoiseParams, make_filter
from ccgins.pipeline import Config
from ccgins.sim import DEG, ImuSpec

P0 = np.diag(np.r_[np.full(3, (1 * DEG) ** 2), np.full(3, 0.1**2), np.full(3, 0.5**2),
                   np.full(3, (10 * DEG / 3600) ** 2), np.full(3, (2e-3 * 9.81) ** 2)])


def _noise():
    d = ImuSpec().densities()
    return NoiseParams(d["gyro"], d["accel"], d["gyro_bias_rw"], d["accel_bias_rw"])


def _gnss_model(P_pos):
    H = np.zeros((3, 15))
    H[:, 6:9] = np.eye(3)
    P = np.zeros((15, 15))
    P[6:9, 6:9] = np.diag(P_pos)
    return H, P


def test_adaptive_r_converges_to_4x_true_noise():
    # Nominal sigma = [0.2, 0.3, 0.4] m; the simulated measurement noise is 2x sigma,
    # i.e. R_true = 4 * R_nominal.
    R_nominal = np.diag(np.array([0.2, 0.3, 0.4]) ** 2)
    R_true = 4.0 * R_nominal
    P_pos = np.array([0.3, 0.4, 0.5]) ** 2
    H, P = _gnss_model(P_pos)

    rng = np.random.default_rng(7)
    S = np.diag(P_pos) + R_true
    innovations = rng.multivariate_normal(np.zeros(3), S, size=5000)

    est = AdaptiveR(R_nominal, window=300, forgetting=0.995,
                    min_scale=0.1, max_scale=30.0)
    for r in innovations:
        R_hat = est.update(r, H, P)

    assert np.allclose(R_hat, R_hat.T)
    assert np.all(np.diag(R_hat) > 0.0)
    ratio = np.diag(R_hat) / np.diag(R_true)
    assert np.all(ratio > 0.75) and np.all(ratio < 1.25), ratio
    # Sanity check relative to the nominal (unscaled) R.
    assert np.allclose(np.diag(R_hat), 4.0 * np.diag(R_nominal), rtol=0.25)


def test_adaptive_r_clamps_to_upper_and_lower_bounds():
    R_nominal = np.diag(np.array([0.2, 0.3, 0.4]) ** 2)
    H, P = _gnss_model(np.zeros(3))       # no predicted contribution: raw R = r r^T

    est_hi = AdaptiveR(R_nominal, window=20, forgetting=0.9,
                       min_scale=0.5, max_scale=4.0)
    for _ in range(20):
        est_hi.update(np.array([10.0, 10.0, 10.0]), H, P)
    assert np.allclose(np.diag(est_hi.R), 4.0 * np.diag(R_nominal))

    est_lo = AdaptiveR(R_nominal, window=20, forgetting=0.9,
                       min_scale=0.5, max_scale=4.0)
    for _ in range(20):
        est_lo.update(np.zeros(3), H, P)
    assert np.allclose(np.diag(est_lo.R), 0.5 * np.diag(R_nominal))


def test_gnss_update_without_estimator_matches_nominal_update():
    nav_a = make_filter("eskf", np.eye(3), np.zeros(3), np.zeros(3), P0, _noise())
    nav_b = nav_a.copy()
    R = np.diag([0.01, 0.04, 0.09])
    z = nav_a.p + np.array([0.05, -0.02, 0.01])

    info = gnss_update(nav_a, z, np.zeros(3), R)
    h, H = nav_b.gnss_model(np.zeros(3))
    ref = nav_b.update(z - h, H, R)

    assert info.accepted and ref.accepted
    assert np.allclose(nav_a.p, nav_b.p)
    assert np.allclose(nav_a.P, nav_b.P)


def test_adaptive_r_is_off_by_default():
    assert Config().adaptive_r is False
