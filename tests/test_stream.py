import numpy as np

from ccgins.pipeline import Config, run
from ccgins.sim import GnssSpec, ImuSpec, generate_trajectory, simulate_gnss, simulate_imu
from ccgins.stream import StreamingNavigator

DURATION = 40.0
SEED = 12


def _data():
    truth = generate_trajectory(DURATION, seed=SEED)
    imu_spec = ImuSpec()
    gnss_spec = GnssSpec(rate_hz=1.0)
    imu = simulate_imu(truth, imu_spec, seed=SEED)
    gnss = simulate_gnss(truth, gnss_spec, seed=SEED)
    cfg = Config(filter="inekf", pseudo="none", monitor="none", adaptive_r=False)
    return truth, imu_spec, gnss_spec, imu, gnss, cfg


def _feed(nav, truth, imu, captured=None):
    for k in range(len(truth.t)):
        nav.push_imu(truth.t[k], imu.accel[k], imu.gyro[k])
        if captured is not None and k % 10 == 0:
            captured[k] = nav.get_state()


def _gnss_args(gnss, m):
    return gnss.t[m], gnss.pos[m], np.diag(gnss.sigma ** 2)


def test_stream_matches_batch():
    truth, imu_spec, gnss_spec, imu, gnss, cfg = _data()
    log = run(truth, imu, gnss, imu_spec, gnss_spec, cfg)

    nav = StreamingNavigator.from_truth(truth, imu_spec, gnss_spec, cfg)
    for m in range(len(gnss.t)):
        nav.push_gnss(*_gnss_args(gnss, m))
    states = {}
    _feed(nav, truth, imu, states)

    kk = np.rint(log.t / truth.dt).astype(int)
    pos = np.array([states[k].position for k in kk])
    vel = np.array([states[k].velocity for k in kk])
    pos_cov = np.array([states[k].position_covariance for k in kk])

    np.testing.assert_allclose(pos, log.p_est, atol=1e-8, rtol=0)
    np.testing.assert_allclose(vel, log.v_est, atol=1e-8, rtol=0)
    np.testing.assert_allclose(pos_cov, log.P_pos, atol=1e-8, rtol=0)


def test_out_of_order_gnss_matches_in_order():
    truth, imu_spec, gnss_spec, imu, gnss, cfg = _data()

    ordered = StreamingNavigator.from_truth(truth, imu_spec, gnss_spec, cfg)
    shuffled = StreamingNavigator.from_truth(truth, imu_spec, gnss_spec, cfg)
    for m in range(len(gnss.t)):
        ordered.push_gnss(*_gnss_args(gnss, m))
    rng = np.random.default_rng(7)
    for m in rng.permutation(len(gnss.t)):
        shuffled.push_gnss(*_gnss_args(gnss, m))

    _feed(ordered, truth, imu)
    _feed(shuffled, truth, imu)

    a, b = ordered.get_state(), shuffled.get_state()
    assert a.t == b.t
    np.testing.assert_allclose(a.position, b.position, atol=1e-12, rtol=0)
    np.testing.assert_allclose(a.velocity, b.velocity, atol=1e-12, rtol=0)
    np.testing.assert_allclose(a.attitude, b.attitude, atol=1e-12, rtol=0)
    np.testing.assert_allclose(a.covariance, b.covariance, atol=1e-12, rtol=0)
    np.testing.assert_allclose(a.position_covariance, b.position_covariance, atol=1e-12, rtol=0)
