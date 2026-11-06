import numpy as np
import pytest
from ccgins.pipeline import Config, run
from ccgins.stream import StreamingNavigator
from ccgins.zupt import OdometryData, OdometryStopGuard
from test_stream import _data, _gnss_args


@pytest.mark.parametrize('kind', ['eskf', 'inekf'])
@pytest.mark.parametrize('wheel_guard', [False, True])
def test_motion_parity(kind, wheel_guard):
    truth, spec, gs, imu, gnss, _ = _data()
    cfg = Config(filter=kind, pseudo='none', monitor='none', zupt=True,
                 zupt_odometry_stop=wheel_guard,
                 earth_rate_enu=(0., 5e-5, 5e-5), local_gravity=9.8)
    ii = np.arange(10, len(truth.t), 10)
    angle = .12
    C = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    lever = np.array([.4, -.2, .1])
    vb = np.einsum('nji,nj->ni', truth.R, truth.v)
    speed = (vb[ii] @ C.T)[:, 0]
    odo = OdometryData(truth.t[ii], speed, .02, C, lever)
    batch = run(truth, imu, gnss, spec, gs, cfg, odometry=odo)
    nav = StreamingNavigator.from_truth(truth, spec, gs, cfg)
    for m in range(len(gnss.t)):
        nav.push_gnss(*_gnss_args(gnss, m))
    for j, t in enumerate(odo.t):
        nav.push_odometry(t, odo.speed[j], odo.sigma[j], C, lever)
    states = {}
    for k, t in enumerate(truth.t):
        nav.push_imu(t, imu.accel[k], imu.gyro[k])
        if k % 10 == 0:
            states[k] = nav.get_state()
    ix = np.rint(batch.t/truth.dt).astype(int)
    np.testing.assert_allclose([states[k].position for k in ix], batch.p_est, atol=1e-8, rtol=0)
    np.testing.assert_allclose([states[k].velocity for k in ix], batch.v_est, atol=1e-8, rtol=0)
    np.testing.assert_allclose([states[k].position_covariance for k in ix], batch.P_pos, atol=1e-8, rtol=0)
    assert nav.motion == batch.extra['motion']


def test_wheel_guard_expiry_and_veto():
    g = OdometryStopGuard()
    g.observe(1., 0, .01, True)
    assert not g.confirmed(1.)
    g.observe(1.1, 0, .01, True)
    assert g.confirmed(1.2)
    assert not g.confirmed(1.31)
    g.observe(1.2, .2, .01, True)
    assert not g.confirmed(1.2)
    g.observe(1.3, 0, .01, False)
    assert not g.confirmed(1.3)


def test_stream_wheel_validation():
    truth, spec, gs, _, _, cfg = _data()
    nav = StreamingNavigator.from_truth(truth, spec, gs, cfg)
    nav.push_odometry(.1, 0, .01, np.eye(3), np.zeros(3))
    with pytest.raises(ValueError):
        nav.push_odometry(.1, 0, .01, np.eye(3), np.zeros(3))
    with pytest.raises(ValueError):
        nav.push_odometry(.2, 0, 0, np.eye(3), np.zeros(3))


@pytest.mark.parametrize('kind', ['eskf', 'inekf'])
@pytest.mark.parametrize('wheel', [False, True])
def test_stationary_outage_confirmation(kind, wheel):
    from ccgins.sim import Truth, ImuData, ImuSpec, GnssSpec, GnssData
    t = np.arange(0, 8.01, .01)
    n = len(t)
    zero = np.zeros((n, 3))
    acc = np.tile([0., 0., 9.81], (n, 1))
    truth = Truth(t, np.tile(np.eye(3), (n, 1, 1)), zero.copy(), zero.copy(),
                  zero.copy(), zero.copy(), acc.copy(), .01)
    imu = ImuData(t, zero.copy(), acc, zero.copy(), zero.copy(), .01)
    spec = ImuSpec(); gs = GnssSpec(lever_arm=np.zeros(3))
    # No GNSS stop evidence at all: a new stop must be wheel-confirmed.
    gnss = GnssData(t=np.array([]), idx=np.array([], int), pos=np.empty((0, 3)),
                    valid=np.array([], bool), spoofed=np.array([], bool), sigma=np.full(3, .01))
    cfg = Config(filter=kind, pseudo='none', monitor='none', zupt=True,
                 zupt_odometry_stop=wheel)
    ii = np.arange(10, n, 10)
    odo = OdometryData(t[ii], np.zeros(len(ii)), .01, np.eye(3), np.zeros(3))
    log = run(truth, imu, gnss, spec, gs, cfg, odometry=odo, log_covariance=True)
    nav = StreamingNavigator.from_truth(truth, spec, gs, cfg)
    pp=[]; vv=[]; cov=[]
    for k, epoch in enumerate(t):
        if k in ii:
            nav.push_odometry(epoch, 0, .01, np.eye(3), np.zeros(3))
        nav.push_imu(epoch, acc[k], zero[k])
        if k > 0 and k % 10 == 0:
            state = nav.get_state()
            pp.append(state.position); vv.append(state.velocity); cov.append(state.covariance)
    np.testing.assert_allclose(pp, log.p_est, atol=1e-12, rtol=0)
    np.testing.assert_allclose(vv, log.v_est, atol=1e-12, rtol=0)
    np.testing.assert_allclose(cov, log.extra["covariance"], atol=1e-12, rtol=0)
    assert nav.motion == log.extra['motion']
    assert (nav.motion['zupt_accepted'] > 0) == wheel
    if not wheel:
        assert nav.motion['unconfirmed_stop_vetoes'] > 0


def test_misaligned_wheel_rejected_before_propagation():
    truth, spec, gs, imu, _, cfg = _data()
    nav = StreamingNavigator.from_truth(truth, spec, gs, cfg)
    nav.push_imu(0., imu.accel[0], imu.gyro[0])
    nav.push_odometry(.015, 0., .01, np.eye(3), np.zeros(3))
    nav.push_imu(.01, imu.accel[1], imu.gyro[1])
    state = nav.get_state()
    with pytest.raises(ValueError):
        nav.push_imu(.02, imu.accel[2], imu.gyro[2])
    assert nav.t == state.t
    np.testing.assert_array_equal(nav.get_state().position, state.position)


@pytest.mark.parametrize('kind', ['eskf', 'inekf'])
def test_gnss_stop_confirmation_and_rejected_motion_revocation(kind):
    from ccgins.filters import NoiseParams
    from ccgins.sim import ImuSpec
    cfg = Config(filter=kind, zupt=True)
    spec = ImuSpec()
    nav = StreamingNavigator(np.eye(3), np.zeros(3), np.zeros(3), np.eye(15)*.01,
                             NoiseParams.from_spec(spec), kind=kind,
                             motion_cfg=cfg, imu_spec=spec, dt=.01)
    for t in (.6, .7, .8):
        nav.push_gnss(t, np.zeros(3), 1e-6)
    nav.push_gnss(3., [10000, 0, 0], 1e-6)
    for t in np.arange(0, 4.01, .01):
        nav.push_imu(t, [0, 0, 9.81], [0, 0, 0])
    assert nav.motion['zupt_accepted'] > 0
    assert max(nav.motion['zupt_times']) <= 3.
    assert nav._rejects == 1  # moving fix vetoes rest even when filter rejects it
    assert not nav._stop
