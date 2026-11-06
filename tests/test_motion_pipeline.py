import numpy as np
import pytest

from ccgins.zupt import (AdaptiveZeroVelocity, OdometryData, mounted_odometry_model,
                         velocity_update)
from ccgins.filters import NoiseParams, make_filter
from ccgins.pipeline import Config, run, mount_matrix
from ccgins.sim import ImuSpec, GnssSpec, generate_trajectory, simulate_imu, simulate_gnss


def calibration(scale=1):
    rng = np.random.default_rng(8)
    g = rng.normal(0, .0003*scale, (2000, 3))
    a = rng.normal(0, .02*scale, (2000, 3)) + [0, 0, 9.81]
    return AdaptiveZeroVelocity.calibrate(g, a, ImuSpec())


def test_calibration_adapts_and_caps_vibration():
    low, high = calibration(), calibration(3)
    assert high.accel_var > low.accel_var
    assert high.gyro_var > low.gyro_var
    extreme = calibration(100)
    assert extreme.accel_var <= 3.**2
    assert extreme.gyro_var <= .05**2


def test_causal_dwell_and_motion_vetoes():
    d = calibration()
    assert not any(d.step([0, 0, 0], [0, 0, 9.81], 0) for _ in range(148))
    assert d.step([0, 0, 0], [0, 0, 9.81], 0)
    # Smooth translation is indistinguishable by IMU alone; speed veto is mandatory.
    assert not d.step([0, 0, 0], [0, 0, 9.81], 2)
    assert not any(d.step([0, 0, .1], [0, 0, 9.81], 0) for _ in range(200))
    assert not any(d.step([0, 0, 0], [0, 0, 10.5], 0) for _ in range(200))
    assert not d.step([np.nan, 0, 0], [0, 0, 9.81], 0)


@pytest.mark.parametrize('kind', ['eskf', 'inekf'])
def test_mounted_odometry_and_velocity_jacobians(kind):
    f = make_filter(kind, np.eye(3), [3, 1, .2], np.zeros(3), np.eye(15),
                    NoiseParams.from_spec(ImuSpec()),
                    earth={'w_ie': np.array([0, 6e-5, 4e-5]), 'g': np.array([0, 0, -9.81])})
    C, lever, gyro = mount_matrix(.2, -.3), np.array([1, .5, .2]), np.array([0, 0, .4])
    h, H = mounted_odometry_model(f, gyro, C, lever)
    for j in range(15):
        ff = f.copy(); dx = np.zeros(15); dx[j] = 1e-6; ff._inject(dx)
        hh, _ = mounted_odometry_model(ff, gyro, C, lever)
        assert np.allclose((hh-h)/1e-6, H[:, j], atol=3e-6)
    captured = {}
    def capture(r, H, Rm, **kw):
        captured['H'] = H
    f.update = capture
    velocity_update(f, np.zeros(3), .1)
    for j in range(6):
        ff = f.copy(); dx = np.zeros(15); dx[j] = 1e-6; ff._inject(dx)
        assert np.allclose((ff.v-f.v)/1e-6, captured['H'][:, j], atol=3e-6)


@pytest.mark.parametrize('kind', ['eskf', 'inekf'])
def test_main_pipeline_applies_motion_and_odometry(kind):
    tr = generate_trajectory(5, seed=0)
    # Controlled static IMU replay with no GNSS: verifies actual main-loop updates.
    tr.v[:] = 0; tr.p[:] = 0; tr.R[:] = np.eye(3)
    spec, gs = ImuSpec(), GnssSpec()
    imu = simulate_imu(tr, spec, seed=0)
    imu.gyro[:] = 0; imu.accel[:] = [0, 0, 9.81]
    gn = simulate_gnss(tr, gs, seed=0); gn.valid[:] = False
    odo = OdometryData([1., 2., 3.], [0, 0, 0], .1, np.eye(3), np.zeros(3), [True, False, True])
    log = run(tr, imu, gn, spec, gs, Config(filter=kind, zupt=True, zupt_require_gnss_stop=False), odometry=odo)
    m = log.extra['motion']
    assert m['zupt_accepted'] > 20
    assert m['odometry_attempted'] == m['odometry_accepted'] == 2
    assert np.linalg.norm(log.v_est[-1]) < .01
    assert np.linalg.eigvalsh(log.P_pos).min() > 0
    with pytest.raises(ValueError, match='aligned'):
        run(tr, imu, gn, spec, gs, Config(), odometry=OdometryData(
            [1.005], [0], .1, np.eye(3), np.zeros(3)))


def test_odometry_requires_valid_geometry():
    with pytest.raises(ValueError):
        OdometryData([1], [1], .1, np.zeros((3, 3)), np.zeros(3))
    with pytest.raises(ValueError):
        OdometryData([1, 1], [1, 1], .1, np.eye(3), np.zeros(3))


@pytest.mark.parametrize('kind', ['eskf', 'inekf'])
def test_odometry_turn_lever_correction_and_outlier_gate(kind):
    from ccgins.zupt import mounted_odometry_update
    f = make_filter(kind, np.eye(3), [2., 0., 0.], np.zeros(3), np.eye(15)*.01,
                    NoiseParams.from_spec(ImuSpec()),
                    earth={'w_ie': None, 'g': np.array([0, 0, -9.81])})
    C = mount_matrix(0., np.pi/2)
    lever, gyro = np.array([1., 0., 0.]), np.array([0., 0., 1.])
    # Wheel point velocity is [2,1,0] in body; vehicle forward is body +y.
    h, _ = mounted_odometry_model(f, gyro, C, lever)
    assert np.allclose(h, [1.])
    v = f.v.copy()
    assert mounted_odometry_update(f, 1., .1, gyro, C, lever).accepted
    assert np.allclose(f.v, v)
    assert not mounted_odometry_update(f, 100., .1, gyro, C, lever).accepted
    assert np.allclose(f.v, v)


def test_default_zupt_requires_independent_stop_confirmation():
    tr = generate_trajectory(5, seed=0)
    tr.v[:] = 0; tr.p[:] = 0; tr.R[:] = np.eye(3)
    spec, gs = ImuSpec(), GnssSpec()
    imu = simulate_imu(tr, spec, seed=0)
    imu.gyro[:] = 0; imu.accel[:] = [0, 0, 9.81]
    gn = simulate_gnss(tr, gs, seed=0); gn.valid[:] = False
    log = run(tr, imu, gn, spec, gs, Config(zupt=True))
    assert log.extra['motion']['zupt_accepted'] == 0
    assert log.extra['motion']['unconfirmed_stop_vetoes'] > 0
    # Three accurate stationary fixes confirm rest; a moving fix revokes it.
    gn.valid[:] = True; gn.pos[:] = 0; gn.sigma[:] = .01
    gs.lever_arm = np.zeros(3)
    log = run(tr, imu, gn, spec, gs, Config(zupt=True))
    assert log.extra['motion']['zupt_accepted'] > 0
    assert min(log.extra['motion']['zupt_times']) > 3
    gn.pos[gn.t >= 4] = [50, 0, 0]
    log = run(tr, imu, gn, spec, gs, Config(zupt=True))
    assert max(log.extra['motion']['zupt_times']) <= 4
