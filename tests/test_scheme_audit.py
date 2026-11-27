"""Audit regressions: sampled noise and scoring boundaries."""
from types import SimpleNamespace
import numpy as np
import pytest
from ccgins.filters import NoiseParams
from ccgins.metrics import summarize
from ccgins.sim import ImuSpec, simulate_imu


@pytest.mark.parametrize('dt', [.005, .02])
def test_noise_density_matches_simulated_samples(dt):
    # Many independent white samples, fixed speed, biases disabled.
    n = 80000
    spec = ImuSpec(gyro_bias_sigma=0, accel_bias_sigma=0,
                   gyro_bias_rw=0, accel_bias_rw=0)
    truth = SimpleNamespace(t=np.arange(n)*dt, dt=dt,
        gyro=np.zeros((n,3)), accel=np.zeros((n,3)),
        v_body=np.tile([12.,0.,0.], (n,1)))
    imu = simulate_imu(truth, spec, seed=71)
    noise = NoiseParams.from_spec(spec, dt=dt)
    assert np.std(imu.gyro) == pytest.approx(noise.gyro/np.sqrt(dt), rel=.01)
    assert np.std(imu.accel) == pytest.approx(noise.accel/np.sqrt(dt), rel=.01)


def test_outage_metrics_respect_scoring_start_and_repeated_durations():
    t = np.arange(1., 11.)
    error = np.zeros((10,3))
    error[:,0] = [100,100,1,2,3,4,5,6,7,8]
    log = SimpleNamespace(t=t, p_est=error, p_true=np.zeros_like(error),
                          P_pos=np.tile(np.eye(3),(10,1,1)), runtime_per_step_us=0.)
    m = summarize(log, [(1.,3.),(6.,3.),(20.,2.)], t_start=3.)
    assert m['max_err_outage_3s_start_1s'] == 1.
    assert m['end_err_outage_3s_start_6s'] == 6.
    assert m['max_err_outage_2s'] is None
    assert m['anees_pos_outage'] == pytest.approx((1+16+25+36)/4/3)


def test_multipath_uses_actual_rounded_epoch_spacing():
    from ccgins.sim import GnssSpec, simulate_gnss
    truth = SimpleNamespace(dt=.1, t=np.arange(101)*.1,
                            p=np.zeros((101,3)), R=np.tile(np.eye(3),(101,1,1)))
    # Both requests round to the same .3s epochs; same physical GM process.
    g1 = simulate_gnss(truth, GnssSpec(rate_hz=3., mp_tau=2.), seed=7)
    g2 = simulate_gnss(truth, GnssSpec(rate_hz=1/.3, mp_tau=2.), seed=7)
    np.testing.assert_array_equal(g1.t, g2.t)
    np.testing.assert_array_equal(g1.pos, g2.pos)


def test_gnss_rate_above_grid_rejected():
    from ccgins.sim import GnssSpec, simulate_gnss
    truth = SimpleNamespace(dt=.1)
    with pytest.raises(ValueError, match='grid'):
        simulate_gnss(truth, GnssSpec(rate_hz=100.))


def test_bias_diagnostics_are_optional_and_do_not_change_navigation():
    from ccgins.pipeline import Config, run
    from ccgins.sim import GnssSpec, generate_trajectory, simulate_gnss
    truth = generate_trajectory(2., dt=.02, seed=9)
    spec, gs = ImuSpec(), GnssSpec()
    imu = simulate_imu(truth, spec, seed=9)
    gnss = simulate_gnss(truth, gs, seed=9)
    plain = run(truth, imu, gnss, spec, gs, Config(pseudo='nhc'))
    diag = run(truth, imu, gnss, spec, gs, Config(pseudo='nhc'), log_covariance=True)
    assert plain.extra['bias_est'] is None
    assert diag.extra['bias_est'].shape == (len(diag.t), 6)
    assert np.all(np.isfinite(diag.extra['bias_est']))
    np.testing.assert_array_equal(plain.p_est, diag.p_est)
    np.testing.assert_array_equal(plain.P_pos, diag.P_pos)
    # First logged state is before any GNSS epoch; this guards against
    # accidentally logging simulator bias truth as the estimated state.
    assert not np.array_equal(diag.extra['bias_est'][0,:3], imu.gyro_bias[10])
