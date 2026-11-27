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


@pytest.mark.parametrize("rate", [10.1, 15., 100.])
def test_gnss_rate_above_grid_rejected(rate):
    from ccgins.sim import GnssSpec, simulate_gnss
    truth = SimpleNamespace(dt=.1)
    with pytest.raises(ValueError, match='grid'):
        simulate_gnss(truth, GnssSpec(rate_hz=rate))


@pytest.mark.parametrize("rate", [0., -1., np.nan, np.inf])
def test_invalid_gnss_rate_rejected(rate):
    from ccgins.sim import GnssSpec, simulate_gnss
    with pytest.raises(ValueError, match="finite and positive"):
        simulate_gnss(SimpleNamespace(dt=.1), GnssSpec(rate_hz=rate))


def test_gnss_rate_at_grid_limit():
    from ccgins.sim import GnssSpec, simulate_gnss
    truth = SimpleNamespace(dt=.1, t=np.arange(4)*.1,
                            p=np.zeros((4, 3)), R=np.tile(np.eye(3), (4, 1, 1)))
    gnss = simulate_gnss(truth, GnssSpec(rate_hz=10.))
    np.testing.assert_array_equal(gnss.idx, [1, 2, 3])


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


def test_fractional_outage_boundaries_agree_between_simulator_and_metrics():
    from ccgins.cli import _extend_metrics
    from ccgins.sim import GnssSpec, simulate_gnss
    from scripts.audit_scheme import segment_masks

    t = np.arange(22) * .03
    outages = [(.33, .27)]
    assert t[11] < .33
    assert t[20] < sum(outages[0])
    truth = SimpleNamespace(dt=.03, t=t, p=np.zeros((22, 3)),
                            R=np.tile(np.eye(3), (22, 1, 1)))
    gnss = simulate_gnss(truth, GnssSpec(rate_hz=1 / truth.dt, outages=outages))
    np.testing.assert_array_equal(gnss.idx[~gnss.valid], np.arange(11, 20))

    error = np.zeros((22, 3))
    error[[11, 19, 20], 0] = [7, 4, 100]
    log = SimpleNamespace(t=t, p_est=error, p_true=truth.p, P_pos=truth.R,
                          runtime_per_step_us=0.)
    metrics = _extend_metrics(log, outages, t_start=.33)
    assert metrics['n_metrics_epochs'] == 11
    assert metrics['max_err_outage_0s'] == metrics['outage_metrics'][0]['max_err_h'] == 7
    assert metrics['end_err_outage_0s'] == metrics['outage_metrics'][0]['end_err_h'] == 4
    masks = segment_masks(t, outages, t_start=.33)
    np.testing.assert_array_equal(np.flatnonzero(masks['outage_0.33_0.6']), np.arange(11, 20))
    np.testing.assert_array_equal(np.flatnonzero(masks['available_0.6_0.63']), [20])


def test_off_grid_outage_is_not_rounded_to_nearest_epoch():
    from ccgins.timing import in_window

    t = np.arange(22) * .03
    np.testing.assert_array_equal(np.flatnonzero(in_window(t, .331, .601)), np.arange(12, 21))


def test_timing_preserves_nonfinite_ordered_comparisons():
    from ccgins.timing import at_or_after, in_window

    t = np.array([1., np.finfo(float).max, np.inf, np.nan])
    np.testing.assert_array_equal(at_or_after(t, np.inf), [False, False, True, False])
    np.testing.assert_array_equal(in_window(t, 0., np.inf), [True, True, False, False])
    assert not in_window(t, 0., np.nan).any()
