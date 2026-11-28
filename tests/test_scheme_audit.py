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


def test_logged_gnss_age_matches_posterior_and_ignores_rejected_fixes():
    from ccgins.pipeline import Config, run
    from ccgins.sim import GnssSpec, generate_trajectory, simulate_gnss

    truth = generate_trajectory(5., dt=.01, seed=900)
    spec = ImuSpec()
    gs = GnssSpec(outages=[(2., 1.)])
    imu = simulate_imu(truth, spec, seed=900)
    gnss = simulate_gnss(truth, gs, seed=900)
    gnss.pos[gnss.t == 3., 0] += 1e6

    log = run(truth, imu, gnss, spec, gs, Config(pseudo="nhc"))
    epochs = [1., 2., 3., 4., 4.1]
    idx = np.searchsorted(log.t, epochs)
    np.testing.assert_allclose(log.t[idx], epochs)
    # A missing fix at 2 s and a rejected fix at 3 s preserve the 1 s reference.
    np.testing.assert_allclose(log.extra["since_gnss"][idx], [0., 1., 2., 0., .1])


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


@pytest.mark.parametrize('bounds,actual', [((.331, .27), (.36, .6)),
                                         ((.33, .27), (.33, .6))])
def test_pure_ins_uses_only_contained_complete_steps(bounds, actual):
    from scripts.audit_scheme import pure_ins_budget
    t = np.arange(22) * .03
    truth = SimpleNamespace(t=t, dt=.03, R=np.tile(np.eye(3), (22, 1, 1)),
                            p=np.zeros((22, 3)), v=np.zeros((22, 3)),
                            v_body=np.zeros((22, 3)))
    # Constant 2m/s² horizontal error: p(T)=T². Vertical gravity cancels.
    imu = SimpleNamespace(gyro=np.zeros((22, 3)),
                          accel=np.tile([2., 0., 9.81], (22, 1)))
    row = pure_ins_budget(truth, imu, [bounds])[0]
    assert row['actual_start'] == pytest.approx(actual[0])
    assert row['actual_end'] == pytest.approx(actual[1])
    assert row['end_h'] == pytest.approx((actual[1] - actual[0])**2)
    assert row['end_v'] == pytest.approx(0.)
    assert row['propagated_duration'] == pytest.approx(row['n_steps'] * truth.dt)


@pytest.mark.parametrize('outage', [(-.01, .3), (.5, .5), (.331, .01), (.3, 0),
                                     (.3, np.nan), (.3, np.inf)])
def test_pure_ins_rejects_unrepresentable_intervals(outage):
    from scripts.audit_scheme import pure_ins_budget
    truth = SimpleNamespace(t=np.arange(22)*.03)
    with pytest.raises(ValueError, match='pure INS outage'):
        pure_ins_budget(truth, None, [outage])


def test_budget_nondefault_scoring_start_and_empty_window():
    from scripts.audit_scheme import budget
    t = np.arange(22)*.03
    e = np.zeros((22, 3))
    e[:11, 0] = 1000.
    e[11:, 0] = 2.
    rows = budget(t, e, [(.33, .27)], t_start=.33)
    assert rows['overall']['n'] == 11
    assert rows['overall']['rmse_h'] == 2.
    assert rows['outage_0.33_0.6']['n'] == 9
    with pytest.raises(ValueError, match='no epochs'):
        budget(t, e, [], 1.)


def test_audit_cli_honors_configured_scoring_and_fractional_duration(tmp_path, monkeypatch):
    import json
    import sys
    import yaml
    from scripts.audit_scheme import main
    config = dict(scenario=dict(seed=9, duration=.63, dt=.03),
                  gnss=dict(rate_hz=1/.03, outages=[dict(start=.331, duration=.27)]),
                  metrics=dict(t_start=.33))
    path = tmp_path / 'scenario.yaml'
    path.write_text(yaml.safe_dump(config))
    out = tmp_path / 'evidence'
    monkeypatch.setattr(sys, 'argv', ['audit_scheme', '--config', str(path),
                                     '--out', str(out), '--seeds', '9'])
    main()
    result = json.loads((out / 'budget.json').read_text())
    row = result['rows'][0]
    assert 't>=0.33;' in result['scoring']
    assert row['segments']['overall']['n'] == 1  # navigation logs every tenth IMU step
    assert row['segments']['outage_0.331_0.601']['n'] == 1
    assert row['gnss_only']['n'] == 2  # .33 and .63; .60 remains in the gap
    assert len(row['bins']) == 1
    assert row['bins'][0]['start'] == .33
    assert row['bins'][0]['n'] == 1  # .60 is logged; .63 is not
    assert row['pure_ins_truth_initialized'][0]['propagated_duration'] == pytest.approx(.24)

    config['gnss']['outages'] = []
    path.write_text(yaml.safe_dump(config))
    main()
    segments = json.loads((out / 'budget.json').read_text())['rows'][0]['segments']
    assert segments['available_0.33_0.63']['n'] == 1
    assert 'terminal_available' not in segments


@pytest.mark.parametrize('start', [-1., float('nan'), float('inf')])
def test_audit_cli_rejects_invalid_scoring_start(tmp_path, monkeypatch, start):
    import sys
    import yaml
    from scripts.audit_scheme import main
    config = tmp_path / 'invalid.yaml'
    config.write_text(yaml.safe_dump(dict(metrics=dict(t_start=start))))
    monkeypatch.setattr(sys, 'argv', ['audit_scheme', '--config', str(config),
                                     '--out', str(tmp_path / 'evidence')])
    with pytest.raises(ValueError, match='metrics.t_start'):
        main()


@pytest.mark.parametrize('start,duration,first,last', [
    (.331, .27, 12, 20), (.33, .27, 11, 20),
])
def test_component_replay_contains_steps_and_matches_budget(start, duration, first, last):
    from scripts.audit_ins_components import integrate, metrics
    from scripts.audit_scheme import pure_ins_budget
    t = np.arange(22)*.03
    truth = SimpleNamespace(t=t, dt=.03, R=np.tile(np.eye(3), (22, 1, 1)),
                            p=np.zeros((22, 3)), v=np.zeros((22, 3)),
                            v_body=np.zeros((22, 3)))
    gyro = np.zeros((22, 3))
    accel = np.tile([2., 0., 9.81], (22, 1))
    # Samples outside the contained interval must never enter the replay.
    accel[:first] = 1e6
    accel[last:] = 1e6
    epochs, errors = integrate(truth, gyro, accel, start, duration)
    np.testing.assert_array_equal(epochs, t[first+1:last+1])
    assert metrics(errors)['end_h'] == pytest.approx((t[last]-t[first])**2)
    row = pure_ins_budget(truth, SimpleNamespace(gyro=gyro, accel=accel), [(start, duration)])[0]
    assert metrics(errors)['rmse_h'] == row['rmse_h']
    assert metrics(errors)['end_h'] == row['end_h']


@pytest.mark.parametrize('start,duration', [(-.01, .3), (.5, .5), (.331, .01),
                                            (.3, 0), (.3, np.nan), (.3, np.inf)])
def test_component_replay_rejects_unrepresentable_interval(start, duration):
    from scripts.audit_ins_components import integrate
    truth = SimpleNamespace(t=np.arange(22)*.03, dt=.03)
    with pytest.raises(ValueError, match='pure INS outage'):
        integrate(truth, None, None, start, duration)
