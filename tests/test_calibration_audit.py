from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from ccgins.sim import GnssSpec, generate_trajectory, simulate_gnss
from scripts.audit_calibration import position_consistency


def test_multipath_ablation_preserves_white_draws_and_schedule():
    truth = generate_trajectory(duration=4., dt=.01, seed=900)
    spec = GnssSpec(mp_sigma=.5)
    nominal = simulate_gnss(truth, spec, seed=900)
    white = simulate_gnss(truth, replace(spec, mp_sigma=0.), seed=900)
    rng = np.random.default_rng(20900)
    expected = rng.standard_normal((len(white.t), 3)) * white.sigma
    antenna = truth.p[white.idx] + np.einsum('nij,j->ni', truth.R[white.idx], spec.lever_arm)
    np.testing.assert_allclose(white.pos - antenna, expected, atol=1e-13)
    np.testing.assert_array_equal(nominal.idx, white.idx)
    np.testing.assert_array_equal(nominal.valid, white.valid)
    np.testing.assert_allclose(nominal.pos[0], white.pos[0])  # MP starts at zero
    assert np.linalg.norm(nominal.pos[1:] - white.pos[1:]) > 0


def test_position_consistency_uses_full_covariance_and_half_open_windows():
    t = np.array([30., 59.9, 60., 89.9, 90., 119.9, 120., 179.9, 180.])
    covariance = np.array([[2., 1., 0.], [1., 2., 0.], [0., 0., 1.]])
    error = np.tile([2., 2., 1.], (len(t), 1))
    log = SimpleNamespace(t=t, p_est=error, p_true=np.zeros_like(error),
                          P_pos=np.tile(covariance, (len(t), 1, 1)))
    result = position_consistency(log, [(60., 30.), (120., 60.)])
    expected = (8/3 + 1)/3
    for name in ('available_30_60', 'available_90_120', 'outage_60_90', 'outage_120_180'):
        row = result['segments'][name]
        assert row['n'] == 2
        np.testing.assert_allclose(row['position_nees_per_dim'], expected)
        assert row['position_95pct_coverage'] == 1.
    assert result['segments']['terminal_available']['n'] == 1
    assert len(result['checkpoints']) == 5


def test_position_consistency_respects_changed_outages_and_scoring_start():
    t = np.arange(10., 101., 10.)
    error = np.column_stack([t / 10, np.zeros((len(t), 2))])
    log = SimpleNamespace(t=t, p_est=error, p_true=np.zeros_like(error),
                          P_pos=np.tile(np.eye(3), (len(t), 1, 1)))
    result = position_consistency(log, [(40., 20.)], t_start=20.)['segments']
    assert 'outage_60_90' not in result
    assert result['outage_40_60']['n'] == 2
    np.testing.assert_allclose(result['outage_40_60']['position_nees_per_dim'], (16+25)/6)
    np.testing.assert_allclose(result['available']['position_nees_per_dim'],
                               np.mean(np.array([2, 3, 6, 7, 8, 9, 10])**2)/3)
    assert result['overall']['n'] == 9
    assert result['terminal_available']['position_nees_per_dim'] == 100/3
