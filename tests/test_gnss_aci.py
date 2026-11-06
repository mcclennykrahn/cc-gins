from dataclasses import replace
import numpy as np
import pytest
from scipy.stats import chi2
from ccgins.aci import GNSSACICalibrator
from ccgins.learning import grouped_correlation_factor, correlation_factor
from ccgins.pipeline import Config, run
from ccgins.scenarios import make_scenario
from ccgins.stream import StreamingNavigator


def test_joint_nis_scale_with_projected_uncertainty():
    rng = np.random.default_rng(43)
    R = np.array([[1., .3, 0], [.3, 2., .2], [0, .2, .7]])
    C = np.diag([.5, .8, .2])
    cal = GNSSACICalibrator(gamma=0, window=3000, min_samples=2000, update_every=2000)
    rs = rng.multivariate_normal(np.zeros(3), C+4*R, 2000)
    for r in rs:
        cal.observe(r, R, C)
    assert cal.q_target == chi2.ppf(.95, 3)
    assert 3.5 < cal.kappa < 4.5
    assert abs(np.quantile(cal._nis(rs, R, C, cal.kappa), .95)-cal.q_target) < 1e-6
    # Orthogonal coordinate change preserves joint NIS, including cross covariance.
    Q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    np.testing.assert_allclose(cal._nis(rs@Q.T, Q@R@Q.T, Q@C@Q.T, cal.kappa),
                               cal._nis(rs, R, C, cal.kappa), atol=1e-12)


def test_pipeline_gnss_scale_is_next_fix_only_and_outage_freezes(monkeypatch):
    from ccgins.filters import NavFilter
    used_covariances = []
    original = NavFilter.update_gnss
    def capture(self, z, lever, Rm, **kwargs):
        used_covariances.append(Rm.copy())
        return original(self, z, lever, Rm, **kwargs)
    monkeypatch.setattr(NavFilter, "update_gnss", capture)
    sc = make_scenario(700, duration=85., outages=[(65., 10.)])
    cfg = Config(pseudo='nhc', gnss_aci=True, gnss_gate=1e20)
    log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, cfg)
    rows = log.extra['gnss_aci']
    assert rows and any(r['kappa_next'] != 1 for r in rows)
    assert all(not (65 <= r['t'] < 75) for r in rows)
    assert rows[0]['kappa_used'] == 1
    for row, covariance in zip(rows, used_covariances):
        np.testing.assert_allclose(covariance, row['kappa_used']*np.diag(sc.gnss.sigma**2))
    for a, b in zip(rows, rows[1:]):
        assert b['kappa_used'] == a['kappa_next']
        if not b['eligible']:
            assert b['kappa_used'] == b['kappa_next']
    with pytest.raises(ValueError, match='cannot be combined'):
        run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec, replace(cfg, adaptive_r=True))
    with pytest.raises(ValueError, match='streaming supports'):
        StreamingNavigator.from_truth(sc.truth, sc.imu_spec, sc.gnss_spec, cfg)


def test_grouped_correlation_has_no_between_drive_mean_or_boundary_pairs():
    rng = np.random.default_rng(8)
    a, b = rng.normal(size=(1000, 2)), rng.normal(size=(800, 2))
    expected = (correlation_factor(a)+correlation_factor(b))/2
    np.testing.assert_allclose(grouped_correlation_factor([a+1000, b-1000]), expected, atol=1e-11)
    assert np.all(correlation_factor(np.concatenate([a+1000, b-1000])) > 100)
    np.testing.assert_array_equal(grouped_correlation_factor([np.ones((3, 2))]), [1, 1])
    with pytest.raises(ValueError):
        grouped_correlation_factor([])


@pytest.mark.parametrize('options', [dict(gnss_gate=-1., max_consecutive_rejects=10000),
    dict(gnss_gate=-1., max_consecutive_rejects=0),
    dict(monitor='single', cusum_h=-1.)])
def test_rejected_forced_and_alarm_excluded_fixes_never_train_gnss_scale(options):
    sc = make_scenario(701, duration=40.)
    log = run(sc.truth, sc.imu, sc.gnss, sc.imu_spec, sc.gnss_spec,
              Config(gnss_aci=True, **options))
    assert all(not r['eligible'] and r['kappa_next'] == 1 for r in log.extra['gnss_aci'])
    if options.get('monitor') == 'single':
        assert log.alarm_time is not None
        assert log.extra['gnss_aci'] == []
