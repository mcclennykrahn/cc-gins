import numpy as np
from scipy.stats import chi2

from ccgins.aci import ACICalibrator
from ccgins.integrity import Cusum


def test_aci_recovers_scale_and_coverage():
    rng = np.random.default_rng(0)
    cal = ACICalibrator(2, alpha=0.05, window=400)
    s2 = np.array([0.04, 0.01])
    true_scale = np.array([9.0, 0.25])          # net over-/under-confident
    hits = []
    for t in range(4000):
        r = rng.standard_normal(2) * np.sqrt(true_scale * s2)
        var = cal.effective_var(s2)
        if t > 1000:
            hits.append(r**2 / var <= chi2.ppf(0.95, 1))
        cal.observe(r, s2, np.zeros(2))
    assert np.allclose(cal.kappa, true_scale, rtol=0.35)
    cov = np.mean(hits, axis=0)
    assert np.all(np.abs(cov - 0.95) < 0.03)


def test_outage_inflation_and_disable():
    cal = ACICalibrator(1, beta=0.5, t_ref=60.0)
    cal.kappa[:] = 2.0
    assert np.isclose(cal.effective_var(np.array([1.0]), 120.0)[0], 2.0 * 2.0)
    off = ACICalibrator(1, enabled=False)
    assert off.effective_var(np.array([3.0]), 100.0)[0] == 3.0


def test_cusum_detects_mean_shift():
    rng = np.random.default_rng(1)
    c = Cusum(k=3.0, h=40.0)
    q0 = chi2.rvs(3, size=500, random_state=rng)
    assert not any(c.step(q) for q in q0)
    alarms = [c.step(q + 10.0) for q in chi2.rvs(3, size=50, random_state=rng)]
    assert any(alarms) and alarms.index(True) < 10
